// Diagnostic-only offline path-family feasibility audit.
//
// This translation unit deliberately compiles the production planner implementation directly.
// The access-specifier override is confined to this BUILD_TESTING executable and only exposes
// read-only geometry/validation entry points that are otherwise private.  The production library
// and local_planner_node are not linked to this executable and remain unchanged.

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include <algorithm>
#include <array>
#include <bitset>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#define private public
#include "local_planning/raceline_spline_planner.hpp"
#undef private

#include "local_planning/obstacle_guard.hpp"
#include "obstacle_detector/aabb_frenet_projector.hpp"
#include "../../src/local_planning/src/raceline_spline_planner.cpp"

namespace path_family_audit
{

using local_planning::EgoFrenetState;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;

constexpr double kAuditEpsilon = 1.0e-9;

struct Frame
{
  std::int64_t logical_stamp_ns{0};
  std::int64_t record_stamp_ns{0};
  EgoFrenetState ego;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};

struct SnapshotStream
{
  std::string scenario;
  std::string source_bag;
  std::string config_path;
  std::string config_sha256;
  std::map<std::string, double> scalars;
  std::map<std::string, std::vector<double>> vectors;
  f110_msgs::msg::WpntArray reference;
  std::vector<Frame> frames;
};

struct CandidateSummary
{
  std::size_t generation_index{0U};
  bool feasible{false};
  bool go_left{false};
  double target_d{std::numeric_limits<double>::quiet_NaN()};
  std::array<double, 3> anchors{
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  double requested_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double effective_entry_length_m{std::numeric_limits<double>::quiet_NaN()};
  double exit_length_m{std::numeric_limits<double>::quiet_NaN()};
  double minimum_normalized_safety_slack{-std::numeric_limits<double>::infinity()};
  double peak_lateral_slope{std::numeric_limits<double>::infinity()};
  double peak_curvature_radpm{std::numeric_limits<double>::infinity()};
  double peak_curvature_rate_radpm2{std::numeric_limits<double>::infinity()};
  double footprint_wall_clearance_m{-std::numeric_limits<double>::infinity()};
  double centerline_wall_clearance_m{-std::numeric_limits<double>::infinity()};
  double obstacle_clearance_m{-std::numeric_limits<double>::infinity()};
  double velocity_loss{std::numeric_limits<double>::infinity()};
  double global_path_deviation_m{std::numeric_limits<double>::infinity()};
  double footprint_violation_m{std::numeric_limits<double>::infinity()};
  double obstacle_violation_m{std::numeric_limits<double>::infinity()};
  double curvature_excess_radpm{std::numeric_limits<double>::infinity()};
  double slope_excess{std::numeric_limits<double>::infinity()};
  double curvature_rate_excess_radpm2{std::numeric_limits<double>::infinity()};
  double normalized_worst_violation{std::numeric_limits<double>::infinity()};
  std::string rejection_reason;
  std::string normalized_rejection_reason;
  std::string geometry_hash;
};

struct SideResult
{
  bool go_left{false};
  std::string gate_reason;
  std::vector<CandidateSummary> candidates;
  std::map<std::string, std::size_t> rejection_histogram;
  std::optional<CandidateSummary> best;
  double runtime_ms{0.0};
};

struct FamilyResult
{
  std::string name;
  SideResult left;
  SideResult right;
  std::map<std::string, std::size_t> rejection_histogram;
  std::optional<CandidateSummary> best;
  std::size_t generated{0U};
  std::size_t feasible{0U};
  double runtime_ms{0.0};
  bool evaluated{true};
};

struct ScenarioResult
{
  SnapshotStream stream;
  Frame frame;
  std::size_t frame_index{0U};
  std::string input_reconstruction;
  std::vector<FamilyResult> families;
  std::string classification;
  std::string dominant_constraint;
  std::string secondary_constraints;
  std::string digest;
};

struct ExpectedBaseline
{
  std::size_t left_generated{0U};
  std::size_t right_generated{0U};
  std::string left_mode;
  std::string right_mode;
};

struct Representative
{
  std::size_t frame_index{0U};
  Frame planning_frame;
  FamilyResult family_a;
  std::string input_reconstruction;
};

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> tokens;
  std::size_t start = 0U;
  while (true) {
    const auto end = line.find('\t', start);
    tokens.push_back(line.substr(start, end == std::string::npos ? end : end - start));
    if (end == std::string::npos) {
      break;
    }
    start = end + 1U;
  }
  return tokens;
}

double parseDouble(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite numeric token: " + token);
  }
  return value;
}

std::int64_t parseInt64(const std::string & token)
{
  std::size_t consumed = 0U;
  const auto value = std::stoll(token, &consumed);
  if (consumed != token.size()) {
    throw std::runtime_error("invalid integer token: " + token);
  }
  return value;
}

std::size_t parseSize(const std::string & token)
{
  const auto value = parseInt64(token);
  if (value < 0) {
    throw std::runtime_error("negative size token: " + token);
  }
  return static_cast<std::size_t>(value);
}

SnapshotStream readStream(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open snapshot stream: " + path.string());
  }
  SnapshotStream stream;
  stream.reference.header.frame_id = "map";
  std::string line;
  if (!std::getline(input, line) || line != "PATH_FAMILY_STREAM_V1") {
    throw std::runtime_error("unsupported snapshot stream: " + path.string());
  }
  while (std::getline(input, line)) {
    const auto tokens = splitTabs(line);
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "SCENARIO" && tokens.size() == 2U) {
      stream.scenario = tokens[1];
    } else if (tokens[0] == "SOURCE_BAG" && tokens.size() == 2U) {
      stream.source_bag = tokens[1];
    } else if (tokens[0] == "CONFIG_PATH" && tokens.size() == 2U) {
      stream.config_path = tokens[1];
    } else if (tokens[0] == "CONFIG_SHA256" && tokens.size() == 2U) {
      stream.config_sha256 = tokens[1];
    } else if (tokens[0] == "PARAM" && tokens.size() == 3U) {
      stream.scalars[tokens[1]] = parseDouble(tokens[2]);
    } else if (tokens[0] == "PARAMV" && tokens.size() >= 3U) {
      const std::size_t count = parseSize(tokens[2]);
      if (tokens.size() != count + 3U) {
        throw std::runtime_error("parameter vector count mismatch: " + tokens[1]);
      }
      auto & values = stream.vectors[tokens[1]];
      values.reserve(count);
      for (std::size_t i = 0U; i < count; ++i) {
        values.push_back(parseDouble(tokens[i + 3U]));
      }
    } else if (tokens[0] == "REFERENCE" && tokens.size() == 2U) {
      stream.reference.wpnts.reserve(parseSize(tokens[1]));
    } else if (tokens[0] == "W" && tokens.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(parseInt64(tokens[1]));
      waypoint.s_m = parseDouble(tokens[2]);
      waypoint.d_m = parseDouble(tokens[3]);
      waypoint.x_m = parseDouble(tokens[4]);
      waypoint.y_m = parseDouble(tokens[5]);
      waypoint.d_right = parseDouble(tokens[6]);
      waypoint.d_left = parseDouble(tokens[7]);
      waypoint.psi_rad = parseDouble(tokens[8]);
      waypoint.kappa_radpm = parseDouble(tokens[9]);
      waypoint.vx_mps = parseDouble(tokens[10]);
      waypoint.ax_mps2 = parseDouble(tokens[11]);
      stream.reference.wpnts.push_back(waypoint);
    } else if (tokens[0] == "FRAME" && tokens.size() == 7U) {
      Frame frame;
      frame.logical_stamp_ns = parseInt64(tokens[1]);
      frame.record_stamp_ns = parseInt64(tokens[2]);
      frame.ego.s = parseDouble(tokens[3]);
      frame.ego.d = parseDouble(tokens[4]);
      frame.ego.speed = parseDouble(tokens[5]);
      const std::size_t obstacle_count = parseSize(tokens[6]);
      frame.obstacles.reserve(obstacle_count);
      for (std::size_t i = 0U; i < obstacle_count; ++i) {
        if (!std::getline(input, line)) {
          throw std::runtime_error("truncated obstacle frame");
        }
        const auto obstacle_tokens = splitTabs(line);
        if (obstacle_tokens.size() != 10U || obstacle_tokens[0] != "O") {
          throw std::runtime_error("malformed obstacle record");
        }
        f110_msgs::msg::Obstacle obstacle;
        obstacle.id = static_cast<std::int32_t>(parseInt64(obstacle_tokens[1]));
        obstacle.s_center = parseDouble(obstacle_tokens[2]);
        obstacle.s_start = parseDouble(obstacle_tokens[3]);
        obstacle.s_end = parseDouble(obstacle_tokens[4]);
        obstacle.d_right = parseDouble(obstacle_tokens[5]);
        obstacle.d_left = parseDouble(obstacle_tokens[6]);
        obstacle.size = parseDouble(obstacle_tokens[7]);
        obstacle.s_var = parseDouble(obstacle_tokens[8]);
        obstacle.d_var = parseDouble(obstacle_tokens[9]);
        obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
        obstacle.is_static = true;
        obstacle.is_visible = true;
        frame.obstacles.push_back(obstacle);
      }
      if (!std::getline(input, line) || line != "END_FRAME") {
        throw std::runtime_error("missing END_FRAME marker");
      }
      stream.frames.push_back(std::move(frame));
    } else if (tokens[0] == "END_STREAM") {
      break;
    } else {
      throw std::runtime_error("unknown snapshot record: " + line);
    }
  }
  if (stream.scenario.empty() || stream.reference.wpnts.empty() || stream.frames.empty()) {
    throw std::runtime_error("incomplete snapshot stream: " + path.string());
  }
  return stream;
}

RacelineSplineParameters parametersFromStream(const SnapshotStream & stream)
{
  RacelineSplineParameters parameters;
  const auto scalar = [&](const std::string & name) {return stream.scalars.at(name);};
  const auto vector = [&](const std::string & name) {return stream.vectors.at(name);};
  parameters.detection_lookahead_m = scalar("detection_lookahead_m");
  parameters.obstacle_cluster_gap_m = scalar("obstacle_cluster_gap_m");
  parameters.obstacle_longitudinal_padding_m = scalar("obstacle_longitudinal_padding_m");
  parameters.vehicle_length_m = scalar("vehicle_length_m");
  parameters.vehicle_half_width_m = scalar("vehicle_half_width_m");
  parameters.safety_margin_m = scalar("safety_margin_m");
  parameters.tracking_error_reserve_m = scalar("tracking_error_reserve_m");
  parameters.tracking_error_lut_speed_bins_mps =
    vector("tracking_error_lut_speed_bins_mps");
  parameters.tracking_error_lut_curvature_bins_radpm =
    vector("tracking_error_lut_curvature_bins_radpm");
  parameters.tracking_error_lut_values_m = vector("tracking_error_lut_values_m");
  parameters.avoidance_velocity_limit_speed_bins_mps =
    vector("avoidance_velocity_limit_speed_bins_mps");
  parameters.avoidance_velocity_limit_lateral_accel_mps2 =
    vector("avoidance_velocity_limit_lateral_accel_mps2");
  parameters.wall_safety_margin_m = scalar("wall_safety_margin_m");
  parameters.fallback_track_half_width_m = scalar("fallback_track_half_width_m");
  parameters.pre_apex_distances_m = vector("pre_apex_distances_m");
  parameters.post_apex_distances_m = vector("post_apex_distances_m");
  parameters.entry_transition_fractions = vector("entry_transition_fractions");
  parameters.transition_distance_scales = vector("transition_distance_scales");
  parameters.outside_line_transition_scale = scalar("outside_line_transition_scale");
  parameters.post_merge_lookahead_m = scalar("post_merge_lookahead_m");
  parameters.post_merge_min_time_sec = scalar("post_merge_min_time_sec");
  parameters.minimum_target_offset_m = scalar("minimum_target_offset_m");
  parameters.maximum_target_offset_m = scalar("maximum_target_offset_m");
  parameters.target_d_candidate_count =
    static_cast<int>(std::llround(scalar("target_d_candidate_count")));
  parameters.maximum_lateral_slope = scalar("maximum_lateral_slope");
  parameters.maximum_curvature_radpm = scalar("maximum_curvature_radpm");
  parameters.maximum_curvature_rate_radpm2 = scalar("maximum_curvature_rate_radpm2");
  parameters.safe_stop_buffer_m = scalar("safe_stop_buffer_m");
  parameters.safe_stop_deceleration_mps2 = scalar("safe_stop_deceleration_mps2");
  parameters.minimum_path_points =
    static_cast<int>(std::llround(scalar("minimum_path_points")));
  if (!parameters.trackingErrorLutValid() || !parameters.avoidanceVelocityLimitValid()) {
    throw std::runtime_error("snapshot contains an invalid production LUT");
  }
  return parameters;
}

std::string normalizeReason(const std::string & reason)
{
  if (reason.find("footprint_track_bound") != std::string::npos ||
    reason.find("track bounds") != std::string::npos)
  {
    return "track_bound";
  }
  if (reason.find("maximum_lateral_slope") != std::string::npos) {
    return "maximum_lateral_slope";
  }
  if (reason.find("maximum_curvature_rate_radpm2") != std::string::npos) {
    return "maximum_curvature_rate";
  }
  if (reason.find("maximum_curvature_radpm") != std::string::npos) {
    return "maximum_curvature";
  }
  if (reason.find("static-obstacle") != std::string::npos) {
    return "obstacle_clearance";
  }
  if (reason.find("target d exceeds track bound") != std::string::npos) {
    return "target_range_track_bound";
  }
  if (reason.empty()) {
    return "feasible";
  }
  return reason;
}

std::uint64_t fnvAppend(std::uint64_t hash, const void * data, std::size_t size)
{
  const auto * bytes = static_cast<const unsigned char *>(data);
  for (std::size_t i = 0U; i < size; ++i) {
    hash ^= static_cast<std::uint64_t>(bytes[i]);
    hash *= 1099511628211ULL;
  }
  return hash;
}

std::uint64_t fnvAppendString(std::uint64_t hash, const std::string & value)
{
  return fnvAppend(hash, value.data(), value.size());
}

std::string hexHash(std::uint64_t hash)
{
  std::ostringstream stream;
  stream << std::hex << std::setfill('0') << std::setw(16) << hash;
  return stream.str();
}

std::string geometryHash(const f110_msgs::msg::WpntArray & path)
{
  std::uint64_t hash = 1469598103934665603ULL;
  const std::uint64_t count = path.wpnts.size();
  hash = fnvAppend(hash, &count, sizeof(count));
  for (const auto & waypoint : path.wpnts) {
    for (const double value : std::array<double, 8>{
        waypoint.s_m, waypoint.d_m, waypoint.x_m, waypoint.y_m,
        waypoint.psi_rad, waypoint.kappa_radpm, waypoint.vx_mps, waypoint.ax_mps2})
    {
      std::uint64_t bits = 0U;
      static_assert(sizeof(bits) == sizeof(value));
      std::memcpy(&bits, &value, sizeof(value));
      hash = fnvAppend(hash, &bits, sizeof(bits));
    }
  }
  return hexHash(hash);
}

double peakSlope(
  const RacelineSplinePlanner & planner,
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path)
{
  double peak = 0.0;
  if (path.wpnts.size() < 2U) {
    return peak;
  }
  double previous_forward = planner.forwardDistance(ego.s, path.wpnts.front().s_m);
  double previous_d = path.wpnts.front().d_m;
  for (std::size_t i = 1U; i < path.wpnts.size(); ++i) {
    const double forward = planner.forwardDistance(ego.s, path.wpnts[i].s_m);
    const double ds = forward - previous_forward;
    if (ds > kAuditEpsilon) {
      peak = std::max(peak, std::abs(path.wpnts[i].d_m - previous_d) / ds);
    }
    previous_forward = forward;
    previous_d = path.wpnts[i].d_m;
  }
  return peak;
}

CandidateSummary summarizeCandidate(
  const RacelineSplinePlanner & planner,
  const EgoFrenetState & ego,
  const RacelineSplinePlanner::Candidate & candidate,
  const RacelineSplineParameters & parameters,
  const std::array<double, 3> & anchors)
{
  CandidateSummary summary;
  summary.generation_index = candidate.audit_index;
  summary.feasible = candidate.valid;
  summary.go_left = candidate.go_left;
  summary.target_d = candidate.target_d;
  summary.anchors = anchors;
  summary.requested_entry_length_m = candidate.requested_entry_length_m;
  summary.effective_entry_length_m = candidate.effective_entry_length_m;
  summary.exit_length_m = candidate.exit_length_m;
  summary.minimum_normalized_safety_slack = candidate.minimum_normalized_safety_slack;
  summary.peak_lateral_slope = peakSlope(planner, ego, candidate.path);
  summary.peak_curvature_radpm = candidate.peak_curvature_radpm;
  summary.peak_curvature_rate_radpm2 = candidate.peak_curvature_rate_radpm2;
  summary.footprint_wall_clearance_m = candidate.rectangular_footprint_wall_clearance_m;
  summary.centerline_wall_clearance_m = candidate.centerline_wall_clearance_m;
  summary.obstacle_clearance_m = candidate.obstacle_clearance_m;
  summary.velocity_loss = candidate.velocity_loss;
  summary.global_path_deviation_m = candidate.global_path_deviation_m;
  summary.footprint_violation_m = std::max(0.0, -summary.footprint_wall_clearance_m);
  summary.obstacle_violation_m = std::max(0.0, -summary.obstacle_clearance_m);
  summary.curvature_excess_radpm = std::max(
    0.0, summary.peak_curvature_radpm - parameters.maximum_curvature_radpm);
  summary.slope_excess = std::max(
    0.0, summary.peak_lateral_slope - parameters.maximum_lateral_slope);
  summary.curvature_rate_excess_radpm2 = std::max(
    0.0, summary.peak_curvature_rate_radpm2 - parameters.maximum_curvature_rate_radpm2);
  const double lateral_norm = std::max(kAuditEpsilon, parameters.maximum_target_offset_m);
  summary.normalized_worst_violation = std::max({
      summary.footprint_violation_m / lateral_norm,
      summary.obstacle_violation_m / lateral_norm,
      summary.curvature_excess_radpm /
      std::max(kAuditEpsilon, parameters.maximum_curvature_radpm),
      summary.slope_excess / std::max(kAuditEpsilon, parameters.maximum_lateral_slope),
      summary.curvature_rate_excess_radpm2 /
      std::max(kAuditEpsilon, parameters.maximum_curvature_rate_radpm2)});
  summary.rejection_reason = candidate.reason;
  summary.normalized_rejection_reason = candidate.valid ?
    "feasible" : normalizeReason(candidate.reason);
  summary.geometry_hash = geometryHash(candidate.path);
  return summary;
}

bool betterFeasible(const CandidateSummary & first, const CandidateSummary & second)
{
  const double slack_delta =
    first.minimum_normalized_safety_slack - second.minimum_normalized_safety_slack;
  if (std::abs(slack_delta) > kAuditEpsilon) {
    return slack_delta > 0.0;
  }
  const double velocity_delta = first.velocity_loss - second.velocity_loss;
  if (std::abs(velocity_delta) > kAuditEpsilon) {
    return velocity_delta < 0.0;
  }
  const double deviation_delta = first.global_path_deviation_m - second.global_path_deviation_m;
  if (std::abs(deviation_delta) > kAuditEpsilon) {
    return deviation_delta < 0.0;
  }
  return first.generation_index < second.generation_index;
}

bool betterClosest(const CandidateSummary & first, const CandidateSummary & second)
{
  if (std::abs(first.normalized_worst_violation - second.normalized_worst_violation) >
    kAuditEpsilon)
  {
    return first.normalized_worst_violation < second.normalized_worst_violation;
  }
  if (std::abs(first.minimum_normalized_safety_slack -
    second.minimum_normalized_safety_slack) > kAuditEpsilon)
  {
    return first.minimum_normalized_safety_slack > second.minimum_normalized_safety_slack;
  }
  return first.generation_index < second.generation_index;
}

void finalizeSide(SideResult & side)
{
  const CandidateSummary * best_feasible = nullptr;
  const CandidateSummary * best_closest = nullptr;
  for (const auto & candidate : side.candidates) {
    if (!candidate.feasible) {
      ++side.rejection_histogram[candidate.normalized_rejection_reason];
    }
    if (candidate.feasible &&
      (best_feasible == nullptr || betterFeasible(candidate, *best_feasible)))
    {
      best_feasible = &candidate;
    }
    if (best_closest == nullptr || betterClosest(candidate, *best_closest)) {
      best_closest = &candidate;
    }
  }
  if (best_feasible != nullptr) {
    side.best = *best_feasible;
  } else if (best_closest != nullptr) {
    side.best = *best_closest;
  }
  if (side.candidates.empty() && !side.gate_reason.empty()) {
    ++side.rejection_histogram[normalizeReason(side.gate_reason)];
  }
}

std::vector<double> uniqueSorted(std::vector<double> values)
{
  std::sort(values.begin(), values.end());
  values.erase(
    std::unique(
      values.begin(), values.end(),
      [](double first, double second) {return std::abs(first - second) <= 1.0e-12;}),
    values.end());
  return values;
}

std::vector<double> entryScales(
  const RacelineSplineParameters & parameters, bool extended)
{
  auto values = parameters.entry_transition_fractions;
  if (extended) {
    values.push_back(
      parameters.detection_lookahead_m / parameters.pre_apex_distances_m.front());
  }
  return uniqueSorted(std::move(values));
}

std::vector<double> exitScales(
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const EgoFrenetState & ego,
  double cluster_end,
  bool go_left,
  bool outside_is_left,
  bool extended)
{
  auto values = parameters.transition_distance_scales;
  if (!extended || values.empty()) {
    return uniqueSorted(std::move(values));
  }
  const double outside_multiplier = go_left == outside_is_left ?
    parameters.outside_line_transition_scale : 1.0;
  const double nominal_post_far = parameters.post_apex_distances_m.back();
  const double current_max_raw = *std::max_element(values.begin(), values.end());
  const double current_effective_length =
    nominal_post_far * current_max_raw * outside_multiplier;
  const double tail = std::max(
    parameters.post_merge_lookahead_m,
    std::abs(ego.speed) * parameters.post_merge_min_time_sec);
  const double horizon_limit = std::max(
    0.0, planner.trackLength() - cluster_end - tail - 1.0e-3);
  for (const double factor : {1.5, 2.0}) {
    const double requested = current_effective_length * factor;
    const double effective = std::min(requested, horizon_limit);
    if (effective > current_effective_length + kAuditEpsilon &&
      nominal_post_far * outside_multiplier > kAuditEpsilon)
    {
      values.push_back(effective / (nominal_post_far * outside_multiplier));
    }
  }
  return uniqueSorted(std::move(values));
}

SideResult runConstantSide(
  const SnapshotStream & stream,
  const Frame & frame,
  bool go_left,
  bool extended_entry,
  bool extended_exit)
{
  const auto started = std::chrono::steady_clock::now();
  const auto parameters = parametersFromStream(stream);
  RacelineSplinePlanner planner(parameters);
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid production reference: " + error);
  }
  SideResult side;
  side.go_left = go_left;
  const auto visible = planner.expandVisibleObstacles(frame.ego, frame.obstacles);
  const auto cluster = planner.nearestCluster(visible);
  if (cluster.empty()) {
    side.gate_reason = "no static obstacle blocks the global race line";
    finalizeSide(side);
    return side;
  }
  const bool outside_is_left = planner.outsideIsLeft(frame.ego, cluster);
  double cluster_start = 0.0;
  double cluster_end = 0.0;
  double minimum_target = 0.0;
  double maximum_target = 0.0;
  if (!planner.computeSideTargetRange(
      frame.ego, cluster, go_left, cluster_start, cluster_end,
      minimum_target, maximum_target, side.gate_reason))
  {
    finalizeSide(side);
    side.runtime_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
    return side;
  }
  const int requested_count = std::max(1, parameters.target_d_candidate_count);
  const bool collapsed = std::abs(maximum_target - minimum_target) <= kAuditEpsilon;
  const int target_count = collapsed ? 1 : requested_count;
  const auto entries = entryScales(parameters, extended_entry);
  const auto exits = exitScales(
    planner, parameters, frame.ego, cluster_end, go_left, outside_is_left, extended_exit);
  std::size_t generation = 0U;
  for (int target_index = 0; target_index < target_count; ++target_index) {
    const double ratio = target_count == 1 ? 0.0 :
      static_cast<double>(target_index) / static_cast<double>(target_count - 1);
    const double target = minimum_target + ratio * (maximum_target - minimum_target);
    for (const double entry : entries) {
      for (const double exit : exits) {
        auto candidate = planner.buildCandidate(
          frame.ego, visible, go_left, entry, exit, outside_is_left,
          cluster_start, cluster_end, target);
        candidate.audit_index = generation++;
        side.candidates.push_back(summarizeCandidate(
            planner, frame.ego, candidate, parameters,
            {target, target, target}));
      }
    }
  }
  finalizeSide(side);
  side.runtime_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return side;
}

struct QuinticSegment
{
  double start{0.0};
  double end{0.0};
  std::array<double, 6> coefficients{};

  double evaluate(double s) const
  {
    const double t = std::clamp((s - start) / (end - start), 0.0, 1.0);
    double result = coefficients[5];
    for (int i = 4; i >= 0; --i) {
      result = result * t + coefficients[static_cast<std::size_t>(i)];
    }
    return result;
  }
};

QuinticSegment quinticHermite(
  double start, double end,
  double d0, double v0, double a0,
  double d1, double v1, double a1)
{
  const double h = end - start;
  if (!(h > kAuditEpsilon)) {
    throw std::runtime_error("non-positive quintic-Hermite segment");
  }
  QuinticSegment segment;
  segment.start = start;
  segment.end = end;
  auto & c = segment.coefficients;
  c[0] = d0;
  c[1] = h * v0;
  c[2] = 0.5 * h * h * a0;
  const double r0 = d1 - c[0] - c[1] - c[2];
  const double r1 = h * v1 - c[1] - 2.0 * c[2];
  const double r2 = h * h * a1 - 2.0 * c[2];
  c[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
  c[4] = -15.0 * r0 + 7.0 * r1 - r2;
  c[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
  return segment;
}

std::vector<QuinticSegment> makeC2Profile(
  const std::array<double, 5> & stations,
  const std::array<double, 5> & offsets)
{
  std::array<double, 5> derivative{};
  std::array<double, 5> acceleration{};
  for (std::size_t i = 1U; i + 1U < stations.size(); ++i) {
    const double h_previous = stations[i] - stations[i - 1U];
    const double h_next = stations[i + 1U] - stations[i];
    const double slope_previous = (offsets[i] - offsets[i - 1U]) / h_previous;
    const double slope_next = (offsets[i + 1U] - offsets[i]) / h_next;
    if (slope_previous * slope_next > 0.0) {
      derivative[i] = (h_previous + h_next) /
        (h_previous / slope_previous + h_next / slope_next);
    } else {
      derivative[i] = 0.0;
    }
    acceleration[i] = 2.0 * (slope_next - slope_previous) /
      (h_previous + h_next);
  }
  // Production assumes zero lateral velocity and acceleration at the ego-side start and at the
  // final d=0 merge.  Internal anchors share the same finite derivative/acceleration from both
  // adjacent quintics, so d, d', and d'' are continuous without artificial all-zero joins.
  derivative.front() = 0.0;
  derivative.back() = 0.0;
  acceleration.front() = 0.0;
  acceleration.back() = 0.0;

  std::vector<QuinticSegment> segments;
  segments.reserve(4U);
  for (std::size_t i = 0U; i + 1U < stations.size(); ++i) {
    segments.push_back(quinticHermite(
        stations[i], stations[i + 1U],
        offsets[i], derivative[i], acceleration[i],
        offsets[i + 1U], derivative[i + 1U], acceleration[i + 1U]));
  }
  return segments;
}

double evaluateProfile(
  const std::vector<QuinticSegment> & segments,
  double forward_s,
  double ego_d)
{
  if (forward_s <= segments.front().start) {
    return ego_d;
  }
  for (const auto & segment : segments) {
    if (forward_s <= segment.end) {
      return segment.evaluate(forward_s);
    }
  }
  return 0.0;
}

std::vector<double> localAnchorSamples(
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const EgoFrenetState & ego,
  const std::vector<RacelineSplinePlanner::ExpandedObstacle> & cluster,
  double station,
  bool go_left,
  std::string & error)
{
  const auto & reference = planner.reference_.wpnts[
    planner.nearestReferenceIndex(planner.wrapS(ego.s + station))];
  const double reserve = parameters.trackBoundaryReserve(
    reference.vx_mps, reference.kappa_radpm);
  const double left_width = reference.d_left > 0.05 ?
    reference.d_left : parameters.fallback_track_half_width_m;
  const double right_width = reference.d_right > 0.05 ?
    reference.d_right : parameters.fallback_track_half_width_m;

  std::vector<const RacelineSplinePlanner::ExpandedObstacle *> local_obstacles;
  for (const auto & obstacle : cluster) {
    if (station >= obstacle.start - kAuditEpsilon && station <= obstacle.end + kAuditEpsilon) {
      local_obstacles.push_back(&obstacle);
    }
  }
  if (local_obstacles.empty()) {
    const auto nearest = std::min_element(
      cluster.begin(), cluster.end(),
      [station](const auto & first, const auto & second) {
        const double first_distance = std::min(
          std::abs(station - first.start), std::abs(station - first.end));
        const double second_distance = std::min(
          std::abs(station - second.start), std::abs(station - second.end));
        return first_distance < second_distance;
      });
    local_obstacles.push_back(&*nearest);
  }

  double obstacle_limit = go_left ?
    -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
  for (const auto * obstacle : local_obstacles) {
    obstacle_limit = go_left ?
      std::max(obstacle_limit, obstacle->d_left) :
      std::min(obstacle_limit, obstacle->d_right);
  }
  if (go_left) {
    obstacle_limit = std::max(obstacle_limit, parameters.minimum_target_offset_m);
    const double track_limit = std::min(
      parameters.maximum_target_offset_m, left_width - reserve);
    if (obstacle_limit > track_limit + kAuditEpsilon) {
      error = "local left obstacle/track interval is empty";
      return {};
    }
    return uniqueSorted({obstacle_limit, 0.5 * (obstacle_limit + track_limit), track_limit});
  }
  obstacle_limit = std::min(obstacle_limit, -parameters.minimum_target_offset_m);
  const double track_limit = std::max(
    -parameters.maximum_target_offset_m, -right_width + reserve);
  if (track_limit > obstacle_limit + kAuditEpsilon) {
    error = "local right obstacle/track interval is empty";
    return {};
  }
  return uniqueSorted({track_limit, 0.5 * (track_limit + obstacle_limit), obstacle_limit});
}

RacelineSplinePlanner::Candidate buildVariableCandidate(
  RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const Frame & frame,
  const std::vector<RacelineSplinePlanner::ExpandedObstacle> & visible,
  bool go_left,
  double cluster_start,
  double cluster_end,
  const std::array<double, 3> & anchors,
  double entry_scale,
  double raw_exit_scale,
  bool outside_is_left,
  std::size_t generation)
{
  RacelineSplinePlanner::Candidate candidate;
  candidate.audit_index = generation;
  candidate.go_left = go_left;
  candidate.target_d = anchors[1];
  candidate.entry_transition_scale = entry_scale;
  candidate.exit_transition_scale = raw_exit_scale;
  double effective_exit_scale = raw_exit_scale;
  if (go_left == outside_is_left) {
    effective_exit_scale *= parameters.outside_line_transition_scale;
  }
  candidate.effective_exit_transition_scale = effective_exit_scale;
  const double requested_entry = parameters.pre_apex_distances_m.front() * entry_scale;
  const double effective_entry_fraction = requested_entry / parameters.detection_lookahead_m;
  if (!(effective_entry_fraction > kAuditEpsilon) ||
    effective_entry_fraction > 1.0 + kAuditEpsilon)
  {
    candidate.reason = "requested entry fraction must be within the available-distance interval";
    return candidate;
  }
  const double entry_length = cluster_start * effective_entry_fraction;
  const double exit_length = parameters.post_apex_distances_m.back() * effective_exit_scale;
  if (!(entry_length > kAuditEpsilon) || !(exit_length > kAuditEpsilon)) {
    candidate.reason = "quintic transition length is not positive";
    return candidate;
  }
  candidate.requested_entry_length_m = requested_entry;
  candidate.effective_entry_length_m = entry_length;
  candidate.effective_entry_transition_scale = effective_entry_fraction;
  candidate.exit_length_m = exit_length;
  const double entry_start = cluster_start - entry_length;
  const double cluster_mid = 0.5 * (cluster_start + cluster_end);
  const double exit_end = cluster_end + exit_length;
  if (!(entry_start + kAuditEpsilon < cluster_start &&
    cluster_start + kAuditEpsilon < cluster_mid &&
    cluster_mid + kAuditEpsilon < cluster_end &&
    cluster_end + kAuditEpsilon < exit_end))
  {
    candidate.reason = "variable-d quintic stations are not strictly increasing";
    return candidate;
  }
  const std::array<double, 5> stations{
    entry_start, cluster_start, cluster_mid, cluster_end, exit_end};
  const std::array<double, 5> offsets{
    frame.ego.d, anchors[0], anchors[1], anchors[2], 0.0};
  const auto segments = makeC2Profile(stations, offsets);
  const double tail = std::max(
    parameters.post_merge_lookahead_m,
    std::abs(frame.ego.speed) * parameters.post_merge_min_time_sec);
  const double path_end = exit_end + tail;
  const std::size_t first_index = planner.nextReferenceIndex(frame.ego.s);
  candidate.path.header = planner.reference_.header;
  for (std::size_t k = 0U; k < planner.reference_.wpnts.size(); ++k) {
    const std::size_t index = (first_index + k) % planner.reference_.wpnts.size();
    const auto & global = planner.reference_.wpnts[index];
    const double forward = planner.forwardDistance(frame.ego.s, global.s_m);
    if (forward > path_end + kAuditEpsilon) {
      break;
    }
    auto waypoint = global;
    waypoint.id = static_cast<std::int32_t>(candidate.path.wpnts.size());
    waypoint.d_m = evaluateProfile(segments, forward, frame.ego.d);
    waypoint.x_m = global.x_m - waypoint.d_m * std::sin(global.psi_rad);
    waypoint.y_m = global.y_m + waypoint.d_m * std::cos(global.psi_rad);
    candidate.path.wpnts.push_back(waypoint);
  }
  if (candidate.path.wpnts.size() < static_cast<std::size_t>(parameters.minimum_path_points)) {
    candidate.reason = "spline segment has too few global race-line samples";
    return candidate;
  }
  planner.updateGeometryAndAcceleration(candidate.path);
  planner.applyAvoidanceVelocityLimit(candidate.path);
  planner.updateGeometryAndAcceleration(candidate.path);
  planner.measureCandidate(frame.ego, visible, candidate);
  if (!planner.validateCandidate(frame.ego, candidate.path, visible, candidate.reason)) {
    return candidate;
  }
  candidate.valid = true;
  candidate.merge_s = planner.wrapS(frame.ego.s + exit_end);
  return candidate;
}

SideResult runVariableSide(
  const SnapshotStream & stream,
  const Frame & frame,
  bool go_left)
{
  const auto started = std::chrono::steady_clock::now();
  const auto parameters = parametersFromStream(stream);
  RacelineSplinePlanner planner(parameters);
  std::string reference_error;
  if (!planner.setReference(stream.reference, &reference_error)) {
    throw std::runtime_error("invalid production reference: " + reference_error);
  }
  SideResult side;
  side.go_left = go_left;
  const auto visible = planner.expandVisibleObstacles(frame.ego, frame.obstacles);
  const auto cluster = planner.nearestCluster(visible);
  if (cluster.empty()) {
    side.gate_reason = "no static obstacle blocks the global race line";
    finalizeSide(side);
    return side;
  }
  double cluster_start = cluster.front().start;
  double cluster_end = cluster.front().end;
  for (const auto & obstacle : cluster) {
    cluster_start = std::min(cluster_start, obstacle.start);
    cluster_end = std::max(cluster_end, obstacle.end);
  }
  const double cluster_mid = 0.5 * (cluster_start + cluster_end);
  std::array<std::vector<double>, 3> samples;
  for (std::size_t index = 0U; index < samples.size(); ++index) {
    const double station = std::array<double, 3>{
      cluster_start, cluster_mid, cluster_end}[index];
    samples[index] = localAnchorSamples(
      planner, parameters, frame.ego, cluster, station, go_left, side.gate_reason);
    if (samples[index].empty()) {
      finalizeSide(side);
      side.runtime_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started).count();
      return side;
    }
  }
  const bool outside_is_left = planner.outsideIsLeft(frame.ego, cluster);
  const auto entries = entryScales(parameters, true);
  const auto exits = exitScales(
    planner, parameters, frame.ego, cluster_end, go_left, outside_is_left, true);
  std::size_t generation = 0U;
  for (const double start_d : samples[0]) {
    for (const double mid_d : samples[1]) {
      for (const double end_d : samples[2]) {
        const std::array<double, 3> anchors{start_d, mid_d, end_d};
        for (const double entry : entries) {
          for (const double exit : exits) {
            auto candidate = buildVariableCandidate(
              planner, parameters, frame, visible, go_left,
              cluster_start, cluster_end, anchors, entry, exit,
              outside_is_left, generation++);
            side.candidates.push_back(summarizeCandidate(
                planner, frame.ego, candidate, parameters, anchors));
          }
        }
      }
    }
  }
  finalizeSide(side);
  side.runtime_ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - started).count();
  return side;
}

FamilyResult combineFamily(
  const std::string & name, SideResult left, SideResult right, bool evaluated = true)
{
  FamilyResult family;
  family.name = name;
  family.left = std::move(left);
  family.right = std::move(right);
  family.evaluated = evaluated;
  if (!evaluated) {
    return family;
  }
  family.generated = family.left.candidates.size() + family.right.candidates.size();
  family.feasible = static_cast<std::size_t>(std::count_if(
      family.left.candidates.begin(), family.left.candidates.end(),
      [](const CandidateSummary & candidate) {return candidate.feasible;})) +
    static_cast<std::size_t>(std::count_if(
      family.right.candidates.begin(), family.right.candidates.end(),
      [](const CandidateSummary & candidate) {return candidate.feasible;}));
  family.runtime_ms = family.left.runtime_ms + family.right.runtime_ms;
  for (const auto & item : family.left.rejection_histogram) {
    family.rejection_histogram[item.first] += item.second;
  }
  for (const auto & item : family.right.rejection_histogram) {
    family.rejection_histogram[item.first] += item.second;
  }
  for (const auto * side : {&family.left, &family.right}) {
    if (!side->best.has_value()) {
      continue;
    }
    const auto & candidate = side->best.value();
    if (!family.best.has_value() ||
      (candidate.feasible && !family.best->feasible) ||
      (candidate.feasible && family.best->feasible && betterFeasible(candidate, *family.best)) ||
      (!candidate.feasible && !family.best->feasible && betterClosest(candidate, *family.best)))
    {
      family.best = candidate;
    }
  }
  return family;
}

FamilyResult runConstantFamily(
  const SnapshotStream & stream,
  const Frame & frame,
  const std::string & name,
  bool extended_entry,
  bool extended_exit)
{
  return combineFamily(
    name,
    runConstantSide(stream, frame, true, extended_entry, extended_exit),
    runConstantSide(stream, frame, false, extended_entry, extended_exit));
}

FamilyResult runVariableFamily(const SnapshotStream & stream, const Frame & frame)
{
  return combineFamily(
    "C",
    runVariableSide(stream, frame, true),
    runVariableSide(stream, frame, false));
}

ExpectedBaseline expectedFor(const std::string & scenario)
{
  if (scenario == "validation_004") {
    return {45U, 0U, "track_bound", "target_range_track_bound"};
  }
  if (scenario == "validation_014") {
    return {0U, 45U, "target_range_track_bound", "maximum_lateral_slope"};
  }
  if (scenario == "validation_019") {
    // The historical narrow diagnostic reported a right-side target-range gate. A bounded
    // current-code snapshot recovery replay (source stamp 11.74 s) showed that the current
    // footprint-aware planner now constructs 45 right candidates; their dominant hard rejection
    // is maximum_lateral_slope. This current production event is the Family A audit baseline.
    return {45U, 45U, "maximum_curvature", "maximum_lateral_slope"};
  }
  if (scenario == "validation_034") {
    return {0U, 45U, "target_range_track_bound", "maximum_lateral_slope"};
  }
  if (scenario == "validation_039") {
    return {45U, 45U, "track_bound", "maximum_curvature"};
  }
  throw std::runtime_error("no baseline expectation for " + scenario);
}

std::string dominantReason(const SideResult & side)
{
  if (side.candidates.empty()) {
    return normalizeReason(side.gate_reason);
  }
  if (side.rejection_histogram.empty()) {
    return "feasible";
  }
  return std::max_element(
    side.rejection_histogram.begin(), side.rejection_histogram.end(),
    [](const auto & first, const auto & second) {
      if (first.second != second.second) {
        return first.second < second.second;
      }
      return first.first > second.first;
    })->first;
}

bool baselineMatches(const FamilyResult & family, const ExpectedBaseline & expected)
{
  const std::size_t left_feasible = static_cast<std::size_t>(std::count_if(
      family.left.candidates.begin(), family.left.candidates.end(),
      [](const CandidateSummary & candidate) {return candidate.feasible;}));
  const std::size_t right_feasible = static_cast<std::size_t>(std::count_if(
      family.right.candidates.begin(), family.right.candidates.end(),
      [](const CandidateSummary & candidate) {return candidate.feasible;}));
  return family.left.candidates.size() == expected.left_generated &&
         family.right.candidates.size() == expected.right_generated &&
         left_feasible == 0U && right_feasible == 0U &&
         dominantReason(family.left) == expected.left_mode &&
         dominantReason(family.right) == expected.right_mode;
}

double obstacleSpan(
  const RacelineSplinePlanner & planner,
  const f110_msgs::msg::Obstacle & obstacle)
{
  const double forward = planner.forwardDistance(obstacle.s_start, obstacle.s_end);
  const double reverse = planner.forwardDistance(obstacle.s_end, obstacle.s_start);
  const double span = std::min(forward, reverse);
  if (std::isfinite(span) && span > kAuditEpsilon) {
    return span;
  }
  return std::max(0.0, std::abs(obstacle.size));
}

double signedTrackDelta(
  const RacelineSplinePlanner & planner, double from_s, double to_s)
{
  double delta = planner.forwardDistance(from_s, to_s);
  if (delta > 0.5 * planner.trackLength()) {
    delta -= planner.trackLength();
  }
  return delta;
}

f110_msgs::msg::Obstacle mergeObstacleEnvelopes(
  const RacelineSplinePlanner & planner,
  const f110_msgs::msg::Obstacle & first,
  const f110_msgs::msg::Obstacle & second)
{
  auto merged = first;
  const double first_half = 0.5 * obstacleSpan(planner, first);
  const double second_half = 0.5 * obstacleSpan(planner, second);
  const double second_center = signedTrackDelta(planner, first.s_center, second.s_center);
  const double lower = std::min(-first_half, second_center - second_half);
  const double upper = std::max(first_half, second_center + second_half);
  const auto wrap = [&planner](double s) {
      const double wrapped = std::fmod(s, planner.trackLength());
      return wrapped < 0.0 ? wrapped + planner.trackLength() : wrapped;
    };
  merged.s_start = wrap(first.s_center + lower);
  merged.s_end = wrap(first.s_center + upper);
  merged.s_center = wrap(first.s_center + 0.5 * (lower + upper));
  merged.d_right = std::min(first.d_right, second.d_right);
  merged.d_left = std::max(first.d_left, second.d_left);
  merged.d_center = 0.5 * (merged.d_right + merged.d_left);
  merged.size = std::hypot(upper - lower, merged.d_left - merged.d_right);
  const auto conservative_variance = [](double first_value, double second_value) {
      const double finite_first = std::isfinite(first_value) && first_value >= 0.0 ?
        first_value : 0.0;
      const double finite_second = std::isfinite(second_value) && second_value >= 0.0 ?
        second_value : 0.0;
      return std::max(finite_first, finite_second);
    };
  merged.s_var = conservative_variance(first.s_var, second.s_var);
  merged.d_var = conservative_variance(first.d_var, second.d_var);
  return merged;
}

Frame guardedUnionFrame(
  const SnapshotStream & stream,
  const Frame & raw_frame,
  RacelineSplinePlanner & planner,
  std::map<int, f110_msgs::msg::Obstacle> & union_by_id)
{
  for (const auto & obstacle : raw_frame.obstacles) {
    const auto previous = union_by_id.find(obstacle.id);
    union_by_id[obstacle.id] = previous == union_by_id.end() ?
      obstacle : mergeObstacleEnvelopes(planner, previous->second, obstacle);
  }
  local_planning::ObstacleGuardParameters guard_parameters;
  guard_parameters.uncertainty_sigma_scale = stream.scalars.at("uncertainty_sigma_scale");
  guard_parameters.minimum_longitudinal_inflation_m =
    stream.scalars.at("uncertainty_min_longitudinal_inflation_m");
  guard_parameters.minimum_lateral_inflation_m =
    stream.scalars.at("uncertainty_min_lateral_inflation_m");
  guard_parameters.maximum_lateral_inflation_m =
    stream.scalars.at("uncertainty_max_lateral_inflation_m");
  Frame planning_frame = raw_frame;
  planning_frame.obstacles.clear();
  planning_frame.obstacles.reserve(union_by_id.size());
  for (const auto & item : union_by_id) {
    planning_frame.obstacles.push_back(local_planning::buildUncertaintyGuard(
        item.second, planner.trackLength(), guard_parameters));
  }
  return planning_frame;
}

Frame exactGuardedFrame(const SnapshotStream & stream, std::int64_t logical_stamp_ns)
{
  RacelineSplinePlanner input_planner(parametersFromStream(stream));
  std::string reference_error;
  if (!input_planner.setReference(stream.reference, &reference_error)) {
    throw std::runtime_error(
            "invalid reference while reconstructing exact guard: " + reference_error);
  }
  std::map<int, f110_msgs::msg::Obstacle> union_by_id;
  for (const auto & raw_frame : stream.frames) {
    const Frame planning_frame = guardedUnionFrame(
      stream, raw_frame, input_planner, union_by_id);
    if (raw_frame.logical_stamp_ns == logical_stamp_ns) {
      return planning_frame;
    }
  }
  throw std::runtime_error(
          "exact logical stamp is absent from snapshot stream: " +
          std::to_string(logical_stamp_ns));
}

int runExactProductionEntry(int argc, char ** argv)
{
  if (argc != 4 && argc != 5) {
    std::cerr <<
      "usage: path_family_feasibility_audit --exact-production-entry STAMP_NS STREAM [OUTPUT]\n";
    return 2;
  }
  const std::int64_t logical_stamp_ns = parseInt64(argv[2]);
  const SnapshotStream stream = readStream(argv[3]);
  const Frame frame = exactGuardedFrame(stream, logical_stamp_ns);
  const auto parameters = parametersFromStream(stream);
  std::ofstream output_file;
  std::ostream * output = &std::cout;
  if (argc == 5) {
    output_file.open(argv[4]);
    if (!output_file) {
      throw std::runtime_error("cannot open exact-entry output: " + std::string(argv[4]));
    }
    output = &output_file;
  }
  *output << std::setprecision(17);
  *output <<
    "scenario\tlogical_stamp_ns\tside\tgeneration_index\tfeasible\ttarget_d"
    "\trequested_entry_length_m\teffective_entry_length_m\texit_length_m"
    "\tminimum_normalized_safety_slack\tpeak_lateral_slope\tpeak_curvature_radpm"
    "\tpeak_curvature_rate_radpm2\tfootprint_wall_clearance_m"
    "\tobstacle_clearance_m\trejection_reason\tgeometry_hash\n";
  for (const bool go_left : {true, false}) {
    const SideResult side = runConstantSide(stream, frame, go_left, true, false);
    for (const auto & candidate : side.candidates) {
      if (std::abs(
          candidate.requested_entry_length_m - parameters.detection_lookahead_m) >
        kAuditEpsilon)
      {
        continue;
      }
      *output << stream.scenario << '\t' << logical_stamp_ns << '\t'
                << (go_left ? "left" : "right") << '\t'
                << candidate.generation_index << '\t'
                << (candidate.feasible ? "true" : "false") << '\t'
                << candidate.target_d << '\t'
                << candidate.requested_entry_length_m << '\t'
                << candidate.effective_entry_length_m << '\t'
                << candidate.exit_length_m << '\t'
                << candidate.minimum_normalized_safety_slack << '\t'
                << candidate.peak_lateral_slope << '\t'
                << candidate.peak_curvature_radpm << '\t'
                << candidate.peak_curvature_rate_radpm2 << '\t'
                << candidate.footprint_wall_clearance_m << '\t'
                << candidate.obstacle_clearance_m << '\t'
                << candidate.rejection_reason << '\t'
                << candidate.geometry_hash << '\n';
    }
  }
  return 0;
}

Representative selectRepresentative(const SnapshotStream & stream)
{
  const auto expected = expectedFor(stream.scenario);
  RacelineSplinePlanner input_planner(parametersFromStream(stream));
  std::string reference_error;
  if (!input_planner.setReference(stream.reference, &reference_error)) {
    throw std::runtime_error("invalid reference while reconstructing guard: " + reference_error);
  }
  std::map<int, f110_msgs::msg::Obstacle> union_by_id;
  std::optional<std::pair<std::size_t, FamilyResult>> first_post_debounce;
  std::set<std::string> observed_signatures;
  for (std::size_t index = 0U; index < stream.frames.size(); ++index) {
    const Frame planning_frame = guardedUnionFrame(
      stream, stream.frames[index], input_planner, union_by_id);
    // Three exact-stamp detector observations are enough for a static offline representative.
    // Do not reproduce node lifecycle timing here: the audit seeks the first geometry snapshot
    // that matches the already-recorded Family A candidate/rejection signature.
    if (index < 2U) {
      continue;
    }
    if (stream.scenario == "validation_019" &&
      stream.source_bag.find("path_family_capture_019") != std::string::npos &&
      stream.frames[index].logical_stamp_ns != 11740000000LL)
    {
      continue;
    }
    auto raw_family = runConstantFamily(stream, stream.frames[index], "A", false, false);
    observed_signatures.insert(
      "raw=" + std::to_string(raw_family.left.candidates.size()) + "/" +
      dominantReason(raw_family.left) + ":" +
      std::to_string(raw_family.right.candidates.size()) + "/" +
      dominantReason(raw_family.right));
    if (baselineMatches(raw_family, expected)) {
      return {index, stream.frames[index], std::move(raw_family), "raw_exact_stamp"};
    }
    auto family = runConstantFamily(stream, planning_frame, "A", false, false);
    observed_signatures.insert(
      "guarded=" + std::to_string(family.left.candidates.size()) + "/" +
      dominantReason(family.left) +
      ":" + std::to_string(family.right.candidates.size()) + "/" +
      dominantReason(family.right));
    if (!first_post_debounce.has_value()) {
      first_post_debounce = std::make_pair(index, family);
    }
    if (baselineMatches(family, expected)) {
      return {index, planning_frame, std::move(family), "cumulative_union_uncertainty_guard"};
    }
  }
  std::ostringstream message;
  message << "Family A did not reproduce baseline for " << stream.scenario;
  if (first_post_debounce.has_value()) {
    const auto & family = first_post_debounce->second;
    message << "; first eligible left=" << family.left.candidates.size()
            << "/" << dominantReason(family.left)
            << " right=" << family.right.candidates.size()
            << "/" << dominantReason(family.right);
  }
  message << "; observed=";
  bool first_signature = true;
  for (const auto & signature : observed_signatures) {
    if (!first_signature) {
      message << ',';
    }
    first_signature = false;
    message << signature;
  }
  throw std::runtime_error(message.str());
}

std::string histogramToken(const std::map<std::string, std::size_t> & histogram)
{
  std::ostringstream stream;
  bool first = true;
  for (const auto & item : histogram) {
    if (!first) {
      stream << ';';
    }
    first = false;
    stream << item.first << '=' << item.second;
  }
  return stream.str();
}

std::string cleanToken(std::string value)
{
  std::replace(value.begin(), value.end(), '\t', ' ');
  std::replace(value.begin(), value.end(), '\n', ' ');
  std::replace(value.begin(), value.end(), '\r', ' ');
  return value;
}

std::string finiteToken(double value)
{
  if (!std::isfinite(value)) {
    return "";
  }
  std::ostringstream stream;
  stream << std::setprecision(17) << value;
  return stream.str();
}

template<typename GetterT>
std::string uniqueNumericToken(const SideResult & side, GetterT getter)
{
  std::vector<double> values;
  values.reserve(side.candidates.size());
  for (const auto & candidate : side.candidates) {
    const double value = getter(candidate);
    if (std::isfinite(value)) {
      values.push_back(value);
    }
  }
  values = uniqueSorted(std::move(values));
  std::ostringstream stream;
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index > 0U) {
      stream << ';';
    }
    stream << std::setprecision(17) << values[index];
  }
  return stream.str();
}

std::size_t feasibleCount(const SideResult & side)
{
  return static_cast<std::size_t>(std::count_if(
      side.candidates.begin(), side.candidates.end(),
      [](const CandidateSummary & candidate) {return candidate.feasible;}));
}

std::string scenarioDigest(const ScenarioResult & result)
{
  std::uint64_t hash = 1469598103934665603ULL;
  hash = fnvAppendString(hash, result.stream.scenario);
  for (const auto & family : result.families) {
    hash = fnvAppendString(hash, family.name);
    hash = fnvAppend(hash, &family.generated, sizeof(family.generated));
    hash = fnvAppend(hash, &family.feasible, sizeof(family.feasible));
    hash = fnvAppendString(hash, histogramToken(family.rejection_histogram));
    if (family.best.has_value()) {
      hash = fnvAppendString(hash, family.best->geometry_hash);
    }
  }
  return hexHash(hash);
}

std::string dominantConstraint(const FamilyResult & family)
{
  if (family.rejection_histogram.empty()) {
    return family.feasible > 0U ? "none" : "unclassified";
  }
  return std::max_element(
    family.rejection_histogram.begin(), family.rejection_histogram.end(),
    [](const auto & first, const auto & second) {
      if (first.second != second.second) {
        return first.second < second.second;
      }
      return first.first > second.first;
    })->first;
}

ScenarioResult evaluateScenario(
  const SnapshotStream & stream,
  std::size_t frame_index,
  const Frame & planning_frame,
  const std::string & input_reconstruction,
  FamilyResult family_a)
{
  ScenarioResult result;
  result.stream = stream;
  result.frame = planning_frame;
  result.frame_index = frame_index;
  result.input_reconstruction = input_reconstruction;
  result.families.push_back(std::move(family_a));
  result.families.push_back(runConstantFamily(stream, result.frame, "B1", true, false));
  result.families.push_back(runConstantFamily(stream, result.frame, "B2", false, true));
  result.families.push_back(runConstantFamily(stream, result.frame, "B3", true, true));
  const auto & b1 = result.families[1];
  const auto & b2 = result.families[2];
  const auto & b3 = result.families[3];
  if (b1.feasible == 0U && b2.feasible == 0U && b3.feasible == 0U) {
    result.families.push_back(runVariableFamily(stream, result.frame));
  } else {
    result.families.push_back(combineFamily("C", SideResult(), SideResult(), false));
  }
  const auto & family_c = result.families[4];
  if (b1.feasible > 0U) {
    result.classification = "E";
    result.dominant_constraint = "production entry authority";
  } else if (b2.feasible > 0U) {
    result.classification = "X";
    result.dominant_constraint = "production exit authority";
  } else if (b3.feasible > 0U) {
    result.classification = "EX";
    result.dominant_constraint = "combined entry and exit authority";
  } else if (family_c.feasible > 0U) {
    result.classification = "T";
    result.dominant_constraint = "constant target_d across obstacle span";
  } else {
    const std::string dominant = dominantConstraint(family_c);
    result.classification = dominant == "unclassified" ? "M" : "P";
    result.dominant_constraint = dominant;
  }
  std::set<std::string> secondary;
  const auto & terminal = family_c.evaluated ? family_c : result.families[3];
  for (const auto & item : terminal.rejection_histogram) {
    if (item.first != result.dominant_constraint) {
      secondary.insert(item.first);
    }
  }
  std::ostringstream secondary_stream;
  bool first = true;
  for (const auto & item : secondary) {
    if (!first) {
      secondary_stream << ';';
    }
    first = false;
    secondary_stream << item;
  }
  result.secondary_constraints = secondary_stream.str();
  result.digest = scenarioDigest(result);
  return result;
}

void writeSnapshot(
  const ScenarioResult & result,
  const std::filesystem::path & output_path)
{
  std::ofstream output(output_path);
  if (!output) {
    throw std::runtime_error("cannot write snapshot: " + output_path.string());
  }
  output << std::setprecision(17);
  output << "PATH_FAMILY_SNAPSHOT_V1\n";
  output << "SCENARIO\t" << result.stream.scenario << '\n';
  output << "SOURCE_BAG\t" << result.stream.source_bag << '\n';
  output << "CONFIG_PATH\t" << result.stream.config_path << '\n';
  output << "CONFIG_SHA256\t" << result.stream.config_sha256 << '\n';
  output << "SELECTION\tfirst_frame_after_three_observations_matching_Family_A_baseline\n";
  output << "INPUT_RECONSTRUCTION\t" << result.input_reconstruction << '\n';
  output << "SOURCE_FRAME_INDEX\t" << result.frame_index << '\n';
  for (const auto & item : result.stream.scalars) {
    output << "PARAM\t" << item.first << '\t' << item.second << '\n';
  }
  for (const auto & item : result.stream.vectors) {
    output << "PARAMV\t" << item.first << '\t' << item.second.size();
    for (const double value : item.second) {
      output << '\t' << value;
    }
    output << '\n';
  }
  output << "REFERENCE\t" << result.stream.reference.wpnts.size() << '\n';
  for (const auto & waypoint : result.stream.reference.wpnts) {
    output << "W\t" << waypoint.id << '\t' << waypoint.s_m << '\t' << waypoint.d_m
           << '\t' << waypoint.x_m << '\t' << waypoint.y_m
           << '\t' << waypoint.d_right << '\t' << waypoint.d_left
           << '\t' << waypoint.psi_rad << '\t' << waypoint.kappa_radpm
           << '\t' << waypoint.vx_mps << '\t' << waypoint.ax_mps2 << '\n';
  }
  output << "FRAME\t" << result.frame.logical_stamp_ns << '\t'
         << result.frame.record_stamp_ns << '\t' << result.frame.ego.s << '\t'
         << result.frame.ego.d << '\t' << result.frame.ego.speed << '\t'
         << result.frame.obstacles.size() << '\n';
  for (const auto & obstacle : result.frame.obstacles) {
    output << "O\t" << obstacle.id << '\t' << obstacle.s_center << '\t'
           << obstacle.s_start << '\t' << obstacle.s_end << '\t'
           << obstacle.d_right << '\t' << obstacle.d_left << '\t'
           << obstacle.size << '\t' << obstacle.s_var << '\t' << obstacle.d_var << '\n';
  }
  output << "END_FRAME\nEND_SNAPSHOT\n";
}

void writeFamilyRow(
  std::ofstream & output,
  const std::string & scenario,
  const std::string & family_name,
  const std::string & side_name,
  const SideResult & side)
{
  const auto feasible = feasibleCount(side);
  output << scenario << '\t' << family_name << '\t' << side_name << '\t'
         << side.candidates.size() << '\t' << feasible << '\t'
         << finiteToken(side.runtime_ms) << '\t' << cleanToken(side.gate_reason) << '\t'
         << histogramToken(side.rejection_histogram);
  if (side.best.has_value()) {
    const auto & best = side.best.value();
    output << '\t' << (best.feasible ? "feasible" : "closest")
           << '\t' << finiteToken(best.minimum_normalized_safety_slack)
           << '\t' << finiteToken(best.peak_lateral_slope)
           << '\t' << finiteToken(best.peak_curvature_radpm)
           << '\t' << finiteToken(best.peak_curvature_rate_radpm2)
           << '\t' << finiteToken(best.footprint_wall_clearance_m)
           << '\t' << finiteToken(best.obstacle_clearance_m)
           << '\t' << finiteToken(best.effective_entry_length_m)
           << '\t' << finiteToken(best.exit_length_m)
           << '\t' << best.geometry_hash;
  } else {
    output << "\t\t\t\t\t\t\t\t\t\t";
  }
  output << '\t' << uniqueNumericToken(
    side, [](const CandidateSummary & candidate) {
      return candidate.effective_entry_length_m;
    })
         << '\t' << uniqueNumericToken(
    side, [](const CandidateSummary & candidate) {return candidate.exit_length_m;})
         << '\n';
}

void writeRawResults(
  const std::vector<ScenarioResult> & results,
  const std::filesystem::path & output_directory,
  double total_wall_s,
  double total_cpu_s,
  const std::string & determinism_scenario,
  const std::string & first_digest,
  const std::string & second_digest)
{
  std::ofstream scenario_output(output_directory / "raw_scenarios.tsv");
  std::ofstream family_output(output_directory / "raw_families.tsv");
  std::ofstream best_output(output_directory / "raw_best_candidates.tsv");
  if (!scenario_output || !family_output || !best_output) {
    throw std::runtime_error("cannot write raw audit outputs");
  }
  scenario_output << "scenario\tframe_index\tlogical_stamp_ns\tego_s\tego_d\tego_speed"
    "\tinput_reconstruction\tsource_bag\tconfig_sha256\tclassification"
    "\tdominant_constraint\tsecondary_constraints\tdigest\n";
  family_output << "scenario\tfamily\tside\tgenerated\tfeasible\truntime_ms\tgate_reason"
    "\trejection_histogram\tbest_status\tbest_slack\tbest_peak_slope"
    "\tbest_peak_curvature\tbest_peak_curvature_rate\tbest_footprint_clearance"
    "\tbest_obstacle_clearance\tbest_entry_length\tbest_exit_length\tbest_geometry_hash"
    "\tentry_lengths_used\texit_lengths_used\n";
  best_output << "scenario\tfamily\tside\tstatus\tgeneration_index\ttarget_d\td_start"
    "\td_mid\td_end\tentry_length\texit_length\tsafety_slack\tpeak_slope"
    "\tpeak_curvature\tpeak_curvature_rate\tfootprint_clearance\tobstacle_clearance"
    "\tfootprint_violation\tobstacle_violation\tcurvature_excess\tslope_excess"
    "\tcurvature_rate_excess\tnormalized_worst_violation\trejection_reason\tgeometry_hash\n";
  for (const auto & result : results) {
    scenario_output << result.stream.scenario << '\t' << result.frame_index << '\t'
                    << result.frame.logical_stamp_ns << '\t'
                    << finiteToken(result.frame.ego.s) << '\t'
                    << finiteToken(result.frame.ego.d) << '\t'
                    << finiteToken(result.frame.ego.speed) << '\t'
                    << result.input_reconstruction << '\t'
                    << result.stream.source_bag << '\t' << result.stream.config_sha256 << '\t'
                    << result.classification << '\t' << result.dominant_constraint << '\t'
                    << result.secondary_constraints << '\t' << result.digest << '\n';
    for (const auto & family : result.families) {
      if (!family.evaluated) {
        family_output << result.stream.scenario << '\t' << family.name
                      << "\tall\t0\t0\t0\tnot_needed\t\t\t\t\t\t\t\t\t\t\t\t\t\n";
        continue;
      }
      writeFamilyRow(family_output, result.stream.scenario, family.name, "left", family.left);
      writeFamilyRow(family_output, result.stream.scenario, family.name, "right", family.right);
      SideResult aggregate;
      aggregate.candidates.reserve(family.generated);
      aggregate.candidates.insert(
        aggregate.candidates.end(), family.left.candidates.begin(), family.left.candidates.end());
      aggregate.candidates.insert(
        aggregate.candidates.end(), family.right.candidates.begin(), family.right.candidates.end());
      aggregate.rejection_histogram = family.rejection_histogram;
      aggregate.best = family.best;
      aggregate.runtime_ms = family.runtime_ms;
      writeFamilyRow(family_output, result.stream.scenario, family.name, "all", aggregate);
      for (const auto & side : std::array<const SideResult *, 2>{&family.left, &family.right}) {
        if (!side->best.has_value()) {
          continue;
        }
        const auto & best = side->best.value();
        best_output << result.stream.scenario << '\t' << family.name << '\t'
                    << (side->go_left ? "left" : "right") << '\t'
                    << (best.feasible ? "feasible" : "closest") << '\t'
                    << best.generation_index << '\t' << finiteToken(best.target_d) << '\t'
                    << finiteToken(best.anchors[0]) << '\t' << finiteToken(best.anchors[1]) << '\t'
                    << finiteToken(best.anchors[2]) << '\t'
                    << finiteToken(best.effective_entry_length_m) << '\t'
                    << finiteToken(best.exit_length_m) << '\t'
                    << finiteToken(best.minimum_normalized_safety_slack) << '\t'
                    << finiteToken(best.peak_lateral_slope) << '\t'
                    << finiteToken(best.peak_curvature_radpm) << '\t'
                    << finiteToken(best.peak_curvature_rate_radpm2) << '\t'
                    << finiteToken(best.footprint_wall_clearance_m) << '\t'
                    << finiteToken(best.obstacle_clearance_m) << '\t'
                    << finiteToken(best.footprint_violation_m) << '\t'
                    << finiteToken(best.obstacle_violation_m) << '\t'
                    << finiteToken(best.curvature_excess_radpm) << '\t'
                    << finiteToken(best.slope_excess) << '\t'
                    << finiteToken(best.curvature_rate_excess_radpm2) << '\t'
                    << finiteToken(best.normalized_worst_violation) << '\t'
                    << cleanToken(best.rejection_reason) << '\t' << best.geometry_hash << '\n';
      }
    }
  }
  std::ofstream performance(output_directory / "raw_performance.tsv");
  performance << "total_wall_s\ttotal_cpu_s\treplay_required\tdeterminism_scenario"
    "\tfirst_digest\tsecond_digest\tdeterministic\n";
  const bool replay_required = std::any_of(
    results.begin(), results.end(),
    [](const ScenarioResult & result) {
      return result.stream.source_bag.find("path_family_capture") != std::string::npos;
    });
  performance << std::setprecision(17) << total_wall_s << '\t' << total_cpu_s
              << '\t' << (replay_required ? "true" : "false") << '\t'
              << determinism_scenario << '\t' << first_digest << '\t'
              << second_digest << '\t' << (first_digest == second_digest ? "true" : "false")
              << '\n';
}

ScenarioResult rerunScenario(const ScenarioResult & original)
{
  auto selected = selectRepresentative(original.stream);
  return evaluateScenario(
    original.stream, selected.frame_index, selected.planning_frame,
    selected.input_reconstruction,
    std::move(selected.family_a));
}

double availableEntryDistance(const FamilyResult & family)
{
  double available = std::numeric_limits<double>::quiet_NaN();
  for (const auto * side : {&family.left, &family.right}) {
    for (const auto & candidate : side->candidates) {
      if (!std::isfinite(candidate.effective_entry_length_m)) {
        continue;
      }
      if (!std::isfinite(available) || candidate.effective_entry_length_m > available) {
        available = candidate.effective_entry_length_m;
      }
    }
  }
  return available;
}

void writeTimeAxisFamily(std::ostream & output, const FamilyResult & family)
{
  output << '\t' << family.generated << '\t' << family.feasible << '\t'
         << dominantConstraint(family) << '\t' << histogramToken(family.rejection_histogram)
         << '\t' << finiteToken(availableEntryDistance(family));
  if (!family.best.has_value()) {
    output << "\t\t\t\t\t\t\t\t\t";
    return;
  }
  const auto & best = family.best.value();
  output << '\t' << (best.go_left ? "left" : "right")
         << '\t' << finiteToken(best.target_d)
         << '\t' << finiteToken(best.footprint_wall_clearance_m)
         << '\t' << finiteToken(best.obstacle_clearance_m)
         << '\t' << finiteToken(best.peak_curvature_radpm)
         << '\t' << finiteToken(best.peak_lateral_slope)
         << '\t' << finiteToken(best.peak_curvature_rate_radpm2)
         << '\t' << cleanToken(best.normalized_rejection_reason)
         << '\t' << best.geometry_hash;
}

int runTimeAxis(int argc, char ** argv)
{
  if (argc != 9) {
    std::cerr <<
      "usage: path_family_feasibility_audit --time-axis STREAM OUTPUT "
      "GT_S_CENTER GT_S_START GT_S_END GT_D_RIGHT GT_D_LEFT\n";
    return 2;
  }
  const SnapshotStream stream = readStream(argv[2]);
  std::ofstream output(argv[3]);
  if (!output) {
    throw std::runtime_error("cannot open time-axis output: " + std::string(argv[3]));
  }
  f110_msgs::msg::Obstacle gt;
  gt.id = -999;
  gt.s_center = parseDouble(argv[4]);
  gt.s_start = parseDouble(argv[5]);
  gt.s_end = parseDouble(argv[6]);
  gt.d_right = parseDouble(argv[7]);
  gt.d_left = parseDouble(argv[8]);
  gt.d_center = 0.5 * (gt.d_right + gt.d_left);
  gt.size = std::hypot(gt.s_end - gt.s_start, gt.d_left - gt.d_right);
  gt.s_var = 0.0;
  gt.d_var = 0.0;
  gt.is_static = true;
  gt.is_visible = true;

  RacelineSplinePlanner input_planner(parametersFromStream(stream));
  std::string reference_error;
  if (!input_planner.setReference(stream.reference, &reference_error)) {
    throw std::runtime_error("invalid reference in time-axis audit: " + reference_error);
  }
  std::map<int, f110_msgs::msg::Obstacle> real_union;
  std::map<int, f110_msgs::msg::Obstacle> oracle_union;
  output << std::setprecision(17);
  output << "scenario\tevent_index\tlogical_stamp_ns\tego_s\tego_d\tego_speed"
    "\tstatic_obs_present\traw_obstacle_count\tplanning_obstacle_count"
    "\tplanning_obstacle_id\tplanning_obstacle_s_start\tplanning_obstacle_s_end"
    "\tplanning_obstacle_d_right\tplanning_obstacle_d_left";
  for (const auto & prefix : {"RAW_A", "RAW_B1", "A", "B1", "ORACLE_A", "ORACLE_B1"}) {
    output << '\t' << prefix << "_generated\t" << prefix << "_hard_feasible\t"
           << prefix << "_dominant_rejection\t" << prefix << "_rejection_histogram\t"
           << prefix << "_available_entry_distance_m\t" << prefix << "_best_side\t"
           << prefix << "_best_target_d\t" << prefix << "_best_wall_clearance_m\t"
           << prefix << "_best_obstacle_clearance_m\t" << prefix
           << "_best_peak_curvature_radpm\t" << prefix << "_best_peak_lateral_slope\t"
           << prefix << "_best_peak_curvature_rate_radpm2\t" << prefix
           << "_best_status\t" << prefix << "_best_geometry_hash";
  }
  output << '\n';

  for (std::size_t index = 0U; index < stream.frames.size(); ++index) {
    const Frame & raw = stream.frames[index];
    const Frame planning = guardedUnionFrame(
      stream, raw, input_planner, real_union);
    Frame oracle_raw = raw;
    oracle_raw.obstacles = {gt};
    const Frame oracle = guardedUnionFrame(
      stream, oracle_raw, input_planner, oracle_union);
    const auto raw_family_a = runConstantFamily(stream, raw, "RAW_A", false, false);
    const auto raw_family_b1 = runConstantFamily(stream, raw, "RAW_B1", true, false);
    const auto family_a = runConstantFamily(stream, planning, "A", false, false);
    const auto family_b1 = runConstantFamily(stream, planning, "B1", true, false);
    const auto oracle_a = runConstantFamily(stream, oracle, "ORACLE_A", false, false);
    const auto oracle_b1 = runConstantFamily(stream, oracle, "ORACLE_B1", true, false);
    output << stream.scenario << '\t' << index << '\t' << raw.logical_stamp_ns << '\t'
           << raw.ego.s << '\t' << raw.ego.d << '\t' << raw.ego.speed << '\t'
           << (!raw.obstacles.empty() ? "true" : "false") << '\t'
           << raw.obstacles.size() << '\t' << planning.obstacles.size();
    if (planning.obstacles.empty()) {
      output << "\t\t\t\t\t";
    } else {
      const auto & obstacle = planning.obstacles.front();
      output << '\t' << obstacle.id << '\t' << obstacle.s_start << '\t'
             << obstacle.s_end << '\t' << obstacle.d_right << '\t' << obstacle.d_left;
    }
    writeTimeAxisFamily(output, raw_family_a);
    writeTimeAxisFamily(output, raw_family_b1);
    writeTimeAxisFamily(output, family_a);
    writeTimeAxisFamily(output, family_b1);
    writeTimeAxisFamily(output, oracle_a);
    writeTimeAxisFamily(output, oracle_b1);
    output << '\n';
  }
  return 0;
}

// Diagnostic-only causal observability mode. Each stream frame already contains the shadow
// support reconstructed at that exact source timestamp. Evaluate only the current production
// entry family for that support and for the fixed exact-GT obstacle; unlike --time-axis this
// intentionally avoids guarded-union and unrelated ablation families.
int runObservabilityActionability(int argc, char ** argv)
{
  if (argc != 9) {
    std::cerr <<
      "usage: path_family_feasibility_audit --observability-actionability STREAM OUTPUT "
      "GT_S_CENTER GT_S_START GT_S_END GT_D_RIGHT GT_D_LEFT\n";
    return 2;
  }
  const SnapshotStream stream = readStream(argv[2]);
  std::ofstream output(argv[3]);
  if (!output) {
    throw std::runtime_error(
            "cannot open observability actionability output: " + std::string(argv[3]));
  }
  f110_msgs::msg::Obstacle gt;
  gt.id = -999;
  gt.s_center = parseDouble(argv[4]);
  gt.s_start = parseDouble(argv[5]);
  gt.s_end = parseDouble(argv[6]);
  gt.d_right = parseDouble(argv[7]);
  gt.d_left = parseDouble(argv[8]);
  gt.d_center = 0.5 * (gt.d_right + gt.d_left);
  gt.size = std::hypot(gt.s_end - gt.s_start, gt.d_left - gt.d_right);
  gt.s_var = 0.0;
  gt.d_var = 0.0;
  gt.is_static = true;
  gt.is_visible = true;

  output << std::setprecision(17);
  output << "scenario\tevent_index\tlogical_stamp_ns\tego_s\tego_d\tego_speed";
  for (const auto & prefix : {"SHADOW_B1", "GT_B1"}) {
    output << '\t' << prefix << "_generated\t" << prefix << "_hard_feasible\t"
           << prefix << "_dominant_rejection\t" << prefix << "_rejection_histogram\t"
           << prefix << "_available_entry_distance_m\t" << prefix << "_best_side\t"
           << prefix << "_best_target_d\t" << prefix << "_best_wall_clearance_m\t"
           << prefix << "_best_obstacle_clearance_m\t" << prefix
           << "_best_peak_curvature_radpm\t" << prefix << "_best_peak_lateral_slope\t"
           << prefix << "_best_peak_curvature_rate_radpm2\t" << prefix
           << "_best_status\t" << prefix << "_best_geometry_hash";
  }
  output << '\n';
  for (std::size_t index = 0U; index < stream.frames.size(); ++index) {
    const Frame & shadow = stream.frames[index];
    Frame oracle = shadow;
    oracle.obstacles = {gt};
    const auto shadow_family = runConstantFamily(stream, shadow, "SHADOW_B1", true, false);
    const auto gt_family = runConstantFamily(stream, oracle, "GT_B1", true, false);
    output << stream.scenario << '\t' << index << '\t' << shadow.logical_stamp_ns << '\t'
           << shadow.ego.s << '\t' << shadow.ego.d << '\t' << shadow.ego.speed;
    writeTimeAxisFamily(output, shadow_family);
    writeTimeAxisFamily(output, gt_family);
    output << '\n';
  }
  return 0;
}

int runC2Key(int argc, char ** argv)
{
  if (argc != 11) {
    std::cerr <<
      "usage: path_family_feasibility_audit --c2-key STREAM OUTPUT STAMP_NS "
      "INPUT_KIND GT_S_CENTER GT_S_START GT_S_END GT_D_RIGHT GT_D_LEFT\n";
    return 2;
  }
  const SnapshotStream stream = readStream(argv[2]);
  const std::int64_t target_stamp = parseInt64(argv[4]);
  const std::string input_kind = argv[5];
  RacelineSplinePlanner input_planner(parametersFromStream(stream));
  std::string reference_error;
  if (!input_planner.setReference(stream.reference, &reference_error)) {
    throw std::runtime_error("invalid reference in C2 key audit: " + reference_error);
  }
  f110_msgs::msg::Obstacle gt;
  gt.id = -999;
  gt.s_center = parseDouble(argv[6]);
  gt.s_start = parseDouble(argv[7]);
  gt.s_end = parseDouble(argv[8]);
  gt.d_right = parseDouble(argv[9]);
  gt.d_left = parseDouble(argv[10]);
  gt.d_center = 0.5 * (gt.d_right + gt.d_left);
  gt.size = std::hypot(gt.s_end - gt.s_start, gt.d_left - gt.d_right);
  gt.s_var = 0.0;
  gt.d_var = 0.0;
  gt.is_static = true;
  gt.is_visible = true;

  std::map<int, f110_msgs::msg::Obstacle> real_union;
  std::map<int, f110_msgs::msg::Obstacle> oracle_union;
  std::optional<Frame> selected;
  for (const auto & raw : stream.frames) {
    const Frame planning = guardedUnionFrame(stream, raw, input_planner, real_union);
    Frame oracle_raw = raw;
    oracle_raw.obstacles = {gt};
    const Frame oracle = guardedUnionFrame(
      stream, oracle_raw, input_planner, oracle_union);
    if (raw.logical_stamp_ns != target_stamp) {
      continue;
    }
    if (input_kind == "raw") {
      selected = raw;
    } else if (input_kind == "production") {
      selected = planning;
    } else if (input_kind == "oracle") {
      selected = oracle;
    } else {
      throw std::runtime_error("unsupported C2 input kind: " + input_kind);
    }
    break;
  }
  if (!selected.has_value()) {
    throw std::runtime_error("C2 key stamp absent from stream: " + std::to_string(target_stamp));
  }
  const FamilyResult family = runVariableFamily(stream, selected.value());
  std::ofstream output(argv[3]);
  if (!output) {
    throw std::runtime_error("cannot open C2 key output: " + std::string(argv[3]));
  }
  output << "scenario\tlogical_stamp_ns\tinput_kind\tC2_generated\tC2_hard_feasible"
    "\tC2_dominant_rejection\tC2_rejection_histogram\tC2_available_entry_distance_m"
    "\tC2_best_side\tC2_best_target_d\tC2_best_wall_clearance_m"
    "\tC2_best_obstacle_clearance_m\tC2_best_peak_curvature_radpm"
    "\tC2_best_peak_lateral_slope\tC2_best_peak_curvature_rate_radpm2"
    "\tC2_best_status\tC2_best_geometry_hash\n";
  output << stream.scenario << '\t' << target_stamp << '\t' << input_kind;
  writeTimeAxisFamily(output, family);
  output << '\n';
  return 0;
}

// Geometry/clearance-budget mode.  This is deliberately kept in the diagnostic executable so
// every track-footprint query calls the production corner projection above.  World polygons are
// used only for the physical S0-S2 obstacle test; the exact-production row uses the planner's
// Frenet AABB convention and production clearance formula.
struct BudgetPoint
{
  double x{0.0};
  double y{0.0};
};

struct BudgetRepresentation
{
  std::string name;
  std::string source;
  std::vector<BudgetPoint> polygon;
  double s_center{0.0};
  double s_min{0.0};
  double s_max{0.0};
  double d_min{0.0};
  double d_max{0.0};
  bool planner_available{true};
  double pre_guard_s_span{std::numeric_limits<double>::quiet_NaN()};
  double guard_longitudinal_growth{0.0};
  double union_lateral_growth{0.0};
};

struct BudgetSpec
{
  std::int64_t representative_stamp_ns{0};
  std::int64_t false_feasible_stamp_ns{0};
  double manifest_s{0.0};
  std::map<std::string, BudgetRepresentation> representations;
};

BudgetSpec readBudgetSpec(const std::filesystem::path & path)
{
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("cannot open geometry budget spec: " + path.string());
  }
  BudgetSpec spec;
  std::string line;
  if (!std::getline(input, line) || line != "GEOMETRY_BUDGET_SPEC_V1") {
    throw std::runtime_error("unsupported geometry budget spec: " + path.string());
  }
  while (std::getline(input, line)) {
    const auto tokens = splitTabs(line);
    if (tokens.empty()) {
      continue;
    }
    if (tokens[0] == "STAMP" && tokens.size() == 3U) {
      if (tokens[1] == "REPRESENTATIVE") {
        spec.representative_stamp_ns = parseInt64(tokens[2]);
      } else if (tokens[1] == "FALSE_FEASIBLE") {
        spec.false_feasible_stamp_ns = parseInt64(tokens[2]);
      }
    } else if (tokens[0] == "MANIFEST_S" && tokens.size() == 2U) {
      spec.manifest_s = parseDouble(tokens[1]);
    } else if (tokens[0] == "POLYGON" && tokens.size() >= 6U) {
      const std::size_t count = parseSize(tokens[3]);
      if (tokens.size() != 4U + 2U * count) {
        throw std::runtime_error("polygon count mismatch in geometry budget spec");
      }
      BudgetRepresentation representation;
      representation.name = tokens[1];
      representation.source = tokens[2];
      for (std::size_t index = 0U; index < count; ++index) {
        representation.polygon.push_back(
          {parseDouble(tokens[4U + 2U * index]), parseDouble(tokens[5U + 2U * index])});
      }
      spec.representations[representation.name] = std::move(representation);
    } else if (tokens[0] == "FRENET_BOX" && tokens.size() == 16U) {
      BudgetRepresentation representation;
      representation.name = tokens[1];
      representation.source = tokens[2];
      representation.s_center = parseDouble(tokens[3]);
      representation.s_min = parseDouble(tokens[4]);
      representation.s_max = parseDouble(tokens[5]);
      representation.d_min = parseDouble(tokens[6]);
      representation.d_max = parseDouble(tokens[7]);
      representation.planner_available = tokens[8] == "true";
      representation.polygon = {
        {parseDouble(tokens[9]), parseDouble(tokens[11])},
        {parseDouble(tokens[10]), parseDouble(tokens[11])},
        {parseDouble(tokens[10]), parseDouble(tokens[12])},
        {parseDouble(tokens[9]), parseDouble(tokens[12])}};
      representation.pre_guard_s_span = parseDouble(tokens[13]);
      representation.guard_longitudinal_growth = parseDouble(tokens[14]);
      representation.union_lateral_growth = parseDouble(tokens[15]);
      spec.representations[representation.name] = std::move(representation);
    } else if (tokens[0] == "END_SPEC") {
      break;
    } else {
      throw std::runtime_error("unknown geometry budget spec record: " + line);
    }
  }
  if (spec.representative_stamp_ns == 0 || spec.representations.count("G0") == 0U ||
    spec.representations.count("G1") == 0U || spec.representations.count("G2") == 0U)
  {
    throw std::runtime_error("incomplete geometry budget spec");
  }
  return spec;
}

struct BudgetProjection
{
  double s{0.0};
  double d{0.0};
};

BudgetProjection projectBudgetPoint(
  const SnapshotStream & stream, const BudgetPoint & point, double expected_s)
{
  double best_distance_squared = std::numeric_limits<double>::infinity();
  BudgetProjection best;
  const auto & reference = stream.reference.wpnts;
  const double length = reference.back().s_m + std::hypot(
    reference.front().x_m - reference.back().x_m,
    reference.front().y_m - reference.back().y_m);
  for (std::size_t index = 0U; index < reference.size(); ++index) {
    const auto & first = reference[index];
    const auto & second = reference[(index + 1U) % reference.size()];
    const double vx = second.x_m - first.x_m;
    const double vy = second.y_m - first.y_m;
    const double squared = vx * vx + vy * vy;
    if (!(squared > kAuditEpsilon)) {
      continue;
    }
    const double ratio = std::clamp(
      ((point.x - first.x_m) * vx + (point.y - first.y_m) * vy) / squared, 0.0, 1.0);
    const double qx = first.x_m + ratio * vx;
    const double qy = first.y_m + ratio * vy;
    const double dx = point.x - qx;
    const double dy = point.y - qy;
    const double distance_squared = dx * dx + dy * dy;
    if (!(distance_squared < best_distance_squared)) {
      continue;
    }
    const double segment_length = std::sqrt(squared);
    const double next_s = index + 1U == reference.size() ? length : second.s_m;
    double s = first.s_m + ratio * (next_s - first.s_m);
    s = std::fmod(s, length);
    if (s < 0.0) {
      s += length;
    }
    double delta = std::fmod(s - expected_s, length);
    if (delta < 0.0) {
      delta += length;
    }
    if (delta > 0.5 * length) {
      delta -= length;
    }
    best_distance_squared = distance_squared;
    best.s = expected_s + delta;
    best.d = (vx * dy - vy * dx) / segment_length;
  }
  return best;
}

void projectWorldRepresentation(
  const SnapshotStream & stream, BudgetRepresentation & representation, double expected_s)
{
  representation.s_min = std::numeric_limits<double>::infinity();
  representation.s_max = -std::numeric_limits<double>::infinity();
  representation.d_min = std::numeric_limits<double>::infinity();
  representation.d_max = -std::numeric_limits<double>::infinity();
  double s_sum = 0.0;
  for (const auto & point : representation.polygon) {
    const auto projected = projectBudgetPoint(stream, point, expected_s);
    representation.s_min = std::min(representation.s_min, projected.s);
    representation.s_max = std::max(representation.s_max, projected.s);
    representation.d_min = std::min(representation.d_min, projected.d);
    representation.d_max = std::max(representation.d_max, projected.d);
    s_sum += projected.s;
  }
  representation.s_center = s_sum / static_cast<double>(representation.polygon.size());
}

BudgetRepresentation representationFromFrame(
  const SnapshotStream & stream,
  const RacelineSplinePlanner & planner,
  const Frame & frame,
  const std::string & name,
  const std::string & source)
{
  if (frame.obstacles.empty()) {
    throw std::runtime_error("cannot create " + name + " from empty planner input");
  }
  const auto & obstacle = frame.obstacles.front();
  BudgetRepresentation result;
  result.name = name;
  result.source = source;
  result.s_center = obstacle.s_center;
  const double half_span = 0.5 * obstacleSpan(planner, obstacle);
  result.s_min = obstacle.s_center - half_span;
  result.s_max = obstacle.s_center + half_span;
  result.d_min = std::min(obstacle.d_right, obstacle.d_left);
  result.d_max = std::max(obstacle.d_right, obstacle.d_left);
  for (const auto & corner : std::array<std::pair<double, double>, 4>{
      std::make_pair(result.s_min, result.d_min),
      std::make_pair(result.s_max, result.d_min),
      std::make_pair(result.s_max, result.d_max),
      std::make_pair(result.s_min, result.d_max)})
  {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    planner.toCartesian(corner.first, corner.second, x, y, yaw);
    result.polygon.push_back({x, y});
  }
  (void)stream;
  return result;
}

Frame exactUnionFrame(const SnapshotStream & stream, std::int64_t logical_stamp_ns)
{
  RacelineSplinePlanner planner(parametersFromStream(stream));
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference while reconstructing exact union: " + error);
  }
  std::map<int, f110_msgs::msg::Obstacle> union_by_id;
  for (const auto & raw : stream.frames) {
    for (const auto & obstacle : raw.obstacles) {
      const auto previous = union_by_id.find(obstacle.id);
      union_by_id[obstacle.id] = previous == union_by_id.end() ?
        obstacle : mergeObstacleEnvelopes(planner, previous->second, obstacle);
    }
    if (raw.logical_stamp_ns == logical_stamp_ns) {
      Frame frame = raw;
      frame.obstacles.clear();
      for (const auto & entry : union_by_id) {
        frame.obstacles.push_back(entry.second);
      }
      return frame;
    }
  }
  throw std::runtime_error("union stamp absent from stream");
}

double polygonAxisGap(
  const std::vector<BudgetPoint> & first,
  const std::vector<BudgetPoint> & second,
  const BudgetPoint & axis)
{
  const double length = std::hypot(axis.x, axis.y);
  if (!(length > kAuditEpsilon)) {
    return -std::numeric_limits<double>::infinity();
  }
  const double nx = axis.x / length;
  const double ny = axis.y / length;
  auto interval = [&](const std::vector<BudgetPoint> & polygon) {
      double minimum = std::numeric_limits<double>::infinity();
      double maximum = -std::numeric_limits<double>::infinity();
      for (const auto & point : polygon) {
        const double projection = point.x * nx + point.y * ny;
        minimum = std::min(minimum, projection);
        maximum = std::max(maximum, projection);
      }
      return std::make_pair(minimum, maximum);
    };
  const auto a = interval(first);
  const auto b = interval(second);
  return std::max(b.first - a.second, a.first - b.second);
}

double pointSegmentDistance(
  const BudgetPoint & point, const BudgetPoint & first, const BudgetPoint & second)
{
  const double vx = second.x - first.x;
  const double vy = second.y - first.y;
  const double squared = vx * vx + vy * vy;
  const double ratio = squared <= kAuditEpsilon ? 0.0 : std::clamp(
    ((point.x - first.x) * vx + (point.y - first.y) * vy) / squared, 0.0, 1.0);
  return std::hypot(point.x - first.x - ratio * vx, point.y - first.y - ratio * vy);
}

double polygonSignedClearance(
  const std::vector<BudgetPoint> & first, const std::vector<BudgetPoint> & second)
{
  double maximum_axis_gap = -std::numeric_limits<double>::infinity();
  for (const auto * polygon : {&first, &second}) {
    for (std::size_t index = 0U; index < polygon->size(); ++index) {
      const auto & a = (*polygon)[index];
      const auto & b = (*polygon)[(index + 1U) % polygon->size()];
      maximum_axis_gap = std::max(
        maximum_axis_gap, polygonAxisGap(first, second, {-(b.y - a.y), b.x - a.x}));
    }
  }
  if (maximum_axis_gap <= 0.0) {
    return maximum_axis_gap;
  }
  double distance = std::numeric_limits<double>::infinity();
  for (const auto * source : {&first, &second}) {
    const auto * target = source == &first ? &second : &first;
    for (const auto & point : *source) {
      for (std::size_t index = 0U; index < target->size(); ++index) {
        distance = std::min(
          distance,
          pointSegmentDistance(
            point, (*target)[index], (*target)[(index + 1U) % target->size()]));
      }
    }
  }
  return distance;
}

f110_msgs::msg::Wpnt budgetWaypoint(
  const RacelineSplinePlanner & planner,
  const SnapshotStream & stream,
  double s,
  double d)
{
  f110_msgs::msg::Wpnt waypoint;
  waypoint.s_m = planner.wrapS(s);
  waypoint.d_m = d;
  planner.toCartesian(waypoint.s_m, d, waypoint.x_m, waypoint.y_m, waypoint.psi_rad);
  const auto & reference = stream.reference.wpnts[planner.nearestReferenceIndex(waypoint.s_m)];
  waypoint.vx_mps = reference.vx_mps;
  waypoint.kappa_radpm = reference.kappa_radpm;
  return waypoint;
}

std::vector<BudgetPoint> vehiclePolygon(
  const RacelineSplinePlanner & planner,
  const SnapshotStream & stream,
  const RacelineSplineParameters & parameters,
  double s,
  double d)
{
  const auto waypoint = budgetWaypoint(planner, stream, s, d);
  const double cosine = std::cos(waypoint.psi_rad);
  const double sine = std::sin(waypoint.psi_rad);
  std::vector<BudgetPoint> polygon;
  for (const auto & corner : std::array<std::pair<double, double>, 4>{
      std::make_pair(0.5 * parameters.vehicle_length_m, parameters.vehicle_half_width_m),
      std::make_pair(0.5 * parameters.vehicle_length_m, -parameters.vehicle_half_width_m),
      std::make_pair(-0.5 * parameters.vehicle_length_m, -parameters.vehicle_half_width_m),
      std::make_pair(-0.5 * parameters.vehicle_length_m, parameters.vehicle_half_width_m)})
  {
    polygon.push_back({
        waypoint.x_m + corner.first * cosine - corner.second * sine,
        waypoint.y_m + corner.first * sine + corner.second * cosine});
  }
  return polygon;
}

struct TrackInterval
{
  double lower{std::numeric_limits<double>::quiet_NaN()};
  double upper{std::numeric_limits<double>::quiet_NaN()};
};

TrackInterval footprintTrackInterval(
  const SnapshotStream & stream,
  const RacelineSplinePlanner & planner,
  double s)
{
  const auto valid = [&](double d) {
      const auto waypoint = budgetWaypoint(planner, stream, s, d);
      return planner.measureFootprintTrackBound(waypoint, 0U).footprint_clearance_m >=
             -kAuditEpsilon;
    };
  constexpr double lower_search = -2.5;
  constexpr double upper_search = 2.5;
  constexpr double step = 0.01;
  // Track-valid lateral offsets form one interval around the reference line.  The old diagnostic
  // scan rebuilt the same brackets at 501 offsets for every 1 mm longitudinal sample.  Bracket
  // directly from d=0 when it is valid, then use the identical 50-iteration bisection.  Retain the
  // exhaustive fallback for malformed/narrow references where the reference line itself is not
  // footprint-valid.  This is an audit-runtime optimization only; measureFootprintTrackBound and
  // every production threshold remain unchanged.
  if (valid(0.0)) {
    double valid_lower = lower_search;
    if (!valid(lower_search)) {
      double invalid_lower = lower_search;
      valid_lower = 0.0;
      for (int iteration = 0; iteration < 50; ++iteration) {
        const double middle = 0.5 * (invalid_lower + valid_lower);
        if (valid(middle)) {
          valid_lower = middle;
        } else {
          invalid_lower = middle;
        }
      }
    }
    double valid_upper = upper_search;
    if (!valid(upper_search)) {
      valid_upper = 0.0;
      double invalid_upper = upper_search;
      for (int iteration = 0; iteration < 50; ++iteration) {
        const double middle = 0.5 * (valid_upper + invalid_upper);
        if (valid(middle)) {
          valid_upper = middle;
        } else {
          invalid_upper = middle;
        }
      }
    }
    return {valid_lower, valid_upper};
  }
  double first_valid = std::numeric_limits<double>::quiet_NaN();
  double last_valid = std::numeric_limits<double>::quiet_NaN();
  for (double d = lower_search; d <= upper_search + kAuditEpsilon; d += step) {
    if (valid(d)) {
      if (!std::isfinite(first_valid)) {
        first_valid = d;
      }
      last_valid = d;
    }
  }
  if (!std::isfinite(first_valid)) {
    return {};
  }
  double invalid_lower = first_valid - step;
  double valid_lower = first_valid;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const double middle = 0.5 * (invalid_lower + valid_lower);
    if (valid(middle)) {
      valid_lower = middle;
    } else {
      invalid_lower = middle;
    }
  }
  double valid_upper = last_valid;
  double invalid_upper = last_valid + step;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const double middle = 0.5 * (valid_upper + invalid_upper);
    if (valid(middle)) {
      valid_upper = middle;
    } else {
      invalid_upper = middle;
    }
  }
  return {valid_lower, valid_upper};
}

std::optional<std::pair<double, double>> physicalCollisionBand(
  const SnapshotStream & stream,
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const BudgetRepresentation & obstacle,
  double s)
{
  const auto collides = [&](double d) {
      return polygonSignedClearance(
        vehiclePolygon(planner, stream, parameters, s, d), obstacle.polygon) <= kAuditEpsilon;
    };
  constexpr double lower_search = -2.5;
  constexpr double upper_search = 2.5;
  constexpr double step = 0.01;
  double first = std::numeric_limits<double>::quiet_NaN();
  double last = std::numeric_limits<double>::quiet_NaN();
  for (double d = lower_search; d <= upper_search + kAuditEpsilon; d += step) {
    if (!collides(d)) {
      continue;
    }
    if (!std::isfinite(first)) {
      first = d;
    }
    last = d;
  }
  if (!std::isfinite(first)) {
    return std::nullopt;
  }
  double clear_lower = first - step;
  double collision_lower = first;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const double middle = 0.5 * (clear_lower + collision_lower);
    if (collides(middle)) {
      collision_lower = middle;
    } else {
      clear_lower = middle;
    }
  }
  double collision_upper = last;
  double clear_upper = last + step;
  for (int iteration = 0; iteration < 50; ++iteration) {
    const double middle = 0.5 * (collision_upper + clear_upper);
    if (collides(middle)) {
      collision_upper = middle;
    } else {
      clear_upper = middle;
    }
  }
  return std::make_pair(collision_lower, collision_upper);
}

struct CorridorBudgetResult
{
  double left_lower{-std::numeric_limits<double>::infinity()};
  double left_upper{std::numeric_limits<double>::infinity()};
  double right_lower{-std::numeric_limits<double>::infinity()};
  double right_upper{std::numeric_limits<double>::infinity()};
  double left_residual{std::numeric_limits<double>::infinity()};
  double right_residual{std::numeric_limits<double>::infinity()};
  double left_point_min{std::numeric_limits<double>::infinity()};
  double left_point_max{-std::numeric_limits<double>::infinity()};
  double right_point_min{std::numeric_limits<double>::infinity()};
  double right_point_max{-std::numeric_limits<double>::infinity()};
  double left_first_empty_s{std::numeric_limits<double>::quiet_NaN()};
  double right_first_empty_s{std::numeric_limits<double>::quiet_NaN()};
  double left_first_pointwise_empty_s{std::numeric_limits<double>::quiet_NaN()};
  double right_first_pointwise_empty_s{std::numeric_limits<double>::quiet_NaN()};
  double left_worst_s{std::numeric_limits<double>::quiet_NaN()};
  double right_worst_s{std::numeric_limits<double>::quiet_NaN()};
  double minimum_track_width{std::numeric_limits<double>::infinity()};
  double worst_track_s{std::numeric_limits<double>::quiet_NaN()};
  double maximum_yaw_projection{0.0};
  std::size_t samples{0U};
};

std::vector<double> corridorSamples(double start, double end)
{
  constexpr double step = 0.001;
  std::vector<double> samples;
  const std::size_t count = static_cast<std::size_t>(std::ceil((end - start) / step));
  samples.reserve(count + 1U);
  for (std::size_t index = 0U; index <= count; ++index) {
    samples.push_back(start + (end - start) * static_cast<double>(index) /
      static_cast<double>(std::max<std::size_t>(1U, count)));
  }
  return samples;
}

CorridorBudgetResult evaluateCorridorBudget(
  const SnapshotStream & stream,
  const BudgetRepresentation & representation,
  const BudgetRepresentation & g3,
  const std::string & stage,
  const RacelineSplineParameters & parameters)
{
  const bool exact_production = stage == "EXACT_PRODUCTION";
  const bool wall = stage != "S0";
  const bool safety = stage == "S2" || stage == "S3" || stage == "S4" || stage == "S5" ||
    exact_production;
  const bool longitudinal_padding = stage == "S3" || stage == "S4" || stage == "S5" ||
    exact_production;
  const bool tracking = stage == "S4" || stage == "S5" || exact_production;
  const bool use_g3 = stage == "S5" && representation.name == "G2";
  const BudgetRepresentation & obstacle = use_g3 ? g3 : representation;
  RacelineSplinePlanner planner(parameters);
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference in corridor budget: " + error);
  }
  auto track_parameters = parameters;
  track_parameters.wall_safety_margin_m = wall ? parameters.wall_safety_margin_m : 0.0;
  RacelineSplinePlanner track_planner(track_parameters);
  if (!track_planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference in footprint interval: " + error);
  }
  const double start = obstacle.s_min - (longitudinal_padding ?
    parameters.obstacle_longitudinal_padding_m : 0.5 * parameters.vehicle_length_m);
  const double end = obstacle.s_max + (longitudinal_padding ?
    parameters.obstacle_longitudinal_padding_m : 0.5 * parameters.vehicle_length_m);
  CorridorBudgetResult result;
  for (const double s : corridorSamples(start, end)) {
    const auto track = footprintTrackInterval(stream, track_planner, s);
    if (!std::isfinite(track.lower) || !std::isfinite(track.upper)) {
      continue;
    }
    double collision_lower = std::numeric_limits<double>::quiet_NaN();
    double collision_upper = std::numeric_limits<double>::quiet_NaN();
    if (!longitudinal_padding) {
      const auto collision = physicalCollisionBand(stream, planner, parameters, obstacle, s);
      if (!collision.has_value()) {
        continue;
      }
      collision_lower = collision->first;
      collision_upper = collision->second;
    } else {
      const auto vehicle_at_zero = vehiclePolygon(planner, stream, parameters, s, 0.0);
      double minimum_d = std::numeric_limits<double>::infinity();
      double maximum_d = -std::numeric_limits<double>::infinity();
      for (const auto & corner : vehicle_at_zero) {
        const auto projected = projectBudgetPoint(stream, corner, s);
        minimum_d = std::min(minimum_d, projected.d);
        maximum_d = std::max(maximum_d, projected.d);
      }
      if (exact_production) {
        minimum_d = -parameters.vehicle_half_width_m;
        maximum_d = parameters.vehicle_half_width_m;
      }
      collision_lower = obstacle.d_min + minimum_d;
      collision_upper = obstacle.d_max + maximum_d;
      // S3-S5 are additive ablations: longitudinal extrusion must not replace and accidentally
      // weaken the physical polygon collision relation already present in S2.  Union both
      // forbidden bands where the real rectangles overlap, and use the extrusion alone only in
      // the newly padded longitudinal region.  EXACT_PRODUCTION intentionally remains the
      // production centerline-AABB convention rather than this explanatory physical union.
      if (!exact_production) {
        const auto physical = physicalCollisionBand(
          stream, planner, parameters, obstacle, s);
        if (physical.has_value()) {
          collision_lower = std::min(collision_lower, physical->first);
          collision_upper = std::max(collision_upper, physical->second);
        }
      }
      result.maximum_yaw_projection = std::max(
        result.maximum_yaw_projection, std::max(maximum_d, -minimum_d));
    }
    const auto & reference = stream.reference.wpnts[planner.nearestReferenceIndex(planner.wrapS(s))];
    const double tracking_reserve = tracking ? parameters.avoidanceTrackingErrorReserve(
      reference.vx_mps, reference.kappa_radpm) : 0.0;
    const double lateral_margin = (safety ? parameters.safety_margin_m : 0.0) + tracking_reserve;
    const double required_left = collision_upper + lateral_margin;
    const double required_right = collision_lower - lateral_margin;
    result.left_lower = std::max(result.left_lower, required_left);
    result.left_upper = std::min(result.left_upper, track.upper);
    result.right_lower = std::max(result.right_lower, track.lower);
    result.right_upper = std::min(result.right_upper, required_right);
    const double point_left = track.upper - required_left;
    const double point_right = required_right - track.lower;
    if (point_left < result.left_point_min) {
      result.left_point_min = point_left;
      result.left_worst_s = s;
    }
    result.left_point_max = std::max(result.left_point_max, point_left);
    if (!std::isfinite(result.left_first_pointwise_empty_s) && point_left <= 0.0) {
      result.left_first_pointwise_empty_s = s;
    }
    if (point_right < result.right_point_min) {
      result.right_point_min = point_right;
      result.right_worst_s = s;
    }
    result.right_point_max = std::max(result.right_point_max, point_right);
    if (!std::isfinite(result.right_first_pointwise_empty_s) && point_right <= 0.0) {
      result.right_first_pointwise_empty_s = s;
    }
    if (!std::isfinite(result.left_first_empty_s) && result.left_upper - result.left_lower <= 0.0) {
      result.left_first_empty_s = s;
    }
    if (!std::isfinite(result.right_first_empty_s) &&
      result.right_upper - result.right_lower <= 0.0)
    {
      result.right_first_empty_s = s;
    }
    if (track.upper - track.lower < result.minimum_track_width) {
      result.minimum_track_width = track.upper - track.lower;
      result.worst_track_s = s;
    }
    ++result.samples;
  }
  result.left_residual = result.left_upper - result.left_lower;
  result.right_residual = result.right_upper - result.right_lower;
  return result;
}

std::string polygonToken(const std::vector<BudgetPoint> & polygon)
{
  std::ostringstream output;
  output << std::setprecision(17);
  for (std::size_t index = 0U; index < polygon.size(); ++index) {
    if (index > 0U) {
      output << ';';
    }
    output << polygon[index].x << ',' << polygon[index].y;
  }
  return output.str();
}

double exactPathClearance(
  const SnapshotStream & stream,
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const f110_msgs::msg::WpntArray & path,
  const BudgetRepresentation & obstacle)
{
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : path.wpnts) {
    minimum = std::min(
      minimum,
      polygonSignedClearance(
        vehiclePolygon(planner, stream, parameters, waypoint.s_m, waypoint.d_m),
        obstacle.polygon));
  }
  return minimum;
}

double productionObstacleClearance(
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const EgoFrenetState & ego,
  const f110_msgs::msg::WpntArray & path,
  const f110_msgs::msg::Obstacle & obstacle)
{
  const double span_forward = planner.forwardDistance(obstacle.s_start, obstacle.s_end);
  const double span_reverse = planner.forwardDistance(obstacle.s_end, obstacle.s_start);
  const double half_span = 0.5 * std::min(span_forward, span_reverse) +
    parameters.obstacle_longitudinal_padding_m;
  const double center = planner.forwardDistance(ego.s, obstacle.s_center);
  double minimum = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : path.wpnts) {
    const double forward = planner.forwardDistance(ego.s, waypoint.s_m);
    if (forward < center - half_span || forward > center + half_span) {
      continue;
    }
    const double clearance = parameters.obstacleSafetyClearance(
      waypoint.vx_mps, waypoint.kappa_radpm);
    const double right = std::min(obstacle.d_right, obstacle.d_left) - clearance;
    const double left = std::max(obstacle.d_right, obstacle.d_left) + clearance;
    const double signed_clearance = waypoint.d_m >= left ? waypoint.d_m - left :
      waypoint.d_m <= right ? right - waypoint.d_m :
      -std::min(waypoint.d_m - right, left - waypoint.d_m);
    minimum = std::min(minimum, signed_clearance);
  }
  return minimum;
}

void writeFalseFeasibleRows(
  std::ostream & output,
  const SnapshotStream & stream,
  const BudgetSpec & spec,
  const BudgetRepresentation & g1,
  const f110_msgs::msg::Obstacle & g1_obstacle,
  const Frame & g2_frame,
  const Frame & g3_frame)
{
  const auto parameters = parametersFromStream(stream);
  for (const auto & input : std::array<std::pair<std::string, const Frame *>, 2>{
      std::make_pair(std::string("G2"), &g2_frame),
      std::make_pair(std::string("G3"), &g3_frame)})
  {
    if (input.second->obstacles.empty()) {
      continue;
    }
    RacelineSplinePlanner planner(parameters);
    std::string error;
    if (!planner.setReference(stream.reference, &error)) {
      throw std::runtime_error("invalid reference in false-feasible audit: " + error);
    }
    const auto result = planner.plan(input.second->ego, input.second->obstacles);
    if (result.kind != local_planning::SplinePlanKind::kAvoidance) {
      continue;
    }
    const auto selected = std::find_if(
      result.candidate_audits.begin(), result.candidate_audits.end(),
      [](const auto & audit) {return audit.selected;});
    if (selected == result.candidate_audits.end()) {
      throw std::runtime_error("avoidance result has no selected candidate audit");
    }
    std::string validation_error;
    local_planning::PathValidationFailure failure;
    const bool g1_production_valid = planner.validatePath(
      input.second->ego, result.path, {g1_obstacle}, &validation_error, &failure);
    const double exact_clearance = exactPathClearance(
      stream, planner, parameters, result.path, g1);
    const double production_input_clearance = selected->obstacle_clearance_m;
    const double g1_production_clearance = productionObstacleClearance(
      planner, parameters, input.second->ego, result.path, g1_obstacle);
    output << stream.scenario << '\t' << spec.false_feasible_stamp_ns << '\t'
           << input.first << '\t' << (selected->go_left ? "left" : "right") << '\t'
           << selected->target_d << '\t' << selected->requested_entry_length_m << '\t'
           << selected->effective_entry_length_m << '\t' << selected->exit_length_m << '\t'
           << result.path.wpnts.size() << '\t' << geometryHash(result.path) << '\t'
           << production_input_clearance << '\t' << g1_production_clearance << '\t'
           << exact_clearance << '\t' << (exact_clearance <= 0.0 ? "true" : "false") << '\t'
           << (g1_production_valid ? "true" : "false") << '\t'
           << cleanToken(validation_error) << '\t'
           << production_input_clearance - g1_production_clearance << '\t'
           << production_input_clearance - exact_clearance << '\t'
           << selected->rectangular_footprint_wall_clearance_m << '\t'
           << selected->peak_curvature_radpm << '\t'
           << finiteToken(selected->peak_curvature_rate_radpm2) << '\n';
  }
}

struct PathRepresentationClearance
{
  double physical_signed_clearance{std::numeric_limits<double>::infinity()};
  double first_overlap_s{std::numeric_limits<double>::quiet_NaN()};
  double first_overlap_time_s{std::numeric_limits<double>::quiet_NaN()};
  std::size_t first_overlap_index{std::numeric_limits<std::size_t>::max()};
};

PathRepresentationClearance evaluatePathRepresentationClearance(
  const SnapshotStream & stream,
  const RacelineSplinePlanner & planner,
  const RacelineSplineParameters & parameters,
  const f110_msgs::msg::WpntArray & path,
  const BudgetRepresentation & representation)
{
  PathRepresentationClearance result;
  double elapsed = 0.0;
  for (std::size_t index = 0U; index < path.wpnts.size(); ++index) {
    if (index > 0U) {
      const auto & previous = path.wpnts[index - 1U];
      const auto & current = path.wpnts[index];
      const double distance = std::hypot(
        current.x_m - previous.x_m, current.y_m - previous.y_m);
      const double speed = 0.5 * (std::abs(previous.vx_mps) + std::abs(current.vx_mps));
      if (speed > 1.0e-6) {
        elapsed += distance / speed;
      }
    }
    const auto & waypoint = path.wpnts[index];
    const double clearance = polygonSignedClearance(
      vehiclePolygon(planner, stream, parameters, waypoint.s_m, waypoint.d_m),
      representation.polygon);
    result.physical_signed_clearance = std::min(
      result.physical_signed_clearance, clearance);
    if (clearance <= 0.0 && !std::isfinite(result.first_overlap_s)) {
      result.first_overlap_s = waypoint.s_m;
      result.first_overlap_time_s = elapsed;
      result.first_overlap_index = index;
    }
  }
  return result;
}

f110_msgs::msg::Obstacle obstacleFromRepresentation(
  const RacelineSplinePlanner & planner,
  const BudgetRepresentation & representation,
  int id)
{
  f110_msgs::msg::Obstacle obstacle;
  obstacle.id = id;
  obstacle.s_center = planner.wrapS(representation.s_center);
  obstacle.s_start = planner.wrapS(representation.s_min);
  obstacle.s_end = planner.wrapS(representation.s_max);
  obstacle.d_right = representation.d_min;
  obstacle.d_left = representation.d_max;
  obstacle.d_center = 0.5 * (representation.d_min + representation.d_max);
  obstacle.size = std::hypot(
    representation.s_max - representation.s_min,
    representation.d_max - representation.d_min);
  obstacle.is_static = true;
  obstacle.is_visible = true;
  return obstacle;
}

std::array<double, 4> polygonCartesianBounds(const std::vector<BudgetPoint> & polygon)
{
  std::array<double, 4> bounds{
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
    std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (const auto & point : polygon) {
    bounds[0] = std::min(bounds[0], point.x);
    bounds[1] = std::max(bounds[1], point.x);
    bounds[2] = std::min(bounds[2], point.y);
    bounds[3] = std::max(bounds[3], point.y);
  }
  return bounds;
}

void writeRepresentationChainRow(
  std::ostream & output,
  const SnapshotStream & stream,
  std::int64_t stamp,
  const std::string & stage,
  const std::string & native_type,
  const std::string & approximation,
  const BudgetRepresentation & representation)
{
  const auto bounds = polygonCartesianBounds(representation.polygon);
  output << stream.scenario << '\t' << stamp << '\t' << stage << '\t'
         << native_type << '\t' << approximation << '\t'
         << bounds[0] << '\t' << bounds[1] << '\t' << bounds[2] << '\t' << bounds[3]
         << '\t' << bounds[1] - bounds[0] << '\t' << bounds[3] - bounds[2]
         << '\t' << 0.5 * (bounds[0] + bounds[1])
         << '\t' << 0.5 * (bounds[2] + bounds[3])
         << '\t' << representation.s_min << '\t' << representation.s_max
         << '\t' << representation.s_max - representation.s_min
         << '\t' << representation.d_min << '\t' << representation.d_max
         << '\t' << representation.d_max - representation.d_min
         << '\t' << representation.s_center
         << '\t' << 0.5 * (representation.d_min + representation.d_max)
         << '\t' << representation.d_min << '\t'
         << polygonToken(representation.polygon) << '\n';
}

int runRepresentationRootCause(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr <<
      "usage: path_family_feasibility_audit --representation-root-cause "
      "STREAM SPEC OUTPUT_DIR EXPECTED_GEOMETRY_HASH\n";
    return 2;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::clock_t cpu_started = std::clock();
  const SnapshotStream stream = readStream(argv[2]);
  BudgetSpec spec = readBudgetSpec(argv[3]);
  const std::filesystem::path output_directory = argv[4];
  const std::string expected_geometry_hash = argv[5];
  std::filesystem::create_directories(output_directory);
  const auto parameters = parametersFromStream(stream);
  RacelineSplinePlanner planner(parameters);
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference in representation root-cause audit: " + error);
  }

  for (const auto & name : {"G0", "G1", "R2", "R3", "R4", "CF_TEMPORAL", "CF_KNOWN"}) {
    auto found = spec.representations.find(name);
    if (found != spec.representations.end()) {
      projectWorldRepresentation(stream, found->second, spec.manifest_s);
    }
  }
  for (const auto & required : {"G0", "G1", "G2", "R2", "R3", "R4", "R5", "R6", "R7", "R8",
      "CF_TEMPORAL", "CF_KNOWN"})
  {
    if (spec.representations.count(required) == 0U) {
      throw std::runtime_error("representation root-cause spec is missing " + std::string(required));
    }
  }

  const Frame g3_frame = exactGuardedFrame(stream, spec.representative_stamp_ns);
  if (g3_frame.obstacles.empty()) {
    throw std::runtime_error("planner production frame is empty at root-cause event");
  }
  BudgetRepresentation r9 = representationFromFrame(
    stream, planner, g3_frame, "R9", "production_same_id_union_plus_longitudinal_guard");
  spec.representations["R9"] = r9;

  const auto plan = planner.plan(g3_frame.ego, g3_frame.obstacles);
  if (plan.kind != local_planning::SplinePlanKind::kAvoidance) {
    throw std::runtime_error("root-cause event does not reproduce an avoidance candidate");
  }
  const std::string actual_geometry_hash = geometryHash(plan.path);
  if (actual_geometry_hash != expected_geometry_hash) {
    throw std::runtime_error(
            "selected candidate hash mismatch: expected " + expected_geometry_hash +
            ", got " + actual_geometry_hash);
  }
  const auto selected = std::find_if(
    plan.candidate_audits.begin(), plan.candidate_audits.end(),
    [](const auto & audit) {return audit.selected;});
  if (selected == plan.candidate_audits.end()) {
    throw std::runtime_error("selected candidate audit is absent");
  }

  std::ofstream chain(output_directory / "representation_chain_exact.tsv");
  std::ofstream clearance(output_directory / "same_path_clearance_exact.tsv");
  std::ofstream counterfactual(output_directory / "counterfactuals_exact.tsv");
  std::ofstream path(output_directory / "selected_path.tsv");
  if (!chain || !clearance || !counterfactual || !path) {
    throw std::runtime_error("cannot open representation root-cause raw output");
  }
  chain << std::setprecision(17);
  clearance << std::setprecision(17);
  counterfactual << std::setprecision(17);
  path << std::setprecision(17);

  chain << "scenario\tstamp_ns\tstage\tnative_type\tapproximation\tx_min\tx_max\ty_min"
    "\ty_max\tx_span\ty_span\tx_center\ty_center\ts_min\ts_max\ts_span\td_min\td_max"
    "\td_span\ts_center\td_center\tselected_right_boundary\tpolygon_xy\n";
  const std::array<std::tuple<std::string, std::string, std::string, std::string>, 10> stages = {{
    {"R0", "G0", "manifest_rectangle", "nominal scenario polygon"},
    {"R1", "G1", "occupied_raster_bounds", "exact half-open simulator raster"},
    {"R2", "R2", "point_cloud", "recorded ranges at exact GT-hit backend beam angles"},
    {"R3", "R3", "point_cloud", "production adaptive-breakpoint cluster points"},
    {"R4", "R4", "cartesian_aabb", "AABB of R3; all corners independently projected"},
    {"R5", "R5", "detection", "production AABB projector output plus retained Cartesian AABB"},
    {"R6", "R6", "tracker", "KF centre plus smoothed independent Frenet extents"},
    {"R7", "R7", "static_obs", "published visible Cartesian union reprojected once"},
    {"R8", "R8", "planner_input", "field-exact static_obs geometry received by planner"},
    {"R9", "R9", "planner_guard", "same-ID union plus longitudinal uncertainty guard"},
  }};
  for (const auto & stage : stages) {
    writeRepresentationChainRow(
      chain, stream, spec.representative_stamp_ns, std::get<0>(stage), std::get<2>(stage),
      std::get<3>(stage), spec.representations.at(std::get<1>(stage)));
  }

  path << "scenario\tstamp_ns\tgeometry_hash\tindex\ts\td\tx\ty\tyaw\tkappa\tvx\tax\n";
  for (std::size_t index = 0U; index < plan.path.wpnts.size(); ++index) {
    const auto & waypoint = plan.path.wpnts[index];
    path << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
         << actual_geometry_hash << '\t' << index << '\t' << waypoint.s_m << '\t'
         << waypoint.d_m << '\t' << waypoint.x_m << '\t' << waypoint.y_m << '\t'
         << waypoint.psi_rad << '\t' << waypoint.kappa_radpm << '\t'
         << waypoint.vx_mps << '\t' << waypoint.ax_mps2 << '\n';
  }

  clearance << "scenario\tstamp_ns\trepresentation\tgeometry_hash\tphysical_signed_clearance"
    "\tminimum_vehicle_obstacle_distance\tproduction_obstacle_clearance\tfirst_overlap_index"
    "\tfirst_overlap_s\tfirst_overlap_nominal_time_s\thard_valid\thard_rejection\n";
  const auto write_clearance = [&](
    const std::string & name,
    const BudgetRepresentation & physical,
    const f110_msgs::msg::Obstacle & production_obstacle)
    {
      const auto physical_result = evaluatePathRepresentationClearance(
        stream, planner, parameters, plan.path, physical);
      const double production_clearance = productionObstacleClearance(
        planner, parameters, g3_frame.ego, plan.path, production_obstacle);
      std::string validation_error;
      local_planning::PathValidationFailure failure;
      const bool valid = planner.validatePath(
        g3_frame.ego, plan.path, {production_obstacle}, &validation_error, &failure);
      clearance << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
                << name << '\t' << actual_geometry_hash << '\t'
                << physical_result.physical_signed_clearance << '\t'
                << std::max(0.0, physical_result.physical_signed_clearance) << '\t'
                << production_clearance << '\t';
      if (physical_result.first_overlap_index != std::numeric_limits<std::size_t>::max()) {
        clearance << physical_result.first_overlap_index;
      }
      clearance << '\t' << finiteToken(physical_result.first_overlap_s) << '\t'
                << finiteToken(physical_result.first_overlap_time_s) << '\t'
                << (valid ? "true" : "false") << '\t'
                << cleanToken(validation_error) << '\n';
    };
  write_clearance(
    "R4_cluster_aabb", spec.representations.at("R4"),
    obstacleFromRepresentation(planner, spec.representations.at("R5"), -904));
  write_clearance(
    "R7_published_static", spec.representations.at("R7"),
    obstacleFromRepresentation(planner, spec.representations.at("R7"), -907));
  write_clearance("R9_planner_production", r9, g3_frame.obstacles.front());
  write_clearance(
    "R1_exact_raster", spec.representations.at("G1"),
    obstacleFromRepresentation(planner, spec.representations.at("G1"), -901));

  counterfactual << "scenario\tstamp_ns\tcounterfactual\tgeometry_hash"
    "\tphysical_signed_clearance\tproduction_obstacle_clearance\thard_valid"
    "\thard_rejection\tfalse_feasible_removed\n";
  const auto write_counterfactual = [&](const std::string & name, const std::string & key) {
      const auto & representation = spec.representations.at(key);
      const auto physical_result = evaluatePathRepresentationClearance(
        stream, planner, parameters, plan.path, representation);
      const auto obstacle = obstacleFromRepresentation(planner, representation, -950);
      const double production_clearance = productionObstacleClearance(
        planner, parameters, g3_frame.ego, plan.path, obstacle);
      std::string validation_error;
      local_planning::PathValidationFailure failure;
      const bool valid = planner.validatePath(
        g3_frame.ego, plan.path, {obstacle}, &validation_error, &failure);
      counterfactual << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
                     << name << '\t' << actual_geometry_hash << '\t'
                     << physical_result.physical_signed_clearance << '\t'
                     << production_clearance << '\t' << (valid ? "true" : "false") << '\t'
                     << cleanToken(validation_error) << '\t'
                     << (!valid ? "true" : "false") << '\n';
    };
  write_counterfactual("temporal_visible_aabb_union", "CF_TEMPORAL");
  write_counterfactual("known_nominal_rectangle_from_visible_face", "CF_KNOWN");
  write_counterfactual("exact_raster_oracle", "G1");

  std::ofstream metadata(output_directory / "event_exact.tsv");
  metadata << std::setprecision(17)
           << "scenario\tstamp_ns\tgeometry_hash\tside\ttarget_d\trequested_entry_length"
              "\teffective_entry_length\texit_length\twaypoint_count\twall_clearance"
              "\tobstacle_clearance\tpeak_curvature\tpeak_slope\tpeak_curvature_rate\n"
           << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
           << actual_geometry_hash << '\t' << (selected->go_left ? "left" : "right") << '\t'
           << selected->target_d << '\t' << selected->requested_entry_length_m << '\t'
           << selected->effective_entry_length_m << '\t' << selected->exit_length_m << '\t'
           << plan.path.wpnts.size() << '\t'
           << selected->rectangular_footprint_wall_clearance_m << '\t'
           << selected->obstacle_clearance_m << '\t' << selected->peak_curvature_radpm << '\t'
           << peakSlope(planner, g3_frame.ego, plan.path) << '\t'
           << finiteToken(selected->peak_curvature_rate_radpm2) << '\n';

  std::ofstream performance(output_directory / "performance_exact.tsv");
  performance << "scenario\twall_s\tcpu_s\n" << std::setprecision(17) << stream.scenario << '\t'
              << std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count() << '\t'
              << static_cast<double>(std::clock() - cpu_started) /
    static_cast<double>(CLOCKS_PER_SEC) << '\n';
  return 0;
}

double unwrapNear(double wrapped_s, double expected_s, double track_length)
{
  if (!(track_length > 0.0)) {
    return wrapped_s;
  }
  double delta = std::fmod(wrapped_s - expected_s, track_length);
  if (delta < -0.5 * track_length) {
    delta += track_length;
  } else if (delta > 0.5 * track_length) {
    delta -= track_length;
  }
  return expected_s + delta;
}

global_planning::ClcsFrenetConverter::Ptr productionConverter(const SnapshotStream & stream)
{
  std::vector<global_planning::ReferenceWaypoint> reference;
  reference.reserve(stream.reference.wpnts.size());
  for (const auto & waypoint : stream.reference.wpnts) {
    global_planning::ReferenceWaypoint item;
    item.x = waypoint.x_m;
    item.y = waypoint.y_m;
    item.s = waypoint.s_m;
    reference.push_back(item);
  }
  global_planning::ClcsFrenetConfig config;
  return global_planning::ClcsFrenetConverter::create(reference, config, 1U);
}

void projectProductionCartesianAabb(
  const global_planning::ClcsFrenetConverter & converter,
  BudgetRepresentation & representation,
  double expected_s)
{
  const auto cartesian = polygonCartesianBounds(representation.polygon);
  const auto projected = obstacle_detector::projectCartesianAabb(
    converter, cartesian[0], cartesian[1], cartesian[2], cartesian[3]);
  if (!projected.has_value()) {
    throw std::runtime_error(
            "production AABB projection failed for temporal representation " +
            representation.name);
  }
  const double track_length = converter.stats().track_length;
  representation.s_center = unwrapNear(projected->s_center, expected_s, track_length);
  representation.s_min = unwrapNear(projected->s_start, representation.s_center, track_length);
  representation.s_max = unwrapNear(projected->s_end, representation.s_center, track_length);
  if (representation.s_min > representation.s_max) {
    std::swap(representation.s_min, representation.s_max);
  }
  representation.d_min = std::min(projected->d_right, projected->d_left);
  representation.d_max = std::max(projected->d_right, projected->d_left);
}

std::string planKindToken(local_planning::SplinePlanKind kind)
{
  switch (kind) {
    case local_planning::SplinePlanKind::kNoObstacle:
      return "no_obstacle";
    case local_planning::SplinePlanKind::kPreparation:
      return "preparation";
    case local_planning::SplinePlanKind::kAvoidance:
      return "avoidance";
    case local_planning::SplinePlanKind::kSafeStop:
      return "safe_stop";
    case local_planning::SplinePlanKind::kNoSafePath:
      return "no_safe_path";
  }
  return "unknown";
}

// Diagnostic-only observed-envelope mode. Cartesian candidates call the detector-owned
// production AABB projector directly; Frenet candidates retain the detector event bounds supplied
// by the report generator. No result is fed to a ROS node or production target.
int runTemporalEnvelopeAudit(int argc, char ** argv)
{
  if (argc != 6) {
    std::cerr <<
      "usage: path_family_feasibility_audit --temporal-envelope "
      "STREAM SPEC OUTPUT_DIR EXPECTED_GEOMETRY_HASH_OR_DASH\n";
    return 2;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::clock_t cpu_started = std::clock();
  const SnapshotStream stream = readStream(argv[2]);
  BudgetSpec spec = readBudgetSpec(argv[3]);
  const std::filesystem::path output_directory = argv[4];
  const std::string expected_geometry_hash = argv[5];
  const bool fast_corridor_only = expected_geometry_hash == "FAST_CORRIDOR_ONLY";
  const bool corridor_only = expected_geometry_hash == "CORRIDOR_ONLY" || fast_corridor_only;
  std::filesystem::create_directories(output_directory);

  const auto parameters = parametersFromStream(stream);
  RacelineSplinePlanner planner(parameters);
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference in temporal-envelope audit: " + error);
  }
  const auto converter = productionConverter(stream);
  projectWorldRepresentation(stream, spec.representations.at("G0"), spec.manifest_s);
  projectWorldRepresentation(stream, spec.representations.at("G1"), spec.manifest_s);

  std::vector<std::string> strategies;
  for (auto & item : spec.representations) {
    if (item.first.rfind("TC_", 0U) == 0U) {
      projectProductionCartesianAabb(*converter, item.second, spec.manifest_s);
      strategies.push_back(item.first);
    } else if (item.first.rfind("TF_", 0U) == 0U) {
      strategies.push_back(item.first);
    }
  }

  // Diagnostic shadow audits can provide the occupied-cell boundary of a non-rectangular
  // possible set as TC_CELL_* polygons. Project every cell with the detector-owned production
  // AABB projector, then reduce them to exact Frenet support before running the expensive corridor
  // and same-path queries. This changes only the BUILD_TESTING audit executable; no representation
  // is published or passed to a production node.
  std::vector<std::string> cell_strategies;
  std::copy_if(
    strategies.begin(), strategies.end(), std::back_inserter(cell_strategies),
    [](const std::string & name) {return name.rfind("TC_CELL_", 0U) == 0U;});
  if (!cell_strategies.empty()) {
    BudgetRepresentation support;
    support.name = "TF_UNION_SUPPORT_INTERNAL";
    support.source = "production_projected_possible_set_boundary_support";
    support.s_min = std::numeric_limits<double>::infinity();
    support.s_max = -std::numeric_limits<double>::infinity();
    support.d_min = std::numeric_limits<double>::infinity();
    support.d_max = -std::numeric_limits<double>::infinity();
    double x_min = std::numeric_limits<double>::infinity();
    double x_max = -std::numeric_limits<double>::infinity();
    double y_min = std::numeric_limits<double>::infinity();
    double y_max = -std::numeric_limits<double>::infinity();
    for (const auto & name : cell_strategies) {
      const auto & cell = spec.representations.at(name);
      const auto cartesian = polygonCartesianBounds(cell.polygon);
      x_min = std::min(x_min, cartesian[0]);
      x_max = std::max(x_max, cartesian[1]);
      y_min = std::min(y_min, cartesian[2]);
      y_max = std::max(y_max, cartesian[3]);
      support.s_min = std::min(support.s_min, cell.s_min);
      support.s_max = std::max(support.s_max, cell.s_max);
      support.d_min = std::min(support.d_min, cell.d_min);
      support.d_max = std::max(support.d_max, cell.d_max);
    }
    support.s_center = 0.5 * (support.s_min + support.s_max);
    support.polygon = {{x_min, y_min}, {x_max, y_min}, {x_max, y_max}, {x_min, y_max}};
    spec.representations[support.name] = std::move(support);
    strategies.erase(
      std::remove_if(
        strategies.begin(), strategies.end(),
        [](const std::string & name) {return name.rfind("TC_CELL_", 0U) == 0U;}),
      strategies.end());
    strategies.push_back("TF_UNION_SUPPORT_INTERNAL");
  }
  if (strategies.empty()) {
    throw std::runtime_error("temporal-envelope spec has no TC_/TF_ strategy");
  }

  const auto base = std::find_if(
    stream.frames.begin(), stream.frames.end(), [&](const auto & frame) {
      return frame.logical_stamp_ns == spec.representative_stamp_ns;
    });
  if (base == stream.frames.end()) {
    throw std::runtime_error("temporal-envelope representative stamp is absent from stream");
  }

  std::optional<local_planning::RacelineSplineResult> common_plan;
  double exact_gt_clearance = std::numeric_limits<double>::quiet_NaN();
  if (!corridor_only && expected_geometry_hash != "-") {
    const Frame g3_frame = exactGuardedFrame(stream, spec.representative_stamp_ns);
    const auto plan = planner.plan(g3_frame.ego, g3_frame.obstacles);
    if (plan.kind != local_planning::SplinePlanKind::kAvoidance) {
      throw std::runtime_error("common temporal-audit path is not an avoidance plan");
    }
    const std::string actual_hash = geometryHash(plan.path);
    if (actual_hash != expected_geometry_hash) {
      throw std::runtime_error(
              "temporal common-path hash mismatch: expected " + expected_geometry_hash +
              ", got " + actual_hash);
    }
    exact_gt_clearance = evaluatePathRepresentationClearance(
      stream, planner, parameters, plan.path, spec.representations.at("G1"))
      .physical_signed_clearance;
    common_plan = plan;
  }

  std::ofstream output(output_directory / "temporal_strategy_exact.tsv");
  if (!output) {
    throw std::runtime_error("cannot open temporal-envelope exact output");
  }
  output << std::setprecision(17);
  output << "scenario\tstamp_ns\trepresentation\tsource\tx_min\tx_max\ty_min\ty_max"
    "\ts_min\ts_max\tlongitudinal_span\td_min\td_max\tlateral_span"
    "\tphysical_left_corridor\tphysical_right_corridor\tphysical_best_corridor"
    "\tleft_corridor\tright_corridor\tbest_corridor\tdirect_plan_kind"
    "\tgenerated_candidates\tfeasible_candidates\tselected_side"
    "\tsame_path_production_clearance\tsame_path_hard_valid\tsame_path_rejection"
    "\tsame_path_exact_gt_clearance\tsame_path_representation_physical_clearance\n";

  const auto write_row = [&](const std::string & name) {
      const auto & representation = spec.representations.at(name);
      const auto cartesian = polygonCartesianBounds(representation.polygon);
      const auto corridor = evaluateCorridorBudget(
        stream, representation, representation, "EXACT_PRODUCTION", parameters);
      const auto obstacle = obstacleFromRepresentation(planner, representation, -980);
      output << stream.scenario << '\t' << spec.representative_stamp_ns << '\t' << name << '\t'
             << representation.source << '\t' << cartesian[0] << '\t' << cartesian[1] << '\t'
             << cartesian[2] << '\t' << cartesian[3] << '\t'
             << representation.s_min << '\t' << representation.s_max << '\t'
             << representation.s_max - representation.s_min << '\t'
             << representation.d_min << '\t' << representation.d_max << '\t'
             << representation.d_max - representation.d_min << '\t';
      if (fast_corridor_only) {
        output << "\t\t\t";
      } else {
        const auto physical_corridor = evaluateCorridorBudget(
          stream, representation, representation, "S0", parameters);
        output << physical_corridor.left_residual << '\t' << physical_corridor.right_residual
               << '\t' << std::max(
          physical_corridor.left_residual, physical_corridor.right_residual) << '\t';
      }
      output << corridor.left_residual << '\t' << corridor.right_residual << '\t'
             << std::max(corridor.left_residual, corridor.right_residual) << '\t';

      if (corridor_only) {
        output << "\t\t\t\t";
      } else {
        Frame frame = *base;
        frame.obstacles = {obstacle};
        const auto direct_plan = planner.plan(frame.ego, frame.obstacles);
        const std::size_t feasible = static_cast<std::size_t>(std::count_if(
          direct_plan.candidate_audits.begin(), direct_plan.candidate_audits.end(),
          [](const auto & audit) {return audit.feasible;}));
        output << planKindToken(direct_plan.kind) << '\t'
               << direct_plan.candidate_audits.size() << '\t' << feasible << '\t'
               << (direct_plan.kind == local_planning::SplinePlanKind::kAvoidance ?
          (direct_plan.go_left ? "left" : "right") : "") << '\t';
      }

      if (common_plan.has_value()) {
        const double clearance = productionObstacleClearance(
          planner, parameters, base->ego, common_plan->path, obstacle);
        std::string validation_error;
        local_planning::PathValidationFailure failure;
        const bool valid = planner.validatePath(
          base->ego, common_plan->path, {obstacle}, &validation_error, &failure);
        const double physical = evaluatePathRepresentationClearance(
          stream, planner, parameters, common_plan->path, representation)
          .physical_signed_clearance;
        output << clearance << '\t' << (valid ? "true" : "false") << '\t'
               << cleanToken(validation_error) << '\t' << exact_gt_clearance << '\t'
               << physical;
      } else {
        output << "\t\t\t\t";
      }
      output << '\n';
    };

  write_row("G0");
  write_row("G1");
  for (const auto & name : strategies) {
    write_row(name);
  }

  std::ofstream performance(output_directory / "performance_exact.tsv");
  performance << "scenario\twall_s\tcpu_s\n" << std::setprecision(17)
              << stream.scenario << '\t'
              << std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count() << '\t'
              << static_cast<double>(std::clock() - cpu_started) /
    static_cast<double>(CLOCKS_PER_SEC) << '\n';
  return 0;
}

int runGeometryBudget(int argc, char ** argv)
{
  if (argc != 5) {
    std::cerr <<
      "usage: path_family_feasibility_audit --geometry-budget STREAM SPEC OUTPUT_DIR\n";
    return 2;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::clock_t cpu_started = std::clock();
  const SnapshotStream stream = readStream(argv[2]);
  BudgetSpec spec = readBudgetSpec(argv[3]);
  const std::filesystem::path output_directory = argv[4];
  std::filesystem::create_directories(output_directory);
  const auto parameters = parametersFromStream(stream);
  RacelineSplinePlanner planner(parameters);
  std::string error;
  if (!planner.setReference(stream.reference, &error)) {
    throw std::runtime_error("invalid reference in geometry budget audit: " + error);
  }
  projectWorldRepresentation(stream, spec.representations.at("G0"), spec.manifest_s);
  projectWorldRepresentation(stream, spec.representations.at("G1"), spec.manifest_s);

  const Frame g3_frame = exactGuardedFrame(stream, spec.representative_stamp_ns);
  const Frame union_frame = exactUnionFrame(stream, spec.representative_stamp_ns);
  auto g3 = representationFromFrame(
    stream, planner, g3_frame, "G3", "production_same_id_union_plus_longitudinal_guard");
  if (!union_frame.obstacles.empty()) {
    const auto & union_obstacle = union_frame.obstacles.front();
    const double union_span = obstacleSpan(planner, union_obstacle);
    g3.pre_guard_s_span = union_span;
    g3.guard_longitudinal_growth = (g3.s_max - g3.s_min) - union_span;
    const auto & g2 = spec.representations.at("G2");
    g3.union_lateral_growth = (g3.d_max - g3.d_min) - (g2.d_max - g2.d_min);
  }
  spec.representations["G3"] = g3;

  std::ofstream geometry(output_directory / "geometry.tsv");
  std::ofstream corridor(output_directory / "ablation.tsv");
  std::ofstream closest(output_directory / "closest.tsv");
  std::ofstream mismatch(output_directory / "mismatch.tsv");
  if (!geometry || !corridor || !closest || !mismatch) {
    throw std::runtime_error("cannot open geometry budget raw output");
  }
  geometry << std::setprecision(17);
  corridor << std::setprecision(17);
  closest << std::setprecision(17);
  mismatch << std::setprecision(17);
  geometry << "scenario\tlogical_stamp_ns\trepresentation\tsource\tplanner_available"
    "\ts_center\ts_min\ts_max\ts_span\td_center\td_min\td_max\td_span\tpolygon_xy"
    "\tpre_guard_s_span\tguard_longitudinal_growth\tunion_lateral_growth\n";
  for (const auto & name : {"G0", "G1", "G2", "G3"}) {
    const auto & representation = spec.representations.at(name);
    geometry << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
             << representation.name << '\t' << representation.source << '\t'
             << (representation.planner_available ? "true" : "false") << '\t'
             << representation.s_center << '\t' << representation.s_min << '\t'
             << representation.s_max << '\t' << representation.s_max - representation.s_min
             << '\t' << 0.5 * (representation.d_min + representation.d_max) << '\t'
             << representation.d_min << '\t' << representation.d_max << '\t'
             << representation.d_max - representation.d_min << '\t'
             << polygonToken(representation.polygon) << '\t'
             << finiteToken(representation.pre_guard_s_span) << '\t'
             << representation.guard_longitudinal_growth << '\t'
             << representation.union_lateral_growth << '\n';
  }

  corridor << "scenario\tlogical_stamp_ns\trepresentation\tstage\tside\tinterval_lower"
    "\tinterval_upper\tresidual\tpoint_min\tpoint_max\tfirst_empty_s"
    "\tfirst_pointwise_empty_s\tworst_s"
    "\tminimum_track_width\tworst_track_s\tmaximum_yaw_projection\tsamples\n";
  for (const auto & name : {"G0", "G1", "G2", "G3"}) {
    const auto & representation = spec.representations.at(name);
    for (const auto & stage : {"S0", "S1", "S2", "S3", "S4", "S5"}) {
      const auto result = evaluateCorridorBudget(stream, representation, g3, stage, parameters);
      for (const bool left : {true, false}) {
        corridor << stream.scenario << '\t' << spec.representative_stamp_ns << '\t'
                 << name << '\t' << stage << '\t' << (left ? "left" : "right") << '\t'
                 << (left ? result.left_lower : result.right_lower) << '\t'
                 << (left ? result.left_upper : result.right_upper) << '\t'
                 << (left ? result.left_residual : result.right_residual) << '\t'
                 << (left ? result.left_point_min : result.right_point_min) << '\t'
                 << (left ? result.left_point_max : result.right_point_max) << '\t'
                 << finiteToken(left ? result.left_first_empty_s : result.right_first_empty_s)
                 << '\t' << finiteToken(
      left ? result.left_first_pointwise_empty_s : result.right_first_pointwise_empty_s)
                 << '\t' << finiteToken(left ? result.left_worst_s : result.right_worst_s)
                 << '\t' << result.minimum_track_width << '\t' << result.worst_track_s << '\t'
                 << result.maximum_yaw_projection << '\t' << result.samples << '\n';
      }
    }
  }
  const auto production = evaluateCorridorBudget(stream, g3, g3, "EXACT_PRODUCTION", parameters);
  for (const bool left : {true, false}) {
    corridor << stream.scenario << '\t' << spec.representative_stamp_ns
             << "\tG3\tEXACT_PRODUCTION\t" << (left ? "left" : "right") << '\t'
             << (left ? production.left_lower : production.right_lower) << '\t'
             << (left ? production.left_upper : production.right_upper) << '\t'
             << (left ? production.left_residual : production.right_residual) << '\t'
             << (left ? production.left_point_min : production.right_point_min) << '\t'
             << (left ? production.left_point_max : production.right_point_max) << '\t'
             << finiteToken(left ? production.left_first_empty_s : production.right_first_empty_s)
             << '\t' << finiteToken(
    left ? production.left_first_pointwise_empty_s : production.right_first_pointwise_empty_s)
             << '\t' << finiteToken(left ? production.left_worst_s : production.right_worst_s)
             << '\t' << production.minimum_track_width << '\t' << production.worst_track_s << '\t'
             << production.maximum_yaw_projection << '\t' << production.samples << '\n';
  }

  closest << "scenario\tlogical_stamp_ns\tfamily\tside\tstatus\ttarget_d\twall_clearance"
    "\tobstacle_clearance\tpeak_curvature\tpeak_slope\tpeak_curvature_rate"
    "\tcurvature_excess\tslope_excess\tcurvature_rate_excess\tfootprint_deficit"
    "\tobstacle_deficit\trejection_reason\tgeometry_hash\n";
  const FamilyResult c2 = runVariableFamily(stream, g3_frame);
  for (const auto * side : {&c2.left, &c2.right}) {
    if (!side->best.has_value()) {
      continue;
    }
    const auto & best = side->best.value();
    closest << stream.scenario << '\t' << spec.representative_stamp_ns << "\tC2\t"
            << (best.go_left ? "left" : "right") << '\t'
            << (best.feasible ? "feasible" : "closest") << '\t' << best.target_d << '\t'
            << best.footprint_wall_clearance_m << '\t' << best.obstacle_clearance_m << '\t'
            << best.peak_curvature_radpm << '\t' << best.peak_lateral_slope << '\t'
            << best.peak_curvature_rate_radpm2 << '\t' << best.curvature_excess_radpm << '\t'
            << best.slope_excess << '\t' << best.curvature_rate_excess_radpm2 << '\t'
            << best.footprint_violation_m << '\t' << best.obstacle_violation_m << '\t'
            << cleanToken(best.rejection_reason) << '\t' << best.geometry_hash << '\n';
  }

  mismatch << "scenario\tlogical_stamp_ns\tinput_representation\tside\ttarget_d"
    "\trequested_entry_length\teffective_entry_length\texit_length\twaypoint_count"
    "\tgeometry_hash\tproduction_input_obstacle_clearance\tg1_production_clearance"
    "\tg1_exact_footprint_clearance\tg1_exact_overlap\tg1_production_hard_valid"
    "\tg1_production_rejection\tproduction_clearance_optimism"
    "\tphysical_clearance_difference\twall_clearance\tpeak_curvature"
    "\tpeak_curvature_rate\n";
  if (spec.false_feasible_stamp_ns > 0) {
    Frame g2_false;
    g2_false.logical_stamp_ns = spec.false_feasible_stamp_ns;
    const auto raw_at_stamp = std::find_if(
      stream.frames.begin(), stream.frames.end(), [&](const auto & frame) {
        return frame.logical_stamp_ns == spec.false_feasible_stamp_ns;
      });
    if (raw_at_stamp != stream.frames.end()) {
      g2_false = *raw_at_stamp;
      const Frame g3_false = exactGuardedFrame(stream, spec.false_feasible_stamp_ns);
      f110_msgs::msg::Obstacle g1_obstacle;
      const auto & g1 = spec.representations.at("G1");
      g1_obstacle.id = -999;
      g1_obstacle.s_center = g1.s_center;
      g1_obstacle.s_start = planner.wrapS(g1.s_min);
      g1_obstacle.s_end = planner.wrapS(g1.s_max);
      g1_obstacle.d_right = g1.d_min;
      g1_obstacle.d_left = g1.d_max;
      g1_obstacle.d_center = 0.5 * (g1.d_min + g1.d_max);
      g1_obstacle.size = std::hypot(g1.s_max - g1.s_min, g1.d_max - g1.d_min);
      g1_obstacle.is_static = true;
      g1_obstacle.is_visible = true;
      writeFalseFeasibleRows(
        mismatch, stream, spec, g1, g1_obstacle, g2_false, g3_false);
    }
  }

  std::ofstream performance(output_directory / "performance.tsv");
  performance << "scenario\twall_s\tcpu_s\n" << std::setprecision(17) << stream.scenario << '\t'
              << std::chrono::duration<double>(
    std::chrono::steady_clock::now() - started).count() << '\t'
              << static_cast<double>(std::clock() - cpu_started) /
    static_cast<double>(CLOCKS_PER_SEC) << '\n';
  return 0;
}

int run(int argc, char ** argv)
{
  if (argc >= 2 && std::string(argv[1]) == "--temporal-envelope") {
    return runTemporalEnvelopeAudit(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--representation-root-cause") {
    return runRepresentationRootCause(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--geometry-budget") {
    return runGeometryBudget(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--c2-key") {
    return runC2Key(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--time-axis") {
    return runTimeAxis(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--observability-actionability") {
    return runObservabilityActionability(argc, argv);
  }
  if (argc >= 2 && std::string(argv[1]) == "--exact-production-entry") {
    return runExactProductionEntry(argc, argv);
  }
  if (argc < 4 || std::string(argv[1]) != "--output") {
    std::cerr << "usage: path_family_feasibility_audit --output OUTPUT_DIR STREAM...\n";
    return 2;
  }
  const std::filesystem::path output_directory = argv[2];
  std::filesystem::create_directories(output_directory / "snapshots");
  const auto wall_start = std::chrono::steady_clock::now();
  const std::clock_t cpu_start = std::clock();
  std::vector<ScenarioResult> results;
  for (int index = 3; index < argc; ++index) {
    const auto stream = readStream(argv[index]);
    std::cerr << "Selecting Family A snapshot for " << stream.scenario << "...\n";
    auto selected = selectRepresentative(stream);
    std::cerr << "  frame=" << selected.frame_index
              << " stamp=" << selected.planning_frame.logical_stamp_ns << "\n";
    auto result = evaluateScenario(
      stream, selected.frame_index, selected.planning_frame,
      selected.input_reconstruction,
      std::move(selected.family_a));
    writeSnapshot(
      result,
      output_directory / "snapshots" / (stream.scenario + ".snapshot.tsv"));
    std::cerr << "  classification=" << result.classification
              << " digest=" << result.digest << "\n";
    results.push_back(std::move(result));
  }
  if (results.empty()) {
    throw std::runtime_error("no scenarios were evaluated");
  }
  auto representative = std::find_if(
    results.begin(), results.end(),
    [](const ScenarioResult & result) {return result.stream.scenario == "validation_019";});
  if (representative == results.end()) {
    representative = results.begin();
  }
  std::cerr << "Repeating offline audit for determinism: "
            << representative->stream.scenario << "...\n";
  const auto repeated = rerunScenario(*representative);
  if (repeated.digest != representative->digest) {
    throw std::runtime_error("determinism digest mismatch for " + representative->stream.scenario);
  }
  const double wall_s = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - wall_start).count();
  const double cpu_s = static_cast<double>(std::clock() - cpu_start) /
    static_cast<double>(CLOCKS_PER_SEC);
  writeRawResults(
    results, output_directory, wall_s, cpu_s, representative->stream.scenario,
    representative->digest, repeated.digest);
  std::cerr << "Completed offline audit: wall_s=" << wall_s << " cpu_s=" << cpu_s << "\n";
  return 0;
}

}  // namespace path_family_audit

int main(int argc, char ** argv)
{
  try {
    return path_family_audit::run(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "path-family audit failed: " << error.what() << '\n';
    return 1;
  }
}
