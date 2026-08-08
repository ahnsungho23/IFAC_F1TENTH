#include "global_planning/clcs_frenet_converter.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "geometry/clcs_exceptions.h"

namespace global_planning
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kMinSegmentLength = 1.0e-9;

double distance(const ReferenceWaypoint & a, const ReferenceWaypoint & b)
{
  return std::hypot(b.x - a.x, b.y - a.y);
}

double cross(
  const ReferenceWaypoint & a,
  const ReferenceWaypoint & b,
  const ReferenceWaypoint & c)
{
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
}

bool rangesOverlap(double a_min, double a_max, double b_min, double b_max)
{
  if (a_min > a_max) {
    std::swap(a_min, a_max);
  }
  if (b_min > b_max) {
    std::swap(b_min, b_max);
  }
  return std::max(a_min, b_min) <= std::min(a_max, b_max);
}

bool segmentsIntersect(
  const ReferenceWaypoint & a,
  const ReferenceWaypoint & b,
  const ReferenceWaypoint & c,
  const ReferenceWaypoint & d)
{
  if (!rangesOverlap(a.x, b.x, c.x, d.x) || !rangesOverlap(a.y, b.y, c.y, d.y)) {
    return false;
  }

  const double c1 = cross(a, b, c);
  const double c2 = cross(a, b, d);
  const double c3 = cross(c, d, a);
  const double c4 = cross(c, d, b);
  return (c1 * c2 < 0.0) && (c3 * c4 < 0.0);
}

bool finiteWaypoint(const ReferenceWaypoint & waypoint)
{
  return std::isfinite(waypoint.x) && std::isfinite(waypoint.y) && std::isfinite(waypoint.s);
}

// Numerically identical reimplementation of the private
// geometry::Segment::convertToCurvilinearCoords(x, y, lambda), rebuilt on the
// public Segment getters so the vendored library stays unmodified. The
// windowed search in projectInWindow() needs per-segment projections and
// cannot call the private overload directly.
Eigen::Vector2d segmentCurvilinearCoords(
  const geometry::Segment & segment,
  const double x,
  const double y,
  double & lambda)
{
  const Eigen::Vector2d pt_1 = segment.pt_1();
  const Eigen::Vector2d pt_2 = segment.pt_2();
  const double length = segment.length();
  const Eigen::Vector2d tangent = (pt_2 - pt_1).normalized();
  const Eigen::Vector2d normal(-tangent.y(), tangent.x());

  const Eigen::Vector2d point(x, y);
  const Eigen::Vector2d local(tangent.dot(point - pt_1), normal.dot(point - pt_1));

  // Endpoint tangents in the segment-local frame; see Eq. (7) in
  // Hery et al. (2017): Map-based curvilinear coordinates for autonomous
  // vehicles. The vendored code keeps the slopes only, so the ratios below
  // are identical to its m_1_/m_2_.
  const Eigen::Vector2d t_start = segment.tangentSegmentStart();
  const Eigen::Vector2d t_end = segment.tangentSegmentEnd();
  const double m_start = normal.dot(t_start) / tangent.dot(t_start);
  const double m_end = normal.dot(t_end) / tangent.dot(t_end);

  lambda = -1.0;
  const double divider = length - local.y() * (m_end - m_start);
  if (std::isgreater(std::abs(divider), 0.0)) {
    lambda = (local.x() + local.y() * m_start) / divider;
  }

  const Eigen::Vector2d base = lambda * pt_2 + (1.0 - lambda) * pt_1;
  double pseudo_distance = (point - base).norm();
  if (std::isless(local.y(), 0.0)) {
    pseudo_distance = -pseudo_distance;
  }
  return Eigen::Vector2d(lambda * length, pseudo_distance);
}

}  // namespace

ClcsFrenetConverter::Ptr ClcsFrenetConverter::create(
  const std::vector<ReferenceWaypoint> & waypoints,
  const ClcsFrenetConfig & config,
  const std::uint64_t path_version)
{
  const auto start = std::chrono::steady_clock::now();

  ClcsBuildStats stats;
  stats.path_version = path_version;
  stats.input_waypoint_count = waypoints.size();

  const auto reference_points = preprocessReferencePath(waypoints, config, stats);
  if (reference_points.size() < 3) {
    throw std::runtime_error("CLCS reference path requires at least 3 valid points");
  }

  stats.track_length = computePathLength(reference_points);
  if (stats.track_length < config.min_path_length) {
    throw std::runtime_error("CLCS reference path length is too short");
  }

  stats.waypoint_s_max_error = computeWaypointSMaxError(reference_points);
  stats.large_gap_count = countLargeGaps(reference_points, config.large_gap_factor);
  stats.self_intersection_count = countSelfIntersections(reference_points, config.closed_loop);

  auto clcs = std::make_shared<geometry::CurvilinearCoordinateSystem>(
    toEigenPolyline(reference_points),
    config.projection_domain_limit,
    config.projection_domain_epsilon,
    config.projection_domain_eps2,
    "off",
    config.projection_domain_method);

  stats.track_length = clcs->length();
  stats.reference_point_count = reference_points.size();
  const auto end = std::chrono::steady_clock::now();
  stats.build_time_ms =
    std::chrono::duration<double, std::milli>(end - start).count();

  return Ptr(new ClcsFrenetConverter(config, waypoints, reference_points, clcs, stats));
}

ClcsFrenetConverter::ClcsFrenetConverter(
  const ClcsFrenetConfig & config,
  const std::vector<ReferenceWaypoint> & source_waypoints,
  const std::vector<ReferenceWaypoint> & reference_points,
  std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs,
  const ClcsBuildStats & stats)
: config_(config),
  source_waypoints_(source_waypoints),
  reference_points_(reference_points),
  clcs_(std::move(clcs)),
  stats_(stats)
{
}

ClcsConversionResult ClcsFrenetConverter::convert(const ClcsConversionInput & input) const
{
  const auto start = std::chrono::steady_clock::now();

  ClcsConversionResult result = newResult();

  if (!std::isfinite(input.x) || !std::isfinite(input.y) || !std::isfinite(input.yaw)) {
    result.error_message = "non-finite Cartesian pose input";
    return result;
  }

  try {
    int segment_index = -1;
    const Eigen::Vector2d curvilinear =
      clcs_->convertToCurvilinearCoordsAndGetSegmentIdx(input.x, input.y, segment_index, false);

    if (!std::isfinite(curvilinear.x()) || !std::isfinite(curvilinear.y())) {
      result.error_message = "CLCS returned non-finite curvilinear coordinates";
      return result;
    }

    finishConversion(input, start, curvilinear.x(), curvilinear.y(), segment_index, result);
    return result;
  } catch (const geometry::ProjectionDomainError & e) {
    result.error_message = e.what();
  } catch (const std::exception & e) {
    result.error_message = e.what();
  }

  const auto end = std::chrono::steady_clock::now();
  result.conversion_time_us =
    std::chrono::duration<double, std::micro>(end - start).count();
  return result;
}

ClcsConversionResult ClcsFrenetConverter::convertTracked(
  const ClcsConversionInput & input,
  ClcsContinuityState & state) const
{
  const auto start = std::chrono::steady_clock::now();

  ClcsConversionResult result = newResult();
  const auto stamp_time = [&result, &start]() {
      const auto end = std::chrono::steady_clock::now();
      result.conversion_time_us =
        std::chrono::duration<double, std::micro>(end - start).count();
    };

  if (!std::isfinite(input.x) || !std::isfinite(input.y) || !std::isfinite(input.yaw)) {
    result.error_message = "non-finite Cartesian pose input";
    stamp_time();
    return result;
  }

  try {
    double raw_s = 0.0;
    double d = 0.0;
    int segment_index = -1;
    bool projected = false;
    std::string miss_reason;

    if (state.initialized) {
      double s_lo = state.s_prev - config_.backward_tolerance;
      double s_hi = state.s_prev + config_.forward_window;
      if (!config_.closed_loop) {
        s_lo = std::max(0.0, s_lo);
        s_hi = std::min(stats_.track_length, s_hi);
      }
      projected = projectInWindow(input.x, input.y, s_lo, s_hi, raw_s, d, segment_index);
      if (!projected) {
        miss_reason = "no projection within monotonic window";
      } else if (std::isfinite(config_.tracked_max_projection_distance) &&
        config_.tracked_max_projection_distance > 0.0 &&
        std::abs(d) > config_.tracked_max_projection_distance)
      {
        // In-window but too far off the reference: treat as a miss so a
        // teleport onto a nearby unrelated arc cannot keep tracking silently.
        projected = false;
        miss_reason = "tracked projection exceeds tracked_max_projection_distance";
      }
    } else if (config_.initial_seed_window > 0.0) {
      // Skidpad-style first fix: only the configured start slice.
      const double s_hi = std::min(stats_.track_length, config_.initial_seed_window);
      projected = projectInWindow(input.x, input.y, 0.0, s_hi, raw_s, d, segment_index);
      if (!projected) {
        miss_reason = "no projection in initial seed window";
      }
    }

    if (!projected) {
      const bool first_fix_full_search =
        !state.initialized && config_.initial_seed_window <= 0.0;
      if (!first_fix_full_search) {
        state.consecutive_misses += 1;
        const bool reacquire = config_.reacquire_after_misses > 0 &&
          state.consecutive_misses >= config_.reacquire_after_misses;
        if (!reacquire) {
          // Fail closed: NEVER silently fall back to the global search —
          // that is exactly the branch flip convertTracked() prevents.
          result.error_message = miss_reason + " (miss " +
            std::to_string(state.consecutive_misses) + ")";
          stamp_time();
          return result;
        }
        result.reacquired = true;
      }

      const Eigen::Vector2d curvilinear =
        clcs_->convertToCurvilinearCoordsAndGetSegmentIdx(input.x, input.y, segment_index, false);
      raw_s = curvilinear.x();
      d = curvilinear.y();
    }

    if (!std::isfinite(raw_s) || !std::isfinite(d)) {
      result.error_message = "CLCS returned non-finite curvilinear coordinates";
      stamp_time();
      return result;
    }

    finishConversion(input, start, raw_s, d, segment_index, result);
    if (result.valid) {
      state.initialized = true;
      state.s_prev = result.s;
      state.consecutive_misses = 0;
    }
    return result;
  } catch (const geometry::ProjectionDomainError & e) {
    result.error_message = e.what();
  } catch (const std::exception & e) {
    result.error_message = e.what();
  }

  stamp_time();
  return result;
}

ClcsConversionResult ClcsFrenetConverter::newResult() const
{
  ClcsConversionResult result;
  result.track_length = stats_.track_length;
  result.clcs_build_time_ms = stats_.build_time_ms;
  result.waypoint_s_max_error = stats_.waypoint_s_max_error;
  result.path_version = stats_.path_version;
  return result;
}

void ClcsFrenetConverter::finishConversion(
  const ClcsConversionInput & input,
  const std::chrono::steady_clock::time_point & start,
  const double raw_s,
  const double d,
  const int segment_index,
  ClcsConversionResult & result) const
{
  result.raw_s = raw_s;
  result.s = normalizeS(raw_s);
  result.d = d;
  result.segment_index = segment_index;

  if (std::isfinite(config_.max_projection_distance) &&
    config_.max_projection_distance > 0.0 &&
    std::abs(result.d) > config_.max_projection_distance)
  {
    result.error_message = "projection distance exceeds max_projection_distance";
    return;
  }

  result.reference_yaw = referenceYaw(result.raw_s);
  result.heading_error = normalizeAngle(input.yaw - result.reference_yaw);

  double vx_map = input.linear_x;
  double vy_map = input.linear_y;
  if (config_.velocity_frame == VelocityFrame::kBody) {
    const double cos_yaw = std::cos(input.yaw);
    const double sin_yaw = std::sin(input.yaw);
    vx_map = cos_yaw * input.linear_x - sin_yaw * input.linear_y;
    vy_map = sin_yaw * input.linear_x + cos_yaw * input.linear_y;
  }

  if (config_.publish_frenet_velocity) {
    const double cos_ref = std::cos(result.reference_yaw);
    const double sin_ref = std::sin(result.reference_yaw);
    result.v_s = cos_ref * vx_map + sin_ref * vy_map;
    result.v_d = -sin_ref * vx_map + cos_ref * vy_map;
  } else {
    result.v_s = input.linear_x;
    result.v_d = input.linear_y;
  }
  result.yaw_rate = input.yaw_rate;

  const Eigen::Vector2d reconstructed =
    clcs_->convertToCartesianCoords(sForClcsQuery(result.raw_s), result.d, false);
  result.reconstruction_error =
    std::hypot(reconstructed.x() - input.x, reconstructed.y() - input.y);

  const auto end = std::chrono::steady_clock::now();
  result.conversion_time_us =
    std::chrono::duration<double, std::micro>(end - start).count();
  result.valid = std::isfinite(result.s) && std::isfinite(result.d) &&
    std::isfinite(result.reference_yaw) && std::isfinite(result.heading_error) &&
    std::isfinite(result.v_s) && std::isfinite(result.v_d);
  if (!result.valid) {
    result.error_message = "conversion result contains non-finite values";
  }
}

bool ClcsFrenetConverter::projectInWindow(
  const double x,
  const double y,
  const double s_lo,
  const double s_hi,
  double & raw_s,
  double & d,
  int & segment_index) const
{
  const auto & segments = clcs_->getSegmentList();
  const auto & segment_longitudinal = clcs_->segmentsLongitudinalCoordinates();
  const std::size_t num_segments = segments.size();
  const double length = stats_.track_length;
  const bool wrap = config_.closed_loop && length > 0.0;

  // Overlap between an interval [a, b] and the window; for closed loops the
  // +-track-length shifted copies of the window are tested too, so a window
  // reaching past the seam keeps working.
  const auto overlaps_window = [s_lo, s_hi, wrap, length](const double a, const double b) {
      if (b >= s_lo && a <= s_hi) {
        return true;
      }
      if (wrap) {
        if (b >= s_lo - length && a <= s_hi - length) {
          return true;
        }
        if (b >= s_lo + length && a <= s_hi + length) {
          return true;
        }
      }
      return false;
    };

  double best_abs_d = std::numeric_limits<double>::infinity();
  int best_index = -1;
  double best_raw_s = 0.0;
  double best_d = 0.0;

  for (std::size_t i = 0; i < num_segments; ++i) {
    const double seg_start = segment_longitudinal[i];
    const double seg_end = seg_start + segments[i]->length();
    if (!overlaps_window(seg_start, seg_end)) {
      continue;
    }
    double lambda = 0.0;
    const Eigen::Vector2d curvilinear =
      segmentCurvilinearCoords(*segments[i], x, y, lambda);
    // Same acceptance tolerance as the vendored full search (10e-8).
    if (!std::isgreaterequal(lambda + 1.0e-7, 0.0) ||
      !std::islessequal(lambda - 1.0e-7, 1.0))
    {
      continue;
    }
    const double candidate_d = curvilinear.y();
    if (!std::isfinite(curvilinear.x()) || !std::isfinite(candidate_d)) {
      continue;
    }
    const double candidate_raw_s = curvilinear.x() + seg_start;
    // The candidate itself must land inside the window (skidpad semantics),
    // not merely on a segment that touches it.
    constexpr double kWindowSlack = 1.0e-9;
    if (!overlaps_window(candidate_raw_s - kWindowSlack, candidate_raw_s + kWindowSlack)) {
      continue;
    }
    if (std::abs(candidate_d) < best_abs_d) {
      best_abs_d = std::abs(candidate_d);
      best_index = static_cast<int>(i);
      best_raw_s = candidate_raw_s;
      best_d = candidate_d;
    }
  }

  if (best_index < 0) {
    return false;
  }
  raw_s = best_raw_s;
  d = best_d;
  segment_index = best_index;
  return true;
}

bool ClcsFrenetConverter::pathChanged(
  const std::vector<ReferenceWaypoint> & previous,
  const std::vector<ReferenceWaypoint> & current,
  const double tolerance)
{
  if (previous.size() != current.size()) {
    return true;
  }

  for (std::size_t i = 0; i < previous.size(); ++i) {
    if (std::hypot(previous[i].x - current[i].x, previous[i].y - current[i].y) > tolerance) {
      return true;
    }
  }
  return false;
}

double ClcsFrenetConverter::normalizeAngle(double angle)
{
  while (angle >= kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

std::vector<ReferenceWaypoint> ClcsFrenetConverter::preprocessReferencePath(
  const std::vector<ReferenceWaypoint> & waypoints,
  const ClcsFrenetConfig & config,
  ClcsBuildStats & stats)
{
  std::vector<ReferenceWaypoint> reference_points;
  reference_points.reserve(waypoints.size() + 1);

  for (const auto & waypoint : waypoints) {
    if (!finiteWaypoint(waypoint)) {
      ++stats.invalid_point_count;
      continue;
    }
    if (!reference_points.empty() &&
      distance(reference_points.back(), waypoint) <= config.duplicate_point_tolerance)
    {
      ++stats.removed_duplicate_count;
      continue;
    }
    reference_points.push_back(waypoint);
  }

  if (config.closed_loop && reference_points.size() >= 3) {
    if (distance(reference_points.front(), reference_points.back()) <=
      config.duplicate_point_tolerance)
    {
      reference_points.pop_back();
      ++stats.removed_duplicate_count;
    }

    if (distance(reference_points.front(), reference_points.back()) >
      config.duplicate_point_tolerance)
    {
      ReferenceWaypoint closing_point = reference_points.front();
      closing_point.s = computePathLength(reference_points) +
        distance(reference_points.back(), reference_points.front());
      reference_points.push_back(closing_point);
      stats.closed_by_appending_first_point = true;
    }
  }

  return reference_points;
}

geometry::EigenPolyline ClcsFrenetConverter::toEigenPolyline(
  const std::vector<ReferenceWaypoint> & reference_points)
{
  geometry::EigenPolyline polyline;
  polyline.reserve(reference_points.size());
  for (const auto & point : reference_points) {
    polyline.emplace_back(point.x, point.y);
  }
  return polyline;
}

double ClcsFrenetConverter::computePathLength(const std::vector<ReferenceWaypoint> & points)
{
  if (points.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    length += distance(points[i], points[i + 1]);
  }
  return length;
}

double ClcsFrenetConverter::computeWaypointSMaxError(
  const std::vector<ReferenceWaypoint> & points)
{
  if (points.size() < 2) {
    return 0.0;
  }

  double cumulative_s = points.front().s;
  double max_error = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    cumulative_s += distance(points[i - 1], points[i]);
    max_error = std::max(max_error, std::abs(points[i].s - cumulative_s));
  }
  return max_error;
}

std::size_t ClcsFrenetConverter::countLargeGaps(
  const std::vector<ReferenceWaypoint> & points,
  const double large_gap_factor)
{
  if (points.size() < 3 || large_gap_factor <= 0.0) {
    return 0;
  }

  std::vector<double> lengths;
  lengths.reserve(points.size() - 1);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const double segment_length = distance(points[i], points[i + 1]);
    if (segment_length > kMinSegmentLength) {
      lengths.push_back(segment_length);
    }
  }

  if (lengths.empty()) {
    return 0;
  }

  std::sort(lengths.begin(), lengths.end());
  const double median = lengths[lengths.size() / 2];
  return static_cast<std::size_t>(
    std::count_if(lengths.begin(), lengths.end(), [median, large_gap_factor](const double length) {
      return length > median * large_gap_factor;
    }));
}

std::size_t ClcsFrenetConverter::countSelfIntersections(
  const std::vector<ReferenceWaypoint> & points,
  const bool closed_loop)
{
  if (points.size() < 4) {
    return 0;
  }

  std::size_t count = 0;
  const std::size_t segment_count = points.size() - 1;
  for (std::size_t i = 0; i < segment_count; ++i) {
    for (std::size_t j = i + 1; j < segment_count; ++j) {
      if (j == i + 1) {
        continue;
      }
      if (closed_loop && i == 0 && j + 1 == segment_count) {
        continue;
      }
      if (segmentsIntersect(points[i], points[i + 1], points[j], points[j + 1])) {
        ++count;
      }
    }
  }
  return count;
}

double ClcsFrenetConverter::normalizeS(const double s) const
{
  if (!config_.closed_loop || stats_.track_length <= 0.0) {
    return s;
  }

  double normalized = std::fmod(s, stats_.track_length);
  if (normalized < 0.0) {
    normalized += stats_.track_length;
  }
  if (normalized >= stats_.track_length) {
    normalized = 0.0;
  }
  return normalized;
}

double ClcsFrenetConverter::sForClcsQuery(const double s) const
{
  if (stats_.track_length <= 0.0) {
    return s;
  }

  const double eps = std::max(1.0e-6, std::min(config_.tangent_epsilon, stats_.track_length * 0.25));
  if (s <= 0.0) {
    return eps;
  }
  if (s >= stats_.track_length) {
    return stats_.track_length - eps;
  }
  return s;
}

double ClcsFrenetConverter::referenceYaw(const double raw_s) const
{
  const Eigen::Vector2d tangent = clcs_->tangent(sForClcsQuery(raw_s));
  return std::atan2(tangent.y(), tangent.x());
}

VelocityFrame parseVelocityFrame(const std::string & value)
{
  if (value == "map" || value == "Map" || value == "MAP") {
    return VelocityFrame::kMap;
  }
  return VelocityFrame::kBody;
}

std::string toString(const VelocityFrame value)
{
  return value == VelocityFrame::kMap ? "map" : "body";
}

}  // namespace global_planning
