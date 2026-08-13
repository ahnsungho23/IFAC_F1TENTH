// Copyright 2026 2026_IFAC contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace
{

using local_planning::EgoFrenetState;
using local_planning::ObstacleScenarioEvaluation;
using local_planning::RacelineSplineParameters;
using local_planning::RacelineSplinePlanner;
using local_planning::SplinePlanKind;

struct Options
{
  std::string reference_path;
  std::string output_path;
  double ego_lookback_m{7.0};
  double obstacle_size_m{0.20};
  double d_min{-0.5};
  double d_max{0.5};
  double d_step{0.1};
  double hard_collision_margin_m{0.03};
  RacelineSplineParameters parameters;
};

std::vector<std::string> split(const std::string & value, char delimiter)
{
  std::vector<std::string> fields;
  std::stringstream stream(value);
  std::string field;
  while (std::getline(stream, field, delimiter)) {
    fields.push_back(field);
  }
  return fields;
}

std::vector<double> parseDoubleList(const std::string & value)
{
  std::vector<double> values;
  for (const auto & field : split(value, ',')) {
    if (!field.empty()) {
      values.push_back(std::stod(field));
    }
  }
  return values;
}

void setParameter(Options & options, const std::string & assignment)
{
  const std::size_t separator = assignment.find('=');
  if (separator == std::string::npos || separator == 0U) {
    throw std::runtime_error("--param expects key=value: " + assignment);
  }
  const std::string key = assignment.substr(0, separator);
  const std::string value = assignment.substr(separator + 1U);
  auto & p = options.parameters;

  if (key == "detection_lookahead_m") {
    p.detection_lookahead_m = std::stod(value);
  } else if (key == "obstacle_cluster_gap_m") {
    p.obstacle_cluster_gap_m = std::stod(value);
  } else if (key == "obstacle_longitudinal_padding_m") {
    p.obstacle_longitudinal_padding_m = std::stod(value);
  } else if (key == "obstacle_clearance_m") {
    p.obstacle_clearance_m = std::stod(value);
  } else if (key == "blocking_margin_m") {
    p.blocking_margin_m = std::stod(value);
  } else if (key == "vehicle_half_width_m") {
    p.vehicle_half_width_m = std::stod(value);
  } else if (key == "boundary_margin_m") {
    p.boundary_margin_m = std::stod(value);
  } else if (key == "fallback_track_half_width_m") {
    p.fallback_track_half_width_m = std::stod(value);
  } else if (key == "pre_apex_distances_m") {
    p.pre_apex_distances_m = parseDoubleList(value);
  } else if (key == "post_apex_distances_m") {
    p.post_apex_distances_m = parseDoubleList(value);
  } else if (key == "transition_distance_scales") {
    p.transition_distance_scales = parseDoubleList(value);
  } else if (key == "outside_line_transition_scale") {
    p.outside_line_transition_scale = std::stod(value);
  } else if (key == "post_merge_lookahead_m") {
    p.post_merge_lookahead_m = std::stod(value);
  } else if (key == "post_merge_min_time_sec") {
    p.post_merge_min_time_sec = std::stod(value);
  } else if (key == "minimum_target_offset_m") {
    p.minimum_target_offset_m = std::stod(value);
  } else if (key == "maximum_target_offset_m") {
    p.maximum_target_offset_m = std::stod(value);
  } else if (key == "commitment_clearance_reserve_m") {
    p.commitment_clearance_reserve_m = std::stod(value);
  } else if (key == "minimum_avoidance_clearance_m") {
    p.minimum_avoidance_clearance_m = std::stod(value);
  } else if (key == "side_tie_epsilon_m") {
    p.side_tie_epsilon_m = std::stod(value);
  } else if (key == "maximum_lateral_slope") {
    p.maximum_lateral_slope = std::stod(value);
  } else if (key == "maximum_curvature_radpm") {
    p.maximum_curvature_radpm = std::stod(value);
  } else if (key == "maximum_curvature_rate_radpm2") {
    p.maximum_curvature_rate_radpm2 = std::stod(value);
  } else if (key == "safe_stop_buffer_m") {
    p.safe_stop_buffer_m = std::stod(value);
  } else if (key == "safe_stop_deceleration_mps2") {
    p.safe_stop_deceleration_mps2 = std::stod(value);
  } else if (key == "minimum_path_points") {
    p.minimum_path_points = std::stoi(value);
  } else if (key == "hard_collision_margin_m") {
    options.hard_collision_margin_m = std::stod(value);
  } else {
    throw std::runtime_error("unsupported planner parameter: " + key);
  }
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    const auto next = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("missing value after " + argument);
        }
        return argv[++i];
      };
    if (argument == "--reference") {
      options.reference_path = next();
    } else if (argument == "--output") {
      options.output_path = next();
    } else if (argument == "--ego-lookback") {
      options.ego_lookback_m = std::stod(next());
    } else if (argument == "--obstacle-size") {
      options.obstacle_size_m = std::stod(next());
    } else if (argument == "--d-min") {
      options.d_min = std::stod(next());
    } else if (argument == "--d-max") {
      options.d_max = std::stod(next());
    } else if (argument == "--d-step") {
      options.d_step = std::stod(next());
    } else if (argument == "--param") {
      setParameter(options, next());
    } else if (argument == "--help" || argument == "-h") {
      std::cout <<
        "Usage: adaptive_side_evaluator --reference PATH --output PATH [options]\n"
        "  --ego-lookback M       Ego distance behind each obstacle (default 7.0)\n"
        "  --obstacle-size M      Square obstacle size (default 0.20)\n"
        "  --d-min/--d-max/--d-step   Lateral scenario grid\n"
        "  --param key=value      Planner parameter override; repeat as needed\n";
      std::exit(0);
    } else {
      throw std::runtime_error("unknown argument: " + argument);
    }
  }
  if (options.reference_path.empty() || options.output_path.empty()) {
    throw std::runtime_error("--reference and --output are required");
  }
  return options;
}

void validateOptions(const Options & options)
{
  const auto & p = options.parameters;
  if (!(options.ego_lookback_m > 0.0) || !(options.obstacle_size_m > 0.0) ||
    !(options.d_step > 0.0) || options.d_max < options.d_min)
  {
    throw std::runtime_error("scenario distances and obstacle size must be positive");
  }
  const double d_intervals = (options.d_max - options.d_min) / options.d_step;
  if (std::abs(d_intervals - std::round(d_intervals)) > 1.0e-9) {
    throw std::runtime_error("d range must be an integer multiple of d-step");
  }
  if (p.pre_apex_distances_m.size() != 3U || p.post_apex_distances_m.size() != 3U ||
    p.transition_distance_scales.empty())
  {
    throw std::runtime_error("pre/post apex arrays need three values and scales cannot be empty");
  }
  if (!std::is_sorted(
      p.transition_distance_scales.begin(), p.transition_distance_scales.end()) ||
    std::adjacent_find(
      p.transition_distance_scales.begin(), p.transition_distance_scales.end()) !=
    p.transition_distance_scales.end() || p.transition_distance_scales.front() <= 0.0)
  {
    throw std::runtime_error("transition_distance_scales must be strictly increasing and positive");
  }
  if (p.minimum_avoidance_clearance_m + 1.0e-9 <
    p.vehicle_half_width_m + options.hard_collision_margin_m)
  {
    throw std::runtime_error(
            "minimum_avoidance_clearance_m is below vehicle half-width plus hard margin");
  }
  if (p.minimum_avoidance_clearance_m > p.obstacle_clearance_m + 1.0e-9) {
    throw std::runtime_error(
            "minimum_avoidance_clearance_m cannot exceed obstacle_clearance_m");
  }
}

f110_msgs::msg::WpntArray loadReference(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {
    throw std::runtime_error("could not open reference CSV: " + path);
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("reference CSV is empty: " + path);
  }
  const auto header = split(line, ',');
  std::map<std::string, std::size_t> column;
  for (std::size_t i = 0; i < header.size(); ++i) {
    column[header[i]] = i;
  }
  const auto indexOf = [&](const std::string & first, const std::string & second = "") {
      auto found = column.find(first);
      if (found == column.end() && !second.empty()) {found = column.find(second);}
      if (found == column.end()) {throw std::runtime_error("missing reference column: " + first);}
      return found->second;
    };
  const std::size_t id_i = indexOf("id");
  const std::size_t s_i = indexOf("s_m", "s");
  const std::size_t x_i = indexOf("x_m");
  const std::size_t y_i = indexOf("y_m");
  const std::size_t psi_i = indexOf("psi_rad");
  const std::size_t kappa_i = indexOf("kappa_radpm");
  const std::size_t vx_i = indexOf("vx_mps");
  const std::size_t ax_i = indexOf("ax_mps2");
  const std::size_t left_i = indexOf("d_left");
  const std::size_t right_i = indexOf("d_right");

  f110_msgs::msg::WpntArray reference;
  reference.header.frame_id = "map";
  while (std::getline(stream, line)) {
    if (line.empty()) {continue;}
    const auto fields = split(line, ',');
    if (fields.size() < header.size()) {
      throw std::runtime_error("malformed reference CSV row: " + line);
    }
    f110_msgs::msg::Wpnt waypoint;
    waypoint.id = static_cast<int32_t>(std::stoi(fields[id_i]));
    waypoint.s_m = std::stod(fields[s_i]);
    waypoint.x_m = std::stod(fields[x_i]);
    waypoint.y_m = std::stod(fields[y_i]);
    waypoint.psi_rad = std::stod(fields[psi_i]);
    waypoint.kappa_radpm = std::stod(fields[kappa_i]);
    waypoint.vx_mps = std::stod(fields[vx_i]);
    waypoint.ax_mps2 = std::stod(fields[ax_i]);
    waypoint.d_left = std::stod(fields[left_i]);
    waypoint.d_right = std::stod(fields[right_i]);
    reference.wpnts.push_back(waypoint);
  }
  return reference;
}

double wrap(double value, double length)
{
  double wrapped = std::fmod(value, length);
  if (wrapped < 0.0) {wrapped += length;}
  return wrapped;
}

double circularDistance(double first, double second, double length)
{
  const double forward = wrap(second - first, length);
  return std::min(forward, length - forward);
}

double speedAt(
  const f110_msgs::msg::WpntArray & reference, double s, double track_length)
{
  const auto best = std::min_element(
    reference.wpnts.begin(), reference.wpnts.end(),
    [&](const auto & first, const auto & second) {
      return circularDistance(s, first.s_m, track_length) <
             circularDistance(s, second.s_m, track_length);
    });
  return best == reference.wpnts.end() ? 0.0 : std::max(0.0, best->vx_mps);
}

std::string csvField(const std::string & value)
{
  std::string escaped = value;
  std::size_t position = 0U;
  while ((position = escaped.find('"', position)) != std::string::npos) {
    escaped.insert(position, 1U, '"');
    position += 2U;
  }
  return '"' + escaped + '"';
}

std::string planKind(SplinePlanKind kind)
{
  switch (kind) {
    case SplinePlanKind::kNoObstacle: return "no_obstacle";
    case SplinePlanKind::kPreparation: return "preparation";
    case SplinePlanKind::kAvoidance: return "avoidance";
    case SplinePlanKind::kSafeStop: return "safe_stop";
    case SplinePlanKind::kNoSafePath: return "no_safe_path";
  }
  return "unknown";
}

std::string decision(const ObstacleScenarioEvaluation & evaluation)
{
  if (evaluation.result.kind != SplinePlanKind::kAvoidance) {return "safe_stop";}
  return evaluation.result.go_left ? "left" : "right";
}

void writeEvaluation(
  std::ostream & output,
  const f110_msgs::msg::Wpnt & reference,
  double obstacle_d,
  const EgoFrenetState & ego,
  const ObstacleScenarioEvaluation & evaluation)
{
  const auto number = [](double value) {
      if (!std::isfinite(value)) {return std::string();}
      std::ostringstream stream;
      stream << std::setprecision(12) << value;
      return stream.str();
    };
  output << reference.id << ',' << number(reference.s_m) << ',' << number(obstacle_d) << ',' <<
    number(ego.s) << ',' << number(ego.d) << ',' << number(ego.speed) << ',' <<
    decision(evaluation) << ',' << planKind(evaluation.result.kind) << ',' <<
    number(evaluation.result.target_d) << ',' << (evaluation.left.evaluated ? 1 : 0) << ',' <<
    (evaluation.left.valid ? 1 : 0) << ',' << number(evaluation.left.target_d) << ',' <<
    number(evaluation.left.min_headroom) << ',' << (evaluation.right.evaluated ? 1 : 0) << ',' <<
    (evaluation.right.valid ? 1 : 0) << ',' << number(evaluation.right.target_d) << ',' <<
    number(evaluation.right.min_headroom) << ',' <<
    (evaluation.evaluated_reduced_clearance ? 1 : 0) << ',' <<
    (evaluation.selected_reduced_clearance ? 1 : 0) << ',' <<
    evaluation.result.path.wpnts.size() << ',' << csvField(evaluation.left.reason) << ',' <<
    csvField(evaluation.right.reason) << ',' << csvField(evaluation.result.reason) << '\n';
}

int run(const Options & options)
{
  validateOptions(options);
  const auto reference = loadReference(options.reference_path);
  RacelineSplinePlanner planner(options.parameters);
  std::string reference_error;
  if (!planner.setReference(reference, &reference_error)) {
    throw std::runtime_error("invalid reference: " + reference_error);
  }

  std::ofstream output(options.output_path);
  if (!output) {
    throw std::runtime_error("could not open output CSV: " + options.output_path);
  }
  output <<
    "index,s,d,ego_s,ego_d,ego_speed,decision,plan_kind,target_d,"
    "left_evaluated,left_valid,left_target_d,left_headroom,"
    "right_evaluated,right_valid,right_target_d,right_headroom,"
    "evaluated_reduced_clearance,selected_reduced_clearance,path_points,"
    "left_reason,right_reason,result_reason\n";

  const int d_count = static_cast<int>(
    std::llround((options.d_max - options.d_min) / options.d_step)) + 1;
  for (const auto & waypoint : reference.wpnts) {
    const double ego_s = wrap(waypoint.s_m - options.ego_lookback_m, planner.trackLength());
    const EgoFrenetState ego{
      ego_s, 0.0, speedAt(reference, ego_s, planner.trackLength())};
    for (int d_index = 0; d_index < d_count; ++d_index) {
      const double obstacle_d = options.d_min + options.d_step * static_cast<double>(d_index);
      const double half_size = 0.5 * options.obstacle_size_m;
      f110_msgs::msg::Obstacle obstacle;
      obstacle.id = waypoint.id;
      obstacle.s_center = waypoint.s_m;
      obstacle.s_start = waypoint.s_m - half_size;
      obstacle.s_end = waypoint.s_m + half_size;
      obstacle.d_center = obstacle_d;
      obstacle.d_right = obstacle_d - half_size;
      obstacle.d_left = obstacle_d + half_size;
      obstacle.size = options.obstacle_size_m;
      obstacle.is_static = true;
      const auto evaluation = planner.evaluateObstacleScenario(ego, {obstacle});
      writeEvaluation(output, waypoint, obstacle_d, ego, evaluation);
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception & error) {
    std::cerr << "adaptive_side_evaluator: " << error.what() << '\n';
    return 1;
  }
}
