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
//
// PATH_FAMILY_STREAM_V1 시나리오 스트림 리더 (테스트 전용).
//
// 재생 안전망(test_p3_production_parity)과 규정 기반 속성 테스트(test_rule_property)가
// 같은 파서를 쓴다. 파서를 복사하면 두 벌이 조용히 갈라지므로 여기 한 벌만 둔다.
// 기준선까지 스트림에 넣는 이유는 offline_trajectory_generator/output/이 gitignore라
// 다른 환경에서 재현되지 않기 때문이다.

#ifndef P3_SCENARIO_STREAM_HPP_
#define P3_SCENARIO_STREAM_HPP_

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "local_planning/raceline_spline_planner.hpp"

namespace local_planning
{
namespace test_stream
{

struct Frame
{
  EgoFrenetState ego;
  std::vector<f110_msgs::msg::Obstacle> obstacles;
};

struct Stream
{
  std::string scenario;
  std::map<std::string, double> scalars;
  std::map<std::string, std::vector<double>> vectors;
  f110_msgs::msg::WpntArray reference;
  std::vector<Frame> frames;
};

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> tokens;
  std::size_t begin = 0U;
  while (true) {
    const std::size_t end = line.find('\t', begin);
    tokens.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      return tokens;
    }
    begin = end + 1U;
  }
}

double number(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite token: " + token);
  }
  return value;
}

Stream readStream(const std::filesystem::path & path)
{
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "PATH_FAMILY_STREAM_V1") {
    throw std::runtime_error("unsupported scenario stream: " + path.string());
  }
  Stream stream;
  stream.reference.header.frame_id = "map";
  while (std::getline(input, line)) {
    const auto tokens = splitTabs(line);
    if (tokens[0] == "SCENARIO" && tokens.size() == 2U) {
      stream.scenario = tokens[1];
    } else if (tokens[0] == "SOURCE_BAG" || tokens[0] == "CONFIG_PATH") {
      continue;
    } else if (tokens[0] == "PARAM" && tokens.size() == 3U) {
      stream.scalars[tokens[1]] = number(tokens[2]);
    } else if (tokens[0] == "PARAMV" && tokens.size() >= 3U) {
      auto & output = stream.vectors[tokens[1]];
      for (std::size_t index = 3U; index < tokens.size(); ++index) {
        output.push_back(number(tokens[index]));
      }
    } else if (tokens[0] == "REFERENCE") {
      continue;
    } else if (tokens[0] == "W" && tokens.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(std::llround(number(tokens[1])));
      waypoint.s_m = number(tokens[2]);
      waypoint.d_m = number(tokens[3]);
      waypoint.x_m = number(tokens[4]);
      waypoint.y_m = number(tokens[5]);
      waypoint.d_right = number(tokens[6]);
      waypoint.d_left = number(tokens[7]);
      waypoint.psi_rad = number(tokens[8]);
      waypoint.kappa_radpm = number(tokens[9]);
      waypoint.vx_mps = number(tokens[10]);
      waypoint.ax_mps2 = number(tokens[11]);
      stream.reference.wpnts.push_back(waypoint);
    } else if (tokens[0] == "FRAME" && tokens.size() == 7U) {
      Frame frame;
      frame.ego.s = number(tokens[3]);
      frame.ego.d = number(tokens[4]);
      frame.ego.speed = number(tokens[5]);
      const auto obstacles = static_cast<std::size_t>(std::llround(number(tokens[6])));
      for (std::size_t index = 0U; index < obstacles; ++index) {
        if (!std::getline(input, line)) {
          throw std::runtime_error("truncated obstacle frame");
        }
        const auto fields = splitTabs(line);
        if (fields.size() != 10U || fields[0] != "O") {
          throw std::runtime_error("malformed obstacle record");
        }
        f110_msgs::msg::Obstacle obstacle;
        obstacle.id = static_cast<std::int32_t>(std::llround(number(fields[1])));
        obstacle.s_center = number(fields[2]);
        obstacle.s_start = number(fields[3]);
        obstacle.s_end = number(fields[4]);
        obstacle.d_right = number(fields[5]);
        obstacle.d_left = number(fields[6]);
        obstacle.size = number(fields[7]);
        obstacle.s_var = number(fields[8]);
        obstacle.d_var = number(fields[9]);
        obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
        obstacle.is_static = true;
        obstacle.is_visible = true;
        frame.obstacles.push_back(obstacle);
      }
      if (!std::getline(input, line) || line != "END_FRAME") {
        throw std::runtime_error("missing END_FRAME");
      }
      stream.frames.push_back(std::move(frame));
    } else if (tokens[0] == "END_STREAM") {
      break;
    } else if (!line.empty()) {
      throw std::runtime_error("unknown stream record: " + line);
    }
  }
  if (stream.scenario.empty() || stream.reference.wpnts.empty() || stream.frames.empty()) {
    throw std::runtime_error("incomplete scenario stream: " + path.string());
  }
  return stream;
}

// 스트림이 실은 값만 쓴다. at()이 없는 키에 대해 던지므로, 추출기와 이 목록이 어긋나면
// 조용히 구조체 기본값으로 통과하는 대신 테스트가 실패한다.
RacelineSplineParameters parametersOf(const Stream & stream)
{
  RacelineSplineParameters value;
  const auto scalar = [&](const char * name) {return stream.scalars.at(name);};
  const auto vector = [&](const char * name) {return stream.vectors.at(name);};
  value.detection_lookahead_m = scalar("detection_lookahead_m");
  value.obstacle_cluster_gap_m = scalar("obstacle_cluster_gap_m");
  value.obstacle_longitudinal_padding_m = scalar("obstacle_longitudinal_padding_m");
  value.vehicle_length_m = scalar("vehicle_length_m");
  value.vehicle_half_width_m = scalar("vehicle_half_width_m");
  value.safety_margin_m = scalar("safety_margin_m");
  // 시나리오는 예약 게이트 도입(2026-08-22) **이전에** 캡처된 운영 스냅샷이라 LUT 를
  // 그대로 담고 있다. 그 스냅샷을 재생하는 것이 이 재생 안전망의 목적이므로 게이트를
  // 켠다 — 여기서 끄면 회귀망이 검사하던 것과 다른 계획을 비교하게 된다.
  value.obstacle_reserve_from_lut = true;
  value.tracking_error_reserve_m = scalar("tracking_error_reserve_m");
  value.wall_safety_margin_m = scalar("wall_safety_margin_m");
  value.fallback_track_half_width_m = scalar("fallback_track_half_width_m");
  value.outside_line_transition_scale = scalar("outside_line_transition_scale");
  value.post_merge_lookahead_m = scalar("post_merge_lookahead_m");
  value.post_merge_min_time_sec = scalar("post_merge_min_time_sec");
  value.minimum_target_offset_m = scalar("minimum_target_offset_m");
  value.maximum_target_offset_m = scalar("maximum_target_offset_m");
  value.target_d_candidate_count =
    static_cast<int>(std::llround(scalar("target_d_candidate_count")));
  value.maximum_lateral_slope = scalar("maximum_lateral_slope");
  value.maximum_curvature_radpm = scalar("maximum_curvature_radpm");
  value.maximum_curvature_rate_radpm2 = scalar("maximum_curvature_rate_radpm2");
  value.safe_stop_buffer_m = scalar("safe_stop_buffer_m");
  value.safe_stop_deceleration_mps2 = scalar("safe_stop_deceleration_mps2");
  value.minimum_path_points = static_cast<int>(std::llround(scalar("minimum_path_points")));
  value.localization_reserve_m = scalar("localization_reserve_m");
  value.avoidance_minimum_speed_mps = scalar("avoidance_minimum_speed_mps");
  value.margin_pass_speed_cap_mps = scalar("margin_pass_speed_cap_mps");
  value.commitment_retention_reserve_fraction =
    scalar("commitment_retention_reserve_fraction");
  value.tracking_error_lut_speed_bins_mps = vector("tracking_error_lut_speed_bins_mps");
  value.tracking_error_lut_curvature_bins_radpm =
    vector("tracking_error_lut_curvature_bins_radpm");
  value.tracking_error_lut_values_m = vector("tracking_error_lut_values_m");
  value.avoidance_velocity_limit_speed_bins_mps =
    vector("avoidance_velocity_limit_speed_bins_mps");
  value.avoidance_velocity_limit_lateral_accel_mps2 =
    vector("avoidance_velocity_limit_lateral_accel_mps2");
  value.pre_apex_distances_m = vector("pre_apex_distances_m");
  value.post_apex_distances_m = vector("post_apex_distances_m");
  value.entry_transition_fractions = vector("entry_transition_fractions");
  value.transition_distance_scales = vector("transition_distance_scales");
  return value;
}

std::filesystem::path scenarioPath(const std::string & name)
{
  return std::filesystem::path(P3_SCENARIO_DIR) / (name + ".stream");
}

// 각 프레임에서 P3가 신규 회피를 만들어내는지.

}  // namespace test_stream
}  // namespace local_planning

#endif  // P3_SCENARIO_STREAM_HPP_
