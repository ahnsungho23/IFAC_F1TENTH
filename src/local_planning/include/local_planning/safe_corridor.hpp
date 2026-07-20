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

#ifndef LOCAL_PLANNING__SAFE_CORRIDOR_HPP_
#define LOCAL_PLANNING__SAFE_CORRIDOR_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <nav_msgs/msg/occupancy_grid.hpp>

namespace local_planning
{

enum class UnknownCellPolicy
{
  kTreatAsFree,
  kTreatAsOccupied,
  kRejectCandidate
};

struct LateralInterval
{
  double min_d{0.0};
  double max_d{0.0};
  bool contains_unknown{false};
};

struct LateralTargetKnot
{
  int waypoint_offset{0};
  double d{0.0};
};

struct CorridorReferenceSample
{
  int waypoint_offset{0};
  int waypoint_index{0};
  double unwrapped_s{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double d_left{0.0};
  double d_right{0.0};
  double longitudinal_half_span{0.0};
};

struct SafeCorridorSample
{
  CorridorReferenceSample reference;
  double track_min_d{0.0};
  double track_max_d{0.0};
  std::vector<LateralInterval> blocked_intervals;
  std::vector<LateralInterval> feasible_intervals;
  bool center_blocked{false};
  bool left_feasible{false};
  bool right_feasible{false};
  bool center_blocked_by_unknown{false};
  double blocking_raw_min_d{0.0};
  double blocking_raw_max_d{0.0};
  std::vector<int> inflated_cell_indices;
};

struct SafeCorridorResult
{
  std::vector<SafeCorridorSample> samples;
  std::vector<int> inflated_cell_indices;
};

struct SafeCorridorConfig
{
  int occupied_threshold{50};
  UnknownCellPolicy unknown_policy{UnknownCellPolicy::kRejectCandidate};
  double minimum_longitudinal_half_width_m{0.15};
  double vehicle_radius_m{0.20};
  double path_clearance_margin_m{0.05};
  double vehicle_front_extent_m{0.3302};
  double vehicle_rear_extent_m{0.05};
  double vehicle_width_m{0.2413};
  double localization_margin_m{0.05};
  double safety_margin_m{0.05};
  bool preserve_circular_collision_check{true};
};

class SafeCorridorBuilder
{
public:
  explicit SafeCorridorBuilder(SafeCorridorConfig config);

  SafeCorridorResult build(
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<CorridorReferenceSample> & references) const;

  static UnknownCellPolicy parseUnknownCellPolicy(const std::string & value);
  static const char * unknownCellPolicyName(UnknownCellPolicy policy);
  static bool intervalContains(const LateralInterval & interval, double d);
  static bool corridorContains(const SafeCorridorSample & sample, double d);
  static std::vector<LateralInterval> intersectFeasibleIntervals(
    const std::vector<SafeCorridorSample> & samples,
    int begin_index, int end_index, bool left_side);
  static std::vector<double> sampleTargetOffsets(
    const std::vector<LateralInterval> & intervals,
    double preferred_offset, int sample_count, double boundary_inset_m);
  static std::vector<std::vector<LateralTargetKnot>> buildCorridorGuidedProfiles(
    const std::vector<SafeCorridorSample> & samples,
    int begin_index, int end_index, bool left_side,
    double preferred_offset, int lateral_sample_count,
    double boundary_inset_m, int knot_stride, int beam_width);
  static std::vector<std::vector<LateralTargetKnot>> buildFullHorizonGuidedProfiles(
    const std::vector<SafeCorridorSample> & samples,
    int window_begin, int collision_begin, int collision_end, int window_end,
    bool left_side, double start_d, double end_d, double preferred_offset,
    int lateral_sample_count, double boundary_inset_m, int knot_stride,
    int beam_width);

private:
  bool worldToMap(
    const nav_msgs::msg::OccupancyGrid & grid,
    double origin_cos, double origin_sin,
    double x, double y, int & column, int & row) const;
  void mapCellCenter(
    const nav_msgs::msg::OccupancyGrid & grid,
    double origin_cos, double origin_sin,
    int column, int row, double & x, double & y) const;
  bool isBlockingCell(int8_t occupancy, bool & is_unknown) const;

  SafeCorridorConfig config_;
};

}  // namespace local_planning

#endif  // LOCAL_PLANNING__SAFE_CORRIDOR_HPP_
