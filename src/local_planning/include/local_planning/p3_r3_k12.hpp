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

#ifndef LOCAL_PLANNING__P3_R3_K12_HPP_
#define LOCAL_PLANNING__P3_R3_K12_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace local_planning
{

inline constexpr const char * kP3R3K12MethodName =
  "R3_LEXICOGRAPHIC_COVERAGE_RESERVE_K12";
inline constexpr const char * kP3R3K12MethodSha256 =
  "7b861e8c7e23ae168413885ea0dc2d09769046ff48a562700b658e084d116fcc";
inline constexpr std::size_t kP3R3K12CandidateBudget = 12U;
inline constexpr std::size_t kP3R3K12LexicographicQuota = 10U;
inline constexpr std::size_t kP3R3K12CoverageQuota = 2U;

struct P3R3K12Interval
{
  double lower{0.0};
  double upper{0.0};
};

// A reference-waypoint sample of the frozen cheap corridor proxy. The exact validator remains
// the only acceptance authority; these fields are used only to order factors before construction.
struct P3R3K12CorridorSample
{
  double station{0.0};
  double lower{0.0};
  double upper{0.0};
  double center{0.0};
  double width{0.0};
  bool finite{false};
  bool later_obstacle_active{false};
};

struct P3R3K12SideGeometry
{
  bool go_left{false};
  bool outside_is_left{false};
  double ego_d{0.0};
  double ego_speed{0.0};
  double cluster_start{0.0};
  double cluster_end{0.0};
  double domain_lower{0.0};
  double domain_upper{0.0};
  double reference_spacing_m{0.25};
  std::vector<P3R3K12Interval> components;
  std::vector<double> center_values;
  std::vector<P3R3K12CorridorSample> reference_samples;
};

struct P3R3K12ProductionCandidate
{
  bool go_left{false};
  double d_target{0.0};
  double d_mid{0.0};
  double entry_scale{0.0};
  double exit_scale{0.0};
};

struct P3R3K12Factor
{
  bool go_left{false};
  double d_target{0.0};
  double d_mid{0.0};
  double entry_scale{0.0};
  double exit_scale{0.0};
  std::array<double, 5> stations{};
  std::string target_source;
  std::string mid_source;
  int source_priority{0};
  std::size_t source_catalog_index{0U};
  std::size_t lateral_factor_index{0U};
  std::size_t transition_index{0U};
  // Cached frozen Python float.hex() values avoid rebuilding equivalent strings in hot sorts.
  std::string d_target_hex;
  std::string d_mid_hex;
  std::string configuration_key;
  std::array<std::uint64_t, 8> preconstruction_shape_bits{};
  std::string preconstruction_shape_key;

  bool construction_guard_proxy{false};
  bool exit_conflict_proxy{false};
  double maximum_corridor_violation_m{0.0};
  double sum_corridor_violation_m{0.0};
  double slope_excess{0.0};
  double curvature_proxy{0.0};
  double center_error{0.0};
  double minimum_clearance_m{0.0};
  double shape_energy{0.0};
  double entry_normalized{0.0};
  double exit_normalized{0.0};
};

struct P3R3K12Selection
{
  std::size_t lateral_factor_count{0U};
  std::size_t pair_priority_count{0U};
  std::vector<P3R3K12Factor> lexicographic;
  std::vector<P3R3K12Factor> coverage;
};

// Passive timing/count profile for the frozen selector. The selector never reads these values,
// so a null profile pointer and a populated profile have identical factor semantics.
struct P3R3K12SelectionProfile
{
  std::size_t transition_count{0U};
  std::size_t raw_combination_count{0U};
  std::size_t production_excluded_count{0U};
  std::size_t factor_pool_count{0U};
  std::size_t unique_profile_count{0U};
  std::size_t proxy_metric_evaluation_count{0U};
  std::size_t proxy_metric_cache_hit_count{0U};
  std::size_t profile_basis_build_count{0U};
  std::size_t profile_basis_cache_hit_count{0U};
  std::size_t profile_sample_basis_count{0U};
  std::size_t pair_metric_worker_count{0U};
  double transition_generation_us{0.0};
  double lateral_factor_generation_us{0.0};
  double pair_priority_computation_us{0.0};
  double lexicographic_ordering_us{0.0};
  double coverage_ordering_us{0.0};
  double shape_deduplication_us{0.0};
};

// Exact C++ port of the frozen online-generatable factor pool and the two preconstruction rank
// streams. No path, exact-validator result, or oracle label is an input to this function.
P3R3K12Selection selectP3R3K12Factors(
  const std::vector<P3R3K12SideGeometry> & sides,
  const std::vector<P3R3K12ProductionCandidate> & production_candidates,
  P3R3K12SelectionProfile * profile = nullptr);

}  // namespace local_planning

#endif  // LOCAL_PLANNING__P3_R3_K12_HPP_
