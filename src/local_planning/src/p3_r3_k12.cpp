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

#include "local_planning/p3_r3_k12.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace local_planning
{
namespace
{

constexpr double kEpsilon = 1.0e-9;
constexpr double kPreApexFarM = 11.442220427651225;
constexpr double kPostApexFarM = 6.178529850015357;
constexpr double kLookaheadM = 15.0;
constexpr double kOutsideExitMultiplier = 0.4060036444074003;
constexpr double kMaximumLateralSlope = 0.8;
constexpr double kTargetBoundM = 1.5;

constexpr std::array<double, 16> kComponentFractions{
  0.0, 1.0 / 64.0, 1.0 / 32.0, 1.0 / 16.0, 1.0 / 8.0,
  3.0 / 16.0, 1.0 / 4.0, 3.0 / 8.0, 1.0 / 2.0,
  5.0 / 8.0, 2.0 / 3.0, 3.0 / 4.0, 7.0 / 8.0,
  15.0 / 16.0, 63.0 / 64.0, 1.0};
constexpr std::array<double, 6> kTargetLocalOffsetsM{0.01, 0.02, 0.04, 0.06, 0.08, 0.10};
constexpr std::array<double, 8> kMidAbsoluteOffsetsM{
  0.01, 0.02, 0.04, 0.05, 0.08, 0.10, 0.15, 0.20};
constexpr std::array<double, 7> kCenterInterpolationFractions{
  0.125, 0.25, 0.375, 0.4375, 0.5, 0.75, 1.0};
constexpr std::array<double, 3> kReferenceInwardFactors{0.25, 0.5, 0.75};

double clamp(double value, double lower, double upper)
{
  return std::min(upper, std::max(lower, value));
}

// The frozen selector was materialized with CPython 3.12 built-in sum(), whose float fast
// path uses Neumaier compensated summation. Matching it prevents one-ULP proxy differences
// from changing the final lexicographic order.
class PythonFloatSum
{
public:
  void add(double value)
  {
    const double next = result_ + value;
    if (std::abs(result_) >= std::abs(value)) {
      compensation_ += (result_ - next) + value;
    } else {
      compensation_ += (value - next) + result_;
    }
    result_ = next;
  }

  double value() const
  {
    return compensation_ != 0.0 && std::isfinite(compensation_) ?
           result_ + compensation_ : result_;
  }

private:
  double result_{0.0};
  double compensation_{0.0};
};

// Python float.hex() is part of the frozen final lexical tie. Reproduce its fixed 13-hex-digit
// binary64 form rather than relying on implementation-specific iostream hexfloat formatting.
std::string pythonHex(double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  const bool negative = (bits >> 63U) != 0U;
  const std::uint64_t exponent_bits = (bits >> 52U) & 0x7ffU;
  const std::uint64_t fraction = bits & ((std::uint64_t{1U} << 52U) - 1U);
  std::ostringstream output;
  if (negative) {
    output << '-';
  }
  if (exponent_bits == 0U && fraction == 0U) {
    output << "0x0.0p+0";
    return output.str();
  }
  output << "0x" << (exponent_bits == 0U ? '0' : '1') << '.' << std::hex <<
    std::setfill('0') << std::setw(13) << fraction << std::dec << 'p';
  const int exponent = exponent_bits == 0U ? -1022 : static_cast<int>(exponent_bits) - 1023;
  output << (exponent >= 0 ? "+" : "") << exponent;
  return output.str();
}

std::string sideName(bool go_left)
{
  return go_left ? "LEFT" : "RIGHT";
}

std::string join(const std::vector<std::string> & values)
{
  std::ostringstream output;
  for (std::size_t index = 0U; index < values.size(); ++index) {
    if (index != 0U) {
      output << '|';
    }
    output << values[index];
  }
  return output.str();
}

std::string configurationKey(
  bool go_left, double target, double middle, double entry, double exit)
{
  return join({
        sideName(go_left), pythonHex(target), pythonHex(middle), pythonHex(entry),
        pythonHex(exit)});
}

std::string shapeKey(
  bool go_left, double target, double middle, const std::array<double, 5> & stations)
{
  std::vector<std::string> values{sideName(go_left), pythonHex(target), pythonHex(middle)};
  for (const double station : stations) {
    values.push_back(pythonHex(station));
  }
  return join(values);
}

std::vector<double> uniqueExact(std::vector<double> values)
{
  values.erase(
    std::remove_if(values.begin(), values.end(), [](double value) {return !std::isfinite(value);}),
    values.end());
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end()), values.end());
  return values;
}

double normalized(double value, const std::vector<double> & values)
{
  if (values.size() < 2U || values.back() - values.front() <= kEpsilon) {
    return 0.0;
  }
  return (value - values.front()) / (values.back() - values.front());
}

struct QuinticSegment
{
  double start{0.0};
  double end{0.0};
  std::array<double, 6> coefficients{};
};

double harmonic(double h_left, double h_right, double left, double right)
{
  return left * right > 0.0 ?
         (h_left + h_right) / (h_left / left + h_right / right) : 0.0;
}

std::vector<QuinticSegment> makeProfile(
  const std::array<double, 5> & stations, const std::array<double, 5> & offsets)
{
  std::array<double, 5> derivative{};
  std::array<double, 5> acceleration{};
  for (std::size_t index = 1U; index + 1U < stations.size(); ++index) {
    const double h_previous = stations[index] - stations[index - 1U];
    const double h_next = stations[index + 1U] - stations[index];
    const double slope_previous = (offsets[index] - offsets[index - 1U]) / h_previous;
    const double slope_next = (offsets[index + 1U] - offsets[index]) / h_next;
    derivative[index] = harmonic(h_previous, h_next, slope_previous, slope_next);
    acceleration[index] = 2.0 * (slope_next - slope_previous) / (h_previous + h_next);
  }
  std::vector<QuinticSegment> segments;
  for (std::size_t index = 0U; index + 1U < stations.size(); ++index) {
    const double h = stations[index + 1U] - stations[index];
    QuinticSegment segment;
    segment.start = stations[index];
    segment.end = stations[index + 1U];
    auto & c = segment.coefficients;
    c[0] = offsets[index];
    c[1] = h * derivative[index];
    c[2] = 0.5 * h * h * acceleration[index];
    const double r0 = offsets[index + 1U] - c[0] - c[1] - c[2];
    const double r1 = h * derivative[index + 1U] - c[1] - 2.0 * c[2];
    const double r2 = h * h * acceleration[index + 1U] - 2.0 * c[2];
    c[3] = 10.0 * r0 - 4.0 * r1 + 0.5 * r2;
    c[4] = -15.0 * r0 + 7.0 * r1 - r2;
    c[5] = 6.0 * r0 - 3.0 * r1 + 0.5 * r2;
    segments.push_back(segment);
  }
  return segments;
}

double segmentValue(const QuinticSegment & segment, double station)
{
  const double t = clamp(
    (station - segment.start) / (segment.end - segment.start), 0.0, 1.0);
  PythonFloatSum value;
  for (std::size_t index = 0U; index < segment.coefficients.size(); ++index) {
    value.add(segment.coefficients[index] * std::pow(t, static_cast<double>(index)));
  }
  return value.value();
}

double profileValue(
  const std::vector<QuinticSegment> & profile, double ego_d, double station)
{
  if (station <= profile.front().start) {
    return ego_d;
  }
  for (const auto & segment : profile) {
    if (station <= segment.end) {
      return segmentValue(segment, station);
    }
  }
  return 0.0;
}

std::array<double, 5> stationsFor(
  const P3R3K12SideGeometry & geometry, double entry, double exit, double target)
{
  const double multiplier = geometry.go_left == geometry.outside_is_left ?
    kOutsideExitMultiplier : 1.0;
  const double exit_length = kPostApexFarM * exit * multiplier;
  const double required = std::abs(target - geometry.ego_d) / kMaximumLateralSlope;
  double apex = geometry.cluster_start;
  double start = apex - apex * kPreApexFarM * entry / kLookaheadM;
  if (!(apex >= required)) {
    apex = std::max(required, geometry.reference_spacing_m);
    start = 0.0;
  }
  return {
    start, apex, 0.5 * (apex + geometry.cluster_end), geometry.cluster_end,
    geometry.cluster_end + exit_length};
}

using Transition = std::pair<double, double>;

struct TargetCandidate
{
  double value{0.0};
  std::string source;
  int priority{0};
};

using ExactPairKey = std::pair<std::string, std::string>;

std::vector<P3R3K12Factor> lateralFactors(
  const P3R3K12SideGeometry & geometry,
  const std::vector<P3R3K12ProductionCandidate> & production)
{
  const auto same_side = [&geometry](const auto & row) {return row.go_left == geometry.go_left;};
  std::vector<double> production_targets;
  std::set<std::pair<double, double>> production_pairs;
  for (const auto & row : production) {
    if (!same_side(row)) {
      continue;
    }
    production_targets.push_back(row.d_target);
    production_pairs.emplace(row.d_target, row.d_mid);
  }
  production_targets = uniqueExact(std::move(production_targets));

  std::map<std::string, TargetCandidate> targets;
  const auto add_target = [&](double value, const std::string & source, int priority) {
      value = clamp(value, geometry.domain_lower, geometry.domain_upper);
      const std::string key = pythonHex(value);
      const auto found = targets.find(key);
      if (found == targets.end() ||
        std::tie(priority, source) < std::tie(found->second.priority, found->second.source))
      {
        targets[key] = {value, source, priority};
      }
    };
  for (const double value : production_targets) {
    add_target(value, "PRODUCTION_TARGET", 0);
    for (const double offset : kTargetLocalOffsetsM) {
      std::ostringstream minus;
      minus << "PRODUCTION_TARGET_MINUS_" << std::fixed << std::setprecision(2) << offset << 'M';
      std::ostringstream plus;
      plus << "PRODUCTION_TARGET_PLUS_" << std::fixed << std::setprecision(2) << offset << 'M';
      add_target(value - offset, minus.str(), 2);
      add_target(value + offset, plus.str(), 2);
    }
  }
  for (std::size_t component_index = 0U;
    component_index < geometry.components.size(); ++component_index)
  {
    double near = geometry.components[component_index].lower;
    double far = geometry.components[component_index].upper;
    if (std::make_tuple(std::abs(far), far) < std::make_tuple(std::abs(near), near)) {
      std::swap(near, far);
    }
    for (const double fraction : kComponentFractions) {
      std::ostringstream source;
      source << "COMPONENT_C" << component_index << "_NEAR_TO_FAR_F" <<
        std::fixed << std::setprecision(8) << fraction;
      add_target(near + fraction * (far - near), source.str(), 1);
    }
  }
  for (std::size_t index = 0U; index < geometry.center_values.size(); ++index) {
    add_target(geometry.center_values[index], "CORRIDOR_CENTER_SAMPLE_" + std::to_string(index), 1);
  }

  std::vector<TargetCandidate> ordered_targets;
  for (const auto & entry : targets) {
    ordered_targets.push_back(entry.second);
  }
  std::sort(
    ordered_targets.begin(), ordered_targets.end(), [](const auto & first, const auto & second) {
      return std::tie(first.value, first.source, first.priority) <
             std::tie(second.value, second.source, second.priority);
    });

  std::map<ExactPairKey, P3R3K12Factor> factors;
  const auto add_factor = [&](double target, double middle, const std::string & target_source,
    const std::string & mid_source, int source_priority) {
      middle = clamp(middle, -kTargetBoundM, kTargetBoundM);
      const ExactPairKey key{pythonHex(target), pythonHex(middle)};
      P3R3K12Factor row;
      row.go_left = geometry.go_left;
      row.d_target = target;
      row.d_mid = middle;
      row.target_source = target_source;
      row.mid_source = mid_source;
      row.source_priority = source_priority;
      const auto found = factors.find(key);
      if (found == factors.end() ||
        std::tie(source_priority, target_source, mid_source) <
        std::tie(found->second.source_priority, found->second.target_source,
        found->second.mid_source))
      {
        factors[key] = std::move(row);
      }
    };

  for (const auto & target : ordered_targets) {
    if (production_pairs.find({target.value, target.value}) != production_pairs.end()) {
      add_factor(
        target.value, target.value, target.source, "PRODUCTION_OR_EQUAL_TARGET", 0);
    }
    add_factor(target.value, target.value, target.source, "MID_EQUALS_TARGET", target.priority);
    for (std::size_t center_index = 0U;
      center_index < geometry.center_values.size(); ++center_index)
    {
      for (const double fraction : kCenterInterpolationFractions) {
        std::ostringstream source;
        source << "TO_CORRIDOR_CENTER_" << center_index << "_F" <<
          std::fixed << std::setprecision(3) << fraction;
        add_factor(
          target.value,
          target.value + fraction * (geometry.center_values[center_index] - target.value),
          target.source, source.str(), target.priority + 1);
      }
    }
    for (const double factor : kReferenceInwardFactors) {
      std::ostringstream source;
      source << "REFERENCE_INWARD_F" << std::fixed << std::setprecision(2) << factor;
      add_factor(
        target.value, factor * target.value, target.source, source.str(), target.priority + 2);
    }
    for (const double offset : kMidAbsoluteOffsetsM) {
      std::ostringstream minus;
      minus << "TARGET_MINUS_" << std::fixed << std::setprecision(2) << offset << 'M';
      std::ostringstream plus;
      plus << "TARGET_PLUS_" << std::fixed << std::setprecision(2) << offset << 'M';
      add_factor(target.value, target.value - offset, target.source, minus.str(),
            target.priority + 1);
      add_factor(target.value, target.value + offset, target.source, plus.str(),
            target.priority + 1);
    }
  }
  for (const auto & pair : production_pairs) {
    add_factor(pair.first, pair.second, "PRODUCTION_TARGET", "PRODUCTION_MID", 0);
  }

  std::vector<P3R3K12Factor> output;
  for (auto & entry : factors) {
    output.push_back(std::move(entry.second));
  }
  std::sort(output.begin(), output.end(), [](const auto & first, const auto & second) {
      return std::make_tuple(
        first.source_priority, std::abs(first.d_mid - first.d_target),
        std::abs(first.d_target), first.target_source, first.mid_source,
        pythonHex(first.d_target), pythonHex(first.d_mid)) <
             std::make_tuple(
        second.source_priority, std::abs(second.d_mid - second.d_target),
        std::abs(second.d_target), second.target_source, second.mid_source,
        pythonHex(second.d_target), pythonHex(second.d_mid));
    });
  for (std::size_t index = 0U; index < output.size(); ++index) {
    output[index].lateral_factor_index = index;
  }
  return output;
}

void populateMetrics(
  P3R3K12Factor & candidate, const P3R3K12SideGeometry & geometry,
  const std::vector<double> & entries, const std::vector<double> & exits)
{
  candidate.stations = stationsFor(
    geometry, candidate.entry_scale, candidate.exit_scale, candidate.d_target);
  for (std::size_t index = 0U; index + 1U < candidate.stations.size(); ++index) {
    if (!(candidate.stations[index + 1U] - candidate.stations[index] > kEpsilon)) {
      candidate.construction_guard_proxy = true;
      candidate.exit_conflict_proxy = true;
      candidate.maximum_corridor_violation_m = std::numeric_limits<double>::infinity();
      candidate.sum_corridor_violation_m = std::numeric_limits<double>::infinity();
      candidate.slope_excess = std::numeric_limits<double>::infinity();
      candidate.curvature_proxy = std::numeric_limits<double>::infinity();
      candidate.center_error = std::numeric_limits<double>::infinity();
      candidate.minimum_clearance_m = -std::numeric_limits<double>::infinity();
      candidate.shape_energy = std::numeric_limits<double>::infinity();
      return;
    }
  }
  const auto profile = makeProfile(
    candidate.stations,
    {geometry.ego_d, candidate.d_target, candidate.d_mid, candidate.d_target, 0.0});
  std::vector<double> violations;
  std::vector<double> clearances;
  std::vector<double> center_errors;
  std::vector<double> later_violations;
  std::vector<std::pair<double, double>> discrete;
  const double sample_limit = candidate.stations[4] + std::max(5.0, std::abs(geometry.ego_speed));
  for (const auto & sample : geometry.reference_samples) {
    if (sample.station > sample_limit + kEpsilon) {
      break;
    }
    if (sample.station < 0.0) {
      continue;
    }
    const double value = profileValue(profile, geometry.ego_d, sample.station);
    double violation = kTargetBoundM;
    double clearance = -kTargetBoundM;
    if (sample.finite) {
      violation = std::max({0.0, sample.lower - value, value - sample.upper});
      clearance = std::min(value - sample.lower, sample.upper - value);
      if (sample.station >= geometry.cluster_start - kEpsilon &&
        sample.station <= geometry.cluster_end + kEpsilon)
      {
        center_errors.push_back(
          std::abs(value - sample.center) / std::max(0.05, sample.width));
      }
    }
    violations.push_back(violation);
    clearances.push_back(clearance);
    if (sample.station > geometry.cluster_end + kEpsilon && sample.later_obstacle_active) {
      later_violations.push_back(violation);
    }
    discrete.emplace_back(sample.station, value);
  }

  std::vector<double> slopes;
  for (std::size_t index = 1U; index < discrete.size(); ++index) {
    const double ds = discrete[index].first - discrete[index - 1U].first;
    if (ds > kEpsilon) {
      slopes.push_back(std::abs(discrete[index].second - discrete[index - 1U].second) / ds);
    }
  }
  std::vector<double> second;
  for (std::size_t index = 2U; index < discrete.size(); ++index) {
    const double left_ds = discrete[index - 1U].first - discrete[index - 2U].first;
    const double right_ds = discrete[index].first - discrete[index - 1U].first;
    if (left_ds > kEpsilon && right_ds > kEpsilon) {
      const double left = (discrete[index - 1U].second - discrete[index - 2U].second) / left_ds;
      const double right = (discrete[index].second - discrete[index - 1U].second) / right_ds;
      second.push_back(
        std::abs(right - left) /
        std::max(kEpsilon, 0.5 * (discrete[index].first - discrete[index - 2U].first)));
    }
  }
  const double peak_slope = slopes.empty() ? std::numeric_limits<double>::infinity() :
    *std::max_element(slopes.begin(), slopes.end());
  PythonFloatSum shape_energy;
  for (const double value : slopes) {
    shape_energy.add(value * value);
  }
  const double mean_shape_energy =
    shape_energy.value() / static_cast<double>(std::max<std::size_t>(1U, slopes.size()));
  candidate.exit_conflict_proxy =
    !later_violations.empty() && *std::max_element(
    later_violations.begin(), later_violations.end()) > kEpsilon;
  candidate.maximum_corridor_violation_m = violations.empty() ?
    std::numeric_limits<double>::infinity() :
    *std::max_element(violations.begin(), violations.end());
  PythonFloatSum violation_sum;
  for (const double value : violations) {
    violation_sum.add(value);
  }
  candidate.sum_corridor_violation_m = violation_sum.value();
  candidate.slope_excess = std::max(0.0, peak_slope - kMaximumLateralSlope);
  candidate.curvature_proxy = second.empty() ? std::numeric_limits<double>::infinity() :
    *std::max_element(second.begin(), second.end());
  PythonFloatSum center_error_sum;
  for (const double value : center_errors) {
    center_error_sum.add(value);
  }
  candidate.center_error = center_errors.empty() ? std::numeric_limits<double>::infinity() :
    center_error_sum.value() / static_cast<double>(center_errors.size());
  candidate.minimum_clearance_m = clearances.empty() ?
    -std::numeric_limits<double>::infinity() :
    *std::min_element(clearances.begin(), clearances.end());
  candidate.shape_energy = mean_shape_energy;
  candidate.entry_normalized = normalized(candidate.entry_scale, entries);
  candidate.exit_normalized = normalized(candidate.exit_scale, exits);
}

double baseScore(const P3R3K12Factor & candidate)
{
  if (!std::isfinite(candidate.maximum_corridor_violation_m)) {
    return 1.0e9;
  }
  const double clearance_penalty = std::max(0.0, 0.02 - candidate.minimum_clearance_m);
  return 40.0 * candidate.maximum_corridor_violation_m +
         2.0 * candidate.sum_corridor_violation_m +
         8.0 * candidate.slope_excess +
         0.12 * candidate.curvature_proxy +
         0.25 * candidate.center_error +
         2.0 * clearance_penalty +
         0.02 * candidate.shape_energy +
         0.002 * static_cast<double>(candidate.source_priority);
}

auto stableTie(const P3R3K12Factor & candidate)
{
  return std::make_tuple(
    candidate.source_priority, candidate.lateral_factor_index, candidate.transition_index,
    candidate.go_left, candidate.configuration_key);
}

std::vector<P3R3K12Factor> deduplicateShape(std::vector<P3R3K12Factor> rows)
{
  std::vector<P3R3K12Factor> output;
  std::set<std::string> seen;
  for (auto & row : rows) {
    if (seen.insert(row.preconstruction_shape_key).second) {
      output.push_back(std::move(row));
    }
  }
  return output;
}

std::vector<P3R3K12Factor> lexicographicOrder(const std::vector<P3R3K12Factor> & pool)
{
  std::vector<P3R3K12Factor> feasible;
  std::vector<P3R3K12Factor> guarded;
  for (const auto & row : pool) {
    (row.construction_guard_proxy ? guarded : feasible).push_back(row);
  }
  std::sort(feasible.begin(), feasible.end(), [](const auto & first, const auto & second) {
      return std::make_tuple(
        first.exit_conflict_proxy, first.maximum_corridor_violation_m > kEpsilon,
        first.maximum_corridor_violation_m, first.slope_excess > kEpsilon,
        first.slope_excess, first.sum_corridor_violation_m, first.curvature_proxy,
        first.center_error, -first.minimum_clearance_m, first.shape_energy, stableTie(first)) <
             std::make_tuple(
        second.exit_conflict_proxy, second.maximum_corridor_violation_m > kEpsilon,
        second.maximum_corridor_violation_m, second.slope_excess > kEpsilon,
        second.slope_excess, second.sum_corridor_violation_m, second.curvature_proxy,
        second.center_error, -second.minimum_clearance_m, second.shape_energy, stableTie(second));
    });
  std::sort(guarded.begin(), guarded.end(), [](const auto & first, const auto & second) {
      return stableTie(first) < stableTie(second);
    });
  feasible.insert(feasible.end(), guarded.begin(), guarded.end());
  return deduplicateShape(std::move(feasible));
}

std::vector<P3R3K12Factor> coverageOrder(const std::vector<P3R3K12Factor> & pool)
{
  std::vector<P3R3K12Factor> remaining;
  std::vector<P3R3K12Factor> guarded;
  for (const auto & row : pool) {
    (row.construction_guard_proxy ? guarded : remaining).push_back(row);
  }
  std::sort(remaining.begin(), remaining.end(), [](const auto & first, const auto & second) {
      return std::make_tuple(first.exit_conflict_proxy, baseScore(first), stableTie(first)) <
             std::make_tuple(second.exit_conflict_proxy, baseScore(second), stableTie(second));
    });
  std::vector<P3R3K12Factor> selected;
  while (!remaining.empty()) {
    std::size_t best = 0U;
    if (!selected.empty()) {
      const auto priority = [&selected](const P3R3K12Factor & row) {
          double distance = std::numeric_limits<double>::infinity();
          for (const auto & chosen : selected) {
            distance = std::min(
              distance,
              (row.go_left != chosen.go_left ? 0.5 : 0.0) +
              std::abs(row.d_target - chosen.d_target) / 1.5 +
              std::abs(row.d_mid - chosen.d_mid) / 3.0 +
              0.20 * std::abs(row.entry_normalized - chosen.entry_normalized) +
              0.20 * std::abs(row.exit_normalized - chosen.exit_normalized));
          }
          return std::make_tuple(
            row.exit_conflict_proxy, baseScore(row) - 2.0 * std::min(1.0, distance),
            baseScore(row), stableTie(row));
        };
      for (std::size_t index = 1U; index < remaining.size(); ++index) {
        if (priority(remaining[index]) < priority(remaining[best])) {
          best = index;
        }
      }
    }
    selected.push_back(std::move(remaining[best]));
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
    if (selected.size() >= 48U) {
      break;
    }
  }
  selected.insert(selected.end(), remaining.begin(), remaining.end());
  std::sort(guarded.begin(), guarded.end(), [](const auto & first, const auto & second) {
      return stableTie(first) < stableTie(second);
    });
  selected.insert(selected.end(), guarded.begin(), guarded.end());
  return deduplicateShape(std::move(selected));
}

}  // namespace

P3R3K12Selection selectP3R3K12Factors(
  const std::vector<P3R3K12SideGeometry> & sides,
  const std::vector<P3R3K12ProductionCandidate> & production_candidates)
{
  P3R3K12Selection result;
  std::set<Transition> transitions;
  std::set<std::string> production_keys;
  std::vector<double> entries;
  std::vector<double> exits;
  for (const auto & candidate : production_candidates) {
    transitions.emplace(candidate.entry_scale, candidate.exit_scale);
    entries.push_back(candidate.entry_scale);
    exits.push_back(candidate.exit_scale);
    production_keys.insert(configurationKey(
        candidate.go_left, candidate.d_target, candidate.d_mid,
        candidate.entry_scale, candidate.exit_scale));
  }
  entries = uniqueExact(std::move(entries));
  exits = uniqueExact(std::move(exits));

  std::vector<P3R3K12SideGeometry> ordered_sides = sides;
  std::sort(ordered_sides.begin(), ordered_sides.end(),
    [](const auto & first, const auto & second) {
      return first.go_left < second.go_left;  // RIGHT before LEFT
    });
  std::vector<P3R3K12Factor> pool;
  for (const auto & geometry : ordered_sides) {
    auto lateral = lateralFactors(geometry, production_candidates);
    result.lateral_factor_count += lateral.size();
    for (const auto & factor : lateral) {
      std::size_t transition_index = 0U;
      for (const auto & transition : transitions) {
        P3R3K12Factor candidate = factor;
        candidate.entry_scale = transition.first;
        candidate.exit_scale = transition.second;
        candidate.transition_index = transition_index++;
        candidate.configuration_key = configurationKey(
          candidate.go_left, candidate.d_target, candidate.d_mid,
          candidate.entry_scale, candidate.exit_scale);
        if (production_keys.find(candidate.configuration_key) != production_keys.end()) {
          continue;
        }
        populateMetrics(candidate, geometry, entries, exits);
        candidate.preconstruction_shape_key = shapeKey(
          candidate.go_left, candidate.d_target, candidate.d_mid, candidate.stations);
        pool.push_back(std::move(candidate));
      }
    }
  }
  result.pair_priority_count = pool.size();
  result.lexicographic = lexicographicOrder(pool);
  result.coverage = coverageOrder(pool);
  return result;
}

}  // namespace local_planning
