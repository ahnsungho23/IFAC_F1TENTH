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
#include <charconv>
#include <chrono>
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
#include <thread>
#include <tuple>
#include <unordered_map>
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
constexpr std::size_t kPairMetricWorkerLimit = 4U;
constexpr std::size_t kMinimumFactorsPerPairMetricWorker = 256U;

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
constexpr std::array<const char *, 6> kTargetLocalMinusSources{
  "PRODUCTION_TARGET_MINUS_0.01M", "PRODUCTION_TARGET_MINUS_0.02M",
  "PRODUCTION_TARGET_MINUS_0.04M", "PRODUCTION_TARGET_MINUS_0.06M",
  "PRODUCTION_TARGET_MINUS_0.08M", "PRODUCTION_TARGET_MINUS_0.10M"};
constexpr std::array<const char *, 6> kTargetLocalPlusSources{
  "PRODUCTION_TARGET_PLUS_0.01M", "PRODUCTION_TARGET_PLUS_0.02M",
  "PRODUCTION_TARGET_PLUS_0.04M", "PRODUCTION_TARGET_PLUS_0.06M",
  "PRODUCTION_TARGET_PLUS_0.08M", "PRODUCTION_TARGET_PLUS_0.10M"};
constexpr std::array<const char *, 8> kMidMinusSources{
  "TARGET_MINUS_0.01M", "TARGET_MINUS_0.02M", "TARGET_MINUS_0.04M",
  "TARGET_MINUS_0.05M", "TARGET_MINUS_0.08M", "TARGET_MINUS_0.10M",
  "TARGET_MINUS_0.15M", "TARGET_MINUS_0.20M"};
constexpr std::array<const char *, 8> kMidPlusSources{
  "TARGET_PLUS_0.01M", "TARGET_PLUS_0.02M", "TARGET_PLUS_0.04M",
  "TARGET_PLUS_0.05M", "TARGET_PLUS_0.08M", "TARGET_PLUS_0.10M",
  "TARGET_PLUS_0.15M", "TARGET_PLUS_0.20M"};
constexpr std::array<const char *, 7> kCenterFractionSources{
  "F0.125", "F0.250", "F0.375", "F0.438", "F0.500", "F0.750", "F1.000"};
constexpr std::array<const char *, 3> kReferenceInwardSources{
  "REFERENCE_INWARD_F0.25", "REFERENCE_INWARD_F0.50", "REFERENCE_INWARD_F0.75"};

using ProfileClock = std::chrono::steady_clock;

double profileElapsedUs(const ProfileClock::time_point & start)
{
  return std::chrono::duration<double, std::micro>(ProfileClock::now() - start).count();
}

double clamp(double value, double lower, double upper)
{
  return std::min(upper, std::max(lower, value));
}

std::uint64_t exactBits(double value)
{
  std::uint64_t bits = 0U;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(bits));
  return bits;
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
  std::array<char, 64> buffer{};
  char * output = buffer.data();
  if (negative) {
    *output++ = '-';
  }
  if (exponent_bits == 0U && fraction == 0U) {
    constexpr char zero[] = "0x0.0p+0";
    output = std::copy(std::begin(zero), std::end(zero) - 1, output);
    return std::string(buffer.data(), output);
  }
  *output++ = '0';
  *output++ = 'x';
  *output++ = exponent_bits == 0U ? '0' : '1';
  *output++ = '.';
  constexpr char digits[] = "0123456789abcdef";
  for (int shift = 48; shift >= 0; shift -= 4) {
    *output++ = digits[(fraction >> static_cast<unsigned int>(shift)) & 0xfU];
  }
  *output++ = 'p';
  const int exponent = exponent_bits == 0U ? -1022 : static_cast<int>(exponent_bits) - 1023;
  if (exponent >= 0) {
    *output++ = '+';
  }
  const auto converted = std::to_chars(output, buffer.data() + buffer.size(), exponent);
  return std::string(buffer.data(), converted.ptr);
}

std::string sideName(bool go_left)
{
  return go_left ? "LEFT" : "RIGHT";
}

std::array<std::uint64_t, 5> configurationKeyBits(
  bool go_left, double target, double middle, double entry, double exit)
{
  return {
    go_left ? 1U : 0U, exactBits(target), exactBits(middle), exactBits(entry), exactBits(exit)};
}

std::string configurationKey(
  bool go_left, const std::string & target_hex, const std::string & middle_hex,
  double entry, double exit)
{
  const std::string entry_hex = pythonHex(entry);
  const std::string exit_hex = pythonHex(exit);
  std::string result;
  result.reserve(
    4U + target_hex.size() + middle_hex.size() + entry_hex.size() + exit_hex.size());
  result.append(sideName(go_left)).push_back('|');
  result.append(target_hex).push_back('|');
  result.append(middle_hex).push_back('|');
  result.append(entry_hex).push_back('|');
  result.append(exit_hex);
  return result;
}

std::string shapeKey(
  bool go_left, const std::string & target_hex, const std::string & middle_hex,
  const std::array<double, 5> & stations)
{
  std::string result;
  result.reserve(160U);
  result.append(sideName(go_left)).push_back('|');
  result.append(target_hex).push_back('|');
  result.append(middle_hex);
  for (const double station : stations) {
    result.push_back('|');
    result.append(pythonHex(station));
  }
  return result;
}

std::array<std::uint64_t, 8> shapeKeyBits(
  bool go_left, double target, double middle, const std::array<double, 5> & stations)
{
  std::array<std::uint64_t, 8> result{};
  result[0] = go_left ? 1U : 0U;
  result[1] = exactBits(target);
  result[2] = exactBits(middle);
  for (std::size_t index = 0U; index < stations.size(); ++index) {
    result[index + 3U] = exactBits(stations[index]);
  }
  return result;
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

std::array<QuinticSegment, 4> makeProfile(
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
  std::array<QuinticSegment, 4> segments;
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
    segments[index] = segment;
  }
  return segments;
}

struct ProfileSampleBasis
{
  int segment_index{-1};  // -1: ego value, -2: post-profile zero.
  std::array<double, 6> powers{};
  double station{0.0};
  double lower{0.0};
  double upper{0.0};
  double center{0.0};
  double center_width_denominator{0.05};
  double slope_ds{0.0};
  double left_ds{0.0};
  double right_ds{0.0};
  double second_difference_denominator{0.0};
  bool finite{false};
  bool center_active{false};
  bool later_obstacle_active{false};
};

struct ProfileEvaluationBasis
{
  std::vector<ProfileSampleBasis> samples;
};

ProfileEvaluationBasis makeProfileEvaluationBasis(
  const std::array<double, 5> & stations, const P3R3K12SideGeometry & geometry)
{
  ProfileEvaluationBasis result;
  result.samples.reserve(geometry.reference_samples.size());
  const double sample_limit = stations[4] + std::max(5.0, std::abs(geometry.ego_speed));
  for (std::size_t index = 0U; index < geometry.reference_samples.size(); ++index) {
    const auto & sample = geometry.reference_samples[index];
    if (sample.station > sample_limit + kEpsilon) {
      break;
    }
    if (sample.station < 0.0) {
      continue;
    }
    ProfileSampleBasis basis;
    basis.station = sample.station;
    basis.lower = sample.lower;
    basis.upper = sample.upper;
    basis.center = sample.center;
    basis.center_width_denominator = std::max(0.05, sample.width);
    basis.finite = sample.finite;
    basis.center_active = sample.finite &&
      sample.station >= geometry.cluster_start - kEpsilon &&
      sample.station <= geometry.cluster_end + kEpsilon;
    basis.later_obstacle_active =
      sample.station > geometry.cluster_end + kEpsilon && sample.later_obstacle_active;
    if (!result.samples.empty()) {
      basis.slope_ds = basis.station - result.samples.back().station;
    }
    if (result.samples.size() >= 2U) {
      basis.left_ds = result.samples.back().station -
        result.samples[result.samples.size() - 2U].station;
      basis.right_ds = basis.station - result.samples.back().station;
      basis.second_difference_denominator =
        0.5 * (basis.station - result.samples[result.samples.size() - 2U].station);
    }
    if (sample.station <= stations.front()) {
      basis.segment_index = -1;
    } else {
      basis.segment_index = -2;
      for (std::size_t segment = 0U; segment + 1U < stations.size(); ++segment) {
        if (sample.station <= stations[segment + 1U]) {
          basis.segment_index = static_cast<int>(segment);
          const double t = clamp(
            (sample.station - stations[segment]) /
            (stations[segment + 1U] - stations[segment]), 0.0, 1.0);
          for (std::size_t power = 0U; power < basis.powers.size(); ++power) {
            basis.powers[power] = std::pow(t, static_cast<double>(power));
          }
          break;
        }
      }
    }
    result.samples.push_back(basis);
  }
  return result;
}

double profileValue(
  const std::array<QuinticSegment, 4> & profile, const ProfileSampleBasis & basis,
  double ego_d)
{
  if (basis.segment_index == -1) {
    return ego_d;
  }
  if (basis.segment_index == -2) {
    return 0.0;
  }
  const auto & segment = profile[static_cast<std::size_t>(basis.segment_index)];
  PythonFloatSum value;
  for (std::size_t index = 0U; index < segment.coefficients.size(); ++index) {
    value.add(segment.coefficients[index] * basis.powers[index]);
  }
  return value.value();
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

using ExactPairKey = std::pair<std::uint64_t, std::uint64_t>;

struct ExactPairHash
{
  std::size_t operator()(const ExactPairKey & key) const
  {
    const std::size_t first = std::hash<std::uint64_t>{}(key.first);
    const std::size_t second = std::hash<std::uint64_t>{}(key.second);
    return first ^ (second + 0x9e3779b97f4a7c15ULL + (first << 6U) + (first >> 2U));
  }
};

template<std::size_t Size>
struct ExactArrayHash
{
  std::size_t operator()(const std::array<std::uint64_t, Size> & key) const
  {
    std::size_t result = 0U;
    for (const std::uint64_t value : key) {
      const std::size_t next = std::hash<std::uint64_t>{}(value);
      result ^= next + 0x9e3779b97f4a7c15ULL + (result << 6U) + (result >> 2U);
    }
    return result;
  }
};

std::vector<P3R3K12Factor> lateralFactors(
  const P3R3K12SideGeometry & geometry,
  const std::vector<P3R3K12ProductionCandidate> & production)
{
  const auto same_side = [&geometry](const auto & row) {return row.go_left == geometry.go_left;};
  std::vector<double> production_targets;
  std::set<std::pair<double, double>> production_pairs;
  production_targets.reserve(production.size());
  for (const auto & row : production) {
    if (!same_side(row)) {
      continue;
    }
    production_targets.push_back(row.d_target);
    production_pairs.emplace(row.d_target, row.d_mid);
  }
  production_targets = uniqueExact(std::move(production_targets));

  std::unordered_map<std::uint64_t, TargetCandidate> targets;
  targets.reserve(
    production_targets.size() * (1U + 2U * kTargetLocalOffsetsM.size()) +
    geometry.components.size() * kComponentFractions.size() + geometry.center_values.size());
  const auto add_target = [&](double value, const std::string & source, int priority) {
      value = clamp(value, geometry.domain_lower, geometry.domain_upper);
      const std::uint64_t key = exactBits(value);
      const auto found = targets.find(key);
      if (found == targets.end() ||
        std::tie(priority, source) < std::tie(found->second.priority, found->second.source))
      {
        targets[key] = {value, source, priority};
      }
    };
  for (const double value : production_targets) {
    add_target(value, "PRODUCTION_TARGET", 0);
    for (std::size_t index = 0U; index < kTargetLocalOffsetsM.size(); ++index) {
      add_target(value - kTargetLocalOffsetsM[index], kTargetLocalMinusSources[index], 2);
      add_target(value + kTargetLocalOffsetsM[index], kTargetLocalPlusSources[index], 2);
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
  ordered_targets.reserve(targets.size());
  for (const auto & entry : targets) {
    ordered_targets.push_back(entry.second);
  }
  std::sort(
    ordered_targets.begin(), ordered_targets.end(), [](const auto & first, const auto & second) {
      return std::tie(first.value, first.source, first.priority) <
             std::tie(second.value, second.source, second.priority);
    });

  std::unordered_map<ExactPairKey, P3R3K12Factor, ExactPairHash> factors;
  factors.reserve(ordered_targets.size() * 40U + production_pairs.size());
  const auto add_factor = [&](double target, double middle, const std::string & target_source,
    const std::string & mid_source, int source_priority) {
      middle = clamp(middle, -kTargetBoundM, kTargetBoundM);
      const ExactPairKey key{exactBits(target), exactBits(middle)};
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

  std::vector<std::array<std::string, kCenterInterpolationFractions.size()>> center_sources(
    geometry.center_values.size());
  for (std::size_t center_index = 0U; center_index < center_sources.size(); ++center_index) {
    for (std::size_t fraction_index = 0U;
      fraction_index < kCenterInterpolationFractions.size(); ++fraction_index)
    {
      center_sources[center_index][fraction_index] =
        "TO_CORRIDOR_CENTER_" + std::to_string(center_index) + "_" +
        kCenterFractionSources[fraction_index];
    }
  }
  for (const auto & target : ordered_targets) {
    if (production_pairs.find({target.value, target.value}) != production_pairs.end()) {
      add_factor(
        target.value, target.value, target.source, "PRODUCTION_OR_EQUAL_TARGET", 0);
    }
    add_factor(target.value, target.value, target.source, "MID_EQUALS_TARGET", target.priority);
    for (std::size_t center_index = 0U;
      center_index < geometry.center_values.size(); ++center_index)
    {
      for (std::size_t fraction_index = 0U;
        fraction_index < kCenterInterpolationFractions.size(); ++fraction_index)
      {
        const double fraction = kCenterInterpolationFractions[fraction_index];
        add_factor(
          target.value,
          target.value + fraction * (geometry.center_values[center_index] - target.value),
          target.source, center_sources[center_index][fraction_index], target.priority + 1);
      }
    }
    for (std::size_t index = 0U; index < kReferenceInwardFactors.size(); ++index) {
      add_factor(
        target.value, kReferenceInwardFactors[index] * target.value, target.source,
        kReferenceInwardSources[index], target.priority + 2);
    }
    for (std::size_t index = 0U; index < kMidAbsoluteOffsetsM.size(); ++index) {
      add_factor(target.value, target.value - kMidAbsoluteOffsetsM[index], target.source,
            kMidMinusSources[index],
            target.priority + 1);
      add_factor(target.value, target.value + kMidAbsoluteOffsetsM[index], target.source,
            kMidPlusSources[index],
            target.priority + 1);
    }
  }
  for (const auto & pair : production_pairs) {
    add_factor(pair.first, pair.second, "PRODUCTION_TARGET", "PRODUCTION_MID", 0);
  }

  std::vector<P3R3K12Factor> factor_rows;
  factor_rows.reserve(factors.size());
  for (auto & entry : factors) {
    factor_rows.push_back(std::move(entry.second));
  }
  std::vector<std::size_t> order(factor_rows.size());
  std::iota(order.begin(), order.end(), 0U);
  std::sort(order.begin(), order.end(), [&factor_rows](std::size_t first_index,
    std::size_t second_index) {
      const auto & first = factor_rows[first_index];
      const auto & second = factor_rows[second_index];
      if (first.source_priority != second.source_priority) {
        return first.source_priority < second.source_priority;
      }
      const double first_middle_distance = std::abs(first.d_mid - first.d_target);
      const double second_middle_distance = std::abs(second.d_mid - second.d_target);
      if (first_middle_distance != second_middle_distance) {
        return first_middle_distance < second_middle_distance;
      }
      const double first_target_distance = std::abs(first.d_target);
      const double second_target_distance = std::abs(second.d_target);
      if (first_target_distance != second_target_distance) {
        return first_target_distance < second_target_distance;
      }
      if (first.target_source != second.target_source) {
        return first.target_source < second.target_source;
      }
      if (first.mid_source != second.mid_source) {
        return first.mid_source < second.mid_source;
      }
      return std::make_pair(pythonHex(first.d_target), pythonHex(first.d_mid)) <
             std::make_pair(pythonHex(second.d_target), pythonHex(second.d_mid));
    });
  std::vector<P3R3K12Factor> output;
  output.reserve(order.size());
  for (const std::size_t index : order) {
    factor_rows[index].lateral_factor_index = output.size();
    output.push_back(std::move(factor_rows[index]));
  }
  return output;
}

// Compact hot-pool row. Frozen source strings and Python-compatible keys are materialized only
// for the selected 10+2 outputs; carrying six std::string objects through a 71k-row pool caused
// substantial allocation, cache, and destruction overhead without participating in rank fields.
struct RankedFactor
{
  bool go_left{false};
  bool profile_representative{false};
  double d_target{0.0};
  double d_mid{0.0};
  double entry_scale{0.0};
  double exit_scale{0.0};
  std::array<double, 5> stations{};
  int source_priority{0};
  std::size_t source_catalog_index{0U};
  std::size_t lateral_factor_index{0U};
  std::size_t transition_index{0U};
  std::array<std::uint64_t, 8> preconstruction_shape_bits{};

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

void populateMetrics(
  RankedFactor & candidate, const P3R3K12SideGeometry & geometry,
  const ProfileEvaluationBasis & evaluation_basis)
{
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
  PythonFloatSum violation_sum;
  PythonFloatSum center_error_sum;
  PythonFloatSum shape_energy;
  double maximum_violation = 0.0;
  double minimum_clearance = 0.0;
  double maximum_later_violation = 0.0;
  double peak_slope = 0.0;
  double curvature_proxy = 0.0;
  bool has_violation = false;
  bool has_clearance = false;
  bool has_later_violation = false;
  bool has_slope = false;
  bool has_curvature = false;
  std::size_t center_error_count = 0U;
  std::size_t slope_count = 0U;
  std::size_t discrete_count = 0U;
  std::pair<double, double> previous_two;
  std::pair<double, double> previous_one;
  const auto update_maximum = [](double value, double & maximum, bool & populated) {
      if (!populated) {
        maximum = value;
        populated = true;
      } else if (maximum < value) {
        maximum = value;
      }
    };
  const auto update_minimum = [](double value, double & minimum, bool & populated) {
      if (!populated) {
        minimum = value;
        populated = true;
      } else if (value < minimum) {
        minimum = value;
      }
    };
  for (const auto & basis : evaluation_basis.samples) {
    const double value = profileValue(profile, basis, geometry.ego_d);
    double violation = kTargetBoundM;
    double clearance = -kTargetBoundM;
    if (basis.finite) {
      violation = std::max({0.0, basis.lower - value, value - basis.upper});
      clearance = std::min(value - basis.lower, basis.upper - value);
      if (basis.center_active) {
        center_error_sum.add(std::abs(value - basis.center) / basis.center_width_denominator);
        ++center_error_count;
      }
    }
    update_maximum(violation, maximum_violation, has_violation);
    violation_sum.add(violation);
    update_minimum(clearance, minimum_clearance, has_clearance);
    if (basis.later_obstacle_active) {
      update_maximum(
        violation, maximum_later_violation, has_later_violation);
    }
    const std::pair<double, double> current{basis.station, value};
    if (discrete_count >= 1U) {
      const double ds = basis.slope_ds;
      if (ds > kEpsilon) {
        const double slope = std::abs(current.second - previous_one.second) / ds;
        update_maximum(slope, peak_slope, has_slope);
        shape_energy.add(slope * slope);
        ++slope_count;
      }
    }
    if (discrete_count >= 2U) {
      const double left_ds = basis.left_ds;
      const double right_ds = basis.right_ds;
      if (left_ds > kEpsilon && right_ds > kEpsilon) {
        const double left = (previous_one.second - previous_two.second) / left_ds;
        const double right = (current.second - previous_one.second) / right_ds;
        const double second =
          std::abs(right - left) /
          std::max(kEpsilon, basis.second_difference_denominator);
        update_maximum(second, curvature_proxy, has_curvature);
      }
    }
    if (discrete_count >= 1U) {
      previous_two = previous_one;
    }
    previous_one = current;
    ++discrete_count;
  }
  const double mean_shape_energy =
    shape_energy.value() / static_cast<double>(std::max<std::size_t>(1U, slope_count));
  candidate.exit_conflict_proxy =
    has_later_violation && maximum_later_violation > kEpsilon;
  candidate.maximum_corridor_violation_m = has_violation ?
    maximum_violation : std::numeric_limits<double>::infinity();
  candidate.sum_corridor_violation_m = violation_sum.value();
  candidate.slope_excess = std::max(
    0.0, (has_slope ? peak_slope : std::numeric_limits<double>::infinity()) -
    kMaximumLateralSlope);
  candidate.curvature_proxy = has_curvature ?
    curvature_proxy : std::numeric_limits<double>::infinity();
  candidate.center_error = center_error_count == 0U ?
    std::numeric_limits<double>::infinity() :
    center_error_sum.value() / static_cast<double>(center_error_count);
  candidate.minimum_clearance_m = has_clearance ?
    minimum_clearance : -std::numeric_limits<double>::infinity();
  candidate.shape_energy = mean_shape_energy;
}

void copyProxyMetrics(RankedFactor & destination, const RankedFactor & source)
{
  destination.construction_guard_proxy = source.construction_guard_proxy;
  destination.exit_conflict_proxy = source.exit_conflict_proxy;
  destination.maximum_corridor_violation_m = source.maximum_corridor_violation_m;
  destination.sum_corridor_violation_m = source.sum_corridor_violation_m;
  destination.slope_excess = source.slope_excess;
  destination.curvature_proxy = source.curvature_proxy;
  destination.center_error = source.center_error;
  destination.minimum_clearance_m = source.minimum_clearance_m;
  destination.shape_energy = source.shape_energy;
}

double baseScore(const RankedFactor & candidate)
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

template<typename Value>
int orderedCompare(const Value & first, const Value & second)
{
  if (first < second) {
    return -1;
  }
  if (second < first) {
    return 1;
  }
  return 0;
}

bool stableTieLess(const RankedFactor & first, const RankedFactor & second)
{
  int comparison = orderedCompare(first.source_priority, second.source_priority);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.lateral_factor_index, second.lateral_factor_index);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.transition_index, second.transition_index);
  if (comparison != 0) {
    return comparison < 0;
  }
  return orderedCompare(first.go_left, second.go_left) < 0;
}

bool lexicographicProxyLess(const RankedFactor & first, const RankedFactor & second)
{
  int comparison = orderedCompare(first.exit_conflict_proxy, second.exit_conflict_proxy);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(
    first.maximum_corridor_violation_m > kEpsilon,
    second.maximum_corridor_violation_m > kEpsilon);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(
    first.maximum_corridor_violation_m, second.maximum_corridor_violation_m);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.slope_excess > kEpsilon, second.slope_excess > kEpsilon);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.slope_excess, second.slope_excess);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(
    first.sum_corridor_violation_m, second.sum_corridor_violation_m);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.curvature_proxy, second.curvature_proxy);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.center_error, second.center_error);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(-first.minimum_clearance_m, -second.minimum_clearance_m);
  if (comparison != 0) {
    return comparison < 0;
  }
  comparison = orderedCompare(first.shape_energy, second.shape_energy);
  return comparison != 0 ? comparison < 0 : stableTieLess(first, second);
}

std::vector<P3R3K12Factor> materializeDeduplicatedShapeOrder(
  const std::vector<RankedFactor> & pool, const std::vector<std::size_t> & order,
  std::size_t maximum_output, P3R3K12SelectionProfile * profile)
{
  const auto start = ProfileClock::now();
  std::vector<P3R3K12Factor> output;
  std::set<std::array<std::uint64_t, 8>> seen;
  output.reserve(order.size());
  for (const std::size_t index : order) {
    const auto & row = pool[index];
    if (seen.insert(row.preconstruction_shape_bits).second) {
      P3R3K12Factor materialized;
      materialized.go_left = row.go_left;
      materialized.d_target = row.d_target;
      materialized.d_mid = row.d_mid;
      materialized.entry_scale = row.entry_scale;
      materialized.exit_scale = row.exit_scale;
      materialized.stations = row.stations;
      materialized.source_priority = row.source_priority;
      materialized.source_catalog_index = row.source_catalog_index;
      materialized.lateral_factor_index = row.lateral_factor_index;
      materialized.transition_index = row.transition_index;
      materialized.preconstruction_shape_bits = row.preconstruction_shape_bits;
      materialized.construction_guard_proxy = row.construction_guard_proxy;
      materialized.exit_conflict_proxy = row.exit_conflict_proxy;
      materialized.maximum_corridor_violation_m = row.maximum_corridor_violation_m;
      materialized.sum_corridor_violation_m = row.sum_corridor_violation_m;
      materialized.slope_excess = row.slope_excess;
      materialized.curvature_proxy = row.curvature_proxy;
      materialized.center_error = row.center_error;
      materialized.minimum_clearance_m = row.minimum_clearance_m;
      materialized.shape_energy = row.shape_energy;
      materialized.entry_normalized = row.entry_normalized;
      materialized.exit_normalized = row.exit_normalized;
      materialized.d_target_hex = pythonHex(row.d_target);
      materialized.d_mid_hex = pythonHex(row.d_mid);
      materialized.configuration_key = configurationKey(
        row.go_left, materialized.d_target_hex, materialized.d_mid_hex,
        row.entry_scale, row.exit_scale);
      materialized.preconstruction_shape_key = shapeKey(
        row.go_left, materialized.d_target_hex, materialized.d_mid_hex, row.stations);
      output.push_back(std::move(materialized));
      if (output.size() >= maximum_output) {
        break;
      }
    }
  }
  if (profile != nullptr) {
    profile->shape_deduplication_us += profileElapsedUs(start);
  }
  return output;
}

std::vector<P3R3K12Factor> lexicographicOrder(
  const std::vector<RankedFactor> & pool, P3R3K12SelectionProfile * profile)
{
  const auto start = ProfileClock::now();
  std::vector<std::size_t> feasible;
  std::vector<std::size_t> guarded;
  feasible.reserve(pool.size());
  guarded.reserve(pool.size());
  for (std::size_t index = 0U; index < pool.size(); ++index) {
    if (!pool[index].profile_representative) {
      continue;
    }
    (pool[index].construction_guard_proxy ? guarded : feasible).push_back(index);
  }
  const auto feasible_less = [&pool](std::size_t first_index, std::size_t second_index) {
      return lexicographicProxyLess(pool[first_index], pool[second_index]);
    };
  const auto guarded_less = [&pool](std::size_t first, std::size_t second) {
      return stableTieLess(pool[first], pool[second]);
    };
  std::vector<std::size_t> selected;
  selected.reserve(kP3R3K12LexicographicQuota);
  const auto select_until_quota = [&selected](auto candidates,
    const auto & less) {
      const std::size_t available = kP3R3K12LexicographicQuota - selected.size();
      if (available == 0U) {
        return;
      }
      const std::size_t count = std::min(available, candidates.size());
      std::partial_sort(candidates.begin(), candidates.begin() + count, candidates.end(), less);
      for (std::size_t index = 0U; index < count; ++index) {
        selected.push_back(candidates[index]);
      }
    };
  select_until_quota(feasible, feasible_less);
  select_until_quota(guarded, guarded_less);
  if (profile != nullptr) {
    profile->lexicographic_ordering_us += profileElapsedUs(start);
  }
  return materializeDeduplicatedShapeOrder(
    pool, selected, kP3R3K12LexicographicQuota, profile);
}

std::vector<P3R3K12Factor> coverageOrder(
  const std::vector<RankedFactor> & pool, P3R3K12SelectionProfile * profile)
{
  const auto start = ProfileClock::now();
  std::vector<std::size_t> remaining;
  std::vector<std::size_t> guarded;
  std::vector<double> base_scores;
  remaining.reserve(pool.size());
  guarded.reserve(pool.size());
  base_scores.reserve(pool.size());
  for (std::size_t index = 0U; index < pool.size(); ++index) {
    base_scores.push_back(baseScore(pool[index]));
    (pool[index].construction_guard_proxy ? guarded : remaining).push_back(index);
  }
  const auto base_less = [&pool, &base_scores](std::size_t first, std::size_t second) {
      int comparison = orderedCompare(
        pool[first].exit_conflict_proxy, pool[second].exit_conflict_proxy);
      if (comparison != 0) {
        return comparison < 0;
      }
      comparison = orderedCompare(base_scores[first], base_scores[second]);
      return comparison != 0 ? comparison < 0 : stableTieLess(pool[first], pool[second]);
    };
  std::vector<std::size_t> selected;
  selected.reserve(pool.size());
  std::set<std::array<std::uint64_t, 8>> selected_shapes;
  while (!remaining.empty() && selected_shapes.size() < kP3R3K12CoverageQuota &&
    selected.size() < 48U)
  {
    std::size_t best = 0U;
    if (selected.empty()) {
      for (std::size_t index = 1U; index < remaining.size(); ++index) {
        if (base_less(remaining[index], remaining[best])) {
          best = index;
        }
      }
    } else {
      const auto adjusted_score = [&pool, &selected, &base_scores](std::size_t row_index) {
          const auto & row = pool[row_index];
          double distance = std::numeric_limits<double>::infinity();
          for (const std::size_t chosen_index : selected) {
            const auto & chosen = pool[chosen_index];
            distance = std::min(
              distance,
              (row.go_left != chosen.go_left ? 0.5 : 0.0) +
              std::abs(row.d_target - chosen.d_target) / 1.5 +
              std::abs(row.d_mid - chosen.d_mid) / 3.0 +
              0.20 * std::abs(row.entry_normalized - chosen.entry_normalized) +
              0.20 * std::abs(row.exit_normalized - chosen.exit_normalized));
          }
          return base_scores[row_index] - 2.0 * std::min(1.0, distance);
        };
      double best_adjusted = adjusted_score(remaining[best]);
      for (std::size_t index = 1U; index < remaining.size(); ++index) {
        const std::size_t candidate = remaining[index];
        const double candidate_adjusted = adjusted_score(candidate);
        int comparison = orderedCompare(
          pool[candidate].exit_conflict_proxy, pool[remaining[best]].exit_conflict_proxy);
        if (comparison == 0) {
          comparison = orderedCompare(candidate_adjusted, best_adjusted);
        }
        if (comparison == 0) {
          comparison = orderedCompare(base_scores[candidate], base_scores[remaining[best]]);
        }
        if (comparison < 0 ||
          (comparison == 0 && stableTieLess(pool[candidate], pool[remaining[best]])))
        {
          best = index;
          best_adjusted = candidate_adjusted;
        }
      }
    }
    selected.push_back(remaining[best]);
    selected_shapes.insert(pool[remaining[best]].preconstruction_shape_bits);
    remaining.erase(remaining.begin() + static_cast<std::ptrdiff_t>(best));
  }
  if (selected_shapes.size() < kP3R3K12CoverageQuota) {
    std::sort(remaining.begin(), remaining.end(), base_less);
  }
  for (const std::size_t index : remaining) {
    if (selected_shapes.size() >= kP3R3K12CoverageQuota) {
      break;
    }
    selected.push_back(index);
    selected_shapes.insert(pool[index].preconstruction_shape_bits);
  }
  std::sort(guarded.begin(), guarded.end(), [&pool](std::size_t first, std::size_t second) {
      return stableTieLess(pool[first], pool[second]);
  });
  for (const std::size_t index : guarded) {
    if (selected_shapes.size() >= kP3R3K12CoverageQuota) {
      break;
    }
    selected.push_back(index);
    selected_shapes.insert(pool[index].preconstruction_shape_bits);
  }
  if (profile != nullptr) {
    profile->coverage_ordering_us += profileElapsedUs(start);
  }
  return materializeDeduplicatedShapeOrder(
    pool, selected, kP3R3K12CoverageQuota, profile);
}

}  // namespace

P3R3K12Selection selectP3R3K12Factors(
  const std::vector<P3R3K12SideGeometry> & sides,
  const std::vector<P3R3K12ProductionCandidate> & production_candidates,
  P3R3K12SelectionProfile * profile)
{
  P3R3K12Selection result;
  const auto transition_start = ProfileClock::now();
  std::set<Transition> transitions;
  std::set<std::array<std::uint64_t, 5>> production_keys;
  std::vector<double> entries;
  std::vector<double> exits;
  for (const auto & candidate : production_candidates) {
    transitions.emplace(candidate.entry_scale, candidate.exit_scale);
    entries.push_back(candidate.entry_scale);
    exits.push_back(candidate.exit_scale);
    production_keys.insert(configurationKeyBits(
        candidate.go_left, candidate.d_target, candidate.d_mid,
        candidate.entry_scale, candidate.exit_scale));
  }
  entries = uniqueExact(std::move(entries));
  exits = uniqueExact(std::move(exits));
  if (profile != nullptr) {
    profile->transition_count = transitions.size();
    profile->transition_generation_us = profileElapsedUs(transition_start);
  }

  std::vector<P3R3K12SideGeometry> ordered_sides = sides;
  std::sort(ordered_sides.begin(), ordered_sides.end(),
    [](const auto & first, const auto & second) {
      return first.go_left < second.go_left;  // RIGHT before LEFT
    });
  std::vector<RankedFactor> pool;
  std::vector<std::pair<std::string, std::string>> source_catalog;
  for (const auto & geometry : ordered_sides) {
    const auto lateral_start = ProfileClock::now();
    auto lateral = lateralFactors(geometry, production_candidates);
    if (profile != nullptr) {
      profile->lateral_factor_generation_us += profileElapsedUs(lateral_start);
    }
    result.lateral_factor_count += lateral.size();
    if (profile != nullptr) {
      profile->raw_combination_count += lateral.size() * transitions.size();
    }
    pool.reserve(pool.size() + lateral.size() * transitions.size());
    source_catalog.reserve(source_catalog.size() + lateral.size());
    const auto pair_start = ProfileClock::now();
    const std::size_t side_pool_begin = pool.size();
    std::unordered_map<std::array<std::uint64_t, 5>, std::size_t, ExactArrayHash<5>>
    profile_basis_indices;
    profile_basis_indices.reserve(transitions.size() * 2U);
    std::vector<ProfileEvaluationBasis> profile_bases;
    std::vector<std::size_t> candidate_basis_indices;
    std::vector<std::size_t> candidate_metric_source_indices;
    std::vector<std::size_t> unique_metric_indices;
    std::unordered_map<std::array<std::uint64_t, 8>, std::size_t, ExactArrayHash<8>>
    profile_metric_indices;
    candidate_basis_indices.reserve(lateral.size() * transitions.size());
    candidate_metric_source_indices.reserve(lateral.size() * transitions.size());
    unique_metric_indices.reserve(lateral.size() * transitions.size());
    profile_metric_indices.reserve(lateral.size() * transitions.size());
    for (auto & factor : lateral) {
      const std::size_t source_catalog_index = source_catalog.size();
      source_catalog.emplace_back(
        std::move(factor.target_source), std::move(factor.mid_source));
      std::size_t transition_index = 0U;
      for (const auto & transition : transitions) {
        RankedFactor candidate;
        candidate.go_left = factor.go_left;
        candidate.d_target = factor.d_target;
        candidate.d_mid = factor.d_mid;
        candidate.source_priority = factor.source_priority;
        candidate.source_catalog_index = source_catalog_index;
        candidate.lateral_factor_index = factor.lateral_factor_index;
        candidate.entry_scale = transition.first;
        candidate.exit_scale = transition.second;
        candidate.entry_normalized = normalized(candidate.entry_scale, entries);
        candidate.exit_normalized = normalized(candidate.exit_scale, exits);
        candidate.transition_index = transition_index++;
        if (production_keys.find(configurationKeyBits(
            candidate.go_left, candidate.d_target, candidate.d_mid,
            candidate.entry_scale, candidate.exit_scale)) != production_keys.end())
        {
          if (profile != nullptr) {
            ++profile->production_excluded_count;
          }
          continue;
        }
        candidate.stations = stationsFor(
          geometry, candidate.entry_scale, candidate.exit_scale, candidate.d_target);
        candidate.preconstruction_shape_bits = shapeKeyBits(
          candidate.go_left, candidate.d_target, candidate.d_mid, candidate.stations);
        std::array<std::uint64_t, 5> basis_key{};
        for (std::size_t index = 0U; index < candidate.stations.size(); ++index) {
          basis_key[index] = exactBits(candidate.stations[index]);
        }
        const std::size_t local_index = candidate_basis_indices.size();
        const auto metric = profile_metric_indices.emplace(
          candidate.preconstruction_shape_bits, local_index);
        candidate.profile_representative = metric.second;
        std::size_t basis_index = 0U;
        if (metric.second) {
          unique_metric_indices.push_back(local_index);
          auto basis = profile_basis_indices.find(basis_key);
          if (basis == profile_basis_indices.end()) {
            basis_index = profile_bases.size();
            profile_bases.push_back(makeProfileEvaluationBasis(candidate.stations, geometry));
            profile_basis_indices.emplace(basis_key, basis_index);
            if (profile != nullptr) {
              ++profile->profile_basis_build_count;
              profile->profile_sample_basis_count += profile_bases.back().samples.size();
            }
          } else {
            basis_index = basis->second;
            if (profile != nullptr) {
              ++profile->profile_basis_cache_hit_count;
            }
          }
        } else {
          basis_index = candidate_basis_indices[metric.first->second];
        }
        pool.push_back(std::move(candidate));
        candidate_basis_indices.push_back(basis_index);
        candidate_metric_source_indices.push_back(metric.first->second);
      }
    }
    const std::size_t candidate_count = candidate_basis_indices.size();
    const std::size_t metric_count = unique_metric_indices.size();
    if (profile != nullptr) {
      profile->unique_profile_count += metric_count;
      profile->proxy_metric_evaluation_count += metric_count;
      profile->proxy_metric_cache_hit_count += candidate_count - metric_count;
    }
    const unsigned int hardware_workers = std::max(1U, std::thread::hardware_concurrency());
    const std::size_t worker_count = metric_count < kMinimumFactorsPerPairMetricWorker ? 1U :
      std::min<std::size_t>(kPairMetricWorkerLimit, hardware_workers);
    if (profile != nullptr) {
      profile->pair_metric_worker_count = std::max(
        profile->pair_metric_worker_count, worker_count);
    }
    const auto evaluate_range = [&](std::size_t begin, std::size_t end) {
        for (std::size_t metric_index = begin; metric_index < end; ++metric_index) {
          const std::size_t local_index = unique_metric_indices[metric_index];
          auto & candidate = pool[side_pool_begin + local_index];
          populateMetrics(
            candidate, geometry, profile_bases[candidate_basis_indices[local_index]]);
        }
      };
    if (worker_count == 1U) {
      evaluate_range(0U, metric_count);
    } else {
      std::vector<std::thread> workers;
      workers.reserve(worker_count);
      const std::size_t chunk_size = (metric_count + worker_count - 1U) / worker_count;
      for (std::size_t worker = 0U; worker < worker_count; ++worker) {
        const std::size_t begin = worker * chunk_size;
        const std::size_t end = std::min(metric_count, begin + chunk_size);
        if (begin < end) {
          workers.emplace_back(evaluate_range, begin, end);
        }
      }
      for (auto & worker : workers) {
        worker.join();
      }
    }
    for (std::size_t local_index = 0U; local_index < candidate_count; ++local_index) {
      const std::size_t source_index = candidate_metric_source_indices[local_index];
      if (source_index != local_index) {
        copyProxyMetrics(
          pool[side_pool_begin + local_index], pool[side_pool_begin + source_index]);
      }
    }
    if (profile != nullptr) {
      profile->pair_priority_computation_us += profileElapsedUs(pair_start);
    }
  }
  result.pair_priority_count = pool.size();
  if (profile != nullptr) {
    profile->factor_pool_count = pool.size();
  }
  result.lexicographic = lexicographicOrder(pool, profile);
  result.coverage = coverageOrder(pool, profile);
  const auto restore_sources = [&source_catalog](auto & factors) {
      for (auto & factor : factors) {
        const auto & source = source_catalog.at(factor.source_catalog_index);
        factor.target_source = source.first;
        factor.mid_source = source.second;
      }
    };
  restore_sources(result.lexicographic);
  restore_sources(result.coverage);
  return result;
}

}  // namespace local_planning
