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
// 대회 규정 기반 **속성** 테스트.
//
// 왜 필요한가 (2026-08-17). 기존 안전망 8종은 전부 녹화 재생이다 — "17:02 백의 이 프레임에서
// 회피가 성립해야 한다". 봤던 배치만 지킨다. 그런데 규정상 장애물 위치는 **사전에 공지되지
// 않고**, 놓인 뒤에는 코드 수정도 금지다. 본 적 없는 배치에서 어떻게 되는지를 재생 테스트는
// 아무것도 보장하지 못한다.
//
// 그래서 "이 입력에서 이 출력" 대신 **"모든 합법 입력에서 이 성질"**을 검사한다. 성질은 하나다:
//
//     기하적으로 통로가 존재하면 플래너는 그 통로를 찾아야 하고,
//     존재하지 않으면 찾았다고 해서는 안 된다.
//
// 판정자(oracle)는 플래너 코드를 **재사용하지 않는다**. 재사용하면 아무것도 증명하지 못한다.
// 회랑에서 장애물을 뺀 자유 구간을 s를 따라 1차원 도달성으로 이어붙이는 독립 계산이다.
//
// 양방향 오탐을 피하려고 판정자를 두 벌 쓴다:
//   보수 판정자  플래너보다 **엄격**하다(기울기 절반, 여유 10% 추가). 여기서 통로가 나오면
//                플래너는 여유를 두고 성공해야 한다. 실패하면 확실한 우리 결함이다.
//   관대 판정자  플래너보다 **느슨**하다(기울기 무제한, 여유 정확히 동일). 여기서 통로가
//                없으면 어떤 경로도 물리적으로 불가능하다. 플래너가 성공하면 거짓 통과다.
// 두 판정자 사이(관대는 통과, 보수는 불가)는 회색지대로 두고 아무것도 주장하지 않는다.
//
// 등급이 둘이다:
//   필수(kMandatory)  현재 예약으로 통로가 존재하는 배치. 우리 코드의 성적표 — 전부 통과해야.
//   목표(kAspiration) 규정 최소 자유폭 0.5 m까지 포함. 현재 예약으로는 물리적으로 불가능하다
//                     (필요 0.72~0.92 m). MCL·제어 개선 대기 항목이라 실패해도 빌드는 통과시키고
//                     대신 "얼마나 모자란지"를 숫자로 보고한다. 이 숫자가 팀에 전달할 목표치다.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "local_planning/p3_shadow.hpp"
#include "local_planning/raceline_spline_planner.hpp"
#include "p3_scenario_stream.hpp"

namespace local_planning
{
namespace
{

using test_stream::parametersOf;
using test_stream::readStream;
using test_stream::scenarioPath;

// ── 대회 규정 (2026 IFAC F1TENTH, 8. 정적 장애물) ────────────────────────────────────────
constexpr double kMaxObstacleSideM = 0.50;      // 각 장애물은 0.5 x 0.5 보다 작다
constexpr double kMinObstacleGapM = 1.00;       // 장애물 간 최소 거리 1 m
constexpr double kMinFreeWidthM = 0.50;         // 장애물이 있어도 자유폭 >= 0.5 m
constexpr double kStartExclusionM = 1.00;       // 출발 위치 기준 1 m 내 금지
constexpr std::size_t kQualifyingObstacles = 2U;   // 예선: 심판이 2개
constexpr std::size_t kFinalsObstacles = 3U;       // 본선: 팀·심판 합쳐 3개

struct Interval
{
  double lower{0.0};
  double upper{0.0};
  bool empty() const {return !(upper > lower);}
};

struct Placement
{
  std::vector<f110_msgs::msg::Obstacle> obstacles;
  double narrowest_free_width_m{0.0};   // 이 배치가 남긴 가장 좁은 통로
};

// 자유 구간에서 장애물 상자를 뺀다. 결과는 s가 아니라 하나의 s에서의 d 구간 집합이다.
std::vector<Interval> subtract(const std::vector<Interval> & source, const Interval & blocked)
{
  std::vector<Interval> result;
  for (const auto & interval : source) {
    if (blocked.upper <= interval.lower || blocked.lower >= interval.upper) {
      result.push_back(interval);
      continue;
    }
    if (blocked.lower > interval.lower) {
      result.push_back({interval.lower, std::min(interval.upper, blocked.lower)});
    }
    if (blocked.upper < interval.upper) {
      result.push_back({std::max(interval.lower, blocked.upper), interval.upper});
    }
  }
  result.erase(
    std::remove_if(
      result.begin(), result.end(), [](const Interval & i) {return i.empty();}),
    result.end());
  return result;
}

// 판정자 설정. 플래너보다 엄격하게(보수) 또는 느슨하게(관대) 만들기 위한 두 손잡이.
struct OracleSettings
{
  double clearance_scale{1.0};    // 횡 여유 배율. >1 이면 플래너보다 엄격.
  double slope_scale{1.0};        // 기울기 한계 배율. <1 이면 플래너보다 엄격.
  bool ignore_slope{false};       // true 면 횡이동 속도 무제한 (관대 판정자)
};

class CorridorOracle
{
public:
  CorridorOracle(
    const f110_msgs::msg::WpntArray & reference,
    const RacelineSplineParameters & parameters,
    const OracleSettings & settings)
  : reference_(reference), parameters_(parameters), settings_(settings) {}

  // 자차 d에서 출발해 장애물 구간을 통과하는 경로가 존재하는가.
  //
  // R(s) = 지금까지의 제약을 모두 만족하며 s 에서 도달 가능한 d 집합.
  //   R(s + ds) = dilate(R(s), slope * ds)  ∩  Free(s + ds)
  // 어느 지점에서든 R 이 비면 통로가 없다. 플래너 코드는 전혀 쓰지 않는다.
  bool corridorExists(
    double ego_s, double ego_d,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double * narrowest_width_m = nullptr) const
  {
    const std::size_t count = reference_.wpnts.size();
    const double track_length = reference_.wpnts.back().s_m;
    const std::size_t first = firstIndexAhead(ego_s);
    std::vector<Interval> reachable{{ego_d, ego_d}};
    double narrowest = std::numeric_limits<double>::infinity();
    double last_s = ego_s;
    bool saw_obstacle = false;

    for (std::size_t step = 0; step < count; ++step) {
      const std::size_t index = (first + step) % count;
      const auto & waypoint = reference_.wpnts[index];
      double forward = waypoint.s_m - ego_s;
      while (forward < 0.0) {forward += track_length;}
      if (forward > 0.5 * track_length) {break;}

      double ds = waypoint.s_m - last_s;
      while (ds < 0.0) {ds += track_length;}
      last_s = waypoint.s_m;

      if (!settings_.ignore_slope && ds > 0.0) {
        const double reach = parameters_.maximum_lateral_slope * settings_.slope_scale * ds;
        for (auto & interval : reachable) {
          interval.lower -= reach;
          interval.upper += reach;
        }
      } else if (settings_.ignore_slope) {
        reachable.assign(1U, {-1.0e3, 1.0e3});
      }

      auto free_space = freeAt(waypoint, obstacles, ego_s, track_length);
      std::vector<Interval> next;
      for (const auto & r : reachable) {
        for (const auto & f : free_space) {
          const Interval hit{std::max(r.lower, f.lower), std::min(r.upper, f.upper)};
          if (!hit.empty()) {next.push_back(hit);}
        }
      }
      const bool blocking = blocksHere(waypoint, obstacles, ego_s, track_length);
      if (blocking) {
        saw_obstacle = true;
        double widest = 0.0;
        for (const auto & f : free_space) {widest = std::max(widest, f.upper - f.lower);}
        narrowest = std::min(narrowest, widest);
      }
      if (next.empty()) {
        if (narrowest_width_m != nullptr) {
          *narrowest_width_m = std::numeric_limits<double>::quiet_NaN();
        }
        return false;
      }
      reachable = std::move(next);
      // 장애물 구간을 모두 지났으면 통로가 있는 것이다.
      if (saw_obstacle && !blocking && forward > lastObstacleEnd(obstacles, ego_s, track_length)) {
        break;
      }
    }
    if (narrowest_width_m != nullptr) {
      *narrowest_width_m = std::isfinite(narrowest) ? narrowest :
        std::numeric_limits<double>::quiet_NaN();
    }
    // 장애물을 한 번도 통과하지 않았다면 이 판정자는 아무것도 검사하지 않은 것이다.
    // "통로 있음"으로 반환하면 플래너에게 근거 없는 요구를 하게 되므로 판정 불가로 돌린다.
    return saw_obstacle;
  }

private:
  // 자차보다 **앞선** 첫 waypoint. 최근접 점을 쓰면 그것이 자차보다 뒤일 때 forward가 한 바퀴
  // 감겨(track_length 근처) 첫 스텝에서 곧바로 break 되고, 장애물을 한 번도 보지 않은 채
  // "통로 있음"을 돌려준다. 그러면 판정자가 공허해져 멀쩡한 플래너를 실패로 몬다.
  std::size_t firstIndexAhead(double s) const
  {
    const double track_length = reference_.wpnts.back().s_m;
    std::size_t best = 0U;
    double best_forward = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < reference_.wpnts.size(); ++i) {
      double forward = reference_.wpnts[i].s_m - s;
      while (forward < 0.0) {forward += track_length;}
      if (forward < best_forward) {best_forward = forward; best = i;}
    }
    return best;
  }

  // 장애물 면에서 경로 중심선까지 필요한 거리.
  //
  // 🔴 판정자가 플래너보다 **관대**하면 안 된다. 처음에는 vehicle_half_width_m +
  // safety_margin_m (=0.158)만 썼는데, 플래너는 여기에 추종오차 예약을 더한다
  // (expandVisibleObstacles: obstacleBaseClearance() + maximumReferenceTrackingErrorReserve).
  // 그 차이가 약 0.26 m다. 그래서 판정자가 "통로 있음"이라고 한 배치의 상당수가 사실은
  // 플래너 기준으로 통로가 없었고, 멀쩡한 기각을 결함으로 세고 있었다.
  double lateralClearance() const
  {
    return (parameters_.obstacleBaseClearance() +
           parameters_.avoidanceTrackingErrorReserve(reference_speed_mps_, 0.0)) *
           settings_.clearance_scale;
  }

  double wallClearance() const
  {
    return (parameters_.vehicle_half_width_m + parameters_.wall_safety_margin_m) *
           settings_.clearance_scale;
  }

  static bool spans(
    const f110_msgs::msg::Obstacle & obstacle, double s, double ego_s, double track_length)
  {
    auto forward = [&](double value) {
        double d = value - ego_s;
        while (d < 0.0) {d += track_length;}
        return d;
      };
    const double here = forward(s);
    return here >= forward(obstacle.s_start) && here <= forward(obstacle.s_end);
  }

  bool blocksHere(
    const f110_msgs::msg::Wpnt & waypoint,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double ego_s, double track_length) const
  {
    for (const auto & obstacle : obstacles) {
      if (spans(obstacle, waypoint.s_m, ego_s, track_length)) {return true;}
    }
    return false;
  }

  double lastObstacleEnd(
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double ego_s, double track_length) const
  {
    double last = 0.0;
    for (const auto & obstacle : obstacles) {
      double d = obstacle.s_end - ego_s;
      while (d < 0.0) {d += track_length;}
      last = std::max(last, d);
    }
    return last;
  }

  std::vector<Interval> freeAt(
    const f110_msgs::msg::Wpnt & waypoint,
    const std::vector<f110_msgs::msg::Obstacle> & obstacles,
    double ego_s, double track_length) const
  {
    std::vector<Interval> free_space{
      {-waypoint.d_right + wallClearance(), waypoint.d_left - wallClearance()}};
    for (const auto & obstacle : obstacles) {
      if (!spans(obstacle, waypoint.s_m, ego_s, track_length)) {continue;}
      free_space = subtract(
        free_space,
        {obstacle.d_right - lateralClearance(), obstacle.d_left + lateralClearance()});
    }
    return free_space;
  }

  const f110_msgs::msg::WpntArray & reference_;
  const RacelineSplineParameters & parameters_;
  OracleSettings settings_;
  // 추종오차 예약은 속도에 따라 커진다. 판정자는 플래너가 실제로 쓰는 속도(속성 테스트의
  // 자차 속도)로 평가한다.
  double reference_speed_mps_{3.0};
};

// 규정을 만족하는 배치를 생성한다. 만들 수 없으면 빈 배치를 돌려준다.
Placement generatePlacement(
  const f110_msgs::msg::WpntArray & reference,
  std::size_t obstacle_count,
  double minimum_free_width_m,
  std::mt19937 & rng)
{
  const double track_length = reference.wpnts.back().s_m;
  std::uniform_real_distribution<double> station(0.0, track_length);
  std::uniform_real_distribution<double> side(0.0, 1.0);
  Placement placement;
  placement.narrowest_free_width_m = std::numeric_limits<double>::infinity();

  for (std::size_t attempt = 0; attempt < 200U && placement.obstacles.size() < obstacle_count;
    ++attempt)
  {
    const double s_center = station(rng);
    if (s_center < kStartExclusionM || s_center > track_length - kStartExclusionM) {continue;}
    bool too_close = false;
    for (const auto & existing : placement.obstacles) {
      double gap = std::abs(existing.s_center - s_center);
      gap = std::min(gap, track_length - gap);
      if (gap < kMinObstacleGapM) {too_close = true; break;}
    }
    if (too_close) {continue;}

    const auto & waypoint = *std::min_element(
      reference.wpnts.begin(), reference.wpnts.end(),
      [s_center](const auto & a, const auto & b) {
        return std::abs(a.s_m - s_center) < std::abs(b.s_m - s_center);
      });
    const double track_lower = -waypoint.d_right;
    const double track_upper = waypoint.d_left;
    const double track_width = track_upper - track_lower;
    // 규정: 장애물을 놓고도 자유폭이 minimum_free_width_m 이상 남아야 한다.
    const double width = std::min(kMaxObstacleSideM, track_width - minimum_free_width_m);
    if (!(width > 0.05)) {continue;}
    // 남은 폭을 좌우로 나눠 한쪽이 minimum_free_width_m 이상이 되게 놓는다.
    const double slack = track_width - width - minimum_free_width_m;
    if (slack < 0.0) {continue;}
    const double left_gap = minimum_free_width_m + side(rng) * slack;
    const double d_low = track_lower + (track_width - width - left_gap);

    f110_msgs::msg::Obstacle obstacle;
    obstacle.id = static_cast<int>(placement.obstacles.size()) + 1;
    obstacle.s_center = s_center;
    obstacle.s_start = s_center - 0.5 * kMaxObstacleSideM;
    obstacle.s_end = s_center + 0.5 * kMaxObstacleSideM;
    obstacle.d_right = d_low;
    obstacle.d_left = d_low + width;
    obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
    obstacle.size = width;
    obstacle.is_static = true;
    obstacle.is_visible = true;
    placement.obstacles.push_back(obstacle);
    placement.narrowest_free_width_m = std::min(
      placement.narrowest_free_width_m, std::max(left_gap, track_width - width - left_gap));
  }
  if (placement.obstacles.size() < obstacle_count) {placement.obstacles.clear();}
  return placement;
}

struct Verdict
{
  std::size_t generated{0U};
  std::size_t mandatory_checked{0U};
  std::size_t mandatory_failed{0U};
  std::size_t false_pass{0U};
  std::map<std::string, std::size_t> failure_kinds;
  double worst_free_width_m{std::numeric_limits<double>::infinity()};
};

Verdict sweep(
  const f110_msgs::msg::WpntArray & reference,
  const RacelineSplineParameters & parameters,
  std::size_t obstacle_count,
  double minimum_free_width_m,
  std::size_t samples,
  std::uint32_t seed,
  bool verbose)
{
  OracleSettings strict;
  strict.clearance_scale = 1.10;   // 여유 10% 더
  strict.slope_scale = 0.50;       // 기울기 절반
  OracleSettings loose;
  loose.ignore_slope = true;
  const CorridorOracle conservative(reference, parameters, strict);
  const CorridorOracle permissive(reference, parameters, loose);

  RacelineSplinePlanner planner(parameters);
  EXPECT_TRUE(planner.setReference(reference));
  const double track_length = reference.wpnts.back().s_m;

  std::mt19937 rng(seed);
  std::uniform_real_distribution<double> ego_station(0.0, track_length);
  Verdict verdict;
  for (std::size_t sample = 0; sample < samples; ++sample) {
    const auto placement = generatePlacement(
      reference, obstacle_count, minimum_free_width_m, rng);
    if (placement.obstacles.empty()) {continue;}
    ++verdict.generated;

    // 자차 위치는 접근 중부터 **이미 옆에 붙은** 상태까지 고루 뽑는다.
    //
    // 처음에는 4~10 m 뒤만 뽑았는데, 그러면 cluster_start가 항상 크게 잡혀
    // "자차가 클러스터 옆에 있는" 영역을 한 번도 시험하지 않는다. 실주행에서 정지가 나는
    // 자리가 정확히 거기다(2026-08-17 01:55 백의 잔여 정지 5건 전부 cluster_start <= 1.1).
    // 음수까지 허용해 자차가 클러스터 앞단을 막 지난 상태도 포함한다.
    const double lead = placement.obstacles.front().s_start;
    std::uniform_real_distribution<double> back(-0.4, 10.0);
    double ego_s = lead - back(rng);
    while (ego_s < 0.0) {ego_s += track_length;}
    // 횡오프셋도 0만 쓰면 회피 도중 상태를 못 만든다. 다만 **그 지점의 트랙 폭 안에서만**
    // 뽑아야 한다.
    //
    // 처음에는 [-0.6, +0.6]에서 균등하게 뽑았는데, 그러면 차가 이미 벽에 박힌 상태로
    // 시작하는 배치가 생긴다. 그때 플래너는 첫 waypoint에서 footprint_track_bound로 기각하고
    // (기각이 옳다) 판정자는 그것을 "통로가 있는데 실패했다"로 센다. 즉 테스트가 스스로
    // 만들어낸 불가능한 상태를 플래너 결함으로 보고했다. 실제로 s=14.07/d=+0.523 배치에서
    // 그 지점의 허용 범위는 [-0.932, +0.482]였다.
    // 발자국은 차 길이의 절반(±0.28 m)만큼 앞뒤로 뻗으므로, 자차 위치 한 점의 트랙 폭만
    // 보면 부족하다. 그 구간에서 가장 좁은 폭을 써야 한다. 종전에는 최근접 waypoint 하나만
    // 보아, 바로 옆 waypoint에서 발자국이 트랙을 벗어나는 자차 상태를 계속 만들어냈다
    // (s=42.92에서 뽑은 d=+0.628이 s=42.93에서 기각됐다).
    const double body = parameters.vehicle_half_width_m + parameters.wall_safety_margin_m;
    const double reach = 0.5 * parameters.vehicle_length_m;
    double d_low = -1.0e3;
    double d_high = 1.0e3;
    for (const auto & waypoint : reference.wpnts) {
      double gap = waypoint.s_m - ego_s;
      while (gap < -0.5 * track_length) {gap += track_length;}
      while (gap > 0.5 * track_length) {gap -= track_length;}
      if (std::abs(gap) > reach) {continue;}
      d_low = std::max(d_low, -waypoint.d_right + body);
      d_high = std::min(d_high, waypoint.d_left - body);
    }
    if (!(d_high > d_low)) {continue;}   // 차가 설 수 없는 지점이면 표본에서 제외
    std::uniform_real_distribution<double> lateral(d_low, d_high);
    EgoFrenetState ego{ego_s, lateral(rng), 3.0};

    // 판정자와 플래너가 **같은 장애물 집합**을 봐야 공정하다. 플래너는 detection_lookahead_m
    // 안의 가장 가까운 군집만 계획한다(그 밖은 계획 대상이 아니며, 무시하는 것이 정상이다).
    // 지평 밖 장애물까지 통로를 요구하면 판정자가 플래너에게 계약에 없는 일을 묻게 된다.
    std::vector<f110_msgs::msg::Obstacle> in_horizon;
    for (const auto & obstacle : placement.obstacles) {
      double forward = obstacle.s_start - ego.s;
      while (forward < 0.0) {forward += track_length;}
      if (forward <= parameters.detection_lookahead_m) {in_horizon.push_back(obstacle);}
    }
    if (in_horizon.empty()) {continue;}

    const auto result = planner.evaluateP3Shadow(ego, in_horizon, 0, 0U, 1U, "PROPERTY");
    const bool planner_ok = result.would_recover ||
      result.failure_classification == "no static obstacle blocks the global race line";

    double width = 0.0;
    const bool conservative_ok = conservative.corridorExists(
      ego.s, ego.d, in_horizon, &width);
    const bool permissive_ok = permissive.corridorExists(ego.s, ego.d, in_horizon);

    if (conservative_ok) {
      ++verdict.mandatory_checked;
      if (!planner_ok) {
        ++verdict.mandatory_failed;
        ++verdict.failure_kinds[result.failure_classification];
        verdict.worst_free_width_m = std::min(verdict.worst_free_width_m, width);
        if (verbose && verdict.mandatory_failed <= 8U) {
          std::printf(
            "  MISS  ego s=%.2f | 통로 있음(가장 좁은 여유폭 %.3f m)인데 플래너 실패: %s\n",
            ego.s, width, result.failure_classification.c_str());
          for (const auto & obstacle : in_horizon) {
            std::printf(
              "          장애물 s=[%.2f,%.2f] d=[%+.3f,%+.3f]\n",
              obstacle.s_start, obstacle.s_end, obstacle.d_right, obstacle.d_left);
          }
          std::printf(
            "          clus=[%.3f,%.3f] L[%+.3f,%+.3f]%s R[%+.3f,%+.3f]%s 후보 %zu개\n",
            result.cluster_start_forward_m, result.cluster_end_forward_m,
            result.left_domain.minimum_target, result.left_domain.maximum_target,
            result.left_domain.valid ? "" : "(무효)",
            result.right_domain.minimum_target, result.right_domain.maximum_target,
            result.right_domain.valid ? "" : "(무효)",
            result.candidates.size());
          for (const auto & candidate : result.candidates) {
            std::printf(
              "            %s entry=%.3f exit=%.3f d=%+.3f peakK=%.3f slope=%.3f "
              "| 실패 s=%.2f d=%+.3f | %s\n",
              candidate.go_left ? "L" : "R", candidate.entry_scale, candidate.exit_scale,
              candidate.d_target, candidate.peak_curvature_radpm,
              candidate.peak_lateral_slope, candidate.validation.failure_waypoint_s,
              candidate.validation.failure_waypoint_d, candidate.rejection_reason.c_str());
          }
        }
      }
    }
    if (!permissive_ok && planner_ok) {
      ++verdict.false_pass;
      if (verbose && verdict.false_pass <= 4U) {
        std::printf("  FALSE ego s=%.2f | 통로가 없는데 플래너가 회피를 만들었다\n", ego.s);
      }
    }
  }
  return verdict;
}

const f110_msgs::msg::WpntArray & raceReference()
{
  static const auto stream = readStream(scenarioPath("passing_mixed"));
  return stream.reference;
}

RacelineSplineParameters raceParameters()
{
  static const auto stream = readStream(scenarioPath("passing_mixed"));
  return parametersOf(stream);
}

}  // namespace

// ── 필수 등급 ────────────────────────────────────────────────────────────────────────────
// 보수 판정자(플래너보다 엄격)가 통로를 찾은 배치에서는 플래너도 반드시 찾아야 한다.
// 여기서 실패하는 것은 예약 부족이 아니라 **우리 코드의 결함**이다 — 판정자가 이미 플래너보다
// 큰 여유와 절반의 기울기로 통로를 확인했기 때문이다.
TEST(RuleProperty, PlannerFindsEveryCorridorAConservativeOracleFinds)
{
  const bool verbose = std::getenv("RULE_PROPERTY_DUMP") != nullptr;
  const auto & reference = raceReference();
  const auto parameters = raceParameters();
  ASSERT_GT(reference.wpnts.size(), 100U);

  std::size_t checked = 0U;
  std::size_t failed = 0U;
  for (const std::size_t count : {kQualifyingObstacles, kFinalsObstacles}) {
    // 자유폭은 규정 최소(0.5)가 아니라 현재 예약으로 통과 가능한 범위에서 뽑는다.
    // 규정 최소는 아래 목표 등급이 따로 잰다.
    const auto verdict = sweep(reference, parameters, count, 1.20, 300U, 20260817U, verbose);
    if (verbose) {
      std::printf(
        "장애물 %zu개 / 자유폭 >= 1.20 m: 생성 %zu, 필수 %zu, 실패 %zu, 거짓통과 %zu\n",
        count, verdict.generated, verdict.mandatory_checked, verdict.mandatory_failed,
        verdict.false_pass);
    }
    if (verbose) {
      for (const auto & kind : verdict.failure_kinds) {
        std::printf("    사유 %4zu회  %s\n", kind.second, kind.first.c_str());
      }
    }
    EXPECT_EQ(verdict.false_pass, 0U)
      << "장애물 " << count << "개: 통로가 없는데 회피를 만들었다 — 충돌 위험";
    checked += verdict.mandatory_checked;
    failed += verdict.mandatory_failed;
  }
  ASSERT_GT(checked, 50U) << "표본이 너무 적어 성질을 주장할 수 없다";

  // 🔴 래칫. 목표는 0이지만 지금은 아니다. 현재 수치를 상한으로 박아 **회귀만** 막고,
  // 수리해서 내려가면 이 숫자를 같이 내린다. 0으로 두면 빌드가 계속 빨개서 신호가 죽고,
  // 상한 없이 두면 조용히 나빠진다.
  //
  // 2026-08-17 1차: 597개 중 36개(6.0%). 그때 생성기는 자차를 장애물 4~10 m 뒤에만 두어
  //   접근 구간만 재고 있었다. 실주행에서 정지가 나는 자리는 자차가 클러스터 **옆에 붙은**
  //   상태인데(01:55 백의 잔여 정지 5건 전부 cluster_start <= 1.1) 그 영역을 한 번도 시험하지
  //   않았다. 생성기를 -0.4~10 m + 횡오프셋 ±0.6 m로 넓혔다.
  //
  // 2026-08-17 2차: 546개 중 102개. 생성기를 넓힌 결과이지 코드가 나빠진 것이 아니다.
  //
  // 2026-08-17 3차(현재 기준): 538개 중 99개(18.4%). 이 사이에 **판정자와 생성기 자체의
  //   결함 셋**을 잡았다. 그 전 숫자들에는 테스트가 스스로 만든 허수가 섞여 있었다:
  //     - 자차 d를 트랙 폭과 무관하게 뽑아, 차가 이미 벽에 박힌 상태로 시작하는 배치를
  //       만들었다. 플래너는 첫 waypoint에서 옳게 기각하는데 그것을 결함으로 셌다.
  //     - 그 제약을 최근접 waypoint 하나로만 걸어, 발자국이 뻗는 ±0.28 m 안의 더 좁은
  //       지점에서 같은 문제가 남았다.
  //     - 판정자의 장애물 여유가 vehicle_half_width_m + safety_margin_m(0.158)뿐이어서
  //       플래너보다 약 0.26 m 관대했다. 플래너는 여기에 추종오차 예약을 더한다.
  //       판정자가 플래너보다 관대하면 멀쩡한 기각이 전부 결함으로 집계된다.
  //
  //   현재 사유 분포: NO_HARD_VALID_M1_CANDIDATE 95 / NO_VALID_SIDE_DOMAIN 3 /
  //                   BOUNDARY_HANDOFF_UNRESOLVED 1,  거짓 통과 0.
  //
  //   남은 실패의 지배적 원인은 진입 눈금이 아니라 **기동 계열의 경직성**이다:
  //     - 램프 형상이 대상 클러스터만 보고 정해진다. 전이 경로 위에 (클러스터가 아닌) 다른
  //       장애물이 놓이면 모든 후보가 그 상자와 교차해 죽는데, 형상을 바꿀 수단이 없다.
  //     - 도메인이 스팬 전체에서 유효한 하나의 상수 목표 d를 요구한다. 회랑이 스팬 안에서
  //       좁아지면 s에 따라 d가 변하는 경로가 존재해도 도메인이 통째로 무효가 된다.
  constexpr std::size_t kKnownFailureCeiling = 99U;
  EXPECT_LE(failed, kKnownFailureCeiling)
    << "보수 판정자가 통로를 찾은 배치 " << checked << "개 중 " << failed
    << "개에서 플래너가 실패했다 — 종전 " << kKnownFailureCeiling
    << "개보다 늘었다(회귀). RULE_PROPERTY_DUMP=1 로 상세 출력";
  if (failed < kKnownFailureCeiling) {
    std::printf(
      "\n  ✅ 실패가 %zu → %zu 로 줄었다. kKnownFailureCeiling 을 %zu 로 내릴 것.\n",
      kKnownFailureCeiling, failed, failed);
  }
}

// ── 목표 등급 ────────────────────────────────────────────────────────────────────────────
// 규정이 보장하는 최소 자유폭 0.5 m 까지 좁힌다. 현재 예약으로는 물리적으로 불가능하다
// (한쪽을 지나는 데 0.72~0.92 m 필요 — local_planning_sim.yaml의 08/13 주석 참고).
// 그래서 실패를 빌드 실패로 만들지 않고, **어느 자유폭부터 통과 가능한지**를 이분법으로
// 역산해 보고한다. 그 숫자가 MCL·제어 담당자에게 전달할 목표치다.
TEST(RuleProperty, ReportTheNarrowestGapWeCanActuallyPass)
{
  const auto & reference = raceReference();
  const auto parameters = raceParameters();

  double passable = 0.0;
  double blocked = 0.0;
  double wide_rate = 0.0;
  std::printf("자유폭별 통과 성적 (본선 3장애물, 표본 200):\n");
  for (const double width : {2.0, 1.6, 1.4, 1.2, 1.0, 0.8, 0.6, kMinFreeWidthM}) {
    const auto verdict = sweep(reference, parameters, kFinalsObstacles, width, 200U,
        20260817U, false);
    const double rate = verdict.mandatory_checked == 0U ? 0.0 :
      1.0 - static_cast<double>(verdict.mandatory_failed) /
      static_cast<double>(verdict.mandatory_checked);
    std::printf(
      "  자유폭 %.2f m: 통로 있는 배치 %3zu개 중 통과 %5.1f%%  (거짓통과 %zu)\n",
      width, verdict.mandatory_checked, 100.0 * rate, verdict.false_pass);
    if (width > 1.9) {wide_rate = rate;}
    if (verdict.mandatory_checked > 0U && verdict.mandatory_failed == 0U) {
      passable = width;
    } else if (verdict.mandatory_checked > 0U && blocked == 0.0) {
      blocked = width;
    }
    EXPECT_EQ(verdict.false_pass, 0U)
      << "자유폭 " << width << " m: 통로가 없는데 회피를 만들었다 — 충돌 위험";
  }
  // 어떤 자유폭에서도 100%가 아니므로 "완전 통과 폭"은 존재하지 않는다. 그것 자체가 결과다 —
  // 좁아질수록 통과율이 떨어지는 기울기가 곧 남은 작업량이다.
  std::printf(
    "\n  → 규정 보장 자유폭 %.2f m 에서의 통과율이 목표치다. 현재 어떤 폭에서도 100%%가\n"
    "     아니므로, 좁은 폭의 실패는 예약 부족과 후보 생성 결함이 섞여 있다.\n"
    "     넓은 폭(2.0 m)의 실패 %.1f%% 는 순수한 후보 생성 결함이다 — 거기가 우리 몫이다.\n",
    kMinFreeWidthM, 100.0 * (1.0 - wide_rate));
  (void)passable;
  (void)blocked;
  // 목표 등급은 빌드를 막지 않는다. 거짓 통과(위험한 방향)만 막는다.
  SUCCEED();
}

}  // namespace local_planning
