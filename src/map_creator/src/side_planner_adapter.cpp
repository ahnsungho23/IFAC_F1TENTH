// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include "map_creator/side_planner_adapter.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace map_creator
{

SidePlannerAdapter::SidePlannerAdapter(local_planning::RacelineSplineParameters parameters)
: planner_(std::move(parameters))
{
}

bool SidePlannerAdapter::setReference(
  const f110_msgs::msg::WpntArray & reference, std::string * error)
{
  if (!planner_.setReference(reference, error)) {
    return false;
  }
  reference_ = reference;
  return true;
}

double SidePlannerAdapter::speedAtS(double s) const
{
  double best_speed = 0.0;
  double best_ds = std::numeric_limits<double>::infinity();
  const double track_length = planner_.trackLength();
  for (const auto & wpnt : reference_.wpnts) {
    double ds = std::abs(wpnt.s_m - s);
    if (track_length > 0.0) {
      ds = std::min(ds, track_length - ds);
    }
    if (ds < best_ds) {
      best_ds = ds;
      best_speed = wpnt.vx_mps;
    }
  }
  return best_speed;
}

SideDecision SidePlannerAdapter::decide(
  const f110_msgs::msg::Obstacle & obstacle, double lookback_m) const
{
  SideDecision decision;
  if (!planner_.ready()) {
    decision.reason = "planner reference not set";
    return decision;
  }

  local_planning::EgoFrenetState ego;
  const double track_length = planner_.trackLength();
  double ego_s = obstacle.s_center - lookback_m;
  if (track_length > 0.0) {
    while (ego_s < 0.0) {ego_s += track_length;}
    while (ego_s >= track_length) {ego_s -= track_length;}
  }
  ego.s = ego_s;
  ego.d = 0.0;
  ego.speed = speedAtS(ego_s);

  // 후보 생성기는 local_planning 쪽에서 P3(analytic corridor) 하나로 통일됐다
  // (2026-08-15, P0 quintic 격자 삭제). plan()은 그 P3 생성기를 그대로 호출하고,
  // 실현 가능한 후보를 exit_reaches_next_obstacle -> velocity_loss -> safety slack ->
  // global 이탈량 순으로 정렬해 1위의 측을 돌려준다. 즉 이 어댑터는 여전히 런타임과
  // 같은 생성기·같은 하드 검증기·같은 순위 계약을 쓴다.
  //
  // ⚠️ 오프라인 프로토콜은 장애물을 하나씩 넘기므로 최우선 정렬항인
  // exit_reaches_next_obstacle은 항상 false로 무력하다. 그 뒤 항(velocity_loss)이
  // 사실상 1순위이며, 런타임(군집 단위 판정)과 갈릴 수 있는 유일한 지점이다.
  const auto result = planner_.plan(ego, {obstacle});

  decision.reason = result.reason;
  switch (result.kind) {
    case local_planning::SplinePlanKind::kAvoidance:
      // margin slow pass도 여기로 온다. 그 경로는 라인 위(target_d = 0)에 그대로
      // 머물고 go_left는 "레이스라인이 장애물의 어느 쪽으로 지나가는가"만 기록한다
      // (raceline_spline_planner.cpp buildMarginSlowPass: go_left = raw_d_sum < 0).
      // 굽지 않을 쪽을 벽으로 칠하는 목적에는 그 값이 그대로 정답이다.
      decision.side =
        result.go_left ? SideDecision::Side::kLeft : SideDecision::Side::kRight;
      decision.target_d = result.target_d;
      break;
    case local_planning::SplinePlanKind::kNoObstacle:
      // plan()은 여전히 isBlockingRaceline으로 군집을 고르므로(nearestCluster),
      // 레이스라인이 이미 비켜 가는 장애물은 spline 평가 자체가 없다. 그럴 때는
      // 장애물의 횡부호를 그대로 쓴다: Frenet d > 0이면 라인 왼쪽에 있는 장애물이니
      // 라인은 그 오른쪽으로 지나간다.
      decision.side = obstacle.d_center < 0.0 ?
        SideDecision::Side::kLeft : SideDecision::Side::kRight;
      decision.target_d = 0.0;
      break;
    case local_planning::SplinePlanKind::kPreparation:
    case local_planning::SplinePlanKind::kSafeStop:
    case local_planning::SplinePlanKind::kNoSafePath:
    default:
      // 양측 모두 하드 검증 실패. plan()이 사유에 P3 실패 분류와 (해당되면)
      // "탈출 가능한 정지점 없음" 경고까지 담아 주므로 노드 로그에 그대로 남는다.
      decision.side = SideDecision::Side::kSafeStop;
      break;
  }
  return decision;
}

}  // namespace map_creator
