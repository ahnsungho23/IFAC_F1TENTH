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

  const auto evaluation =
    planner_.evaluateObstacleScenario(ego, {obstacle});

  decision.left = evaluation.left;
  decision.right = evaluation.right;
  decision.reason = evaluation.result.reason;
  if (evaluation.result.kind == local_planning::SplinePlanKind::kAvoidance) {
    decision.side =
      evaluation.result.go_left ? SideDecision::Side::kLeft : SideDecision::Side::kRight;
    decision.target_d = evaluation.result.target_d;
  } else {
    decision.side = SideDecision::Side::kSafeStop;
  }
  return decision;
}

}  // namespace map_creator
