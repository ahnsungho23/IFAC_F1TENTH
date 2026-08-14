// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__SIDE_PLANNER_ADAPTER_HPP_
#define MAP_CREATOR__SIDE_PLANNER_ADAPTER_HPP_

#include <string>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <local_planning/raceline_spline_planner.hpp>

namespace map_creator
{

struct SideDecision
{
  enum class Side {kLeft, kRight, kSafeStop};
  Side side{Side::kSafeStop};
  double target_d{0.0};
  std::string reason;
};

// Wraps a RacelineSplinePlanner instance configured with map_creator's OWN
// decision parameter snapshot (which intentionally differs from the runtime
// local_planning yaml) and runs the shared plan() entry point with the fixed
// offline ego protocol (s = obstacle.s - lookback, d = 0, speed = reference vx
// at ego s). No Python replication, no guard inflation.
class SidePlannerAdapter
{
public:
  explicit SidePlannerAdapter(local_planning::RacelineSplineParameters parameters);

  bool setReference(const f110_msgs::msg::WpntArray & reference, std::string * error);
  bool ready() const {return planner_.ready();}
  double trackLength() const {return planner_.trackLength();}
  const local_planning::RacelineSplinePlanner & planner() const {return planner_;}

  SideDecision decide(const f110_msgs::msg::Obstacle & obstacle, double lookback_m) const;

private:
  double speedAtS(double s) const;

  local_planning::RacelineSplinePlanner planner_;
  f110_msgs::msg::WpntArray reference_;
};

}  // namespace map_creator

#endif  // MAP_CREATOR__SIDE_PLANNER_ADAPTER_HPP_
