// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__OBSTACLE_LEDGER_HPP_
#define MAP_CREATOR__OBSTACLE_LEDGER_HPP_

#include <cstddef>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>

namespace map_creator
{

struct LedgerEntry
{
  f110_msgs::msg::Obstacle obstacle;  // latest snapshot
  int first_seen_lap{0};
  int last_seen_lap{0};
  int observation_count{0};
};

// Lap-scoped static obstacle ledger: accumulates /static_obs observations during
// lap 1, matches repeated detections wrap-aware, and provides the frozen set the
// map_creator pipeline bakes. ROS-free (message structs only).
class ObstacleLedger
{
public:
  void setTrackLength(double track_length_m) {track_length_ = track_length_m;}
  void setMatchThresholds(double max_ds_m, double max_dd_m)
  {
    max_ds_ = max_ds_m;
    max_dd_ = max_dd_m;
  }

  // Adds all static obstacles from one /static_obs message observed during `lap`.
  void addObservations(const std::vector<f110_msgs::msg::Obstacle> & obstacles, int lap);

  // Entries observed at least `min_observations` times (the freeze snapshot).
  std::vector<LedgerEntry> confirmed(int min_observations) const;

  // Entries not seen for `miss_laps` completed laps (removal hysteresis).
  std::vector<std::size_t> removalCandidates(int current_lap, int miss_laps) const;
  void removeAt(const std::vector<std::size_t> & sorted_indices);

  bool empty() const {return entries_.empty();}
  std::size_t size() const {return entries_.size();}
  void clear() {entries_.clear();}

private:
  double wrapDelta(double a, double b) const;

  double track_length_{0.0};
  double max_ds_{1.0};
  double max_dd_{0.3};
  std::vector<LedgerEntry> entries_;
};

}  // namespace map_creator

#endif  // MAP_CREATOR__OBSTACLE_LEDGER_HPP_
