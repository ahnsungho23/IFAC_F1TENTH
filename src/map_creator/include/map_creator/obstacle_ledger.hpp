// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#ifndef MAP_CREATOR__OBSTACLE_LEDGER_HPP_
#define MAP_CREATOR__OBSTACLE_LEDGER_HPP_

#include <cstddef>
#include <optional>
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
  std::optional<int> missing_since_lap;
};

// Lap-scoped static obstacle ledger: accumulates /adaptive_obstacle_map snapshots during
// lap 1 and matches repeated entries by their P0-projected Frenet geometry (the node
// projects each Cartesian AABB onto the immutable P0 reference before updateSnapshot;
// this class itself performs no conversion). ROS-free (message structs only).
class ObstacleLedger
{
public:
  void setTrackLength(double track_length_m) {track_length_ = track_length_m;}
  void setMatchThresholds(double max_ds_m, double max_dd_m)
  {
    max_ds_ = max_ds_m;
    max_dd_ = max_dd_m;
  }

  // Synchronizes one authoritative /adaptive_obstacle_map snapshot received during `lap`.
  // An unmatched stored geometry starts the removal hysteresis; transport silence does not.
  void updateSnapshot(const std::vector<f110_msgs::msg::Obstacle> & obstacles, int lap);

  // Current persistent entries, including entries inside the removal-hysteresis window.
  std::vector<LedgerEntry> snapshot() const {return entries_;}

  // Entries absent from authoritative snapshots for `miss_laps` completed laps.
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
