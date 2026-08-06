// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.

#include "map_creator/obstacle_ledger.hpp"

#include <algorithm>
#include <cmath>

namespace map_creator
{

double ObstacleLedger::wrapDelta(double a, double b) const
{
  double delta = a - b;
  if (track_length_ > 0.0) {
    while (delta > 0.5 * track_length_) {delta -= track_length_;}
    while (delta < -0.5 * track_length_) {delta += track_length_;}
  }
  return delta;
}

void ObstacleLedger::addObservations(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles, int lap)
{
  for (const auto & obstacle : obstacles) {
    if (!obstacle.is_static) {
      continue;
    }
    LedgerEntry * match = nullptr;
    for (auto & entry : entries_) {
      const double ds = std::abs(wrapDelta(obstacle.s_center, entry.obstacle.s_center));
      const double dd = std::abs(obstacle.d_center - entry.obstacle.d_center);
      if (ds < max_ds_ && dd < max_dd_) {
        match = &entry;
        break;
      }
    }
    if (match != nullptr) {
      match->obstacle = obstacle;
      match->last_seen_lap = lap;
      ++match->observation_count;
    } else {
      LedgerEntry entry;
      entry.obstacle = obstacle;
      entry.first_seen_lap = lap;
      entry.last_seen_lap = lap;
      entry.observation_count = 1;
      entries_.push_back(entry);
    }
  }
}

std::vector<LedgerEntry> ObstacleLedger::confirmed(int min_observations) const
{
  std::vector<LedgerEntry> out;
  for (const auto & entry : entries_) {
    if (entry.observation_count >= min_observations) {
      out.push_back(entry);
    }
  }
  return out;
}

std::vector<std::size_t> ObstacleLedger::removalCandidates(
  int current_lap, int miss_laps) const
{
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    if (current_lap - entries_[i].last_seen_lap >= miss_laps) {
      out.push_back(i);
    }
  }
  return out;
}

void ObstacleLedger::removeAt(const std::vector<std::size_t> & sorted_indices)
{
  for (auto it = sorted_indices.rbegin(); it != sorted_indices.rend(); ++it) {
    if (*it < entries_.size()) {
      entries_.erase(entries_.begin() + static_cast<std::ptrdiff_t>(*it));
    }
  }
}

}  // namespace map_creator
