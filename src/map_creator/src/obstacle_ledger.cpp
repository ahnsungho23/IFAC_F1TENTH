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

void ObstacleLedger::updateSnapshot(
  const std::vector<f110_msgs::msg::Obstacle> & obstacles, int lap)
{
  std::vector<bool> present(entries_.size(), false);
  for (const auto & obstacle : obstacles) {
    if (!obstacle.is_static) {
      continue;
    }
    std::optional<std::size_t> match;
    for (std::size_t index = 0; index < entries_.size(); ++index) {
      if (present[index]) {
        continue;
      }
      const auto & entry = entries_[index];
      const double ds = std::abs(wrapDelta(obstacle.s_center, entry.obstacle.s_center));
      const double dd = std::abs(obstacle.d_center - entry.obstacle.d_center);
      if (ds < max_ds_ && dd < max_dd_) {
        match = index;
        break;
      }
    }
    if (match.has_value()) {
      auto & entry = entries_[*match];
      entry.obstacle = obstacle;
      entry.last_seen_lap = lap;
      entry.missing_since_lap.reset();
      ++entry.observation_count;
      present[*match] = true;
    } else {
      LedgerEntry entry;
      entry.obstacle = obstacle;
      entry.first_seen_lap = lap;
      entry.last_seen_lap = lap;
      entry.observation_count = 1;
      entries_.push_back(entry);
      present.push_back(true);
    }
  }

  for (std::size_t index = 0; index < entries_.size(); ++index) {
    if (!present[index] && !entries_[index].missing_since_lap.has_value()) {
      entries_[index].missing_since_lap = lap;
    }
  }
}

std::vector<std::size_t> ObstacleLedger::removalCandidates(
  int current_lap, int miss_laps) const
{
  std::vector<std::size_t> out;
  for (std::size_t i = 0; i < entries_.size(); ++i) {
    const auto & missing_since = entries_[i].missing_since_lap;
    if (missing_since.has_value() && current_lap - *missing_since >= miss_laps) {
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
