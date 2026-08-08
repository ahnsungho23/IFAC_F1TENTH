#include "static_obstacle_map/static_obstacle_map_memory.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace static_obstacle_map
{

StaticObstacleMapMemory::StaticObstacleMapMemory(const MemoryConfig & config)
{
  configure(config);
}

void StaticObstacleMapMemory::configure(const MemoryConfig & config)
{
  config_ = config;
  config_.association_distance_m = std::max(0.0, config_.association_distance_m);
  config_.edge_confirm_frames = std::max(1, config_.edge_confirm_frames);
  config_.edge_match_tolerance_m = std::max(0.0, config_.edge_match_tolerance_m);
  config_.max_obstacle_diagonal_m = std::max(0.0, config_.max_obstacle_diagonal_m);

  for (auto & obstacle : obstacles_) {
    const Aabb clamped = clampAabb(
      Aabb{obstacle.x_min, obstacle.x_max, obstacle.y_min, obstacle.y_max});
    obstacle.x_min = clamped.x_min;
    obstacle.x_max = clamped.x_max;
    obstacle.y_min = clamped.y_min;
    obstacle.y_max = clamped.y_max;
    resetPendingBoundary(obstacle.pending_x_min);
    resetPendingBoundary(obstacle.pending_x_max);
    resetPendingBoundary(obstacle.pending_y_min);
    resetPendingBoundary(obstacle.pending_y_max);
  }
}

std::optional<StaticObstacleMapMemory::Aabb> StaticObstacleMapMemory::obstacleAabb(
  const f110_msgs::msg::Obstacle & obstacle)
{
  if (!obstacle.is_static || !obstacle.is_visible || !obstacle.has_cartesian) {
    return std::nullopt;
  }
  if (!std::isfinite(obstacle.x_min) || !std::isfinite(obstacle.x_max) ||
    !std::isfinite(obstacle.y_min) || !std::isfinite(obstacle.y_max) ||
    obstacle.x_min > obstacle.x_max || obstacle.y_min > obstacle.y_max)
  {
    return std::nullopt;
  }
  return Aabb{obstacle.x_min, obstacle.x_max, obstacle.y_min, obstacle.y_max};
}

StaticObstacleMapMemory::Aabb StaticObstacleMapMemory::clampAabb(const Aabb & input) const
{
  if (config_.max_obstacle_diagonal_m <= 0.0) {
    return input;
  }

  const double width = std::max(0.0, input.x_max - input.x_min);
  const double height = std::max(0.0, input.y_max - input.y_min);
  const double diagonal = std::hypot(width, height);
  if (diagonal <= config_.max_obstacle_diagonal_m || diagonal <= 1.0e-12) {
    return input;
  }

  const double scale = config_.max_obstacle_diagonal_m / diagonal;
  const double center_x = 0.5 * (input.x_min + input.x_max);
  const double center_y = 0.5 * (input.y_min + input.y_max);
  const double half_width = 0.5 * width * scale;
  const double half_height = 0.5 * height * scale;
  return Aabb{
    center_x - half_width, center_x + half_width,
    center_y - half_height, center_y + half_height};
}

double StaticObstacleMapMemory::aabbGap(
  const StoredObstacle & stored, const Aabb & candidate)
{
  const double gap_x = std::max(
    {0.0, stored.x_min - candidate.x_max, candidate.x_min - stored.x_max});
  const double gap_y = std::max(
    {0.0, stored.y_min - candidate.y_max, candidate.y_min - stored.y_max});
  return std::hypot(gap_x, gap_y);
}

double StaticObstacleMapMemory::aabbDiagonal(const Aabb & aabb)
{
  return std::hypot(aabb.x_max - aabb.x_min, aabb.y_max - aabb.y_min);
}

void StaticObstacleMapMemory::resetPendingBoundary(PendingBoundary & pending)
{
  pending = PendingBoundary{};
}

bool StaticObstacleMapMemory::observeLowerExpansion(
  double observation, double stored_edge,
  PendingBoundary & pending, double & confirmed_edge) const
{
  if (observation >= stored_edge) {
    resetPendingBoundary(pending);
    return false;
  }

  if (!pending.active ||
    std::abs(observation - pending.value) > config_.edge_match_tolerance_m)
  {
    pending.value = observation;
    pending.consecutive_hits = 1;
    pending.active = true;
  } else {
    pending.value = std::min(pending.value, observation);
    ++pending.consecutive_hits;
  }

  if (pending.consecutive_hits < config_.edge_confirm_frames) {
    return false;
  }
  confirmed_edge = pending.value;
  resetPendingBoundary(pending);
  return true;
}

bool StaticObstacleMapMemory::observeUpperExpansion(
  double observation, double stored_edge,
  PendingBoundary & pending, double & confirmed_edge) const
{
  if (observation <= stored_edge) {
    resetPendingBoundary(pending);
    return false;
  }

  if (!pending.active ||
    std::abs(observation - pending.value) > config_.edge_match_tolerance_m)
  {
    pending.value = observation;
    pending.consecutive_hits = 1;
    pending.active = true;
  } else {
    pending.value = std::max(pending.value, observation);
    ++pending.consecutive_hits;
  }

  if (pending.consecutive_hits < config_.edge_confirm_frames) {
    return false;
  }
  confirmed_edge = pending.value;
  resetPendingBoundary(pending);
  return true;
}

StaticObstacleMapMemory::FusionResult StaticObstacleMapMemory::fuseBoundedUnion(
  StoredObstacle & stored, const Aabb & candidate) const
{
  Aabb proposed{
    stored.x_min, stored.x_max, stored.y_min, stored.y_max};
  bool has_confirmed_expansion = false;
  double confirmed_edge = 0.0;

  if (observeLowerExpansion(
      candidate.x_min, stored.x_min, stored.pending_x_min, confirmed_edge))
  {
    proposed.x_min = std::min(proposed.x_min, confirmed_edge);
    has_confirmed_expansion = true;
  }
  if (observeUpperExpansion(
      candidate.x_max, stored.x_max, stored.pending_x_max, confirmed_edge))
  {
    proposed.x_max = std::max(proposed.x_max, confirmed_edge);
    has_confirmed_expansion = true;
  }
  if (observeLowerExpansion(
      candidate.y_min, stored.y_min, stored.pending_y_min, confirmed_edge))
  {
    proposed.y_min = std::min(proposed.y_min, confirmed_edge);
    has_confirmed_expansion = true;
  }
  if (observeUpperExpansion(
      candidate.y_max, stored.y_max, stored.pending_y_max, confirmed_edge))
  {
    proposed.y_max = std::max(proposed.y_max, confirmed_edge);
    has_confirmed_expansion = true;
  }

  if (!has_confirmed_expansion) {
    return FusionResult::Unchanged;
  }
  if (config_.max_obstacle_diagonal_m > 0.0 &&
    aabbDiagonal(proposed) > config_.max_obstacle_diagonal_m)
  {
    return FusionResult::RejectedByLimit;
  }

  stored.x_min = proposed.x_min;
  stored.x_max = proposed.x_max;
  stored.y_min = proposed.y_min;
  stored.y_max = proposed.y_max;
  return FusionResult::Expanded;
}

std::optional<std::size_t> StaticObstacleMapMemory::findAssociation(
  const Aabb & candidate, int source_id,
  const std::vector<bool> & already_matched) const
{
  std::optional<std::size_t> best_index;
  double best_score = std::numeric_limits<double>::max();
  for (std::size_t index = 0; index < obstacles_.size(); ++index) {
    if (already_matched[index]) {
      continue;
    }
    const double gap = aabbGap(obstacles_[index], candidate);
    if (gap > config_.association_distance_m) {
      continue;
    }

    const double stored_center_x = 0.5 * (obstacles_[index].x_min + obstacles_[index].x_max);
    const double stored_center_y = 0.5 * (obstacles_[index].y_min + obstacles_[index].y_max);
    const double candidate_center_x = 0.5 * (candidate.x_min + candidate.x_max);
    const double candidate_center_y = 0.5 * (candidate.y_min + candidate.y_max);
    const double center_distance = std::hypot(
      stored_center_x - candidate_center_x, stored_center_y - candidate_center_y);
    // Source ID is a tie-breaker only. Geometry remains authoritative across detector/CLCS resets.
    const double id_bonus =
      obstacles_[index].source_id == source_id ? config_.association_distance_m : 0.0;
    const double score = gap + 0.1 * center_distance - id_bonus;
    if (score < best_score) {
      best_score = score;
      best_index = index;
    }
  }
  return best_index;
}

MemoryUpdateStats StaticObstacleMapMemory::updateConfirmed(
  const f110_msgs::msg::ObstacleArray & message)
{
  MemoryUpdateStats stats;
  std::vector<bool> matched(obstacles_.size(), false);

  for (const auto & obstacle_message : message.obstacles) {
    const auto raw_aabb = obstacleAabb(obstacle_message);
    if (!raw_aabb.has_value()) {
      ++stats.rejected;
      continue;
    }
    const Aabb candidate = clampAabb(*raw_aabb);
    const auto match = findAssociation(candidate, obstacle_message.id, matched);
    if (!match.has_value()) {
      StoredObstacle stored;
      stored.memory_id = next_memory_id_++;
      stored.source_id = obstacle_message.id;
      stored.x_min = candidate.x_min;
      stored.x_max = candidate.x_max;
      stored.y_min = candidate.y_min;
      stored.y_max = candidate.y_max;
      stored.update_count = 1;
      obstacles_.push_back(stored);
      matched.push_back(true);
      ++stats.inserted;
      continue;
    }

    StoredObstacle & stored = obstacles_[*match];
    const FusionResult fusion = fuseBoundedUnion(stored, candidate);
    if (fusion == FusionResult::Expanded) {
      ++stats.geometry_expanded;
    } else if (fusion == FusionResult::RejectedByLimit) {
      ++stats.limit_rejected;
    }
    stored.source_id = obstacle_message.id;
    ++stored.update_count;
    matched[*match] = true;
    ++stats.updated;
  }
  return stats;
}

MemoryUpdateStats StaticObstacleMapMemory::removeDynamic(
  const f110_msgs::msg::ObstacleArray & message)
{
  MemoryUpdateStats stats;
  if (!config_.remove_reclassified_dynamic) {
    return stats;
  }

  for (const auto & dynamic : message.obstacles) {
    if (dynamic.is_static) {
      ++stats.rejected;
      continue;
    }
    const auto old_size = obstacles_.size();
    obstacles_.erase(
      std::remove_if(
        obstacles_.begin(), obstacles_.end(),
        [&dynamic](const StoredObstacle & stored) {
          return stored.source_id == dynamic.id;
        }),
      obstacles_.end());
    stats.removed += old_size - obstacles_.size();
  }
  return stats;
}

std::size_t StaticObstacleMapMemory::clear()
{
  const std::size_t count = obstacles_.size();
  obstacles_.clear();
  return count;
}

f110_msgs::msg::ObstacleArray StaticObstacleMapMemory::buildObstacleArray(
  const std_msgs::msg::Header & header) const
{
  f110_msgs::msg::ObstacleArray output;
  output.header = header;
  output.obstacles.reserve(obstacles_.size());
  for (const auto & stored : obstacles_) {
    f110_msgs::msg::Obstacle obstacle;
    obstacle.id = stored.memory_id;
    obstacle.has_cartesian = true;
    obstacle.x_min = stored.x_min;
    obstacle.x_max = stored.x_max;
    obstacle.y_min = stored.y_min;
    obstacle.y_max = stored.y_max;
    obstacle.x_center = 0.5 * (stored.x_min + stored.x_max);
    obstacle.y_center = 0.5 * (stored.y_min + stored.y_max);
    obstacle.radius = 0.5 * std::hypot(
      stored.x_max - stored.x_min, stored.y_max - stored.y_min);
    obstacle.size = 2.0 * obstacle.radius;
    obstacle.is_static = true;
    obstacle.is_visible = true;
    output.obstacles.push_back(obstacle);
  }
  return output;
}

const std::vector<StoredObstacle> & StaticObstacleMapMemory::obstacles() const
{
  return obstacles_;
}

const MemoryConfig & StaticObstacleMapMemory::config() const
{
  return config_;
}

}  // namespace static_obstacle_map
