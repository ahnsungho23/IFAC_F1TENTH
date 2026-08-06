#ifndef STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_MEMORY_HPP_
#define STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_MEMORY_HPP_

#include <cstddef>
#include <optional>
#include <vector>

#include <f110_msgs/msg/obstacle.hpp>
#include <f110_msgs/msg/obstacle_array.hpp>

namespace static_obstacle_map
{

struct MemoryConfig
{
  // Maximum edge-to-edge AABB gap for treating a new confirmed observation as an update.
  double association_distance_m{0.30};
  // A new outward edge must remain consistent for this many associated observations before the
  // persistent AABB grows to include it. The stored AABB never shrinks during a run.
  int edge_confirm_frames{3};
  // Maximum difference between consecutive observations supporting the same outward edge.
  double edge_match_tolerance_m{0.05};
  // Hard limit on the stored raw map-frame AABB diagonal. <= 0 disables the limit.
  double max_obstacle_diagonal_m{0.80};
  bool remove_reclassified_dynamic{true};
};

struct PendingBoundary
{
  double value{0.0};
  int consecutive_hits{0};
  bool active{false};
};

struct StoredObstacle
{
  int memory_id{0};
  int source_id{0};
  double x_min{0.0};
  double x_max{0.0};
  double y_min{0.0};
  double y_max{0.0};
  std::size_t update_count{0};
  PendingBoundary pending_x_min;
  PendingBoundary pending_x_max;
  PendingBoundary pending_y_min;
  PendingBoundary pending_y_max;
};

struct MemoryUpdateStats
{
  std::size_t inserted{0};
  std::size_t updated{0};
  std::size_t rejected{0};
  std::size_t removed{0};
  std::size_t geometry_expanded{0};
  std::size_t limit_rejected{0};
};

class StaticObstacleMapMemory
{
public:
  explicit StaticObstacleMapMemory(const MemoryConfig & config = MemoryConfig{});

  void configure(const MemoryConfig & config);
  MemoryUpdateStats updateConfirmed(const f110_msgs::msg::ObstacleArray & message);
  MemoryUpdateStats removeDynamic(const f110_msgs::msg::ObstacleArray & message);
  std::size_t clear();

  const std::vector<StoredObstacle> & obstacles() const;
  const MemoryConfig & config() const;

private:
  enum class FusionResult
  {
    Unchanged,
    Expanded,
    RejectedByLimit,
  };

  struct Aabb
  {
    double x_min;
    double x_max;
    double y_min;
    double y_max;
  };

  static std::optional<Aabb> obstacleAabb(const f110_msgs::msg::Obstacle & obstacle);
  static double aabbGap(const StoredObstacle & stored, const Aabb & candidate);
  static double aabbDiagonal(const Aabb & aabb);
  static void resetPendingBoundary(PendingBoundary & pending);
  Aabb clampAabb(const Aabb & input) const;
  bool observeLowerExpansion(
    double observation, double stored_edge,
    PendingBoundary & pending, double & confirmed_edge) const;
  bool observeUpperExpansion(
    double observation, double stored_edge,
    PendingBoundary & pending, double & confirmed_edge) const;
  FusionResult fuseBoundedUnion(
    StoredObstacle & stored, const Aabb & candidate) const;
  std::optional<std::size_t> findAssociation(
    const Aabb & candidate, int source_id,
    const std::vector<bool> & already_matched) const;

  MemoryConfig config_;
  std::vector<StoredObstacle> obstacles_;
  int next_memory_id_{0};
};

}  // namespace static_obstacle_map

#endif  // STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_MEMORY_HPP_
