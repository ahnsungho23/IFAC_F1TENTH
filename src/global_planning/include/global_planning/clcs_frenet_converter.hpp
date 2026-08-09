#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "geometry/curvilinear_coordinate_system.h"

namespace global_planning
{

struct ReferenceWaypoint
{
  double x{0.0};
  double y{0.0};
  double s{0.0};
};

enum class VelocityFrame
{
  kMap,
  kBody
};

struct ClcsFrenetConfig
{
  bool closed_loop{true};
  double path_change_tolerance{1.0e-4};
  double duplicate_point_tolerance{1.0e-3};
  double min_path_length{0.5};
  double large_gap_factor{5.0};
  double max_projection_distance{20.0};
  double projection_domain_limit{20.0};
  double projection_domain_epsilon{0.1};
  double projection_domain_eps2{0.0};
  int projection_domain_method{1};
  double tangent_epsilon{0.05};
  bool publish_frenet_velocity{true};
  VelocityFrame velocity_frame{VelocityFrame::kBody};

  // --- Monotonic s-window tracking (convertTracked) --------------------------
  // Monotonic progress tracking for closed loops: after the
  // first fix only the arc slice [s_prev - backward_tolerance, s_prev +
  // forward_window] (modulo track length when closed_loop) is searched, so
  // reference branches that are close in Euclidean space but far in arc
  // length (hairpin opposite leg) can never capture the projection.
  double forward_window{1.0};        // W [m]
  double backward_tolerance{1.0};    // epsilon [m]
  // First fix: <= 0 searches the whole path (closed track may start
  // anywhere); > 0 restricts the first fix to [0, initial_seed_window]
  // (use only when the start region is known in advance).
  double initial_seed_window{0.0};
  // Euclidean gate applied to tracked fixes only (stateless convert() keeps
  // max_projection_distance). Must exceed the largest legitimate |d|
  // (avoidance excursions, here ~1.1 m) and should stay below the smallest
  // branch gap (hairpin legs 1.41 m).
  double tracked_max_projection_distance{1.5};
  // A window miss fails closed. After this many consecutive misses one loud
  // global re-search re-acquires progress (teleport / 2D Pose Estimate).
  // 0 = never re-acquire (strict fail-closed).
  int reacquire_after_misses{15};
};

struct ClcsBuildStats
{
  std::uint64_t path_version{0};
  std::size_t input_waypoint_count{0};
  std::size_t reference_point_count{0};
  std::size_t removed_duplicate_count{0};
  std::size_t invalid_point_count{0};
  std::size_t large_gap_count{0};
  std::size_t self_intersection_count{0};
  double track_length{0.0};
  double waypoint_s_max_error{0.0};
  double build_time_ms{0.0};
  bool closed_by_appending_first_point{false};
};

struct ClcsConversionInput
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double linear_x{0.0};
  double linear_y{0.0};
  double yaw_rate{0.0};
};

// Monotonic-progress state for convertTracked(). Reset (value-initialize)
// whenever the reference path changes.
struct ClcsContinuityState
{
  bool initialized{false};
  double s_prev{0.0};          // normalized [0, track_length)
  int consecutive_misses{0};
};

struct ClcsConversionResult
{
  bool valid{false};
  // True when this fix came from the bounded global re-search after
  // reacquire_after_misses consecutive window misses (log it loudly).
  bool reacquired{false};
  int segment_index{-1};
  double s{0.0};
  double d{0.0};
  double raw_s{0.0};
  double reference_yaw{0.0};
  double heading_error{0.0};
  double v_s{0.0};
  double v_d{0.0};
  double yaw_rate{0.0};
  double track_length{0.0};
  double conversion_time_us{0.0};
  double clcs_build_time_ms{0.0};
  double waypoint_s_max_error{0.0};
  double reconstruction_error{0.0};
  std::uint64_t path_version{0};
  std::string error_message;
};

class ClcsFrenetConverter
{
public:
  using Ptr = std::shared_ptr<ClcsFrenetConverter>;
  using ConstPtr = std::shared_ptr<const ClcsFrenetConverter>;

  static Ptr create(
    const std::vector<ReferenceWaypoint> & waypoints,
    const ClcsFrenetConfig & config,
    std::uint64_t path_version);

  ClcsConversionResult convert(const ClcsConversionInput & input) const;

  // Monotonic s-window variant of convert() for the single continuously
  // moving ego pose stream. First fix: whole path (or [0,
  // initial_seed_window] when configured). Afterwards only segments whose
  // arc interval intersects [s_prev - backward_tolerance, s_prev +
  // forward_window] are searched, wrap-aware for closed loops. A window miss
  // (no candidate, or |d| beyond tracked_max_projection_distance) fails
  // closed — NO silent global fallback, that is exactly the branch flip this
  // exists to prevent — and only after reacquire_after_misses consecutive
  // misses does one global re-search run (result.reacquired = true). On
  // failure the state is untouched. Do not use for unrelated points
  // (obstacle projection etc.); use convert() there.
  ClcsConversionResult convertTracked(
    const ClcsConversionInput & input,
    ClcsContinuityState & state) const;

  const ClcsBuildStats & stats() const { return stats_; }
  const std::vector<ReferenceWaypoint> & source_waypoints() const { return source_waypoints_; }
  const std::vector<ReferenceWaypoint> & reference_points() const { return reference_points_; }

  static bool pathChanged(
    const std::vector<ReferenceWaypoint> & previous,
    const std::vector<ReferenceWaypoint> & current,
    double tolerance);

  static double normalizeAngle(double angle);

private:
  ClcsFrenetConverter(
    const ClcsFrenetConfig & config,
    const std::vector<ReferenceWaypoint> & source_waypoints,
    const std::vector<ReferenceWaypoint> & reference_points,
    std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs,
    const ClcsBuildStats & stats);

  static std::vector<ReferenceWaypoint> preprocessReferencePath(
    const std::vector<ReferenceWaypoint> & waypoints,
    const ClcsFrenetConfig & config,
    ClcsBuildStats & stats);

  static geometry::EigenPolyline toEigenPolyline(
    const std::vector<ReferenceWaypoint> & reference_points);

  static double computePathLength(const std::vector<ReferenceWaypoint> & points);
  static double computeWaypointSMaxError(const std::vector<ReferenceWaypoint> & points);
  static std::size_t countLargeGaps(
    const std::vector<ReferenceWaypoint> & points,
    double large_gap_factor);
  static std::size_t countSelfIntersections(
    const std::vector<ReferenceWaypoint> & points,
    bool closed_loop);

  double normalizeS(double s) const;
  double sForClcsQuery(double s) const;
  double referenceYaw(double raw_s) const;

  // Prologue shared by convert()/convertTracked(): statistics fields.
  ClcsConversionResult newResult() const;

  // Shared tail of convert()/convertTracked(): everything computable once
  // raw_s/d/segment_index are known (s normalization, projection-distance
  // check, reference yaw, heading error, velocities, reconstruction, timing,
  // validity). Fills result in place.
  void finishConversion(
    const ClcsConversionInput & input,
    const std::chrono::steady_clock::time_point & start,
    double raw_s,
    double d,
    int segment_index,
    ClcsConversionResult & result) const;

  // Nearest in-window projection: only segments whose raw arc interval
  // intersects [s_lo, s_hi] are considered (the interval may extend beyond
  // [0, L); for closed loops the +-L shifted copies are tested too). The
  // accepted candidate's own s must also fall inside the window. Returns
  // false when no in-window segment claims the point.
  bool projectInWindow(
    double x,
    double y,
    double s_lo,
    double s_hi,
    double & raw_s,
    double & d,
    int & segment_index) const;

  ClcsFrenetConfig config_;
  std::vector<ReferenceWaypoint> source_waypoints_;
  std::vector<ReferenceWaypoint> reference_points_;
  std::shared_ptr<geometry::CurvilinearCoordinateSystem> clcs_;
  ClcsBuildStats stats_;
};

VelocityFrame parseVelocityFrame(const std::string & value);
std::string toString(VelocityFrame value);

}  // namespace global_planning
