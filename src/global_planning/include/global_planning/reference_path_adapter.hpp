#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "global_planning/clcs_frenet_converter.hpp"

namespace global_planning
{

// Input waypoint for reference path adaptation: raceline point plus lateral
// distances to the track bounds, measured along the path normal (left = +n).
struct AdapterWaypoint
{
  double x{0.0};
  double y{0.0};
  double d_left{0.0};
  double d_right{0.0};
};

// Configuration for adaptReferencePath(). C++ port of Alg. 1 in Würsching &
// Althoff, "Robust and Efficient Curvilinear Coordinate Transformation with
// Guaranteed Map Coverage for Motion Planning" (IEEE IV 2024), adapted for
// closed racetrack loops:
//  - no lanelets: the inner boundary of each bend is built directly from the
//    per-waypoint d_left/d_right instead of lanelet adjacency traversal;
//  - the termination criterion rho = |kappa| * (inner boundary distance +
//    boundary_margin) < 1 (paper eq. (4)/(6)) is checked pointwise; the
//    paper's per-partition constant cap kappa_Gm = 1/dist(worst point) is
//    NOT enforced, so curvature may redistribute toward low-clearance spots
//    (e.g. bend/straight junctions) while rho < 1 — the unique-projection
//    guarantee — still holds everywhere. Use max_absolute_curvature for an
//    additional global cap when that redistribution is unwanted;
//  - subdivision, curvature, and resampling are wrap-aware (closed loop),
//    there are no fixed endpoints.
struct ReferencePathAdapterConfig
{
  // Lemma 1 only: one cubic subdivision pass (plus optional resampling).
  bool enable_smoothing{false};
  // Full Alg. 1 loop: subdivide -> check rho -> resample -> boundary guard.
  bool enable_curvature_reduction{false};
  // k: Lane-Riesenfeld cubic subdivision refinements per iteration (paper: 5).
  int subdivision_refinements{5};
  // n_iter: cap for the subdivide/resample loop.
  int max_iterations{10};
  // Delta s for the resampling step; <= 0 uses the median input spacing.
  double resample_step{0.0};
  // epsilon: safety margin added to the measured boundary distance (paper
  // over-approximates the distance so the curvature cap stays conservative).
  double boundary_margin{0.05};
  // Global |kappa| cap (paper's kappa_bar); <= 0 disables the cap.
  double max_absolute_curvature{0.0};
  // |kappa| below this counts as "straight"; the middle of every straight run
  // becomes a fixed anchor. The loop is split at the anchors and each bend is
  // subdivided/resampled as an open segment with fixed end points — the
  // closed-loop equivalent of the paper's partitions (Lemma 3 needs fixed
  // ends; without them convex bends contract instead of flattening).
  double anchor_curvature_threshold{0.1};
  double duplicate_point_tolerance{1.0e-3};
};

struct ReferencePathAdapterResult
{
  // Adapted closed polyline without a duplicated closing point; s is the
  // cumulative arc length from the first point (ready for CLCS create()).
  std::vector<ReferenceWaypoint> path;
  bool modified{false};
  int iterations_used{0};
  std::size_t input_point_count{0};
  std::size_t output_point_count{0};
  double initial_max_abs_curvature{0.0};
  double final_max_abs_curvature{0.0};
  // max over all points of rho = |kappa| * (inner boundary distance + margin).
  double initial_max_rho{0.0};
  double final_max_rho{0.0};
  // "disabled" | "too_few_points" | "already_satisfied" | "criterion_met" |
  // "max_iterations" | "boundary_hit" | "smoothing_only" | "resample_only" |
  // "degenerate_bounds"
  std::string stop_reason{"disabled"};
};

// Adapt a closed reference path so every drivable point projects uniquely
// onto it (curvature-singularity type only; see the paper). The input order
// defines the driving direction; the path is treated as closed everywhere.
ReferencePathAdapterResult adaptReferencePath(
  const std::vector<AdapterWaypoint> & input,
  const ReferencePathAdapterConfig & config);

}  // namespace global_planning
