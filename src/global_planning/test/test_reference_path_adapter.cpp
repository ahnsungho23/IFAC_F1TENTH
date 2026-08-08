#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "global_planning/reference_path_adapter.hpp"

namespace
{

using global_planning::AdapterWaypoint;
using global_planning::ReferencePathAdapterConfig;
using global_planning::adaptReferencePath;

constexpr double kPi = 3.14159265358979323846;

// CCW stadium (two straights + two semicircular caps), caps centered at
// (+-straight/2, 0). CCW means the interior is on the left of the direction
// of travel, so d_left is the inner-bound distance at the caps.
std::vector<AdapterWaypoint> makeStadium(
  const double straight,
  const double radius,
  const double spacing,
  const double d_left,
  const double d_right)
{
  std::vector<AdapterWaypoint> out;
  const double half = straight / 2.0;
  const auto add = [&out, d_left, d_right](const double x, const double y) {
      out.push_back({x, y, d_left, d_right});
    };
  const int n_straight = std::max(2, static_cast<int>(std::round(straight / spacing)));
  const int n_cap = std::max(4, static_cast<int>(std::round(kPi * radius / spacing)));

  for (int i = 0; i < n_straight; ++i) {  // bottom straight, heading +x
    add(-half + straight * i / n_straight, -radius);
  }
  for (int i = 0; i < n_cap; ++i) {  // right cap, -90 deg -> +90 deg
    const double theta = -kPi / 2.0 + kPi * i / n_cap;
    add(half + radius * std::cos(theta), radius * std::sin(theta));
  }
  for (int i = 0; i < n_straight; ++i) {  // top straight, heading -x
    add(half - straight * i / n_straight, radius);
  }
  for (int i = 0; i < n_cap; ++i) {  // left cap, +90 deg -> +270 deg
    const double theta = kPi / 2.0 + kPi * i / n_cap;
    add(-half + radius * std::cos(theta), radius * std::sin(theta));
  }
  return out;
}

double closedLength(const std::vector<global_planning::ReferenceWaypoint> & path)
{
  double length = 0.0;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto & a = path[i];
    const auto & b = path[(i + 1) % path.size()];
    length += std::hypot(b.x - a.x, b.y - a.y);
  }
  return length;
}

}  // namespace

TEST(ReferencePathAdapter, DisabledReturnsInputUnmodified)
{
  const auto input = makeStadium(6.0, 1.0, 0.2, 0.5, 0.5);
  ReferencePathAdapterConfig config;  // both flags false, resample_step 0

  const auto result = adaptReferencePath(input, config);

  EXPECT_FALSE(result.modified);
  EXPECT_EQ(result.stop_reason, "disabled");
  ASSERT_EQ(result.path.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(result.path[i].x, input[i].x, 1e-12);
    EXPECT_NEAR(result.path[i].y, input[i].y, 1e-12);
  }
}

TEST(ReferencePathAdapter, SmoothingSubdividesClosedSquare)
{
  // Unit square, one refinement: cubic masks (1,6,1)/8 and (4,4)/8 with
  // wrap-around. Expected points computed by hand.
  const std::vector<AdapterWaypoint> square = {
    {0.0, 0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0, 0.0}, {0.0, 1.0, 0.0, 0.0}};
  ReferencePathAdapterConfig config;
  config.enable_smoothing = true;
  config.subdivision_refinements = 1;

  const auto result = adaptReferencePath(square, config);

  EXPECT_TRUE(result.modified);
  EXPECT_EQ(result.stop_reason, "smoothing_only");
  ASSERT_EQ(result.path.size(), 8u);
  const std::vector<std::pair<double, double>> expected = {
    {0.125, 0.125}, {0.5, 0.0}, {0.875, 0.125}, {1.0, 0.5},
    {0.875, 0.875}, {0.5, 1.0}, {0.125, 0.875}, {0.0, 0.5}};
  for (std::size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(result.path[i].x, expected[i].first, 1e-12) << "point " << i;
    EXPECT_NEAR(result.path[i].y, expected[i].second, 1e-12) << "point " << i;
  }
}

TEST(ReferencePathAdapter, AlreadySatisfiedKeepsPathUntouched)
{
  // Cap radius 1.5 m, inner bound 0.6 m: rho ~ (1/1.5)*(0.6+0.05) = 0.43 < 1.
  const auto input = makeStadium(6.0, 1.5, 0.2, 0.6, 0.6);
  ReferencePathAdapterConfig config;
  config.enable_curvature_reduction = true;

  const auto result = adaptReferencePath(input, config);

  EXPECT_FALSE(result.modified);
  EXPECT_EQ(result.stop_reason, "already_satisfied");
  EXPECT_LT(result.initial_max_rho, 1.0);
  EXPECT_GT(result.initial_max_rho, 0.0);
  ASSERT_EQ(result.path.size(), input.size());
  for (std::size_t i = 0; i < input.size(); ++i) {
    EXPECT_NEAR(result.path[i].x, input[i].x, 1e-12);
    EXPECT_NEAR(result.path[i].y, input[i].y, 1e-12);
  }
}

TEST(ReferencePathAdapter, CurvatureReductionMeetsCriterion)
{
  // Cap radius 0.5 m (kappa = 2), inner bound 0.4 m, margin 0.15 m:
  // rho ~ 2*(0.4+0.15) = 1.1 > 1 -> the loop must flatten the caps until
  // the osculating radius clears the inner bound plus margin.
  const auto input = makeStadium(6.0, 0.5, 0.1, 0.4, 0.4);
  ReferencePathAdapterConfig config;
  config.enable_curvature_reduction = true;
  config.boundary_margin = 0.15;
  config.resample_step = 0.25;

  const auto result = adaptReferencePath(input, config);

  EXPECT_TRUE(result.modified);
  EXPECT_EQ(result.stop_reason, "criterion_met");
  EXPECT_GT(result.initial_max_rho, 1.0);
  EXPECT_LT(result.final_max_rho, 1.0);
  EXPECT_GE(result.iterations_used, 1);

  // What Alg. 1 guarantees is rho < 1 (unique projection over the corridor)
  // with the path inside the bounds — asserted above and below. The exact
  // curvature profile is NOT guaranteed: per the paper's own convergence
  // argument (Thm. 1) the path moves toward the inner bound each iteration,
  // so |kappa| may locally rise while rho still drops. Only a loose global
  // sanity bound is checked here.
  EXPECT_LT(result.final_max_abs_curvature, 6.0);

  // The adapted path must stay clear of the bounds (0.4 m corridor).
  for (const auto & point : result.path) {
    double nearest = 1e9;
    for (std::size_t i = 0; i < input.size(); ++i) {
      const auto & a = input[i];
      const auto & b = input[(i + 1) % input.size()];
      const double ex = b.x - a.x;
      const double ey = b.y - a.y;
      const double len2 = ex * ex + ey * ey;
      const double t = len2 < 1e-12 ? 0.0 :
        std::clamp(((point.x - a.x) * ex + (point.y - a.y) * ey) / len2, 0.0, 1.0);
      nearest = std::min(
        nearest, std::hypot(point.x - (a.x + t * ex), point.y - (a.y + t * ey)));
    }
    EXPECT_LT(nearest, 0.35) << "adapted point strayed toward a track bound";
  }
}

TEST(ReferencePathAdapter, IterationCapReportsEarlyStop)
{
  // Far inner bound (0.7 m) makes rho ~ 2*0.75 = 1.5; a single weak
  // iteration (k=1) cannot reach rho < 1, so the cap must fire.
  const auto input = makeStadium(6.0, 0.5, 0.1, 0.7, 0.7);
  ReferencePathAdapterConfig config;
  config.enable_curvature_reduction = true;
  config.subdivision_refinements = 1;
  config.max_iterations = 1;

  const auto result = adaptReferencePath(input, config);

  EXPECT_TRUE(result.modified);
  EXPECT_EQ(result.stop_reason, "max_iterations");
  EXPECT_EQ(result.iterations_used, 1);
  EXPECT_GT(result.initial_max_rho, 1.0);
}

TEST(ReferencePathAdapter, DegenerateBoundsSkipsReduction)
{
  const auto input = makeStadium(6.0, 0.5, 0.1, 0.0, 0.0);

  ReferencePathAdapterConfig reduction_only;
  reduction_only.enable_curvature_reduction = true;
  const auto skipped = adaptReferencePath(input, reduction_only);
  EXPECT_FALSE(skipped.modified);
  EXPECT_EQ(skipped.stop_reason, "degenerate_bounds");

  ReferencePathAdapterConfig with_smoothing = reduction_only;
  with_smoothing.enable_smoothing = true;
  const auto smoothed = adaptReferencePath(input, with_smoothing);
  EXPECT_TRUE(smoothed.modified);
  EXPECT_EQ(smoothed.stop_reason, "degenerate_bounds");
}

TEST(ReferencePathAdapter, ResampleOnlyUniformSpacing)
{
  const auto input = makeStadium(6.0, 1.0, 0.13, 0.5, 0.5);
  ReferencePathAdapterConfig config;
  config.resample_step = 0.5;

  const auto result = adaptReferencePath(input, config);

  EXPECT_TRUE(result.modified);
  EXPECT_EQ(result.stop_reason, "resample_only");

  std::vector<AdapterWaypoint> raw_input = input;
  double input_length = 0.0;
  for (std::size_t i = 0; i < raw_input.size(); ++i) {
    const auto & a = raw_input[i];
    const auto & b = raw_input[(i + 1) % raw_input.size()];
    input_length += std::hypot(b.x - a.x, b.y - a.y);
  }
  EXPECT_NEAR(closedLength(result.path), input_length, 0.02 * input_length);

  const double step = closedLength(result.path) / static_cast<double>(result.path.size());
  for (std::size_t i = 0; i + 1 < result.path.size(); ++i) {
    const double gap = std::hypot(
      result.path[i + 1].x - result.path[i].x,
      result.path[i + 1].y - result.path[i].y);
    EXPECT_NEAR(gap, step, 0.02 * step) << "gap " << i;
  }
}

TEST(ReferencePathAdapter, AdaptedSIsCumulativeAndMonotonic)
{
  const auto input = makeStadium(6.0, 0.5, 0.1, 0.4, 0.4);
  ReferencePathAdapterConfig config;
  config.enable_curvature_reduction = true;
  config.boundary_margin = 0.15;
  config.resample_step = 0.25;

  const auto result = adaptReferencePath(input, config);

  ASSERT_TRUE(result.modified);
  ASSERT_GE(result.path.size(), 4u);
  EXPECT_NEAR(result.path.front().s, 0.0, 1e-12);
  for (std::size_t i = 1; i < result.path.size(); ++i) {
    EXPECT_GT(result.path[i].s, result.path[i - 1].s);
    const double gap = std::hypot(
      result.path[i].x - result.path[i - 1].x,
      result.path[i].y - result.path[i - 1].y);
    EXPECT_NEAR(result.path[i].s - result.path[i - 1].s, gap, 1e-9);
  }
}

TEST(ReferencePathAdapter, TooFewPointsIsRejected)
{
  const std::vector<AdapterWaypoint> triangle = {
    {0.0, 0.0, 0.5, 0.5}, {1.0, 0.0, 0.5, 0.5}, {0.5, 1.0, 0.5, 0.5}};
  ReferencePathAdapterConfig config;
  config.enable_curvature_reduction = true;

  const auto result = adaptReferencePath(triangle, config);

  EXPECT_FALSE(result.modified);
  EXPECT_EQ(result.stop_reason, "too_few_points");
}
