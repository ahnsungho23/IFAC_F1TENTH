#include "global_planning/reference_path_adapter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include <Eigen/Core>

namespace global_planning
{
namespace
{

using Points = std::vector<Eigen::Vector2d>;

constexpr double kTinyCurvature = 1.0e-9;
constexpr double kTinyLength = 1.0e-12;
// Hard cap on the point count during subdivision (2^k growth backstop).
constexpr std::size_t kMaxSubdividedPoints = std::size_t{1} << 18;

std::size_t wrapIndex(const std::ptrdiff_t index, const std::size_t size)
{
  const auto n = static_cast<std::ptrdiff_t>(size);
  return static_cast<std::size_t>(((index % n) + n) % n);
}

double cross2(const Eigen::Vector2d & a, const Eigen::Vector2d & b)
{
  return a.x() * b.y() - a.y() * b.x();
}

std::vector<AdapterWaypoint> stripDuplicates(
  const std::vector<AdapterWaypoint> & input,
  const double tolerance)
{
  std::vector<AdapterWaypoint> out;
  out.reserve(input.size());
  for (const auto & waypoint : input) {
    if (!std::isfinite(waypoint.x) || !std::isfinite(waypoint.y)) {
      continue;
    }
    if (!out.empty() &&
      std::hypot(waypoint.x - out.back().x, waypoint.y - out.back().y) <= tolerance)
    {
      continue;
    }
    out.push_back(waypoint);
  }
  // Closed-loop input may duplicate the first point at the end; drop it so
  // every wrap-aware operation sees each vertex exactly once.
  if (out.size() >= 2 &&
    std::hypot(out.front().x - out.back().x, out.front().y - out.back().y) <= tolerance)
  {
    out.pop_back();
  }
  return out;
}

Points toPoints(const std::vector<AdapterWaypoint> & waypoints)
{
  Points pts;
  pts.reserve(waypoints.size());
  for (const auto & waypoint : waypoints) {
    pts.emplace_back(waypoint.x, waypoint.y);
  }
  return pts;
}

// One Lane-Riesenfeld cubic B-spline subdivision step for a closed polyline:
// vertex mask (1, 6, 1)/8 and edge mask (4, 4)/8, indices wrapping (Lemma 1).
Points subdivideClosedOnce(const Points & pts)
{
  const std::size_t n = pts.size();
  Points out;
  out.reserve(2 * n);
  for (std::size_t i = 0; i < n; ++i) {
    const auto & prev = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) - 1, n)];
    const auto & curr = pts[i];
    const auto & next = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) + 1, n)];
    out.emplace_back((prev + 6.0 * curr + next) / 8.0);
    out.emplace_back((curr + next) / 2.0);
  }
  return out;
}

Points subdivideClosed(Points pts, const int refinements)
{
  for (int i = 0; i < refinements; ++i) {
    if (pts.size() * 2 > kMaxSubdividedPoints) {
      break;
    }
    pts = subdivideClosedOnce(pts);
  }
  return pts;
}

// Open-segment variant with both end points (the anchors) pinned. Lemma 3's
// flattening argument needs fixed segment ends: without them a convex bend
// contracts toward its curvature center (curvature grows) instead of
// flattening toward its chord.
Points subdivideOpenOnce(const Points & pts)
{
  const std::size_t n = pts.size();
  Points out;
  out.reserve(2 * n);
  for (std::size_t i = 0; i < n; ++i) {
    if (i == 0 || i + 1 == n) {
      out.push_back(pts[i]);
    } else {
      out.emplace_back((pts[i - 1] + 6.0 * pts[i] + pts[i + 1]) / 8.0);
    }
    if (i + 1 < n) {
      out.emplace_back((pts[i] + pts[i + 1]) / 2.0);
    }
  }
  return out;
}

Points subdivideOpen(Points pts, const int refinements)
{
  for (int i = 0; i < refinements; ++i) {
    if (pts.size() * 2 > kMaxSubdividedPoints) {
      break;
    }
    pts = subdivideOpenOnce(pts);
  }
  return pts;
}

// Signed discrete (Menger) curvature per vertex, wrap-aware. Positive kappa
// bends left; the inner side of the bend is the side the curve bends toward.
std::vector<double> signedCurvature(const Points & pts)
{
  const std::size_t n = pts.size();
  std::vector<double> kappa(n, 0.0);
  if (n < 3) {
    return kappa;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const auto & prev = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) - 1, n)];
    const auto & curr = pts[i];
    const auto & next = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) + 1, n)];
    const Eigen::Vector2d v1 = curr - prev;
    const Eigen::Vector2d v2 = next - curr;
    const double denom = v1.norm() * v2.norm() * (next - prev).norm();
    if (denom < kTinyLength) {
      continue;
    }
    kappa[i] = 2.0 * cross2(v1, v2) / denom;
  }
  return kappa;
}

double closedLength(const Points & pts)
{
  double length = 0.0;
  const std::size_t n = pts.size();
  for (std::size_t i = 0; i < n; ++i) {
    length += (pts[(i + 1) % n] - pts[i]).norm();
  }
  return length;
}

double medianSpacing(const Points & pts)
{
  const std::size_t n = pts.size();
  std::vector<double> gaps;
  gaps.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    gaps.push_back((pts[(i + 1) % n] - pts[i]).norm());
  }
  std::nth_element(gaps.begin(), gaps.begin() + gaps.size() / 2, gaps.end());
  return gaps[gaps.size() / 2];
}

// Uniform arc-length resampling of a closed polyline. The first point stays
// anchored (closed-loop analogue of the paper keeping the end points fixed)
// and the step is adjusted so the loop closes exactly.
Points resampleClosed(const Points & pts, const double step)
{
  const double length = closedLength(pts);
  const auto count = static_cast<std::size_t>(
    std::max<double>(4.0, std::round(length / std::max(step, kTinyLength))));
  const double actual_step = length / static_cast<double>(count);

  Points out;
  out.reserve(count);
  out.push_back(pts.front());

  const std::size_t n = pts.size();
  std::size_t segment = 0;
  double segment_used = 0.0;
  for (std::size_t i = 1; i < count; ++i) {
    double remaining = actual_step;
    while (true) {
      const Eigen::Vector2d & a = pts[segment % n];
      const Eigen::Vector2d & b = pts[(segment + 1) % n];
      const double segment_length = (b - a).norm();
      const double available = segment_length - segment_used;
      if (remaining <= available || segment_length < kTinyLength) {
        segment_used += remaining;
        const double ratio = segment_length < kTinyLength ?
          0.0 : segment_used / segment_length;
        out.emplace_back(a + ratio * (b - a));
        break;
      }
      remaining -= available;
      segment_used = 0.0;
      ++segment;
    }
  }
  return out;
}

// Uniform arc-length resampling of an open segment; both end points stay
// pinned exactly (closed-loop equivalent of the paper "keeping the end points
// fixed" during the resampling step).
Points resampleOpen(const Points & pts, const double step)
{
  std::vector<double> cumulative(pts.size(), 0.0);
  for (std::size_t i = 1; i < pts.size(); ++i) {
    cumulative[i] = cumulative[i - 1] + (pts[i] - pts[i - 1]).norm();
  }
  const double length = cumulative.back();
  const auto count = static_cast<std::size_t>(
    std::max<double>(2.0, std::round(length / std::max(step, kTinyLength))));

  Points out;
  out.reserve(count + 1);
  out.push_back(pts.front());
  std::size_t j = 0;
  for (std::size_t i = 1; i < count; ++i) {
    const double target = length * static_cast<double>(i) / static_cast<double>(count);
    while (j + 2 < pts.size() && cumulative[j + 1] < target) {
      ++j;
    }
    const double segment_length = cumulative[j + 1] - cumulative[j];
    const double ratio = segment_length < kTinyLength ?
      0.0 : (target - cumulative[j]) / segment_length;
    out.emplace_back(pts[j] + ratio * (pts[j + 1] - pts[j]));
  }
  out.push_back(pts.back());
  return out;
}

// Anchors = middles of maximal runs with |kappa| below the threshold (the
// straights), wrap-aware. They play the role of the paper's partition
// boundaries: fixed points between which each bend is refined as an open
// segment. Fallback when fewer than two runs exist: the flattest point and
// its arc-length antipode.
std::vector<std::size_t> findAnchors(
  const Points & pts,
  const std::vector<double> & kappa,
  const double threshold)
{
  const std::size_t n = pts.size();
  std::vector<bool> flat(n, false);
  std::size_t flat_count = 0;
  for (std::size_t i = 0; i < n; ++i) {
    flat[i] = std::abs(kappa[i]) < threshold;
    flat_count += flat[i] ? 1u : 0u;
  }

  std::vector<std::size_t> anchors;
  if (flat_count == n) {
    anchors = {0, n / 2};
  } else if (flat_count > 0) {
    for (std::size_t i = 0; i < n; ++i) {
      if (!flat[i] || flat[wrapIndex(static_cast<std::ptrdiff_t>(i) - 1, n)]) {
        continue;  // not the start of a flat run
      }
      std::size_t run_length = 1;
      while (run_length < n && flat[(i + run_length) % n]) {
        ++run_length;
      }
      anchors.push_back((i + run_length / 2) % n);
    }
  }

  std::sort(anchors.begin(), anchors.end());
  anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
  if (anchors.size() < 2) {
    const std::size_t base = anchors.empty() ?
      static_cast<std::size_t>(std::distance(
        kappa.begin(),
        std::min_element(
          kappa.begin(), kappa.end(),
          [](const double a, const double b) {return std::abs(a) < std::abs(b);}))) :
      anchors.front();
    // Arc-length antipode of the base anchor.
    std::vector<double> cumulative(n + 1, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
      cumulative[i + 1] = cumulative[i] + (pts[(i + 1) % n] - pts[i]).norm();
    }
    const double total = cumulative[n];
    const double target = std::fmod(cumulative[base] + total / 2.0, total);
    std::size_t antipode = 0;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      double delta = std::abs(cumulative[i] - target);
      delta = std::min(delta, total - delta);
      if (delta < best) {
        best = delta;
        antipode = i;
      }
    }
    anchors = {std::min(base, antipode), std::max(base, antipode)};
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());
  }
  if (anchors.size() < 2) {
    anchors = {0, n / 2};  // n >= 4 is guaranteed by the caller
  }
  return anchors;
}

// Split the closed polyline at the anchors. Each segment contains both of its
// end anchors; consecutive segments share the boundary anchor.
std::vector<Points> splitAtAnchors(
  const Points & pts,
  const std::vector<std::size_t> & anchors)
{
  std::vector<Points> segments;
  const std::size_t n = pts.size();
  const std::size_t m = anchors.size();
  segments.reserve(m);
  for (std::size_t a = 0; a < m; ++a) {
    const std::size_t end = anchors[(a + 1) % m];
    Points segment;
    for (std::size_t i = anchors[a]; ; i = (i + 1) % n) {
      segment.push_back(pts[i]);
      if (i == end) {
        break;
      }
    }
    segments.push_back(std::move(segment));
  }
  return segments;
}

Points concatSegments(const std::vector<Points> & segments)
{
  Points out;
  for (const auto & segment : segments) {
    out.insert(out.end(), segment.begin(), segment.end() - 1);  // shared anchor
  }
  return out;
}

Eigen::Vector2d leftNormal(const Points & pts, const std::size_t i)
{
  const std::size_t n = pts.size();
  const auto & prev = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) - 1, n)];
  const auto & next = pts[wrapIndex(static_cast<std::ptrdiff_t>(i) + 1, n)];
  Eigen::Vector2d tangent = next - prev;
  const double norm = tangent.norm();
  if (norm < kTinyLength) {
    return {0.0, 0.0};
  }
  tangent /= norm;
  return {-tangent.y(), tangent.x()};
}

struct TrackBounds
{
  Points left;
  Points right;
};

TrackBounds buildBounds(const std::vector<AdapterWaypoint> & waypoints, const Points & pts)
{
  TrackBounds bounds;
  bounds.left.reserve(pts.size());
  bounds.right.reserve(pts.size());
  for (std::size_t i = 0; i < pts.size(); ++i) {
    const Eigen::Vector2d normal = leftNormal(pts, i);
    bounds.left.emplace_back(pts[i] + waypoints[i].d_left * normal);
    bounds.right.emplace_back(pts[i] - waypoints[i].d_right * normal);
  }
  return bounds;
}

// Smallest positive ray parameter t where origin + t * dir crosses the closed
// polyline; +inf when the ray misses every segment.
double rayHitDistance(
  const Eigen::Vector2d & origin,
  const Eigen::Vector2d & dir,
  const Points & poly)
{
  double best = std::numeric_limits<double>::infinity();
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = poly[i];
    const Eigen::Vector2d edge = poly[(i + 1) % n] - a;
    const double denom = cross2(dir, edge);
    if (std::abs(denom) < kTinyLength) {
      continue;
    }
    const Eigen::Vector2d r = a - origin;
    const double t = cross2(r, edge) / denom;
    const double u = cross2(r, dir) / denom;
    if (t > 1.0e-9 && u >= 0.0 && u <= 1.0) {
      best = std::min(best, t);
    }
  }
  return best;
}

double nearestDistance(const Eigen::Vector2d & point, const Points & poly)
{
  double best = std::numeric_limits<double>::infinity();
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = poly[i];
    const Eigen::Vector2d edge = poly[(i + 1) % n] - a;
    const double length2 = edge.squaredNorm();
    const double ratio = length2 < kTinyLength ?
      0.0 : std::clamp((point - a).dot(edge) / length2, 0.0, 1.0);
    best = std::min(best, (point - (a + ratio * edge)).norm());
  }
  return best;
}

struct RhoStats
{
  double max_rho{0.0};
  double max_abs_curvature{0.0};
};

// Pointwise evaluation of the paper's criterion: rho_i = |kappa_i| * (distance
// along the inner-side normal to the corresponding track bound + margin).
RhoStats evaluateRho(const Points & path, const TrackBounds & bounds, const double margin)
{
  RhoStats stats;
  const auto kappa = signedCurvature(path);
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double abs_kappa = std::abs(kappa[i]);
    stats.max_abs_curvature = std::max(stats.max_abs_curvature, abs_kappa);
    if (abs_kappa < kTinyCurvature) {
      continue;
    }
    const Eigen::Vector2d normal = leftNormal(path, i);
    if (normal.squaredNorm() < 0.5) {
      continue;
    }
    const Eigen::Vector2d dir = kappa[i] > 0.0 ? normal : Eigen::Vector2d(-normal);
    const Points & boundary = kappa[i] > 0.0 ? bounds.left : bounds.right;
    double dist = rayHitDistance(path[i], dir, boundary);
    if (!std::isfinite(dist)) {
      // Ray missed (boundary gap / degenerate offset): fall back to the
      // nearest Euclidean distance, which under-approximates the lateral
      // extent but keeps the criterion defined.
      dist = nearestDistance(path[i], boundary);
    }
    stats.max_rho = std::max(stats.max_rho, abs_kappa * (dist + margin));
  }
  return stats;
}

bool rangesOverlap(double a_min, double a_max, double b_min, double b_max)
{
  if (a_min > a_max) {
    std::swap(a_min, a_max);
  }
  if (b_min > b_max) {
    std::swap(b_min, b_max);
  }
  return std::max(a_min, b_min) <= std::min(a_max, b_max);
}

bool segmentsIntersect(
  const Eigen::Vector2d & a,
  const Eigen::Vector2d & b,
  const Eigen::Vector2d & c,
  const Eigen::Vector2d & d)
{
  if (!rangesOverlap(a.x(), b.x(), c.x(), d.x()) ||
    !rangesOverlap(a.y(), b.y(), c.y(), d.y()))
  {
    return false;
  }
  const double c1 = cross2(b - a, c - a);
  const double c2 = cross2(b - a, d - a);
  const double c3 = cross2(d - c, a - c);
  const double c4 = cross2(d - c, b - c);
  return (c1 * c2 < 0.0) && (c3 * c4 < 0.0);
}

// Paper Fig. 6: chords p_i -> p_(i+3) (p_B = 3) of the resampled control
// polyline must not cross a track bound; checked against both bounds because
// crossing either is fatal for a raceline.
bool chordsCrossBounds(const Points & path, const TrackBounds & bounds)
{
  const std::size_t n = path.size();
  if (n < 4) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const Eigen::Vector2d & a = path[i];
    const Eigen::Vector2d & b = path[(i + 3) % n];
    for (const Points * boundary : {&bounds.left, &bounds.right}) {
      const std::size_t m = boundary->size();
      for (std::size_t j = 0; j < m; ++j) {
        if (segmentsIntersect(a, b, (*boundary)[j], (*boundary)[(j + 1) % m])) {
          return true;
        }
      }
    }
  }
  return false;
}

std::vector<ReferenceWaypoint> toReferenceWaypoints(const Points & pts)
{
  std::vector<ReferenceWaypoint> out;
  out.reserve(pts.size());
  double s = 0.0;
  for (std::size_t i = 0; i < pts.size(); ++i) {
    if (i > 0) {
      s += (pts[i] - pts[i - 1]).norm();
    }
    out.push_back({pts[i].x(), pts[i].y(), s});
  }
  return out;
}

}  // namespace

ReferencePathAdapterResult adaptReferencePath(
  const std::vector<AdapterWaypoint> & input,
  const ReferencePathAdapterConfig & config)
{
  ReferencePathAdapterResult result;
  result.input_point_count = input.size();

  const auto stripped = stripDuplicates(input, config.duplicate_point_tolerance);
  const Points pts0 = toPoints(stripped);
  result.output_point_count = pts0.size();
  result.path = toReferenceWaypoints(pts0);

  if (pts0.size() < 4) {
    result.stop_reason = "too_few_points";
    return result;
  }

  const int refinements = std::max(1, config.subdivision_refinements);
  const double step = config.resample_step > 0.0 ?
    config.resample_step : medianSpacing(pts0);

  const auto finalize = [&result](const Points & pts) {
      result.path = toReferenceWaypoints(pts);
      result.output_point_count = pts.size();
      result.modified = true;
    };

  if (!config.enable_curvature_reduction) {
    if (config.enable_smoothing) {
      Points pts = subdivideClosed(pts0, refinements);
      if (config.resample_step > 0.0) {
        pts = resampleClosed(pts, step);
      }
      finalize(pts);
      result.stop_reason = "smoothing_only";
    } else if (config.resample_step > 0.0) {
      finalize(resampleClosed(pts0, step));
      result.stop_reason = "resample_only";
    }
    return result;
  }

  // --- Full Alg. 1 loop (curvature reduction) ---
  double max_extent = 0.0;
  for (const auto & waypoint : stripped) {
    max_extent = std::max(max_extent, waypoint.d_left + waypoint.d_right);
  }
  if (max_extent < 1.0e-6) {
    // No usable track bounds: the rho criterion is vacuous. Degrade to
    // smoothing when requested instead of pretending the guarantee holds.
    if (config.enable_smoothing) {
      finalize(resampleClosed(subdivideClosed(pts0, refinements), step));
    }
    result.stop_reason = "degenerate_bounds";
    return result;
  }

  const TrackBounds bounds = buildBounds(stripped, pts0);
  const double kappa_cap = config.max_absolute_curvature;

  const RhoStats initial = evaluateRho(pts0, bounds, config.boundary_margin);
  result.initial_max_rho = initial.max_rho;
  result.initial_max_abs_curvature = initial.max_abs_curvature;
  result.final_max_rho = initial.max_rho;
  result.final_max_abs_curvature = initial.max_abs_curvature;

  // Short-circuit (deviation from the paper, which subdivides before the
  // first check): an input that already satisfies the criterion everywhere is
  // returned untouched so the frame stays identical to the raw raceline.
  if (initial.max_rho < 1.0 &&
    (kappa_cap <= 0.0 || initial.max_abs_curvature <= kappa_cap))
  {
    result.stop_reason = "already_satisfied";
    return result;
  }

  Points pts = pts0;
  result.stop_reason = "max_iterations";
  const int max_iterations = std::max(1, config.max_iterations);
  for (int iteration = 1; iteration <= max_iterations; ++iteration) {
    result.iterations_used = iteration;

    // Partition the loop at the anchors (fixed points inside the straights)
    // and refine every bend as an open segment with pinned ends — the paper's
    // per-partition treatment; Lemma 3's flattening requires the fixed ends.
    const auto kappa = signedCurvature(pts);
    const auto anchors = findAnchors(pts, kappa, config.anchor_curvature_threshold);
    auto segments = splitAtAnchors(pts, anchors);
    std::vector<Points> dense;
    dense.reserve(segments.size());
    for (auto & segment : segments) {
      dense.push_back(subdivideOpen(std::move(segment), refinements));
    }
    pts = concatSegments(dense);

    const RhoStats stats = evaluateRho(pts, bounds, config.boundary_margin);
    result.final_max_rho = stats.max_rho;
    result.final_max_abs_curvature = stats.max_abs_curvature;
    if (stats.max_rho < 1.0 &&
      (kappa_cap <= 0.0 || stats.max_abs_curvature <= kappa_cap))
    {
      result.stop_reason = "criterion_met";
      break;
    }

    std::vector<Points> coarse;
    coarse.reserve(dense.size());
    for (const auto & segment : dense) {
      coarse.push_back(resampleOpen(segment, step));
    }
    const Points candidate = concatSegments(coarse);
    if (chordsCrossBounds(candidate, bounds)) {
      // Fig. 6 guard: accepting the resample would risk crossing a bound.
      result.stop_reason = "boundary_hit";
      break;
    }
    pts = candidate;
  }

  // Keep the reference light for downstream CLCS/search: bring the converged
  // dense polyline back to the working spacing (anchors stay pinned so the
  // flattened shape is preserved), then re-measure and report the actually
  // published values.
  if (result.stop_reason == "criterion_met") {
    const auto kappa = signedCurvature(pts);
    const auto anchors = findAnchors(pts, kappa, config.anchor_curvature_threshold);
    const auto segments = splitAtAnchors(pts, anchors);
    std::vector<Points> coarse;
    coarse.reserve(segments.size());
    for (const auto & segment : segments) {
      coarse.push_back(resampleOpen(segment, step));
    }
    pts = concatSegments(coarse);
    const RhoStats stats = evaluateRho(pts, bounds, config.boundary_margin);
    result.final_max_rho = stats.max_rho;
    result.final_max_abs_curvature = stats.max_abs_curvature;
  }

  finalize(pts);
  return result;
}

}  // namespace global_planning
