#include "trajectory_core.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

#include <Eigen/Core>
#include <LBFGSB.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>
#ifdef HAVE_OPENCV_XIMGPROC
#include <opencv2/ximgproc.hpp>
#endif

namespace otg {

namespace {

using json = nlohmann::ordered_json;

constexpr int kVelocityLimitColumns = 4;

[[noreturn]] void fail(const std::string& message) { throw std::runtime_error(message); }

double norm2(const cv::Point2d& p) { return std::hypot(p.x, p.y); }

int wrap_index(int i, int n) { return ((i % n) + n) % n; }

// Python round()/np.round use banker's rounding (half to even); pixel indices
// must match or raycasts land one map pixel off at exact .5 boundaries.
long py_round(double v) { return static_cast<long>(std::nearbyint(v)); }

// np.interp equivalent: linear interpolation with edge clamping. xs must be
// strictly increasing.
double lin_interp(double x, const std::vector<double>& xs, const std::vector<double>& ys) {
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back()) return ys.back();
  const auto it = std::upper_bound(xs.begin(), xs.end(), x);
  const size_t hi = static_cast<size_t>(it - xs.begin());
  const size_t lo = hi - 1;
  const double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
  return ys[lo] + t * (ys[hi] - ys[lo]);
}

// scipy.ndimage.gaussian_filter1d(mode="wrap", truncate=4.0) equivalent.
std::vector<double> gaussian_wrap(const std::vector<double>& values, double sigma) {
  const int n = static_cast<int>(values.size());
  if (sigma <= 0.0 || n == 0) return values;
  const int lw = static_cast<int>(4.0 * sigma + 0.5);
  std::vector<double> weights(2 * lw + 1);
  double sum = 0.0;
  for (int i = -lw; i <= lw; ++i) {
    weights[i + lw] = std::exp(-0.5 * (i / sigma) * (i / sigma));
    sum += weights[i + lw];
  }
  for (double& w : weights) w /= sum;
  std::vector<double> out(n, 0.0);
  for (int i = 0; i < n; ++i) {
    double acc = 0.0;
    for (int j = -lw; j <= lw; ++j) acc += weights[j + lw] * values[wrap_index(i + j, n)];
    out[i] = acc;
  }
  return out;
}

std::vector<cv::Point2d> gaussian_wrap_points(const std::vector<cv::Point2d>& pts, double sigma) {
  std::vector<double> x(pts.size()), y(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) {
    x[i] = pts[i].x;
    y[i] = pts[i].y;
  }
  x = gaussian_wrap(x, sigma);
  y = gaussian_wrap(y, sigma);
  std::vector<cv::Point2d> out(pts.size());
  for (size_t i = 0; i < pts.size(); ++i) out[i] = {x[i], y[i]};
  return out;
}

std::vector<cv::Point2d> remove_consecutive_duplicates(const std::vector<cv::Point2d>& points,
                                                       double eps = 1e-9) {
  if (points.empty()) return points;
  std::vector<cv::Point2d> out;
  out.push_back(points.front());
  for (size_t i = 1; i < points.size(); ++i) {
    if (norm2(points[i] - out.back()) > eps) out.push_back(points[i]);
  }
  if (out.size() > 1 && norm2(out.front() - out.back()) <= eps) out.pop_back();
  return out;
}

// Cumulative arc length of the CLOSED polygon: n+1 entries, last = total.
std::pair<std::vector<double>, double> cumulative_s(const std::vector<cv::Point2d>& pts) {
  const size_t n = pts.size();
  std::vector<double> s(n + 1, 0.0);
  for (size_t i = 0; i < n; ++i) {
    const cv::Point2d& a = pts[i];
    const cv::Point2d& b = pts[(i + 1) % n];
    s[i + 1] = s[i] + norm2(b - a);
  }
  return {s, s.back()};
}

struct HeadingCurvature {
  std::vector<double> s, psi, kappa;
};

HeadingCurvature headings_and_curvature(const std::vector<cv::Point2d>& pts) {
  const int n = static_cast<int>(pts.size());
  HeadingCurvature out;
  out.s.assign(n, 0.0);
  out.psi.assign(n, 0.0);
  out.kappa.assign(n, 0.0);
  for (int i = 0; i < n; ++i) {
    const cv::Point2d& prev = pts[wrap_index(i - 1, n)];
    const cv::Point2d& cur = pts[i];
    const cv::Point2d& next = pts[wrap_index(i + 1, n)];
    const cv::Point2d chord = next - prev;
    out.psi[i] = std::atan2(chord.y, chord.x);
    const double a = norm2(cur - prev);
    const double b = norm2(next - cur);
    const double c = norm2(chord);
    const double cross =
        (cur.x - prev.x) * (next.y - prev.y) - (cur.y - prev.y) * (next.x - prev.x);
    const double denom = a * b * c;
    out.kappa[i] = denom > 1e-9 ? 2.0 * cross / denom : 0.0;
  }
  for (int i = 1; i < n; ++i) out.s[i] = out.s[i - 1] + norm2(pts[i] - pts[i - 1]);
  return out;
}

std::vector<cv::Point2d> normals_from_heading(const std::vector<double>& psi) {
  std::vector<cv::Point2d> normals(psi.size());
  for (size_t i = 0; i < psi.size(); ++i) normals[i] = {-std::sin(psi[i]), std::cos(psi[i])};
  return normals;
}

std::pair<MapInfo, cv::Mat> load_map_image(const fs::path& map_yaml) {
  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(map_yaml.string());
  } catch (const std::exception& exc) {
    fail("Could not read map YAML " + map_yaml.string() + ": " + exc.what());
  }
  fs::path image_path = cfg["image"].as<std::string>();
  if (!image_path.is_absolute()) image_path = map_yaml.parent_path() / image_path;

  cv::Mat image = cv::imread(image_path.string(), cv::IMREAD_GRAYSCALE);
  if (image.empty()) fail("Could not read map image: " + image_path.string());

  MapInfo info;
  info.yaml_path = map_yaml;
  info.image_path = image_path;
  info.resolution = cfg["resolution"].as<double>();
  if (cfg["origin"]) {
    const auto origin = cfg["origin"].as<std::vector<double>>();
    if (!origin.empty()) info.origin_x = origin[0];
    if (origin.size() > 1) info.origin_y = origin[1];
    if (origin.size() > 2) info.origin_yaw = origin[2];
  }
  info.negate = cfg["negate"] ? cfg["negate"].as<int>() : 0;
  info.occupied_thresh = cfg["occupied_thresh"] ? cfg["occupied_thresh"].as<double>() : 0.65;
  info.free_thresh = cfg["free_thresh"] ? cfg["free_thresh"].as<double>() : 0.196;
  info.height = image.rows;
  info.width = image.cols;
  return {info, image};
}

std::string map_mode(const fs::path& map_yaml) {
  const YAML::Node cfg = YAML::LoadFile(map_yaml.string());
  std::string mode = cfg["mode"] ? cfg["mode"].as<std::string>() : "trinary";
  std::transform(mode.begin(), mode.end(), mode.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return mode;
}

// occupancy = image/255 if negate else 1 - image/255
cv::Mat occupancy_from(const cv::Mat& image, int negate) {
  cv::Mat occupancy;
  image.convertTo(occupancy, CV_64F, 1.0 / 255.0);
  if (!negate) occupancy = 1.0 - occupancy;
  return occupancy;
}

cv::Mat free_mask_from(const cv::Mat& image, const MapInfo& info, const std::string& mode,
                       bool unknown_as_free) {
  const cv::Mat occupancy = occupancy_from(image, info.negate);
  cv::Mat free_mask;
  if (unknown_as_free) {
    cv::compare(occupancy, info.occupied_thresh, free_mask, cv::CMP_LT);
  } else if (mode == "trinary") {
    // ROS map_saver commonly writes unknown cells as gray around value 205.
    // Treat only near-free endpoint pixels as drivable so the generator does
    // not route through unknown background outside a cropped SLAM map.
    if (info.negate) {
      cv::compare(image, 5, free_mask, cv::CMP_LE);
    } else {
      cv::compare(image, 250, free_mask, cv::CMP_GE);
    }
    if (cv::countNonZero(free_mask) == 0) {
      cv::compare(occupancy, info.free_thresh, free_mask, cv::CMP_LE);
    }
  } else {
    cv::compare(occupancy, info.free_thresh, free_mask, cv::CMP_LE);
  }
  cv::Mat out;
  cv::threshold(free_mask, out, 0, 1, cv::THRESH_BINARY);
  return out;  // CV_8U 0/1
}

cv::Mat occupied_mask_from(const cv::Mat& image, const MapInfo& info) {
  const cv::Mat occupancy = occupancy_from(image, info.negate);
  cv::Mat occupied;
  cv::compare(occupancy, info.occupied_thresh, occupied, cv::CMP_GE);
  cv::Mat out;
  cv::threshold(occupied, out, 0, 1, cv::THRESH_BINARY);
  return out;
}

cv::Mat remove_small_components(const cv::Mat& binary_mask, int min_area) {
  cv::Mat mask;
  cv::threshold(binary_mask, mask, 0, 1, cv::THRESH_BINARY);
  if (min_area <= 0) return mask;
  cv::Mat labels, stats, centroids;
  const int num_labels = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
  if (num_labels <= 1) return mask;
  cv::Mat cleaned = cv::Mat::zeros(mask.size(), CV_8U);
  for (int label = 1; label < num_labels; ++label) {
    if (stats.at<int>(label, cv::CC_STAT_AREA) >= min_area) {
      cleaned.setTo(1, labels == label);
    }
  }
  return cleaned;
}

cv::Mat cleanup_free_mask(const cv::Mat& free_mask, int median_kernel, int morph_kernel,
                          int open_iterations, int close_iterations,
                          const cv::Mat& occupied_mask) {
  cv::Mat mask;
  cv::threshold(free_mask, mask, 0, 255, cv::THRESH_BINARY);
  int median_size = std::max(1, median_kernel);
  if (median_size % 2 == 0) ++median_size;
  if (median_size > 1) cv::medianBlur(mask, mask, median_size);

  int kernel_size = std::max(1, morph_kernel);
  if (kernel_size % 2 == 0) ++kernel_size;
  const cv::Mat kernel = cv::Mat::ones(kernel_size, kernel_size, CV_8U);

  if (close_iterations > 0) {
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), close_iterations);
  }
  if (open_iterations > 0) {
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), open_iterations);
  }

  // Cleanup may only reclaim unknown/noise pixels — never measured walls.
  // Median blur / morphological closing with a large kernel can otherwise
  // swallow a thin interior wall entirely, and every later stage (widths,
  // corridor bounds, off-map validation) would then believe the wall's area is
  // drivable. Sub-speckle occupied blobs stay removable so LiDAR salt noise
  // inside the track does not needlessly pinch the corridor.
  const int speckle_area = std::max(9, median_size * median_size);
  const cv::Mat walls = remove_small_components(occupied_mask, speckle_area);
  mask.setTo(0, walls > 0);

  mask.row(0).setTo(0);
  mask.row(mask.rows - 1).setTo(0);
  mask.col(0).setTo(0);
  mask.col(mask.cols - 1).setTo(0);

  cv::Mat binary, labels, stats, centroids;
  cv::threshold(mask, binary, 0, 1, cv::THRESH_BINARY);
  const int num_labels = cv::connectedComponentsWithStats(binary, labels, stats, centroids, 8);
  if (num_labels <= 1) fail("No free-space component was found in the map.");
  int largest = 1;
  for (int label = 2; label < num_labels; ++label) {
    if (stats.at<int>(label, cv::CC_STAT_AREA) > stats.at<int>(largest, cv::CC_STAT_AREA)) {
      largest = label;
    }
  }
  cv::Mat out = cv::Mat::zeros(mask.size(), CV_8U);
  out.setTo(255, labels == largest);
  return out;
}

#ifndef HAVE_OPENCV_XIMGPROC
// Zhang-Suen thinning fallback: topology-preserving 1-px skeleton that keeps
// the track loop connected when OpenCV was built without the ximgproc module.
cv::Mat zhang_suen_thinning(const cv::Mat& binary) {
  cv::Mat img;
  cv::threshold(binary, img, 0, 1, cv::THRESH_BINARY);
  bool changed = true;
  while (changed) {
    changed = false;
    for (int step = 0; step < 2; ++step) {
      std::vector<cv::Point> remove;
      for (int r = 0; r < img.rows; ++r) {
        for (int c = 0; c < img.cols; ++c) {
          if (!img.at<uint8_t>(r, c)) continue;
          auto at = [&](int rr, int cc) -> int {
            if (rr < 0 || rr >= img.rows || cc < 0 || cc >= img.cols) return 0;
            return img.at<uint8_t>(rr, cc);
          };
          const int p2 = at(r - 1, c), p3 = at(r - 1, c + 1), p4 = at(r, c + 1),
                    p5 = at(r + 1, c + 1), p6 = at(r + 1, c), p7 = at(r + 1, c - 1),
                    p8 = at(r, c - 1), p9 = at(r - 1, c - 1);
          const int b = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
          if (b < 2 || b > 6) continue;
          const int ring[9] = {p2, p3, p4, p5, p6, p7, p8, p9, p2};
          int a = 0;
          for (int k = 0; k < 8; ++k) a += (ring[k] == 0 && ring[k + 1] == 1);
          if (a != 1) continue;
          const bool cond = step == 0 ? (p2 * p4 * p6 == 0 && p4 * p6 * p8 == 0)
                                      : (p2 * p4 * p8 == 0 && p2 * p6 * p8 == 0);
          if (cond) remove.emplace_back(c, r);
        }
      }
      for (const cv::Point& p : remove) img.at<uint8_t>(p) = 0;
      if (!remove.empty()) changed = true;
    }
  }
  cv::Mat out;
  cv::threshold(img, out, 0, 255, cv::THRESH_BINARY);
  return out;
}
#endif

cv::Mat skeletonize(const cv::Mat& binary_mask) {
  cv::Mat image;
  cv::threshold(binary_mask, image, 0, 255, cv::THRESH_BINARY);
#ifdef HAVE_OPENCV_XIMGPROC
  cv::Mat thinned;
  cv::ximgproc::thinning(image, thinned, cv::ximgproc::THINNING_ZHANGSUEN);
  return thinned;
#else
  return zhang_suen_thinning(image);
#endif
}

cv::Mat prune_skeleton(const cv::Mat& skeleton, int prune_iterations, int min_component_area) {
  cv::Mat mask = remove_small_components(skeleton, min_component_area);
  const cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
  for (int i = 0; i < std::max(0, prune_iterations); ++i) {
    cv::Mat neighbors;
    cv::filter2D(mask, neighbors, CV_16S, kernel, cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);
    std::vector<cv::Point> endpoints;
    for (int r = 0; r < mask.rows; ++r) {
      for (int c = 0; c < mask.cols; ++c) {
        if (mask.at<uint8_t>(r, c) &&
            neighbors.at<int16_t>(r, c) - mask.at<uint8_t>(r, c) <= 1) {
          endpoints.emplace_back(c, r);
        }
      }
    }
    if (endpoints.empty()) break;
    for (const cv::Point& p : endpoints) mask.at<uint8_t>(p) = 0;
  }
  mask = remove_small_components(mask, min_component_area);
  // Deliberately no keep-largest-component here: pixel count is a poor
  // discriminator against noise blobs. The contour stage picks the loop with
  // the largest ENCLOSED area instead, which is the actual track loop.
  cv::Mat out;
  cv::threshold(mask, out, 0, 255, cv::THRESH_BINARY);
  return out;
}

std::vector<cv::Point2d> extract_centerline_pixels(cv::Mat skeleton, int prune_iterations,
                                                   int min_component_area, int min_points = 20) {
  const cv::Mat pruned = prune_skeleton(skeleton, prune_iterations, min_component_area);
  if (cv::countNonZero(pruned) >= min_points) skeleton = pruned;

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(skeleton, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

  struct Candidate {
    double length, area;
    const std::vector<cv::Point>* points;
  };
  std::vector<Candidate> candidates;
  for (const auto& contour : contours) {
    if (static_cast<int>(contour.size()) < min_points) continue;
    const double length = cv::arcLength(contour, true);
    const double area = std::abs(cv::contourArea(contour));
    candidates.push_back({length, area, &contour});
  }
  if (candidates.empty()) {
    fail("Could not extract a closed centerline from the skeletonized map.");
  }
  // Enclosed area first: the real track loop encircles the map interior,
  // while long-but-thin noise contours (scan artifacts, hatching) enclose
  // almost nothing even though they beat the loop on arc length.
  const auto best = std::max_element(
      candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return std::make_pair(a.area, a.length) < std::make_pair(b.area, b.length);
      });
  std::vector<cv::Point2d> points;
  points.reserve(best->points->size());
  for (const cv::Point& p : *best->points) points.emplace_back(p.x, p.y);
  return remove_consecutive_duplicates(points);
}

std::vector<cv::Point2d> pixel_to_world(const std::vector<cv::Point2d>& points_col_row,
                                        const MapInfo& info, bool flip_y) {
  std::vector<cv::Point2d> out(points_col_row.size());
  for (size_t i = 0; i < points_col_row.size(); ++i) {
    const double x = info.origin_x + points_col_row[i].x * info.resolution;
    const double y = flip_y
        ? info.origin_y + (info.height - 1 - points_col_row[i].y) * info.resolution
        : info.origin_y + points_col_row[i].y * info.resolution;
    out[i] = {x, y};
  }
  return out;
}

cv::Point2d world_to_pixel_one(const cv::Point2d& p, const MapInfo& info, bool flip_y) {
  const double col = (p.x - info.origin_x) / info.resolution;
  const double row = flip_y ? (info.height - 1) - (p.y - info.origin_y) / info.resolution
                            : (p.y - info.origin_y) / info.resolution;
  return {col, row};
}

std::vector<cv::Point2d> remove_sharp_spikes(const std::vector<cv::Point2d>& points_xy,
                                             double min_angle_deg, int iterations) {
  if (min_angle_deg <= 0.0 || iterations <= 0) return points_xy;
  std::vector<cv::Point2d> filtered = remove_consecutive_duplicates(points_xy);
  for (int iter = 0; iter < iterations; ++iter) {
    const int n = static_cast<int>(filtered.size());
    if (n < 8) break;
    std::vector<double> angles(n);
    std::vector<int> spikes;
    for (int i = 0; i < n; ++i) {
      const cv::Point2d prev_vec = filtered[wrap_index(i - 1, n)] - filtered[i];
      const cv::Point2d next_vec = filtered[wrap_index(i + 1, n)] - filtered[i];
      const double denom = norm2(prev_vec) * norm2(next_vec);
      double cos_angle = 1.0;
      if (denom > 1e-9) cos_angle = prev_vec.dot(next_vec) / denom;
      angles[i] = std::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / M_PI;
      if (angles[i] < min_angle_deg) spikes.push_back(i);
    }
    if (spikes.empty()) break;
    const int max_remove = std::max(1, n / 12);
    if (static_cast<int>(spikes.size()) > max_remove) {
      std::sort(spikes.begin(), spikes.end(),
                [&](int a, int b) { return angles[a] < angles[b]; });
      spikes.resize(max_remove);
    }
    std::vector<bool> keep(n, true);
    for (int idx : spikes) keep[idx] = false;
    std::vector<cv::Point2d> next;
    next.reserve(n);
    for (int i = 0; i < n; ++i) {
      if (keep[i]) next.push_back(filtered[i]);
    }
    filtered = std::move(next);
  }
  return filtered;
}

std::vector<cv::Point2d> filter_and_resample_closed(const std::vector<cv::Point2d>& points_xy,
                                                    double step, const Args& args) {
  std::vector<cv::Point2d> filtered =
      remove_sharp_spikes(points_xy, args.min_centerline_angle, args.spike_filter_iterations);
  // A spike filter that eats a large share of the points is misfiring on the
  // path shape (e.g. a tightly-cut raceline), not removing pin artifacts.
  // Resampling the surviving points would bridge the gaps with wall-crossing
  // chords, so prefer the unfiltered path in that case.
  if (static_cast<int>(filtered.size()) <
      std::max(4, static_cast<int>(0.7 * points_xy.size()))) {
    filtered = points_xy;
  }
  std::vector<cv::Point2d> sampled = resample_closed(filtered, step);
  const std::vector<cv::Point2d> post_filtered =
      remove_sharp_spikes(sampled, args.min_centerline_angle, args.spike_filter_iterations);
  const int lower = std::max(4, static_cast<int>(0.7 * sampled.size()));
  if (lower <= static_cast<int>(post_filtered.size()) &&
      post_filtered.size() < sampled.size()) {
    sampled = resample_closed(post_filtered, step);
  }
  return sampled;
}

double raycast_distance_robust(const cv::Mat& free_mask, const cv::Point2d& point_xy,
                               const cv::Point2d& direction_xy, const MapInfo& info, bool flip_y,
                               double max_distance_m, int min_wall_pixels = 2) {
  // Requires multiple consecutive wall pixels so single-pixel noise never
  // creates a false wall.
  const double step_m = std::max(info.resolution * 0.5, 0.01);
  const int steps = static_cast<int>(max_distance_m / step_m);
  int consecutive_walls = 0;
  int first_wall_idx = -1;
  for (int i = 1; i <= steps; ++i) {
    const cv::Point2d point = point_xy + direction_xy * (i * step_m);
    const cv::Point2d pixel = world_to_pixel_one(point, info, flip_y);
    const int col = static_cast<int>(py_round(pixel.x));
    const int row = static_cast<int>(py_round(pixel.y));
    if (row < 0 || row >= free_mask.rows || col < 0 || col >= free_mask.cols) {
      if (first_wall_idx < 0) first_wall_idx = i - 1;
      if (++consecutive_walls >= min_wall_pixels) {
        return std::max(0.0, first_wall_idx * step_m);
      }
      continue;
    }
    if (free_mask.at<uint8_t>(row, col) == 0) {
      if (first_wall_idx < 0) first_wall_idx = i;
      if (++consecutive_walls >= min_wall_pixels) {
        return std::max(0.0, first_wall_idx * step_m);
      }
    } else {
      consecutive_walls = 0;
      first_wall_idx = -1;
    }
  }
  return max_distance_m;
}

std::vector<double> distance_clearance_m(const std::vector<cv::Point2d>& points_xy,
                                         const cv::Mat& distance_map_px, const MapInfo& info,
                                         bool flip_y) {
  std::vector<double> out(points_xy.size());
  for (size_t i = 0; i < points_xy.size(); ++i) {
    const cv::Point2d pixel = world_to_pixel_one(points_xy[i], info, flip_y);
    const int col = std::clamp(static_cast<int>(py_round(pixel.x)), 0, info.width - 1);
    const int row = std::clamp(static_cast<int>(py_round(pixel.y)), 0, info.height - 1);
    out[i] = distance_map_px.at<float>(row, col) * info.resolution;
  }
  return out;
}

cv::Mat distance_transform_of(const cv::Mat& free_mask) {
  cv::Mat binary, dist;
  cv::threshold(free_mask, binary, 0, 1, cv::THRESH_BINARY);
  cv::distanceTransform(binary, dist, cv::DIST_L2, 5);
  return dist;
}

// Returns {d_right, d_left}.
std::pair<std::vector<double>, std::vector<double>> track_widths(
    const std::vector<cv::Point2d>& points_xy, const cv::Mat& free_mask, const MapInfo& info,
    bool flip_y, double max_distance_m, const std::string& width_mode) {
  const size_t n = points_xy.size();

  if (width_mode == "distance") {
    const cv::Mat dist = distance_transform_of(free_mask);
    std::vector<double> widths = distance_clearance_m(points_xy, dist, info, flip_y);
    for (double& w : widths) w = std::clamp(w, 1e-3, max_distance_m);
    return {widths, widths};
  }

  const HeadingCurvature hc = headings_and_curvature(points_xy);
  const std::vector<cv::Point2d> left_normals = normals_from_heading(hc.psi);
  std::vector<double> d_left(n), d_right(n);
  for (size_t i = 0; i < n; ++i) {
    d_left[i] = raycast_distance_robust(free_mask, points_xy[i], left_normals[i], info, flip_y,
                                        max_distance_m);
    d_right[i] = raycast_distance_robust(free_mask, points_xy[i], -left_normals[i], info, flip_y,
                                         max_distance_m);
  }
  if (width_mode == "raycast") return {d_right, d_left};

  // hybrid: robust directional raycast keeps the left/right asymmetry; the
  // isotropic distance-transform clearance is used ONLY as a failure floor
  // when a ray slipped through a gap (hit nothing although a wall is near).
  const cv::Mat dist = distance_transform_of(free_mask);
  std::vector<double> min_clearance = distance_clearance_m(points_xy, dist, info, flip_y);
  for (size_t i = 0; i < n; ++i) {
    const double floor_m = std::clamp(min_clearance[i], 1e-3, max_distance_m);
    if (d_left[i] >= max_distance_m - 1e-6) d_left[i] = floor_m;
    if (d_right[i] >= max_distance_m - 1e-6) d_right[i] = floor_m;
    d_left[i] = std::clamp(d_left[i], 1e-3, max_distance_m);
    d_right[i] = std::clamp(d_right[i], 1e-3, max_distance_m);
  }
  return {d_right, d_left};
}

std::vector<std::pair<int, int>> circular_true_runs(const std::vector<bool>& mask) {
  const int n = static_cast<int>(mask.size());
  const bool any = std::any_of(mask.begin(), mask.end(), [](bool b) { return b; });
  const bool all = std::all_of(mask.begin(), mask.end(), [](bool b) { return b; });
  if (n == 0 || !any || all) return {};
  int first_false = 0;
  while (mask[first_false]) ++first_false;
  const int start_offset = (first_false + 1) % n;
  std::vector<std::pair<int, int>> runs;
  int i = 0;
  while (i < n) {
    if (!mask[wrap_index(i + start_offset, n)]) {
      ++i;
      continue;
    }
    const int start = i;
    while (i < n && mask[wrap_index(i + start_offset, n)]) ++i;
    runs.emplace_back(wrap_index(start + start_offset, n), wrap_index(i - 1 + start_offset, n));
  }
  return runs;
}

std::vector<int> circular_indices(int start, int end, int count) {
  std::vector<int> out;
  if (start <= end) {
    for (int i = start; i <= end; ++i) out.push_back(i);
  } else {
    for (int i = start; i < count; ++i) out.push_back(i);
    for (int i = 0; i <= end; ++i) out.push_back(i);
  }
  return out;
}

std::vector<cv::Point2d> optimize_min_curvature(const std::vector<cv::Point2d>& center_xy,
                                                const std::vector<double>& d_right,
                                                const std::vector<double>& d_left,
                                                const Args& args) {
  const int n = static_cast<int>(center_xy.size());
  const HeadingCurvature hc = headings_and_curvature(center_xy);
  const std::vector<cv::Point2d> normals = normals_from_heading(hc.psi);
  const double clearance = args.safety_width * 0.5 + args.boundary_margin;

  Eigen::VectorXd lower(n), upper(n);
  bool has_room = false;
  for (int i = 0; i < n; ++i) {
    lower[i] = -std::max(d_right[i] - clearance, 0.0);
    upper[i] = std::max(d_left[i] - clearance, 0.0);
    if (upper[i] > 1e-3 || lower[i] < -1e-3) has_room = true;
  }
  if (!has_room) return center_xy;

  const double max_curv = args.max_curvature;
  const auto objective = [&](const Eigen::VectorXd& alpha) -> double {
    std::vector<cv::Point2d> shifted(n);
    for (int i = 0; i < n; ++i) shifted[i] = center_xy[i] + normals[i] * alpha[i];
    const HeadingCurvature shc = headings_and_curvature(shifted);
    double kappa_sq = 0.0, seg_sum = 0.0, dalpha_sq = 0.0, excess_sq = 0.0;
    for (int i = 0; i < n; ++i) {
      kappa_sq += shc.kappa[i] * shc.kappa[i];
      seg_sum += norm2(shifted[wrap_index(i + 1, n)] - shifted[i]);
      const double dalpha = alpha[wrap_index(i + 1, n)] - alpha[i];
      dalpha_sq += dalpha * dalpha;
      if (max_curv > 0.0) {
        const double excess = std::max(std::abs(shc.kappa[i]) - max_curv, 0.0);
        excess_sq += excess * excess;
      }
    }
    double cost = args.curvature_weight * kappa_sq / n + args.smooth_weight * dalpha_sq / n +
                  args.length_weight * seg_sum / n;
    if (max_curv > 0.0) {
      // Bends sharper than the steering limit are undrivable, not just slow.
      // Mean (not sum) keeps the term smooth enough for the numerical
      // gradients — a stiff sum-based penalty destabilizes the solve.
      cost += 25.0 * excess_sq / n;
    }
    return cost;
  };

  Eigen::VectorXd best_alpha = Eigen::VectorXd::Zero(n);
  double best_cost = objective(best_alpha);
  const auto fun = [&](Eigen::VectorXd& x, Eigen::VectorXd& grad) -> double {
    const double base = objective(x);
    for (int i = 0; i < n; ++i) {
      const double h = 1e-8 * std::max(1.0, std::abs(x[i]));
      const double saved = x[i];
      x[i] = saved + h;
      grad[i] = (objective(x) - base) / h;
      x[i] = saved;
    }
    if (base < best_cost) {
      best_cost = base;
      best_alpha = x;
    }
    return base;
  };

  LBFGSpp::LBFGSBParam<double> param;
  param.max_iterations = args.max_optimizer_iter;
  param.epsilon = 1e-5;
  param.epsilon_rel = 1e-5;
  param.past = 1;
  param.delta = 1e-5;  // ftol-like relative decrease stop
  param.max_linesearch = 25;
  LBFGSpp::LBFGSBSolver<double> solver(param);

  Eigen::VectorXd alpha = Eigen::VectorXd::Zero(n);
  double fx = 0.0;
  try {
    solver.minimize(fun, alpha, fx, lower, upper);
  } catch (const std::exception& exc) {
    // Line-search / iteration-limit stops still leave the best raceline found
    // so far in best_alpha, which is perfectly usable.
    std::fprintf(stderr, "[WARN] optimizer did not fully converge: %s\n", exc.what());
  }
  const Eigen::VectorXd& result = objective(alpha) <= best_cost ? alpha : best_alpha;
  std::vector<cv::Point2d> out(n);
  for (int i = 0; i < n; ++i) out[i] = center_xy[i] + normals[i] * result[i];
  return out;
}

std::vector<cv::Point2d> offset_by_d_ratio(const std::vector<cv::Point2d>& center_xy,
                                           const std::vector<double>& d_right,
                                           const std::vector<double>& d_left, const Args& args) {
  // d_ratio > 0 moves toward d_right, < 0 toward d_left (user spec; this is
  // the OPPOSITE sign of the mincurv alpha / Frenet d convention — do not
  // "fix" it). The ratio scales the usable half-width (wall distance minus
  // clearance), so |d_ratio| = 1 stops clearance short of the wall.
  const int n = static_cast<int>(center_xy.size());
  const HeadingCurvature hc = headings_and_curvature(center_xy);
  const std::vector<cv::Point2d> normals = normals_from_heading(hc.psi);  // left normals
  const double clearance = args.safety_width * 0.5 + args.boundary_margin;
  std::vector<double> usable_right(n), usable_left(n), alpha(n);
  const double ratio = args.d_ratio;
  for (int i = 0; i < n; ++i) {
    usable_right[i] = std::max(d_right[i] - clearance, 0.0);
    usable_left[i] = std::max(d_left[i] - clearance, 0.0);
    alpha[i] = -ratio * (ratio >= 0.0 ? usable_right[i] : usable_left[i]);
  }
  if (args.d_ratio_alpha_smooth_sigma > 0.0) {
    // Smooth the decision variable, never the width measurements: clipping
    // back to the RAW corridor only shrinks |alpha| (toward the centerline),
    // so smoothing errors always land on the safe side.
    alpha = gaussian_wrap(alpha, args.d_ratio_alpha_smooth_sigma);
    for (int i = 0; i < n; ++i) alpha[i] = std::clamp(alpha[i], -usable_right[i], usable_left[i]);
  }
  // Fold guard (last, only ever shrinks |alpha| so the corridor stays valid):
  // offsets toward the local curvature center are capped below the curvature
  // radius, otherwise a hairpin inside-offset folds into a self-loop.
  const std::vector<double> kappa_smooth = gaussian_wrap(hc.kappa, 2.0);
  for (int i = 0; i < n; ++i) {
    const double fold_cap = 0.9 / std::max(std::abs(kappa_smooth[i]), 1e-9);
    if (kappa_smooth[i] * alpha[i] > 0.0) {
      alpha[i] = std::clamp(alpha[i], -fold_cap, fold_cap);
    }
  }
  std::vector<cv::Point2d> out(n);
  for (int i = 0; i < n; ++i) out[i] = center_xy[i] + normals[i] * alpha[i];
  return out;
}

std::vector<cv::Point2d> optimize_raceline(const std::vector<cv::Point2d>& center_xy,
                                           const std::vector<double>& d_right,
                                           const std::vector<double>& d_left, const Args& args) {
  if (args.optimizer == "centerline") return center_xy;
  if (args.optimizer == "d_ratio") return offset_by_d_ratio(center_xy, d_right, d_left, args);
  return optimize_min_curvature(center_xy, d_right, d_left, args);
}

std::vector<cv::Point2d> limit_curvature_spikes(std::vector<cv::Point2d> pts, double max_curv,
                                                int iterations = 40, int max_run = 3) {
  // Flatten ISOLATED curvature spikes above the steering limit. Runs longer
  // than max_run are real corners — touching them here only shifts the kink
  // to the run boundary (measured: it amplified a 6 rad/m corner to
  // 17 rad/m), so they are left intact for the validation warning.
  const int n = static_cast<int>(pts.size());
  for (int iter = 0; iter < iterations; ++iter) {
    const HeadingCurvature hc = headings_and_curvature(pts);
    std::vector<bool> bad(n);
    bool any_bad = false;
    for (int i = 0; i < n; ++i) {
      bad[i] = std::abs(hc.kappa[i]) > max_curv;
      any_bad = any_bad || bad[i];
    }
    if (!any_bad) break;
    std::vector<bool> target(n, false);
    for (const auto& [start, end] : circular_true_runs(bad)) {
      const std::vector<int> idxs = circular_indices(start, end, n);
      if (static_cast<int>(idxs.size()) <= max_run) {
        for (int idx : idxs) target[idx] = true;
      }
    }
    if (std::none_of(target.begin(), target.end(), [](bool b) { return b; })) break;
    std::vector<bool> expanded(n);
    for (int i = 0; i < n; ++i) {
      expanded[i] = target[i] || target[wrap_index(i - 1, n)] || target[wrap_index(i + 1, n)];
    }
    const std::vector<cv::Point2d> snapshot = pts;
    for (int i = 0; i < n; ++i) {
      if (!expanded[i]) continue;
      const cv::Point2d chord_mid =
          0.5 * (snapshot[wrap_index(i - 1, n)] + snapshot[wrap_index(i + 1, n)]);
      pts[i] = 0.7 * snapshot[i] + 0.3 * chord_mid;
    }
  }
  return pts;
}

int count_off_map_waypoints(const std::vector<cv::Point2d>& points_xy, const cv::Mat& free_mask,
                            const MapInfo& info, bool flip_y) {
  // Checked on a densified copy (~2 px spacing) so a segment slicing through
  // a thin wall between two waypoints is caught as well.
  const std::vector<cv::Point2d> dense =
      resample_closed(points_xy, std::max(info.resolution * 2.0, 1e-3));
  std::vector<bool> off(dense.size());
  bool any_off = false;
  for (size_t i = 0; i < dense.size(); ++i) {
    const cv::Point2d pixel = world_to_pixel_one(dense[i], info, flip_y);
    const int col = std::clamp(static_cast<int>(py_round(pixel.x)), 0, info.width - 1);
    const int row = std::clamp(static_cast<int>(py_round(pixel.y)), 0, info.height - 1);
    off[i] = free_mask.at<uint8_t>(row, col) == 0;
    any_off = any_off || off[i];
  }
  if (!any_off) return 0;
  // Map dense hits back to waypoint count: one per waypoint whose span is hit.
  const auto [s_dense, total_dense] = cumulative_s(dense);
  const auto [s_wpts, total_wpts] = cumulative_s(points_xy);
  std::vector<bool> hit(points_xy.size(), false);
  for (size_t i = 0; i < dense.size(); ++i) {
    if (!off[i]) continue;
    const auto it = std::upper_bound(s_wpts.begin(), s_wpts.end(), s_dense[i]);
    const int span = static_cast<int>(it - s_wpts.begin()) - 1;
    hit[std::clamp(span, 0, static_cast<int>(points_xy.size()) - 1)] = true;
  }
  return static_cast<int>(std::count(hit.begin(), hit.end(), true));
}

std::pair<double, int> report_min_clearance(const std::vector<cv::Point2d>& points_xy,
                                            const cv::Mat& free_mask, const MapInfo& info,
                                            bool flip_y, const Args& args) {
  // Dense wall-clearance audit of the final raceline: the off-map count only
  // proves the path stays on free pixels; this measures the distance-transform
  // clearance so car-body wall clipping (clearance < safety_width/2) becomes
  // visible too. Violations are counted against the physical half-width only.
  const std::vector<cv::Point2d> dense =
      resample_closed(points_xy, std::max(info.resolution * 2.0, 1e-3));
  const cv::Mat dist = distance_transform_of(free_mask);
  const std::vector<double> clearance = distance_clearance_m(dense, dist, info, flip_y);
  double min_clearance = clearance.empty() ? 0.0
                                           : *std::min_element(clearance.begin(), clearance.end());
  const int violations = static_cast<int>(std::count_if(
      clearance.begin(), clearance.end(),
      [&](double c) { return c < args.safety_width * 0.5; }));
  return {min_clearance, violations};
}

bool line_has_clearance(const cv::Point2d& start_xy, const cv::Point2d& end_xy,
                        const cv::Mat& free_mask, const cv::Mat& distance_map_px,
                        const MapInfo& info, bool flip_y, double required_clearance_m) {
  const cv::Point2d delta = end_xy - start_xy;
  const double length = norm2(delta);
  if (length <= info.resolution) return false;
  const double step = std::max(info.resolution * 0.5, 0.01);
  const int sample_count = std::max(2, static_cast<int>(std::ceil(length / step)) + 1);
  for (int k = 0; k < sample_count; ++k) {
    const double fraction = static_cast<double>(k) / (sample_count - 1);
    const cv::Point2d sample = start_xy + fraction * delta;
    const cv::Point2d pixel = world_to_pixel_one(sample, info, flip_y);
    const int col = static_cast<int>(py_round(pixel.x));
    const int row = static_cast<int>(py_round(pixel.y));
    if (row < 0 || row >= info.height || col < 0 || col >= info.width) return false;
    if (free_mask.at<uint8_t>(row, col) == 0) return false;
    if (distance_map_px.at<float>(row, col) * info.resolution < required_clearance_m) {
      return false;
    }
  }
  return true;
}

std::vector<cv::Point2d> straighten_straight_segments(const std::vector<cv::Point2d>& points_xy,
                                                      const cv::Mat& free_mask,
                                                      const MapInfo& info, bool flip_y,
                                                      const Args& args, bool& changed) {
  changed = false;
  const int n = static_cast<int>(points_xy.size());
  if (!args.straighten_straights || n < 8) return points_xy;
  if (args.straight_kappa_threshold <= 0.0 || args.straight_min_length <= 0.0) return points_xy;

  const HeadingCurvature hc = headings_and_curvature(points_xy);
  std::vector<double> abs_kappa(n);
  for (int i = 0; i < n; ++i) abs_kappa[i] = std::abs(hc.kappa[i]);
  const std::vector<double> smooth_abs_kappa = gaussian_wrap(abs_kappa, 1.0);
  std::vector<bool> straight_mask(n);
  for (int i = 0; i < n; ++i) {
    straight_mask[i] = smooth_abs_kappa[i] <= args.straight_kappa_threshold;
  }
  const auto runs = circular_true_runs(straight_mask);
  if (runs.empty()) return points_xy;

  const cv::Mat distance_map_px = distance_transform_of(free_mask);
  // Chord validation only needs the car to physically fit (half width plus
  // the straightening margin). boundary_margin is an optimizer-corridor
  // shaping knob; including it here silently disabled straightening on narrow
  // tracks as soon as the margin grew.
  const double required_clearance = args.safety_width * 0.5 + args.straight_clearance_margin;
  std::vector<cv::Point2d> straightened = points_xy;

  for (const auto& [start, end] : runs) {
    const std::vector<int> indices = circular_indices(start, end, n);
    std::vector<double> distances(indices.size(), 0.0);
    for (size_t k = 1; k < indices.size(); ++k) {
      distances[k] = distances[k - 1] + norm2(points_xy[indices[k]] - points_xy[indices[k - 1]]);
    }
    const double length = distances.empty() ? 0.0 : distances.back();
    if (indices.size() < 4 || length < args.straight_min_length) continue;

    const cv::Point2d start_xy = points_xy[indices.front()];
    const cv::Point2d end_xy = points_xy[indices.back()];
    const cv::Point2d chord = end_xy - start_xy;
    const double chord_length = norm2(chord);
    if (chord_length <= info.resolution || chord_length < args.straight_min_length * 0.5) {
      continue;
    }
    if (!line_has_clearance(start_xy, end_xy, free_mask, distance_map_px, info, flip_y,
                            required_clearance)) {
      std::fprintf(stderr,
                   "[WARN] straight run of %.2f m not straightened: chord clearance is below "
                   "%.2f m somewhere along it.\n",
                   length, required_clearance);
      continue;
    }

    for (size_t k = 0; k < indices.size(); ++k) {
      const double fraction = length > 1e-9 ? distances[k] / length : 0.0;
      const cv::Point2d line_point = start_xy + fraction * chord;
      double weight;
      if (args.straight_blend_length > 0.0) {
        const double edge_distance = std::min(distances[k], length - distances[k]);
        const double w = std::clamp(edge_distance / args.straight_blend_length, 0.0, 1.0);
        weight = w * w * (3.0 - 2.0 * w);
      } else {
        weight = (k == 0 || k + 1 == indices.size()) ? 0.0 : 1.0;
      }
      straightened[indices[k]] =
          points_xy[indices[k]] * (1.0 - weight) + line_point * weight;
    }
    changed = true;
  }
  return straightened;
}

// Highest speed satisfying v^2*kappa <= piecewise-linear ay_max(v).
double rightmost_lateral_speed(double kappa_abs,
                               const std::vector<std::array<double, 4>>& limits,
                               double max_speed) {
  if (kappa_abs <= 1e-12) return max_speed;
  std::vector<double> speeds, lateral;
  for (const auto& row : limits) {
    speeds.push_back(row[0]);
    lateral.push_back(row[3]);
  }
  std::vector<double> breakpoints{0.0};
  for (double s : speeds) {
    if (s > 0.0 && s < max_speed) breakpoints.push_back(s);
  }
  breakpoints.push_back(max_speed);

  const auto residual = [&](double speed) {
    return speed * speed * kappa_abs - lin_interp(speed, speeds, lateral);
  };
  const auto bisect_feasible = [&](double left, double right) {
    // left is feasible and right is infeasible. Returning the feasible side
    // keeps the generated profile inside the table limit.
    for (int i = 0; i < 60; ++i) {
      const double middle = 0.5 * (left + right);
      if (residual(middle) <= 0.0) {
        left = middle;
      } else {
        right = middle;
      }
    }
    return left;
  };

  // Scan high-speed intervals first. Within one table interval residual is a
  // convex quadratic, so checking both ends and its vertex finds the
  // rightmost feasible root even for a non-monotonic lateral-accel table.
  for (int index = static_cast<int>(breakpoints.size()) - 1; index > 0; --index) {
    const double lower = breakpoints[index - 1];
    const double upper = breakpoints[index];
    if (residual(upper) <= 0.0) return upper;
    if (residual(lower) <= 0.0) return bisect_feasible(lower, upper);
    const double ay_lower = lin_interp(lower, speeds, lateral);
    const double ay_upper = lin_interp(upper, speeds, lateral);
    const double slope = (ay_upper - ay_lower) / (upper - lower);
    const double vertex = slope / (2.0 * kappa_abs);
    if (lower < vertex && vertex < upper && residual(vertex) <= 0.0) {
      return bisect_feasible(vertex, upper);
    }
  }
  return 0.0;
}

struct VelocityProfile {
  std::vector<double> vx, ax;
  double lap_time = 0.0;
};

VelocityProfile velocity_profile(const std::vector<cv::Point2d>& points_xy,
                                 const std::vector<double>& kappa, double max_speed,
                                 double min_speed,
                                 const std::vector<std::array<double, 4>>& limits) {
  const int n = static_cast<int>(points_xy.size());
  std::vector<double> speeds_col, accel_col, decel_col, lateral_col;
  for (const auto& row : limits) {
    speeds_col.push_back(row[0]);
    accel_col.push_back(row[1]);
    decel_col.push_back(row[2]);
    lateral_col.push_back(row[3]);
  }
  std::vector<double> seg(n);
  for (int i = 0; i < n; ++i) seg[i] = norm2(points_xy[wrap_index(i + 1, n)] - points_xy[i]);

  std::vector<double> v(n);
  int infeasible = 0;
  for (int i = 0; i < n; ++i) {
    const double curve_speed = rightmost_lateral_speed(std::abs(kappa[i]), limits, max_speed);
    if (curve_speed < min_speed - 1e-9) ++infeasible;
    v[i] = std::clamp(curve_speed, min_speed, max_speed);
  }
  if (infeasible > 0) {
    std::fprintf(stderr,
                 "[WARN] min-speed %.3f m/s overrides the velocity-table lateral limit at %d "
                 "waypoints; lower min-speed for a strictly feasible profile.\n",
                 min_speed, infeasible);
  }

  bool converged = false;
  for (int pass = 0; pass < 100; ++pass) {
    const std::vector<double> previous = v;
    for (int i = 0; i < n; ++i) {
      const int j = wrap_index(i + 1, n);
      const double max_accel = lin_interp(v[i], speeds_col, accel_col);
      const double possible = std::sqrt(std::max(v[i] * v[i] + 2.0 * max_accel * seg[i], 0.0));
      v[j] = std::min(v[j], possible);
    }
    for (int i = n - 1; i >= 0; --i) {
      const int j = wrap_index(i - 1, n);
      const double max_decel = lin_interp(v[i], speeds_col, decel_col);
      const double possible = std::sqrt(std::max(v[i] * v[i] + 2.0 * max_decel * seg[j], 0.0));
      v[j] = std::min(v[j], possible);
    }
    double max_change = 0.0;
    for (int i = 0; i < n; ++i) max_change = std::max(max_change, std::abs(v[i] - previous[i]));
    if (max_change <= 1e-10) {
      converged = true;
      break;
    }
  }
  if (!converged) fail("speed-dependent forward/backward profile did not converge.");

  double max_excess = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < n; ++i) {
    const double next_v = v[wrap_index(i + 1, n)];
    if (v[i] > min_speed + 1e-9) {
      const double lateral_excess =
          v[i] * v[i] * std::abs(kappa[i]) - lin_interp(v[i], speeds_col, lateral_col);
      max_excess = std::max(max_excess, lateral_excess);
    }
    const double accel_excess =
        next_v * next_v - v[i] * v[i] - 2.0 * lin_interp(v[i], speeds_col, accel_col) * seg[i];
    const double decel_excess =
        v[i] * v[i] - next_v * next_v - 2.0 * lin_interp(next_v, speeds_col, decel_col) * seg[i];
    max_excess = std::max({max_excess, accel_excess, decel_excess});
  }
  if (max_excess > 1e-7) {
    std::ostringstream oss;
    oss << "velocity profile violates the interpolated table constraints by " << max_excess << ".";
    fail(oss.str());
  }

  VelocityProfile out;
  out.vx = v;
  out.ax.assign(n, 0.0);
  for (int i = 0; i < n; ++i) {
    const double next_v = v[wrap_index(i + 1, n)];
    if (seg[i] > 1e-6) out.ax[i] = (next_v * next_v - v[i] * v[i]) / (2.0 * seg[i]);
    out.lap_time += seg[i] / std::max(v[i], 1e-3);
  }
  return out;
}

std::pair<Trajectory, double> build_trajectory(const std::vector<cv::Point2d>& points_xy,
                                               const cv::Mat& free_mask, const MapInfo& info,
                                               bool flip_y, const Args& args) {
  auto [d_right, d_left] =
      track_widths(points_xy, free_mask, info, flip_y, args.max_width_distance, args.width_mode);
  const HeadingCurvature hc = headings_and_curvature(points_xy);
  const VelocityProfile profile =
      velocity_profile(points_xy, hc.kappa, args.max_speed, args.min_speed, args.velocity_limits);
  Trajectory traj;
  traj.points_xy = points_xy;
  traj.d_right = std::move(d_right);
  traj.d_left = std::move(d_left);
  traj.s_m = hc.s;
  traj.psi_rad = hc.psi;
  traj.kappa_radpm = hc.kappa;
  traj.vx_mps = profile.vx;
  traj.ax_mps2 = profile.ax;
  return {traj, profile.lap_time};
}

std::string csv_number(double value) {
  double rounded = std::round(value * 1e6) / 1e6;
  if (std::abs(rounded) < 0.0000005) rounded = 0.0;
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.6f", rounded);
  std::string text(buffer);
  text.erase(text.find_last_not_of('0') + 1);
  if (!text.empty() && text.back() == '.') text.pop_back();
  return text;
}

json wpnt_array(const Trajectory& traj) {
  json wpnts = json::array();
  for (size_t i = 0; i < traj.points_xy.size(); ++i) {
    wpnts.push_back({
        {"id", static_cast<int>(i)},
        {"s_m", traj.s_m[i]},
        {"d_m", 0.0},
        {"x_m", traj.points_xy[i].x},
        {"y_m", traj.points_xy[i].y},
        {"d_right", traj.d_right[i]},
        {"d_left", traj.d_left[i]},
        {"psi_rad", traj.psi_rad[i]},
        {"kappa_radpm", traj.kappa_radpm[i]},
        {"vx_mps", traj.vx_mps[i]},
        {"ax_mps2", traj.ax_mps2[i]},
    });
  }
  return {{"header", {{"stamp", {{"sec", 0}, {"nanosec", 0}}}, {"frame_id", "map"}}},
          {"wpnts", wpnts}};
}

void draw_closed_polyline(cv::Mat& image, const std::vector<cv::Point2d>& points_xy,
                          const MapInfo& info, bool flip_y, const cv::Scalar& color,
                          int thickness) {
  const std::vector<cv::Point2d> pixels = world_to_pixel(points_xy, info, flip_y);
  std::vector<cv::Point> rounded(pixels.size());
  for (size_t i = 0; i < pixels.size(); ++i) {
    rounded[i] = {std::clamp(static_cast<int>(py_round(pixels[i].x)), 0, info.width - 1),
                  std::clamp(static_cast<int>(py_round(pixels[i].y)), 0, info.height - 1)};
  }
  for (size_t i = 0; i < rounded.size(); ++i) {
    cv::line(image, rounded[i], rounded[(i + 1) % rounded.size()], color, thickness, cv::LINE_AA);
  }
}

}  // namespace

fs::path exe_dir() {
  return fs::canonical("/proc/self/exe").parent_path();
}

fs::path default_output_dir(const fs::path& map_yaml) {
  // <offline_trajectory_generator>/output/<map stem> (binary lives in bin/).
  return exe_dir().parent_path() / "output" / map_yaml.stem();
}

fs::path default_velocity_limits_csv() {
  return exe_dir().parent_path() / "config" / "velocity_limits.csv";
}

std::vector<std::array<double, 4>> load_velocity_limits(const fs::path& path, double max_speed) {
  std::ifstream stream(path);
  if (!stream) fail("Could not read velocity limits CSV " + path.string());
  std::vector<std::array<double, 4>> limits;
  std::string line;
  while (std::getline(stream, line)) {
    const size_t comment = line.find('#');
    if (comment != std::string::npos) line.erase(comment);
    if (line.find_first_not_of(" \t\r\n") == std::string::npos) continue;
    std::array<double, 4> row{};
    std::stringstream fields(line);
    std::string field;
    int column = 0;
    while (std::getline(fields, field, ',')) {
      if (column >= kVelocityLimitColumns) {
        ++column;
        break;
      }
      try {
        row[column] = std::stod(field);
      } catch (const std::exception&) {
        fail("Could not read velocity limits CSV " + path.string() + ": bad number '" + field +
             "'");
      }
      ++column;
    }
    if (column != kVelocityLimitColumns) {
      fail("velocity limits CSV must have four columns: "
           "[speed_mps,max_accel_mps2,max_decel_mps2,max_lateral_accel_mps2].");
    }
    limits.push_back(row);
  }
  if (limits.size() < 2) fail("velocity limits CSV must contain at least two rows.");
  for (const auto& row : limits) {
    for (double value : row) {
      if (!std::isfinite(value)) fail("velocity limits CSV contains NaN or infinite values.");
    }
  }
  if (std::abs(limits[0][0]) > 1e-9) fail("velocity limits CSV must start at speed 0.0 m/s.");
  for (size_t i = 1; i < limits.size(); ++i) {
    if (limits[i][0] - limits[i - 1][0] <= 0.0) {
      fail("velocity limits CSV speeds must be strictly increasing.");
    }
  }
  for (const auto& row : limits) {
    if (row[1] <= 0.0 || row[2] <= 0.0 || row[3] <= 0.0) {
      fail("all acceleration limits in velocity limits CSV must be positive.");
    }
  }
  if (limits.back()[0] + 1e-9 < max_speed) {
    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << "velocity limits CSV ends at " << limits.back()[0]
        << " m/s but max-speed is " << max_speed << " m/s. Extend the table first.";
    fail(oss.str());
  }
  return limits;
}

void validate_args(Args& args) {
  if (args.waypoint_step <= 0.0 || args.optimizer_step <= 0.0) {
    fail("waypoint-step and optimizer-step must be positive.");
  }
  if (args.safety_width <= 0.0) fail("safety-width must be positive.");
  if (args.max_speed <= 0.0 || args.min_speed <= 0.0) fail("speed parameters must be positive.");
  if (args.min_speed > args.max_speed) fail("min-speed must be <= max-speed.");
  if (args.velocity_limits_csv.empty()) args.velocity_limits_csv = default_velocity_limits_csv();
  args.velocity_limits_csv = fs::absolute(args.velocity_limits_csv).lexically_normal();
  args.velocity_limits = load_velocity_limits(args.velocity_limits_csv, args.max_speed);
  if (args.straight_kappa_threshold < 0.0) fail("straight-kappa-threshold must be non-negative.");
  if (args.straight_min_length < 0.0 || args.straight_clearance_margin < 0.0) {
    fail("straight length and clearance parameters must be non-negative.");
  }
  if (args.straight_blend_length < 0.0) fail("straight-blend-length must be non-negative.");
  if (args.d_ratio < -1.0 || args.d_ratio > 1.0) fail("d-ratio must be within [-1, 1].");
  if (args.d_ratio_alpha_smooth_sigma < 0.0) {
    fail("d-ratio-alpha-smooth-sigma must be non-negative.");
  }
  if (args.width_mode != "distance" && args.width_mode != "raycast" &&
      args.width_mode != "hybrid") {
    fail("width-mode must be one of: distance, raycast, hybrid.");
  }
  if (args.optimizer != "mincurv" && args.optimizer != "centerline" &&
      args.optimizer != "d_ratio") {
    fail("optimizer must be one of: mincurv, centerline, d_ratio.");
  }
}

std::vector<cv::Point2d> world_to_pixel(const std::vector<cv::Point2d>& points_xy,
                                        const MapInfo& info, bool flip_y) {
  std::vector<cv::Point2d> out(points_xy.size());
  for (size_t i = 0; i < points_xy.size(); ++i) {
    out[i] = world_to_pixel_one(points_xy[i], info, flip_y);
  }
  return out;
}

std::vector<cv::Point2d> resample_closed(std::vector<cv::Point2d> points_xy, double step) {
  points_xy = remove_consecutive_duplicates(points_xy);
  if (points_xy.size() < 4) {
    fail("Need at least four points to resample a closed trajectory.");
  }
  const auto [s, total] = cumulative_s(points_xy);
  if (total <= step * 4) {
    fail("Extracted centerline is too short for the requested waypoint step.");
  }
  std::vector<cv::Point2d> closed = points_xy;
  closed.push_back(points_xy.front());
  std::vector<double> xs(closed.size()), ys(closed.size());
  for (size_t i = 0; i < closed.size(); ++i) {
    xs[i] = closed[i].x;
    ys[i] = closed[i].y;
  }
  const int sample_count = std::max(4, static_cast<int>(std::floor(total / step)));
  std::vector<cv::Point2d> out(sample_count);
  for (int k = 0; k < sample_count; ++k) {
    const double new_s = total * k / sample_count;
    out[k] = {lin_interp(new_s, s, xs), lin_interp(new_s, s, ys)};
  }
  return out;
}

GenerationResult generate_trajectory(Args& args) {
  validate_args(args);
  auto [map_info, image] = load_map_image(args.map_yaml);
  const std::string mode = map_mode(args.map_yaml);
  const cv::Mat free_raw = free_mask_from(image, map_info, mode, args.unknown_as_free);
  const cv::Mat free_mask =
      cleanup_free_mask(free_raw, args.median_kernel, args.morph_kernel,
                        args.morph_open_iterations, args.morph_close_iterations,
                        occupied_mask_from(image, map_info));
  cv::Mat skeleton = skeletonize(free_mask);
  if (args.min_track_width > 0.0) {
    // The car cannot drive where the corridor is narrower than the track
    // width, so any skeleton pixel there is scan noise, not centerline.
    const cv::Mat clearance_px = distance_transform_of(free_mask);
    cv::Mat too_narrow;
    cv::compare(clearance_px * (2.0 * map_info.resolution), args.min_track_width, too_narrow,
                cv::CMP_LT);
    skeleton.setTo(0, too_narrow);
  }
  const std::vector<cv::Point2d> center_px = extract_centerline_pixels(
      skeleton, args.skeleton_prune_iterations, args.min_skeleton_component_area);
  const bool flip_y = !args.no_flip_y;
  std::vector<cv::Point2d> center_xy = pixel_to_world(center_px, map_info, flip_y);
  if (args.smooth_sigma > 0.0 && center_xy.size() >= 5) {
    center_xy = gaussian_wrap_points(center_xy, args.smooth_sigma);
  }
  center_xy = filter_and_resample_closed(center_xy, args.optimizer_step, args);
  if (args.reverse) std::reverse(center_xy.begin(), center_xy.end());

  auto [center_right, center_left] = track_widths(center_xy, free_mask, map_info, flip_y,
                                                  args.max_width_distance, args.width_mode);
  std::vector<cv::Point2d> optimized_xy =
      optimize_raceline(center_xy, center_right, center_left, args);
  optimized_xy = filter_and_resample_closed(optimized_xy, args.optimizer_step, args);
  std::vector<cv::Point2d> global_xy =
      filter_and_resample_closed(optimized_xy, args.waypoint_step, args);
  if (args.raceline_smooth_sigma > 0.0 && global_xy.size() >= 5) {
    // Remove the piecewise-linear vertex kinks left by the coarse optimizer
    // grid (they read as phantom steering-limit violations at fine spacing).
    global_xy = gaussian_wrap_points(global_xy, args.raceline_smooth_sigma);
  }
  bool straightened = false;
  const std::vector<cv::Point2d> straightened_xy =
      straighten_straight_segments(global_xy, free_mask, map_info, flip_y, args, straightened);
  if (straightened) global_xy = resample_closed(straightened_xy, args.waypoint_step);
  if (args.max_curvature > 0.0) {
    global_xy = limit_curvature_spikes(global_xy, args.max_curvature);
  }

  const std::vector<cv::Point2d> center_output_xy =
      filter_and_resample_closed(center_xy, args.waypoint_step, args);
  GenerationResult result;
  result.map_info = map_info;
  result.image = image;
  result.free_mask = free_mask;
  result.flip_y = flip_y;
  std::tie(result.center_traj, std::ignore) =
      build_trajectory(center_output_xy, free_mask, map_info, flip_y, args);
  std::tie(result.global_traj, result.lap_time) =
      build_trajectory(global_xy, free_mask, map_info, flip_y, args);

  result.off_map_wpnts =
      count_off_map_waypoints(result.global_traj.points_xy, free_mask, map_info, flip_y);
  if (result.off_map_wpnts) {
    std::fprintf(stderr,
                 "[WARN] %d/%zu waypoints lie outside the drivable free space — the extraction "
                 "likely picked a noise region. Check debug_overlay.png; raise "
                 "--min-track-width or the map-cleanup parameters (median/morph kernels).\n",
                 result.off_map_wpnts, result.global_traj.points_xy.size());
  }

  const auto [min_clearance, clearance_violations] =
      report_min_clearance(result.global_traj.points_xy, free_mask, map_info, flip_y, args);
  std::printf(
      "[INFO] min wall clearance along final raceline: %.3f m (half-width %.2f m, corridor "
      "target %.2f m)\n",
      min_clearance, args.safety_width * 0.5, args.safety_width * 0.5 + args.boundary_margin);
  if (clearance_violations) {
    std::fprintf(stderr,
                 "[WARN] %d dense samples of the final raceline are closer to a wall than the "
                 "car half-width %.2f m — post-processing (smoothing/straightening) may have "
                 "pushed the line outward. Check debug_overlay.png near the tightest spots.\n",
                 clearance_violations, args.safety_width * 0.5);
  }

  for (double kappa : result.global_traj.kappa_radpm) {
    result.max_abs_kappa = std::max(result.max_abs_kappa, std::abs(kappa));
  }
  if (args.max_curvature > 0.0) {
    result.kappa_violations = static_cast<int>(std::count_if(
        result.global_traj.kappa_radpm.begin(), result.global_traj.kappa_radpm.end(),
        [&](double kappa) { return std::abs(kappa) > args.max_curvature; }));
    if (result.kappa_violations) {
      std::fprintf(stderr,
                   "[WARN] %d/%zu waypoints exceed the steering limit --max-curvature %.2f rad/m "
                   "(max |kappa| = %.2f) — the raceline is not drivable as-is. Widen "
                   "--safety-width/--boundary-margin, raise --smooth-sigma, or check whether "
                   "the track corner itself is tighter than the car's turning radius.\n",
                   result.kappa_violations, result.global_traj.points_xy.size(),
                   args.max_curvature, result.max_abs_kappa);
    }
  }
  return result;
}

void write_outputs(const fs::path& output_dir, const GenerationResult& result, const Args& args) {
  fs::create_directories(output_dir);
  const Trajectory& center = result.center_traj;
  const Trajectory& global = result.global_traj;

  {
    std::ofstream stream(output_dir / "centerline.csv");
    stream.precision(17);  // full float64 round-trip, like Python's str(float)
    stream << "x_m,y_m,d_right,d_left\n";
    for (size_t i = 0; i < center.points_xy.size(); ++i) {
      stream << center.points_xy[i].x << ',' << center.points_xy[i].y << ','
             << center.d_right[i] << ',' << center.d_left[i] << '\n';
    }
  }
  {
    std::ofstream stream(output_dir / "global_waypoints.csv");
    stream << "id,s,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2,d_left,d_right\n";
    for (size_t i = 0; i < global.points_xy.size(); ++i) {
      stream << i << ',' << csv_number(global.s_m[i]) << ',' << csv_number(global.points_xy[i].x)
             << ',' << csv_number(global.points_xy[i].y) << ',' << csv_number(global.psi_rad[i])
             << ',' << csv_number(global.kappa_radpm[i]) << ',' << csv_number(global.vx_mps[i])
             << ',' << csv_number(global.ax_mps2[i]) << ',' << csv_number(global.d_left[i]) << ','
             << csv_number(global.d_right[i]) << '\n';
    }
  }

  std::ostringstream lap_text;
  lap_text.precision(3);
  lap_text << std::fixed << result.lap_time;
  const json payload = {
      {"map_info_str",
       {{"data", "offline generator; map=" + result.map_info.yaml_path.string() +
                     "; optimizer=" + args.optimizer +
                     "; estimated_lap_time=" + lap_text.str() + "s"}}},
      {"est_lap_time", {{"data", result.lap_time}}},
      {"centerline_markers", {{"markers", json::array()}}},
      {"centerline_waypoints", wpnt_array(center)},
      {"global_traj_markers_iqp", {{"markers", json::array()}}},
      {"global_traj_wpnts_iqp", wpnt_array(global)},
      {"trackbounds_markers", {{"markers", json::array()}}},
  };
  std::ofstream(output_dir / "global_waypoints.json") << payload.dump(2);

  json velocity_limits = json::array();
  for (const auto& row : args.velocity_limits) {
    velocity_limits.push_back({row[0], row[1], row[2], row[3]});
  }
  const json metadata = {
      {"map_yaml", result.map_info.yaml_path.string()},
      {"map_image", result.map_info.image_path.string()},
      {"resolution", result.map_info.resolution},
      {"origin",
       {result.map_info.origin_x, result.map_info.origin_y, result.map_info.origin_yaw}},
      {"waypoint_count", static_cast<int>(global.points_xy.size())},
      {"centerline_count", static_cast<int>(center.points_xy.size())},
      {"estimated_lap_time_sec", result.lap_time},
      // Validation summary (the GUI status bar reads these).
      {"off_map_wpnts", result.off_map_wpnts},
      {"kappa_violations", result.kappa_violations},
      {"max_abs_kappa", result.max_abs_kappa},
      {"args",
       {
           {"map_yaml", args.map_yaml.string()},
           {"output_dir", output_dir.string()},
           {"waypoint_step", args.waypoint_step},
           {"optimizer_step", args.optimizer_step},
           {"safety_width", args.safety_width},
           {"boundary_margin", args.boundary_margin},
           {"max_width_distance", args.max_width_distance},
           {"width_mode", args.width_mode},
           {"max_speed", args.max_speed},
           {"min_speed", args.min_speed},
           {"velocity_limits_csv", args.velocity_limits_csv.string()},
           {"max_curvature", args.max_curvature},
           {"smooth_sigma", args.smooth_sigma},
           {"raceline_smooth_sigma", args.raceline_smooth_sigma},
           {"median_kernel", args.median_kernel},
           {"morph_kernel", args.morph_kernel},
           {"morph_open_iterations", args.morph_open_iterations},
           {"morph_close_iterations", args.morph_close_iterations},
           {"skeleton_prune_iterations", args.skeleton_prune_iterations},
           {"min_skeleton_component_area", args.min_skeleton_component_area},
           {"min_track_width", args.min_track_width},
           {"min_centerline_angle", args.min_centerline_angle},
           {"spike_filter_iterations", args.spike_filter_iterations},
           {"optimizer", args.optimizer},
           {"max_optimizer_iter", args.max_optimizer_iter},
           {"curvature_weight", args.curvature_weight},
           {"smooth_weight", args.smooth_weight},
           {"length_weight", args.length_weight},
           {"d_ratio", args.d_ratio},
           {"d_ratio_alpha_smooth_sigma", args.d_ratio_alpha_smooth_sigma},
           {"straighten_straights", args.straighten_straights},
           {"straight_kappa_threshold", args.straight_kappa_threshold},
           {"straight_min_length", args.straight_min_length},
           {"straight_clearance_margin", args.straight_clearance_margin},
           {"straight_blend_length", args.straight_blend_length},
           {"reverse", args.reverse},
           {"no_flip_y", args.no_flip_y},
           {"unknown_as_free", args.unknown_as_free},
           {"debug_image", args.debug_image},
           {"velocity_limits", velocity_limits},
       }},
  };
  std::ofstream(output_dir / "metadata.json") << metadata.dump(2);
}

void write_debug_image(const fs::path& output_dir, const GenerationResult& result) {
  cv::Mat debug;
  cv::cvtColor(result.image, debug, cv::COLOR_GRAY2BGR);
  draw_closed_polyline(debug, result.center_traj.points_xy, result.map_info, result.flip_y,
                       {255, 0, 0}, 1);
  draw_closed_polyline(debug, result.global_traj.points_xy, result.map_info, result.flip_y,
                       {0, 0, 255}, 1);
  cv::imwrite((output_dir / "debug_overlay.png").string(), debug);
}

void write_preview_image(const fs::path& path, const GenerationResult& result) {
  // Same colors as the GUI preview (BGR here): centerline blue-ish, raceline
  // red-ish, start point green.
  cv::Mat preview;
  cv::cvtColor(result.image, preview, cv::COLOR_GRAY2BGR);
  const int thickness = std::max(
      1, static_cast<int>(std::lround(
             std::max(result.map_info.width, result.map_info.height) / 700.0)));
  draw_closed_polyline(preview, result.center_traj.points_xy, result.map_info, result.flip_y,
                       {255, 120, 0}, thickness);
  draw_closed_polyline(preview, result.global_traj.points_xy, result.map_info, result.flip_y,
                       {45, 70, 255}, thickness + 1);
  if (!result.global_traj.points_xy.empty()) {
    const std::vector<cv::Point2d> start_px =
        world_to_pixel({result.global_traj.points_xy.front()}, result.map_info, result.flip_y);
    const cv::Point start(
        std::clamp(static_cast<int>(py_round(start_px[0].x)), 0, result.map_info.width - 1),
        std::clamp(static_cast<int>(py_round(start_px[0].y)), 0, result.map_info.height - 1));
    cv::circle(preview, start, std::max(3, thickness + 2), {90, 220, 0}, -1, cv::LINE_AA);
  }
  cv::imwrite(path.string(), preview);
}

}  // namespace otg
