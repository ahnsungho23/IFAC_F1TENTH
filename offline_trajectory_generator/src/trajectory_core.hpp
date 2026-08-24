// Standalone global trajectory generator core (C++ port of the former
// generate_global_trajectory.py). No ROS dependencies.
#pragma once

#include <array>
#include <filesystem>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace otg {

namespace fs = std::filesystem;

struct MapInfo {
  fs::path yaml_path;
  fs::path image_path;
  double resolution = 0.05;
  double origin_x = 0.0;
  double origin_y = 0.0;
  double origin_yaw = 0.0;
  int negate = 0;
  double occupied_thresh = 0.65;
  double free_thresh = 0.196;
  int height = 0;
  int width = 0;
};

struct Trajectory {
  std::vector<cv::Point2d> points_xy;
  std::vector<double> d_right, d_left, s_m, psi_rad, kappa_radpm, vx_mps, ax_mps2;
};

// All generator parameters (defaults match the former argparse defaults).
struct Args {
  fs::path map_yaml;
  fs::path output_dir;  // empty -> <exe>/../output/<map stem>
  double waypoint_step = 0.1;
  double optimizer_step = 0.2;
  double safety_width = 0.35;
  double boundary_margin = 0.03;
  double max_width_distance = 5.0;
  std::string width_mode = "hybrid";  // distance | raycast | hybrid
  double max_speed = 4.0;
  double min_speed = 1.0;
  fs::path velocity_limits_csv;  // empty -> <exe>/../config/velocity_limits.csv
  double max_curvature = 1.2;
  double smooth_sigma = 2.0;
  double raceline_smooth_sigma = 1.0;
  int median_kernel = 3;
  int morph_kernel = 5;
  int morph_open_iterations = 1;
  int morph_close_iterations = 1;
  int skeleton_prune_iterations = 80;
  int min_skeleton_component_area = 40;
  double min_track_width = 0.3;
  double min_centerline_angle = 75.0;
  int spike_filter_iterations = 8;
  std::string optimizer = "centerline";  // mincurv | centerline | d_ratio
  int max_optimizer_iter = 200;
  double curvature_weight = 1.0;
  double smooth_weight = 0.04;
  double length_weight = 0.002;
  double d_ratio = 0.0;
  double d_ratio_alpha_smooth_sigma = 0.0;
  bool straighten_straights = true;
  double straight_kappa_threshold = 0.2;
  double straight_min_length = 1.5;
  double straight_clearance_margin = 0.03;
  double straight_blend_length = 0.5;
  bool reverse = false;
  bool no_flip_y = false;
  bool unknown_as_free = false;
  bool debug_image = false;
  // Bake RViz MarkerArrays into global_waypoints.json when requested by the CLI/GUI.
  bool emit_markers = false;
  // Filled by validate_args() from velocity_limits_csv:
  // rows of [speed_mps, max_accel_mps2, max_decel_mps2, max_lateral_accel_mps2]
  std::vector<std::array<double, 4>> velocity_limits;
};

struct GenerationResult {
  MapInfo map_info;
  cv::Mat image;      // raw grayscale map image
  cv::Mat free_mask;  // cleaned drivable free space (0/255)
  Trajectory center_traj;
  Trajectory global_traj;
  double lap_time = 0.0;
  bool flip_y = true;
  // Waypoints outside drivable free space (extraction latched onto noise).
  int off_map_wpnts = 0;
  // Waypoints whose curvature exceeds the steering limit (max_curvature).
  int kappa_violations = 0;
  double max_abs_kappa = 0.0;
};

fs::path exe_dir();
fs::path default_output_dir(const fs::path& map_yaml);
fs::path default_velocity_limits_csv();

// Throws std::runtime_error with a user-facing message on invalid input.
void validate_args(Args& args);
std::vector<std::array<double, 4>> load_velocity_limits(const fs::path& path, double max_speed);

GenerationResult generate_trajectory(Args& args);

void write_outputs(const fs::path& output_dir, const GenerationResult& result, const Args& args);
void write_debug_image(const fs::path& output_dir, const GenerationResult& result);
// Preview overlay (same colors as the GUI preview) used by the obstacle-map
// regeneration driver's --preview-png.
void write_preview_image(const fs::path& path, const GenerationResult& result);

std::vector<cv::Point2d> world_to_pixel(const std::vector<cv::Point2d>& points_xy,
                                        const MapInfo& info, bool flip_y);
std::vector<cv::Point2d> resample_closed(std::vector<cv::Point2d> points_xy, double step);

}  // namespace otg
