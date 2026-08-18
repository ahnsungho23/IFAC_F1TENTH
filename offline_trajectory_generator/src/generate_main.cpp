// CLI: generate a global raceline from a ROS map YAML without running ROS 2.
#include <cstdio>
#include <cstring>
#include <exception>
#include <functional>
#include <map>
#include <string>

#include "trajectory_core.hpp"

namespace {

void print_usage() {
  std::puts(
      "Usage: generate_global_trajectory --map-yaml <path> [options]\n"
      "Options (defaults in brackets):\n"
      "  --map-yaml PATH             SLAM map YAML path (required)\n"
      "  --output-dir PATH           output dir [output/<map_name>]\n"
      "  --waypoint-step M           final waypoint spacing [0.1]\n"
      "  --optimizer-step M          internal optimizer spacing [0.2]\n"
      "  --safety-width M            vehicle+safety width [0.35]\n"
      "  --boundary-margin M         extra margin from walls [0.03]\n"
      "  --max-width-distance M      raycast limit for track width [5.0]\n"
      "  --width-mode MODE           distance|raycast|hybrid [hybrid]\n"
      "  --max-speed MPS             maximum waypoint speed [4.0]\n"
      "  --min-speed MPS             minimum waypoint speed [1.0]\n"
      "  --velocity-limits-csv PATH  4-column limits CSV [config/velocity_limits.csv]\n"
      "  --max-curvature RADPM       steering limit; 0 disables [1.2]\n"
      "  --smooth-sigma S            closed-curve smoothing sigma [2.0]\n"
      "  --raceline-smooth-sigma S   final raceline smoothing sigma [1.0]\n"
      "  --median-kernel N           map denoise kernel [3]\n"
      "  --morph-kernel N            map cleanup kernel [5]\n"
      "  --morph-open-iterations N   [1]\n"
      "  --morph-close-iterations N  [1]\n"
      "  --skeleton-prune-iterations N   [80]\n"
      "  --min-skeleton-component-area N [40]\n"
      "  --min-track-width M         drop skeleton in narrow corridors [0.3]\n"
      "  --min-centerline-angle DEG  spike filter angle [75]\n"
      "  --spike-filter-iterations N [8]\n"
      "  --optimizer NAME            mincurv|centerline|d_ratio [centerline]\n"
      "  --max-optimizer-iter N      [200]\n"
      "  --curvature-weight W        [1.0]\n"
      "  --smooth-weight W           [0.04]\n"
      "  --length-weight W           [0.002]\n"
      "  --d-ratio R                 lateral ratio in [-1,1] for d_ratio [0.0]\n"
      "  --d-ratio-alpha-smooth-sigma S  [0.0]\n"
      "  --no-straighten-straights   disable straight replacement\n"
      "  --straight-kappa-threshold K    [0.2]\n"
      "  --straight-min-length M     [1.5]\n"
      "  --straight-clearance-margin M   [0.03]\n"
      "  --straight-blend-length M   [0.5]\n"
      "  --reverse                   reverse waypoint order\n"
      "  --no-flip-y                 disable ROS map image y-axis flip\n"
      "  --unknown-as-free           treat unknown gray pixels as free\n"
      "  --debug-image               write debug_overlay.png");
}

}  // namespace

int main(int argc, char** argv) {
  otg::Args args;
  std::map<std::string, std::function<void(const std::string&)>> value_options = {
      {"--map-yaml", [&](const std::string& v) { args.map_yaml = v; }},
      {"--output-dir", [&](const std::string& v) { args.output_dir = v; }},
      {"--waypoint-step", [&](const std::string& v) { args.waypoint_step = std::stod(v); }},
      {"--optimizer-step", [&](const std::string& v) { args.optimizer_step = std::stod(v); }},
      {"--safety-width", [&](const std::string& v) { args.safety_width = std::stod(v); }},
      {"--boundary-margin", [&](const std::string& v) { args.boundary_margin = std::stod(v); }},
      {"--max-width-distance",
       [&](const std::string& v) { args.max_width_distance = std::stod(v); }},
      {"--width-mode", [&](const std::string& v) { args.width_mode = v; }},
      {"--max-speed", [&](const std::string& v) { args.max_speed = std::stod(v); }},
      {"--min-speed", [&](const std::string& v) { args.min_speed = std::stod(v); }},
      {"--velocity-limits-csv",
       [&](const std::string& v) { args.velocity_limits_csv = v; }},
      {"--max-curvature", [&](const std::string& v) { args.max_curvature = std::stod(v); }},
      {"--smooth-sigma", [&](const std::string& v) { args.smooth_sigma = std::stod(v); }},
      {"--raceline-smooth-sigma",
       [&](const std::string& v) { args.raceline_smooth_sigma = std::stod(v); }},
      {"--median-kernel", [&](const std::string& v) { args.median_kernel = std::stoi(v); }},
      {"--morph-kernel", [&](const std::string& v) { args.morph_kernel = std::stoi(v); }},
      {"--morph-open-iterations",
       [&](const std::string& v) { args.morph_open_iterations = std::stoi(v); }},
      {"--morph-close-iterations",
       [&](const std::string& v) { args.morph_close_iterations = std::stoi(v); }},
      {"--skeleton-prune-iterations",
       [&](const std::string& v) { args.skeleton_prune_iterations = std::stoi(v); }},
      {"--min-skeleton-component-area",
       [&](const std::string& v) { args.min_skeleton_component_area = std::stoi(v); }},
      {"--min-track-width", [&](const std::string& v) { args.min_track_width = std::stod(v); }},
      {"--min-centerline-angle",
       [&](const std::string& v) { args.min_centerline_angle = std::stod(v); }},
      {"--spike-filter-iterations",
       [&](const std::string& v) { args.spike_filter_iterations = std::stoi(v); }},
      {"--optimizer", [&](const std::string& v) { args.optimizer = v; }},
      {"--max-optimizer-iter",
       [&](const std::string& v) { args.max_optimizer_iter = std::stoi(v); }},
      {"--curvature-weight",
       [&](const std::string& v) { args.curvature_weight = std::stod(v); }},
      {"--smooth-weight", [&](const std::string& v) { args.smooth_weight = std::stod(v); }},
      {"--length-weight", [&](const std::string& v) { args.length_weight = std::stod(v); }},
      {"--d-ratio", [&](const std::string& v) { args.d_ratio = std::stod(v); }},
      {"--d-ratio-alpha-smooth-sigma",
       [&](const std::string& v) { args.d_ratio_alpha_smooth_sigma = std::stod(v); }},
      {"--straight-kappa-threshold",
       [&](const std::string& v) { args.straight_kappa_threshold = std::stod(v); }},
      {"--straight-min-length",
       [&](const std::string& v) { args.straight_min_length = std::stod(v); }},
      {"--straight-clearance-margin",
       [&](const std::string& v) { args.straight_clearance_margin = std::stod(v); }},
      {"--straight-blend-length",
       [&](const std::string& v) { args.straight_blend_length = std::stod(v); }},
  };
  std::map<std::string, std::function<void()>> flag_options = {
      {"--no-straighten-straights", [&] { args.straighten_straights = false; }},
      {"--reverse", [&] { args.reverse = true; }},
      {"--no-flip-y", [&] { args.no_flip_y = true; }},
      {"--unknown-as-free", [&] { args.unknown_as_free = true; }},
      {"--debug-image", [&] { args.debug_image = true; }},
  };

  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    if (option == "-h" || option == "--help") {
      print_usage();
      return 0;
    }
    if (const auto flag = flag_options.find(option); flag != flag_options.end()) {
      flag->second();
      continue;
    }
    const auto value_option = value_options.find(option);
    if (value_option == value_options.end()) {
      std::fprintf(stderr, "Unknown option: %s\n", option.c_str());
      return 2;
    }
    if (i + 1 >= argc) {
      std::fprintf(stderr, "Option %s requires a value.\n", option.c_str());
      return 2;
    }
    try {
      value_option->second(argv[++i]);
    } catch (const std::exception&) {
      std::fprintf(stderr, "Invalid value for %s: %s\n", option.c_str(), argv[i]);
      return 2;
    }
  }
  if (args.map_yaml.empty()) {
    std::fprintf(stderr, "--map-yaml is required.\n");
    return 2;
  }

  try {
    otg::GenerationResult result = otg::generate_trajectory(args);
    otg::fs::path output_dir = args.output_dir;
    if (output_dir.empty()) output_dir = otg::default_output_dir(args.map_yaml);
    otg::write_outputs(output_dir, result, args);
    if (args.debug_image) otg::write_debug_image(output_dir, result);
    std::printf("Wrote %s\n", (output_dir / "global_waypoints.json").c_str());
    std::printf("Wrote %s\n", (output_dir / "global_waypoints.csv").c_str());
    std::printf("Waypoints: %zu, estimated lap time: %.3fs\n",
                result.global_traj.points_xy.size(), result.lap_time);
  } catch (const std::exception& exc) {
    std::fprintf(stderr, "Error: %s\n", exc.what());
    return 1;
  }
  return 0;
}
