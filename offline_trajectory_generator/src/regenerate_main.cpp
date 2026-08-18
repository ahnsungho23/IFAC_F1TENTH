// Headless regeneration driver for the map_creator pipeline.
//
// Loads the exact same gui_params.yaml the trajectory GUI uses, overrides only
// the input map (the painted obstacle map) and the output directory, runs the
// generator, applies the physical swap gates from
// learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md (3.4), and writes
// global_waypoints.json + metadata.json + gate_report.json.
//
// Exit codes: 0 = generated and all physical gates passed,
// 1 = gate failure / generation error, 2 = bad invocation.
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>

#include "trajectory_core.hpp"

namespace {

using json = nlohmann::ordered_json;
namespace fs = otg::fs;

fs::path expand_user(std::string text) {
  if (!text.empty() && text[0] == '~') {
    const char* home = std::getenv("HOME");
    if (home != nullptr) text = home + text.substr(1);
  }
  return text;
}

// Mirrors trajectory_gui load: defaults first, then gui_params.yaml overrides.
// ponytail: skips the GUI's slider-step re-normalization — the GUI already
// writes normalized values; add it here only if hand-edited YAMLs become common.
otg::Args args_from_gui_params(const fs::path& params_yaml) {
  otg::Args args;
  args.width_mode = "distance";  // GUI default differs from the CLI default
  args.debug_image = true;
  YAML::Node node;
  if (fs::exists(params_yaml)) node = YAML::LoadFile(params_yaml.string());

  const auto get_double = [&](const char* key, double fallback) {
    return node[key] ? node[key].as<double>(fallback) : fallback;
  };
  const auto get_int = [&](const char* key, int fallback) {
    return node[key] ? node[key].as<int>(fallback) : fallback;
  };
  const auto get_bool = [&](const char* key, bool fallback) {
    return node[key] ? node[key].as<bool>(fallback) : fallback;
  };

  args.waypoint_step = get_double("waypoint_step", args.waypoint_step);
  args.optimizer_step = get_double("optimizer_step", args.optimizer_step);
  args.raceline_smooth_sigma = get_double("raceline_smooth_sigma", args.raceline_smooth_sigma);
  args.safety_width = get_double("safety_width", args.safety_width);
  args.boundary_margin = get_double("boundary_margin", args.boundary_margin);
  args.max_width_distance = get_double("max_width_distance", args.max_width_distance);
  args.max_speed = get_double("max_speed", args.max_speed);
  args.min_speed = get_double("min_speed", args.min_speed);
  args.max_curvature = get_double("max_curvature", args.max_curvature);
  args.smooth_sigma = get_double("smooth_sigma", args.smooth_sigma);
  args.median_kernel = get_int("median_kernel", args.median_kernel);
  args.morph_kernel = get_int("morph_kernel", args.morph_kernel);
  args.morph_open_iterations = get_int("morph_open_iterations", args.morph_open_iterations);
  args.morph_close_iterations = get_int("morph_close_iterations", args.morph_close_iterations);
  args.skeleton_prune_iterations =
      get_int("skeleton_prune_iterations", args.skeleton_prune_iterations);
  args.min_skeleton_component_area =
      get_int("min_skeleton_component_area", args.min_skeleton_component_area);
  args.min_track_width = get_double("min_track_width", args.min_track_width);
  args.min_centerline_angle = get_double("min_centerline_angle", args.min_centerline_angle);
  args.spike_filter_iterations = get_int("spike_filter_iterations", args.spike_filter_iterations);
  args.max_optimizer_iter = get_int("max_optimizer_iter", args.max_optimizer_iter);
  args.curvature_weight = get_double("curvature_weight", args.curvature_weight);
  args.smooth_weight = get_double("smooth_weight", args.smooth_weight);
  args.length_weight = get_double("length_weight", args.length_weight);
  args.d_ratio = get_double("d_ratio", args.d_ratio);
  args.d_ratio_alpha_smooth_sigma =
      get_double("d_ratio_alpha_smooth_sigma", args.d_ratio_alpha_smooth_sigma);
  args.straight_kappa_threshold =
      get_double("straight_kappa_threshold", args.straight_kappa_threshold);
  args.straight_min_length = get_double("straight_min_length", args.straight_min_length);
  args.straight_clearance_margin =
      get_double("straight_clearance_margin", args.straight_clearance_margin);
  args.straight_blend_length = get_double("straight_blend_length", args.straight_blend_length);
  args.straighten_straights = get_bool("straighten_straights", args.straighten_straights);
  args.reverse = get_bool("reverse", args.reverse);
  args.no_flip_y = get_bool("no_flip_y", args.no_flip_y);
  args.unknown_as_free = get_bool("unknown_as_free", args.unknown_as_free);

  if (node["optimizer"]) args.optimizer = node["optimizer"].as<std::string>();
  if (args.optimizer != "centerline" && args.optimizer != "mincurv" &&
      args.optimizer != "d_ratio") {
    args.optimizer = "mincurv";  // saved YAML may hold a removed optimizer
  }
  if (node["width_mode"]) args.width_mode = node["width_mode"].as<std::string>();

  if (node["velocity_limits_csv"]) {
    fs::path csv = expand_user(node["velocity_limits_csv"].as<std::string>());
    // The GUI stores this path relative to the repository root.
    if (!csv.is_absolute()) csv = otg::exe_dir().parent_path().parent_path() / csv;
    args.velocity_limits_csv = csv;
  }
  return args;
}

std::vector<cv::Point2d> densify_closed(const std::vector<cv::Point2d>& points_xy, double step) {
  std::vector<cv::Point2d> out;
  const size_t n = points_xy.size();
  for (size_t i = 0; i < n; ++i) {
    const cv::Point2d& a = points_xy[i];
    const cv::Point2d seg = points_xy[(i + 1) % n] - a;
    const int count = std::max(1, static_cast<int>(std::ceil(std::hypot(seg.x, seg.y) / step)));
    for (int k = 0; k < count; ++k) {
      out.push_back(a + seg * (static_cast<double>(k) / count));
    }
  }
  return out;
}

double min_distance_to_aabb(const std::vector<cv::Point2d>& points_xy, const json& box) {
  const double x_min = box.at("x_min").get<double>();
  const double x_max = box.at("x_max").get<double>();
  const double y_min = box.at("y_min").get<double>();
  const double y_max = box.at("y_max").get<double>();
  double best = std::numeric_limits<double>::infinity();
  for (const cv::Point2d& p : points_xy) {
    const double dx = std::max({x_min - p.x, 0.0, p.x - x_max});
    const double dy = std::max({y_min - p.y, 0.0, p.y - y_max});
    best = std::min(best, std::hypot(dx, dy));
  }
  return best;
}

}  // namespace

int main(int argc, char** argv) {
  fs::path map_yaml, gui_params, output_dir, obstacles_json;
  double min_clearance_required = 0.42;
  double max_kappa = 3.2;
  std::optional<double> safety_width, smooth_sigma;
  std::optional<int> morph_kernel;
  bool preview_png = false;

  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    const auto next = [&]() -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "Option %s requires a value.\n", option.c_str());
        std::exit(2);
      }
      return argv[++i];
    };
    if (option == "--map-yaml") {
      map_yaml = next();
    } else if (option == "--gui-params") {
      gui_params = next();
    } else if (option == "--output-dir") {
      output_dir = next();
    } else if (option == "--obstacles-json") {
      obstacles_json = next();
    } else if (option == "--min-clearance") {
      min_clearance_required = std::stod(next());
    } else if (option == "--max-kappa") {
      max_kappa = std::stod(next());
    } else if (option == "--safety-width") {
      safety_width = std::stod(next());
    } else if (option == "--smooth-sigma") {
      smooth_sigma = std::stod(next());
    } else if (option == "--morph-kernel") {
      morph_kernel = std::stoi(next());
    } else if (option == "--preview-png") {
      preview_png = true;
    } else {
      std::fprintf(stderr, "Unknown option: %s\n", option.c_str());
      return 2;
    }
  }
  if (map_yaml.empty() || output_dir.empty()) {
    std::fprintf(stderr,
                 "Usage: regenerate_obstacle_map --map-yaml <painted map yaml> --output-dir <dir>"
                 " [--gui-params <yaml>] [--obstacles-json <json>] [--min-clearance M]"
                 " [--max-kappa K] [--safety-width M] [--smooth-sigma S] [--morph-kernel N]"
                 " [--preview-png]\n");
    return 2;
  }
  if (gui_params.empty()) gui_params = otg::exe_dir().parent_path() / "gui_params.yaml";
  if (!fs::is_regular_file(map_yaml)) {
    std::fprintf(stderr, "[regenerate_obstacle_map] map yaml not found: %s\n", map_yaml.c_str());
    return 2;
  }

  otg::Args args = args_from_gui_params(gui_params);
  args.map_yaml = map_yaml;
  args.output_dir = output_dir;
  if (safety_width) args.safety_width = *safety_width;
  if (smooth_sigma) args.smooth_sigma = *smooth_sigma;
  if (morph_kernel) args.morph_kernel = *morph_kernel;

  const auto started = std::chrono::steady_clock::now();
  const auto elapsed_sec = [&] {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  };

  otg::GenerationResult result;
  try {
    result = otg::generate_trajectory(args);
  } catch (const std::exception& exc) {
    const json report = {
        {"status", "generation_error"},
        {"error", exc.what()},
        {"elapsed_sec", elapsed_sec()},
    };
    fs::create_directories(output_dir);
    std::ofstream(output_dir / "gate_report.json") << report.dump(2);
    std::fprintf(stderr, "[regenerate_obstacle_map] generation failed: %s\n", exc.what());
    return 1;
  }

  const otg::Trajectory& traj = result.global_traj;
  json gates = json::object();

  gates["off_map"] = {{"passed", result.off_map_wpnts == 0},
                      {"off_map_wpnts", result.off_map_wpnts}};

  bool finite = true;
  for (size_t i = 0; i < traj.points_xy.size(); ++i) {
    finite = finite && std::isfinite(traj.points_xy[i].x) &&
             std::isfinite(traj.points_xy[i].y) && std::isfinite(traj.s_m[i]) &&
             std::isfinite(traj.kappa_radpm[i]);
  }
  gates["finite"] = {{"passed", finite}};

  bool s_increasing = true;
  for (size_t i = 1; i < traj.s_m.size(); ++i) {
    s_increasing = s_increasing && traj.s_m[i] > traj.s_m[i - 1];
  }
  gates["s_strictly_increasing"] = {{"passed", s_increasing}};

  const double max_abs_kappa = result.max_abs_kappa;
  gates["kappa_physical"] = {{"passed", max_abs_kappa <= max_kappa},
                             {"max_abs_kappa", max_abs_kappa},
                             {"limit", max_kappa}};

  json clearance_info = {{"checked", false}};
  if (!obstacles_json.empty() && fs::is_regular_file(obstacles_json)) {
    json obstacles_doc;
    std::ifstream(obstacles_json) >> obstacles_doc;
    const json obstacles = obstacles_doc.value("obstacles", json::array());
    const std::vector<cv::Point2d> dense = densify_closed(traj.points_xy, 0.02);
    double min_clear = std::numeric_limits<double>::infinity();
    json per_obstacle = json::array();
    for (const json& obs : obstacles) {
      const double dist = min_distance_to_aabb(dense, obs);
      per_obstacle.push_back({{"id", obs.value("id", json())}, {"min_distance_m", dist}});
      min_clear = std::min(min_clear, dist);
    }
    const bool unbounded = std::isinf(min_clear);
    clearance_info = {
        {"checked", true},
        {"min_clearance_m", unbounded ? json() : json(min_clear)},
        {"required_m", min_clearance_required},
        {"per_obstacle", per_obstacle},
    };
    gates["obstacle_clearance"] = {
        {"passed", unbounded || min_clear >= min_clearance_required},
        {"checked", true},
        {"min_clearance_m", unbounded ? json() : json(min_clear)},
        {"required_m", min_clearance_required},
    };
  }

  // Quality-only indicators (never block the swap; MAP_CREATOR_PROPOSAL 3.4).
  const json quality = {
      {"kappa_violations_generator_limit", result.kappa_violations},
      {"generator_max_curvature", args.max_curvature},
      {"waypoint_count", static_cast<int>(traj.s_m.size())},
      {"lap_time_estimate_sec", result.lap_time},
  };

  bool all_passed = true;
  for (const auto& [name, gate] : gates.items()) {
    all_passed = all_passed && gate.at("passed").get<bool>();
  }
  const json report = {
      {"status", all_passed ? "ok" : "gate_failure"},
      {"gates", gates},
      {"quality", quality},
      {"clearance", clearance_info},
      {"elapsed_sec", elapsed_sec()},
      {"map_yaml", map_yaml.string()},
      {"gui_params", gui_params.string()},
  };

  fs::create_directories(output_dir);
  if (all_passed) {
    otg::write_outputs(output_dir, result, args);
    if (preview_png) {
      // 베이스라인 생성 경로의 debug_overlay.png 와 짝이 되는 이름.
      // 이쪽은 칠해진 obstacle_map 위에 재생성 라인을 그린 것이다.
      otg::write_preview_image(output_dir / "obstacle_debug_overlay.png", result);
    }
  }
  std::ofstream(output_dir / "gate_report.json") << report.dump(2);

  std::string gate_summary;
  for (const auto& [name, gate] : gates.items()) {
    if (!gate_summary.empty()) gate_summary += ", ";
    gate_summary += name + std::string(gate.at("passed").get<bool>() ? "=PASS" : "=FAIL");
  }
  std::printf("[regenerate_obstacle_map] %s (elapsed %.1fs, gates: %s)\n",
              all_passed ? "ok" : "gate_failure", elapsed_sec(), gate_summary.c_str());
  return all_passed ? 0 : 1;
}
