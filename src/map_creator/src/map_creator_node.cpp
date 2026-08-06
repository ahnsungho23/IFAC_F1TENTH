// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.
//
// map_creator_node: lap-transition obstacle_map pipeline
// (learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md).
//
// Laps 1-2: accumulate confirmed static obstacles (/static_obs) into a ledger.
// At the lap 2 -> 3 transition: freeze -> per-obstacle left/right decision via
// the SHARED RacelineSplinePlanner::evaluateObstacleScenario (map_creator's own
// tuned parameter snapshot) -> paint the NON-chosen side to the wall on a copy
// of the pristine base map -> run the offline regeneration driver
// (regenerate_obstacle_map.py, gui_params.yaml values) -> when the driver's
// physical gates pass AND /lap_count advances, swap the global line through
// /global_planning/reload_waypoints after the next lap transition.
// Any failure keeps the previous line (local avoidance keeps covering).

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "map_creator/map_painter.hpp"
#include "map_creator/obstacle_ledger.hpp"
#include "map_creator/regeneration_manager.hpp"
#include "map_creator/side_planner_adapter.hpp"

#include <yaml-cpp/yaml.h>

namespace map_creator
{

namespace fs = std::filesystem;
using json = nlohmann::json;

class MapCreatorNode : public rclcpp::Node
{
public:
  MapCreatorNode()
  : Node("map_creator_node")
  {
    declareParameters();
    readParameters();
    painter_ = std::make_unique<MapPainter>(painter_config_);

    if (reseed_on_startup_) {
      std::string why;
      if (reseedBaseline(&why)) {
        RCLCPP_INFO(get_logger(), "reseeded %s from baseline '%s'",
          outputDir().c_str(), baseline_map_name_.c_str());
      } else {
        RCLCPP_WARN(get_logger(), "baseline reseed skipped: %s", why.c_str());
      }
    }

    const auto latched = rclcpp::QoS(1).reliable().transient_local();
    wpnts_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      global_waypoints_topic_, latched,
      std::bind(&MapCreatorNode::wpntsCallback, this, std::placeholders::_1));
    lap_sub_ = create_subscription<std_msgs::msg::Int32>(
      lap_count_topic_, latched,
      std::bind(&MapCreatorNode::lapCallback, this, std::placeholders::_1));
    obs_sub_ = create_subscription<f110_msgs::msg::ObstacleArray>(
      static_obs_topic_, rclcpp::QoS(10),
      std::bind(&MapCreatorNode::obstaclesCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>("/map_creator/status", 10);
    reload_client_ = create_client<std_srvs::srv::Trigger>(reload_service_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MapCreatorNode::tick, this));
    publishStatus("idle: waiting for lap " + std::to_string(trigger_lap_count_));
  }

private:
  enum class Stage {kIdle, kGenerating, kArmed, kMonitoring, kAborted};

  // ---------------------------------------------------------------- parameters
  void declareParameters()
  {
    declare_parameter<std::string>("static_obs_topic", "/static_obs");
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    declare_parameter<std::string>("lap_count_topic", "/lap_count");
    declare_parameter<std::string>(
      "reload_service", "/global_planning/reload_waypoints");

    declare_parameter<int>("trigger_lap_count", 2);
    declare_parameter<double>("match_max_ds_m", 1.0);
    declare_parameter<double>("match_max_dd_m", 0.3);
    declare_parameter<int>("min_observations", 3);
    declare_parameter<int>("removal_miss_laps", 2);

    declare_parameter<double>("ego_lookback_m", 12.0);

    // Side-decision parameter snapshot. Unlisted planner parameters keep the
    // deployed local_planning.yaml values as C++ defaults here.
    declare_parameter<std::vector<double>>(
      "decision.transition_distance_scales", {1.0, 1.25, 1.50});
    declare_parameter<double>("decision.outside_line_transition_scale", 1.35);
    declare_parameter<double>("decision.commitment_clearance_reserve_m", 0.05);
    declare_parameter<double>("decision.minimum_avoidance_clearance_m", 0.18);
    declare_parameter<double>("decision.boundary_margin_m", 0.13);
    declare_parameter<double>("decision.minimum_target_offset_m", 0.20);
    declare_parameter<double>("decision.maximum_lateral_slope", 0.65);
    declare_parameter<double>("decision.obstacle_clearance_m", 0.25);
    declare_parameter<double>("decision.maximum_target_offset_m", 1.50);
    declare_parameter<double>("decision.side_tie_epsilon_m", 0.02);
    declare_parameter<double>("decision.vehicle_half_width_m", 0.121);
    declare_parameter<double>("decision.obstacle_longitudinal_padding_m", 0.3);
    declare_parameter<double>("decision.detection_lookahead_m", 12.0);
    declare_parameter<double>("decision.fallback_track_half_width_m", 1.50);

    declare_parameter<std::string>("base_map_yaml", "");
    declare_parameter<int>("nondrivable_threshold", 250);
    declare_parameter<int>("side_samples_min", 9);
    declare_parameter<double>("ray_step_fraction", 0.25);
    declare_parameter<double>("max_ray_length_m", 5.0);

    declare_parameter<std::string>(
      "generator_driver", "offline_trajectory_generator/regenerate_obstacle_map.py");
    declare_parameter<std::string>(
      "gui_params_yaml", "offline_trajectory_generator/gui_params.yaml");
    declare_parameter<std::string>(
      "output_base_dir", "offline_trajectory_generator/output");
    declare_parameter<std::string>("output_map_name", "obstacle_map");
    declare_parameter<std::string>("baseline_map_name", "map");
    declare_parameter<bool>("reseed_on_startup", true);
    declare_parameter<std::string>("python_executable", "python3");
    declare_parameter<double>("generation_timeout_sec", 120.0);
    declare_parameter<double>("retry_safety_width", 0.5);

    declare_parameter<double>("min_obstacle_clearance_after_m", 0.42);
    declare_parameter<int>("max_swap_deferral_laps", 3);
  }

  void readParameters()
  {
    static_obs_topic_ = get_parameter("static_obs_topic").as_string();
    global_waypoints_topic_ = get_parameter("global_waypoints_topic").as_string();
    lap_count_topic_ = get_parameter("lap_count_topic").as_string();
    reload_service_ = get_parameter("reload_service").as_string();

    trigger_lap_count_ = static_cast<int>(get_parameter("trigger_lap_count").as_int());
    min_observations_ = static_cast<int>(get_parameter("min_observations").as_int());
    removal_miss_laps_ = static_cast<int>(get_parameter("removal_miss_laps").as_int());
    ledger_.setMatchThresholds(
      get_parameter("match_max_ds_m").as_double(),
      get_parameter("match_max_dd_m").as_double());

    ego_lookback_m_ = get_parameter("ego_lookback_m").as_double();

    decision_params_.transition_distance_scales =
      get_parameter("decision.transition_distance_scales").as_double_array();
    decision_params_.outside_line_transition_scale =
      get_parameter("decision.outside_line_transition_scale").as_double();
    decision_params_.commitment_clearance_reserve_m =
      get_parameter("decision.commitment_clearance_reserve_m").as_double();
    decision_params_.minimum_avoidance_clearance_m =
      get_parameter("decision.minimum_avoidance_clearance_m").as_double();
    decision_params_.boundary_margin_m =
      get_parameter("decision.boundary_margin_m").as_double();
    decision_params_.minimum_target_offset_m =
      get_parameter("decision.minimum_target_offset_m").as_double();
    decision_params_.maximum_lateral_slope =
      get_parameter("decision.maximum_lateral_slope").as_double();
    decision_params_.obstacle_clearance_m =
      get_parameter("decision.obstacle_clearance_m").as_double();
    decision_params_.maximum_target_offset_m =
      get_parameter("decision.maximum_target_offset_m").as_double();
    decision_params_.side_tie_epsilon_m =
      get_parameter("decision.side_tie_epsilon_m").as_double();
    decision_params_.vehicle_half_width_m =
      get_parameter("decision.vehicle_half_width_m").as_double();
    decision_params_.obstacle_longitudinal_padding_m =
      get_parameter("decision.obstacle_longitudinal_padding_m").as_double();
    decision_params_.detection_lookahead_m =
      get_parameter("decision.detection_lookahead_m").as_double();
    decision_params_.fallback_track_half_width_m =
      get_parameter("decision.fallback_track_half_width_m").as_double();

    base_map_yaml_ = get_parameter("base_map_yaml").as_string();
    painter_config_.nondrivable_threshold =
      static_cast<int>(get_parameter("nondrivable_threshold").as_int());
    painter_config_.side_samples_min =
      static_cast<int>(get_parameter("side_samples_min").as_int());
    painter_config_.ray_step_fraction = get_parameter("ray_step_fraction").as_double();
    painter_config_.max_ray_length_m = get_parameter("max_ray_length_m").as_double();

    generator_driver_ = get_parameter("generator_driver").as_string();
    gui_params_yaml_ = get_parameter("gui_params_yaml").as_string();
    output_base_dir_ = get_parameter("output_base_dir").as_string();
    output_map_name_ = get_parameter("output_map_name").as_string();
    baseline_map_name_ = get_parameter("baseline_map_name").as_string();
    reseed_on_startup_ = get_parameter("reseed_on_startup").as_bool();
    python_executable_ = get_parameter("python_executable").as_string();
    generation_timeout_sec_ = get_parameter("generation_timeout_sec").as_double();
    retry_safety_width_ = get_parameter("retry_safety_width").as_double();

    min_clearance_after_ = get_parameter("min_obstacle_clearance_after_m").as_double();
    max_swap_deferral_laps_ =
      static_cast<int>(get_parameter("max_swap_deferral_laps").as_int());

    if (base_map_yaml_.empty()) {
      // Fall back to the same map the trajectory GUI uses (gui_params map_yaml),
      // so painting target and generator input are one and the same file family.
      try {
        const auto gui = YAML::LoadFile(gui_params_yaml_);
        if (gui["map_yaml"]) {
          base_map_yaml_ = gui["map_yaml"].as<std::string>();
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "cannot read gui_params for base map: %s", e.what());
      }
    }
  }

  std::string outputDir() const {return output_base_dir_ + "/" + output_map_name_;}

  // ---------------------------------------------------------------- callbacks
  void wpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    if (adapter_) {
      return;  // immutable baseline: the FIRST published line is the reference P0
    }
    auto adapter = std::make_unique<SidePlannerAdapter>(decision_params_);
    std::string error;
    if (!adapter->setReference(*msg, &error)) {
      RCLCPP_WARN(get_logger(), "reference rejected: %s", error.c_str());
      return;
    }
    adapter_ = std::move(adapter);
    ledger_.setTrackLength(adapter_->trackLength());
    RCLCPP_INFO(get_logger(),
      "baseline reference captured (%zu wpnts, track %.2f m)",
      msg->wpnts.size(), adapter_->trackLength());
  }

  void lapCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    lap_count_ = msg->data;
  }

  void obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    if (stage_ == Stage::kIdle || stage_ == Stage::kMonitoring) {
      ledger_.addObservations(msg->obstacles, lap_count_);
    }
  }

  // ---------------------------------------------------------------- pipeline
  void abort(const std::string & why)
  {
    stage_ = Stage::kAborted;
    RCLCPP_ERROR(get_logger(), "pipeline aborted (previous line stays): %s", why.c_str());
    publishStatus("aborted: " + why);
    writeManifest("aborted", why);
  }

  bool reseedBaseline(std::string * why)
  {
    const fs::path src_dir = fs::path(output_base_dir_) / baseline_map_name_;
    const fs::path dst_dir = fs::path(outputDir());
    std::error_code ec;
    fs::create_directories(dst_dir, ec);
    bool copied_any = false;
    for (const char * name : {"global_waypoints.json", "metadata.json"}) {
      const fs::path src = src_dir / name;
      if (!fs::exists(src)) {
        if (why) {*why = "missing " + src.string();}
        continue;
      }
      fs::copy_file(src, dst_dir / name, fs::copy_options::overwrite_existing, ec);
      if (ec) {
        if (why) {*why = "copy failed: " + ec.message();}
        return false;
      }
      copied_any = true;
    }
    return copied_any;
  }

  static json obstacleAabb(
    const f110_msgs::msg::Obstacle & ob,
    const MapPainter::FrenetToCartesian & to_cart)
  {
    double x_min, x_max, y_min, y_max;
    if (ob.has_cartesian) {
      x_min = ob.x_min; x_max = ob.x_max; y_min = ob.y_min; y_max = ob.y_max;
    } else {
      x_min = y_min = std::numeric_limits<double>::infinity();
      x_max = y_max = -std::numeric_limits<double>::infinity();
      for (const double s : {ob.s_start, ob.s_end}) {
        for (const double d : {ob.d_right, ob.d_left}) {
          double x = 0.0, y = 0.0, yaw = 0.0;
          to_cart(s, d, x, y, yaw);
          x_min = std::min(x_min, x); x_max = std::max(x_max, x);
          y_min = std::min(y_min, y); y_max = std::max(y_max, y);
        }
      }
    }
    return json{{"id", ob.id}, {"x_min", x_min}, {"x_max", x_max},
      {"y_min", y_min}, {"y_max", y_max},
      {"s_center", ob.s_center}, {"d_center", ob.d_center},
      {"d_left", ob.d_left}, {"d_right", ob.d_right}};
  }

  MapPainter::FrenetToCartesian makeToCart() const
  {
    const double track_length = adapter_->trackLength();
    const auto * planner = &adapter_->planner();
    return [planner, track_length](double s, double d, double & x, double & y, double & yaw) {
             if (track_length > 0.0) {
               s = std::fmod(s, track_length);
               if (s < 0.0) {s += track_length;}
             }
             planner->toCartesian(s, d, x, y, yaw);
           };
  }

  bool runDecisionAndPaint()
  {
    decisions_.clear();
    baked_.clear();
    for (const auto & entry : frozen_) {
      const auto decision = adapter_->decide(entry.obstacle, ego_lookback_m_);
      decisions_.push_back(decision);
      if (decision.side != SideDecision::Side::kSafeStop) {
        baked_.push_back(entry);
        baked_decisions_.push_back(decision);
      } else {
        RCLCPP_WARN(get_logger(),
          "obstacle id=%d not baked (safe_stop: %s)",
          entry.obstacle.id, decision.reason.c_str());
      }
    }
    if (baked_.empty()) {
      abort("no bakeable obstacle (all safe_stop)");
      return false;
    }

    std::string error;
    if (!painter_->loaded() && !painter_->loadBase(base_map_yaml_, &error)) {
      abort("base map load failed: " + error);
      return false;
    }
    painter_->reset();
    const auto to_cart = makeToCart();
    for (std::size_t i = 0; i < baked_.size(); ++i) {
      // Pass LEFT -> block the RIGHT side; pass RIGHT -> block the LEFT side.
      const bool block_left =
        baked_decisions_[i].side == SideDecision::Side::kRight;
      if (!painter_->paintObstacle(
          baked_[i].obstacle, block_left, adapter_->trackLength(), to_cart, &error))
      {
        abort("painting failed: " + error);
        return false;
      }
    }
    if (!painter_->save(outputDir(), output_map_name_, &error)) {
      abort("map save failed: " + error);
      return false;
    }

    json obstacles = json::array();
    for (const auto & entry : baked_) {
      obstacles.push_back(obstacleAabb(entry.obstacle, to_cart));
    }
    std::ofstream out(fs::path(outputDir()) / "obstacles.json");
    out << json{{"obstacles", obstacles}}.dump(2);
    return true;
  }

  std::string buildDriverCommand(const std::optional<double> & safety_width) const
  {
    std::ostringstream cmd;
    cmd << "timeout " << static_cast<int>(generation_timeout_sec_) << " "
        << python_executable_ << " " << generator_driver_
        << " --map-yaml " << outputDir() << "/" << output_map_name_ << ".yaml"
        << " --gui-params " << gui_params_yaml_
        << " --output-dir " << outputDir()
        << " --obstacles-json " << outputDir() << "/obstacles.json"
        << " --min-clearance " << min_clearance_after_;
    if (safety_width.has_value()) {
      cmd << " --safety-width " << *safety_width;
    }
    cmd << " >> " << outputDir() << "/regen_log.txt 2>&1";
    return cmd.str();
  }

  void startGeneration(const std::optional<double> & safety_width)
  {
    if (!regen_.start(buildDriverCommand(safety_width))) {
      abort("generation already in flight");
      return;
    }
    stage_ = Stage::kGenerating;
    publishStatus(
      safety_width ? "generating (retry, safety_width override)" : "generating");
  }

  void requestSwap()
  {
    if (!reload_client_->service_is_ready()) {
      return;  // keep waiting inside the gate window
    }
    swap_inflight_ = true;
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    reload_client_->async_send_request(
      request,
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        const auto response = future.get();
        swap_inflight_ = false;
        if (response->success) {
          stage_ = Stage::kMonitoring;
          RCLCPP_INFO(get_logger(), "swap done: %s", response->message.c_str());
          publishStatus("swapped: " + response->message);
          writeManifest("swapped", response->message);
        } else {
          abort("reload rejected: " + response->message);
        }
      });
  }

  void writeManifest(const std::string & status, const std::string & note)
  {
    json decisions = json::array();
    for (std::size_t i = 0; i < frozen_.size(); ++i) {
      const auto & decision = decisions_.size() > i ? decisions_[i] : SideDecision{};
      const char * side =
        decision.side == SideDecision::Side::kLeft ? "left" :
        decision.side == SideDecision::Side::kRight ? "right" : "safe_stop";
      decisions.push_back(json{
          {"obstacle_id", frozen_[i].obstacle.id},
          {"s_center", frozen_[i].obstacle.s_center},
          {"d_center", frozen_[i].obstacle.d_center},
          {"first_seen_lap", frozen_[i].first_seen_lap},
          {"observations", frozen_[i].observation_count},
          {"decision", side},
          {"target_d", decision.target_d},
          {"reason", decision.reason},
        });
    }
    json manifest{
      {"status", status},
      {"note", note},
      {"trigger_lap", trigger_lap_count_},
      {"lap_count", lap_count_},
      {"ego_lookback_m", ego_lookback_m_},
      {"base_map_yaml", base_map_yaml_},
      {"gui_params_yaml", gui_params_yaml_},
      {"decisions", decisions},
    };
    std::error_code ec;
    fs::create_directories(outputDir(), ec);
    std::ofstream out(fs::path(outputDir()) / "manifest.json");
    out << manifest.dump(2);
  }

  void publishStatus(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  // ---------------------------------------------------------------- FSM tick
  void tick()
  {
    switch (stage_) {
      case Stage::kIdle:
        if (!fired_ && adapter_ && lap_count_ >= trigger_lap_count_) {
          fired_ = true;
          frozen_ = ledger_.confirmed(min_observations_);
          baked_decisions_.clear();
          if (frozen_.empty()) {
            abort("trigger lap reached but no confirmed static obstacle");
            break;
          }
          RCLCPP_INFO(get_logger(),
            "lap %d transition: frozen %zu obstacle(s), deciding sides",
            lap_count_, frozen_.size());
          if (runDecisionAndPaint()) {
            retried_ = false;
            startGeneration(std::nullopt);
          }
        }
        break;

      case Stage::kGenerating:
        if (regen_.finished()) {
          const int code = regen_.exitCode();
          regen_.reset();
          if (code == 0) {
            armed_after_lap_ = lap_count_;
            stage_ = Stage::kArmed;
            publishStatus("armed: waiting for next lap boundary + GLOBAL gate");
          } else if (code == 1 && !retried_ && retry_safety_width_ > 0.0) {
            retried_ = true;
            RCLCPP_WARN(get_logger(),
              "generation gates failed; retrying with safety_width=%.2f",
              retry_safety_width_);
            startGeneration(retry_safety_width_);
          } else {
            abort("generation failed (exit code " + std::to_string(code) +
              "), see " + outputDir() + "/regen_log.txt");
          }
        }
        break;

      case Stage::kArmed: {
        if (swap_inflight_) {
          break;
        }
        // [User condition 2] swap only after /lap_count advanced past the lap in
        // which generation+validation completed.
        if (lap_count_ <= armed_after_lap_) {
          break;
        }
        if (lap_count_ - armed_after_lap_ > max_swap_deferral_laps_) {
          abort("swap deferred more than " +
            std::to_string(max_swap_deferral_laps_) + " laps");
          break;
        }
        requestSwap();
        break;
      }

      case Stage::kMonitoring: {
        const auto removals = ledger_.removalCandidates(lap_count_, removal_miss_laps_);
        if (!removals.empty()) {
          ledger_.removeAt(removals);
          frozen_ = ledger_.confirmed(min_observations_);
          baked_decisions_.clear();
          if (frozen_.empty()) {
            std::string why;
            if (reseedBaseline(&why)) {
              RCLCPP_INFO(get_logger(), "all obstacles gone: rolling back to baseline");
              publishStatus("rollback: baseline reseeded, reloading");
              requestSwap();
              stage_ = Stage::kAborted;  // terminal; pipeline complete
            } else {
              abort("rollback reseed failed: " + why);
            }
          } else {
            RCLCPP_INFO(get_logger(),
              "obstacle set changed: regenerating with %zu obstacle(s)", frozen_.size());
            if (runDecisionAndPaint()) {
              retried_ = false;
              startGeneration(std::nullopt);
            }
          }
        }
        break;
      }

      case Stage::kAborted:
        break;
    }
  }

  // ---------------------------------------------------------------- members
  std::string static_obs_topic_, global_waypoints_topic_, lap_count_topic_;
  std::string reload_service_;
  int trigger_lap_count_{2};
  int min_observations_{3};
  int removal_miss_laps_{2};
  double ego_lookback_m_{12.0};
  local_planning::RacelineSplineParameters decision_params_;
  std::string base_map_yaml_;
  MapPainterConfig painter_config_;
  std::string generator_driver_, gui_params_yaml_, output_base_dir_;
  std::string output_map_name_, baseline_map_name_, python_executable_;
  bool reseed_on_startup_{true};
  double generation_timeout_sec_{120.0};
  double retry_safety_width_{0.5};
  double min_clearance_after_{0.42};
  int max_swap_deferral_laps_{3};

  ObstacleLedger ledger_;
  std::unique_ptr<SidePlannerAdapter> adapter_;
  std::unique_ptr<MapPainter> painter_;
  RegenerationManager regen_;

  Stage stage_{Stage::kIdle};
  bool fired_{false};
  bool retried_{false};
  std::atomic<bool> swap_inflight_{false};
  int lap_count_{0};
  int armed_after_lap_{0};
  std::vector<LedgerEntry> frozen_;
  std::vector<LedgerEntry> baked_;
  std::vector<SideDecision> decisions_;
  std::vector<SideDecision> baked_decisions_;

  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wpnts_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr lap_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reload_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace map_creator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<map_creator::MapCreatorNode>());
  rclcpp::shutdown();
  return 0;
}
