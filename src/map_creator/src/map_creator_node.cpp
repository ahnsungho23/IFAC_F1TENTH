// Copyright 2026 2026_IFAC contributors
// Licensed under the Apache License, Version 2.0.
//
// map_creator_node: lap-transition obstacle_map pipeline
// (learning_adaptive_globalpath/MAP_CREATOR_PROPOSAL.md).
//
// Laps 1-2: project the persistent confirmed-static snapshot
// (/adaptive_obstacle_map, Cartesian AABB contract) onto the immutable P0
// reference (CLCS) and consume it into a ledger, all-or-nothing per snapshot.
// At the lap 2 -> 3 transition: freeze -> per-obstacle left/right decision via
// the SHARED RacelineSplinePlanner::plan (map_creator's own
// tuned parameter snapshot) -> paint the NON-chosen side to the wall on a copy
// of the pristine base map -> run the offline regeneration driver
// (regenerate_obstacle_map.py, gui_params.yaml values) -> when the driver's
// physical gates pass AND /lap_count advances, swap the global line through
// /global_planning/reload_waypoints after the next lap transition.
// Any failure keeps the previous line (local avoidance keeps covering).
//
// The swap is ONE-WAY and final. Obstacles disappearing later used to trigger a
// regeneration (some gone) or a baseline rollback swap (all gone); both were
// removed, so nothing re-swaps the global line once the pipeline commits it.
// Obstacles appearing after the freeze were never reflected in the line either.
// Everything post-swap is local avoidance's job -- note that the swap also
// disables the GLOBAL->AVOID gate and nothing restores it.

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

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "f110_msgs/msg/obstacle_array.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"

#include "global_planning/clcs_frenet_converter.hpp"

#include "map_creator/frenet_aabb.hpp"
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
      obstacle_map_topic_, latched,
      std::bind(&MapCreatorNode::obstaclesCallback, this, std::placeholders::_1));
    status_pub_ = create_publisher<std_msgs::msg::String>("/map_creator/status", 10);
    reload_client_ = create_client<std_srvs::srv::Trigger>(reload_service_);
    if (disable_avoid_after_swap_) {
      state_machine_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, state_machine_node_name_);
    }
    if (!control_node_name_.empty() && swap_max_speed_mps_ > 0.0) {
      control_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, control_node_name_);
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100), std::bind(&MapCreatorNode::tick, this));
    publishStatus("idle: waiting for lap " + std::to_string(trigger_lap_count_));
  }

private:
  enum class Stage {kIdle, kGenerating, kArmed, kMonitoring, kAborted};

  // ---------------------------------------------------------------- parameters
  void declareParameters()
  {
    declare_parameter<std::string>("obstacle_map_topic", "/adaptive_obstacle_map");
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    declare_parameter<std::string>("lap_count_topic", "/lap_count");
    declare_parameter<std::string>(
      "reload_service", "/global_planning/reload_waypoints");
    declare_parameter<bool>("disable_avoid_after_swap", true);
    declare_parameter<std::string>("state_machine_node_name", "state_machine_node");
    // Post-swap control speed cap: send max_speed to the control node right
    // after a successful obstacle-line swap. <= 0 disables the send.
    // The direction is relative to the control node's launch default (currently
    // 5.0 in control_real.launch.py): laps 1-2 run the baseline line with unknown
    // obstacle positions, so they start slow and this raises the cap once the
    // swapped line routes around the obstacles.
    declare_parameter<std::string>("control_node_name", "control_map_node");
    declare_parameter<double>("swap_max_speed_mps", 0.0);

    declare_parameter<int>("trigger_lap_count", 2);
    declare_parameter<double>("match_max_ds_m", 1.0);
    declare_parameter<double>("match_max_dd_m", 0.3);

    declare_parameter<double>("ego_lookback_m", 12.0);

    // P0 projection: /adaptive_obstacle_map carries Cartesian AABBs only; this
    // node projects them onto the immutable P0 reference itself.
    declare_parameter<double>("reference_alignment_tolerance_m", 0.05);
    declare_parameter<double>("max_obstacle_projection_d_m", 2.0);

    // Side-decision parameter snapshot. plan()이 좌/우 판정에 실제로 읽는 플래너
    // 파라미터를 하나도 빠짐없이 이 노드가 소유한다 — 헤더 C++ 기본값에 기대는 항목이
    // 없어야 "map_creator 자기 파라미터로 판정한다"가 문자 그대로 성립한다.
    // 여기 기본값과 config/map_creator.yaml 값은 2026-08-20 기준 배포
    // local_planning/config/local_planning.yaml과 동일하다.
    //
    // 판정에 쓰이지 않아 일부러 뺀 것: merge_ramp_*(완료 핸드오프 복귀 램프),
    // commitment_retention_reserve_fraction(런타임 커밋 재검증 전용).
    //
    // Clearance 모델: obstacle inflation = vehicle_half_width + safety_margin +
    // tracking-error reserve(LUT) + localization_reserve, wall reserve =
    // wall_safety_margin_m 단독. 구 obstacle_clearance_m / boundary_margin_m /
    // minimum_avoidance_clearance_m 노브는 없다. side_tie_epsilon_m은 P0 quintic
    // 격자와 함께 사라졌다 — 좌우 동률은 plan()의 정렬 계약이 깬다.

    // 검출·군집·차체 기하
    declare_parameter<double>("decision.detection_lookahead_m", 15.0);
    declare_parameter<double>("decision.obstacle_cluster_gap_m", 0.8);
    declare_parameter<double>(
      "decision.obstacle_longitudinal_padding_m", 0.4149924657737441);
    declare_parameter<double>("decision.vehicle_length_m", 0.56);
    declare_parameter<double>("decision.vehicle_half_width_m", 0.1435);

    // 여유(clearance) 모델
    declare_parameter<double>("decision.safety_margin_m", 0.014789254299520768);
    declare_parameter<double>("decision.tracking_error_reserve_m", 0.140);
    declare_parameter<std::vector<double>>(
      "decision.tracking_error_lut_speed_bins_mps", {0.0, 1.5, 3.0, 4.5, 6.5});
    declare_parameter<std::vector<double>>(
      "decision.tracking_error_lut_curvature_bins_radpm",
      {0.0, 0.2, 0.5, 0.9, 1.316266519079011});
    declare_parameter<std::vector<double>>(
      "decision.tracking_error_lut_values_m", {
        0.200, 0.200, 0.200, 0.200, 0.200,
        0.325, 0.395, 0.395, 0.395, 0.395,
        0.330, 0.395, 0.395, 0.395, 0.395,
        0.330, 0.395, 0.395, 0.395, 0.395,
        0.330, 0.395, 0.395, 0.395, 0.395});
    declare_parameter<double>("decision.localization_reserve_m", 0.12);
    declare_parameter<double>("decision.wall_safety_margin_m", 0.10);
    declare_parameter<double>("decision.fallback_track_half_width_m", 1.50);

    // 속도 제한 (새 정렬 계약에서 velocity_loss가 실질 1순위라 좌우 선택에 직접 영향)
    declare_parameter<std::vector<double>>(
      "decision.avoidance_velocity_limit_speed_bins_mps",
      {0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0});
    declare_parameter<std::vector<double>>(
      "decision.avoidance_velocity_limit_lateral_accel_mps2",
      {7.0, 7.0, 7.0, 7.0, 7.0, 7.0, 6.5, 6.5, 6.5, 6.5});
    declare_parameter<double>("decision.avoidance_minimum_speed_mps", 1.0);
    declare_parameter<double>("decision.margin_pass_speed_cap_mps", 2.0);
    declare_parameter<double>("decision.approach_feasibility_decel_mps2", 2.0);
    declare_parameter<double>("decision.approach_feasibility_decel_max_mps2", 3.5);
    declare_parameter<double>("decision.profile_feasibility_decel_mps2", 3.5);

    // 후보 기하 (진입/탈출 길이, 목표 오프셋 격자)
    declare_parameter<std::vector<double>>(
      "decision.pre_apex_distances_m",
      {11.442220427651225, 7.628146951767484, 3.814073475883742});
    declare_parameter<std::vector<double>>(
      "decision.post_apex_distances_m",
      {2.059509950005119, 4.119019900010238, 6.178529850015357});
    declare_parameter<std::vector<double>>(
      "decision.entry_transition_fractions", {0.5145810930150512, 0.75, 1.00});
    declare_parameter<std::vector<double>>(
      "decision.transition_distance_scales",
      {0.4971684162574945, 0.6991537701867223, 3.698773101198193});
    declare_parameter<double>(
      "decision.outside_line_transition_scale", 0.4060036444074003);
    declare_parameter<double>("decision.maximum_exit_length_m", 0.0);
    declare_parameter<double>("decision.post_merge_lookahead_m", 5.0);
    declare_parameter<double>("decision.post_merge_min_time_sec", 1.0);
    declare_parameter<double>("decision.minimum_target_offset_m", 0.15);
    declare_parameter<double>("decision.maximum_target_offset_m", 1.50);
    declare_parameter<int>("decision.target_d_candidate_count", 5);

    // 하드 게이트 (어느 측이 실현 가능한지를 가른다)
    declare_parameter<double>("decision.maximum_lateral_slope", 0.8);
    declare_parameter<double>("decision.maximum_curvature_radpm", 1.316266519079011);
    declare_parameter<double>("decision.maximum_curvature_rate_radpm2", 20.0);
    declare_parameter<int>("decision.minimum_path_points", 8);

    // 안전정지 (여기로 떨어진 장애물은 베이크되지 않는다)
    declare_parameter<double>("decision.safe_stop_buffer_m", 2.60);
    declare_parameter<double>("decision.safe_stop_deceleration_mps2", 1.8);
    declare_parameter<bool>("decision.safe_stop_escape_check_enable", true);
    declare_parameter<double>("decision.safe_stop_escape_retreat_step_m", 0.30);
    declare_parameter<int>("decision.safe_stop_escape_max_retreats", 8);

    declare_parameter<std::string>("base_map_yaml", "");
    declare_parameter<int>("nondrivable_threshold", 250);
    declare_parameter<int>("side_samples_min", 9);
    declare_parameter<double>("ray_step_fraction", 0.25);
    declare_parameter<double>("max_ray_length_m", 5.0);

    declare_parameter<std::string>(
      "generator_driver", "offline_trajectory_generator/bin/regenerate_obstacle_map");
    declare_parameter<std::string>(
      "gui_params_yaml", "offline_trajectory_generator/gui_params.yaml");
    declare_parameter<std::string>(
      "output_base_dir", "offline_trajectory_generator/output");
    declare_parameter<std::string>("output_map_name", "obstacle_map");
    declare_parameter<std::string>("baseline_map_name", "map");
    declare_parameter<bool>("reseed_on_startup", true);
    // Interpreter prefix for the driver; empty = run the driver binary directly.
    declare_parameter<std::string>("python_executable", "");
    declare_parameter<double>("generation_timeout_sec", 120.0);
    declare_parameter<double>("initial_smooth_sigma", 4.1);
    declare_parameter<double>("retry_safety_width", 0.4);
    declare_parameter<double>("retry_smooth_sigma", 2.5);
    // Per-pass morph_kernel override; <= 0 keeps the gui_params value.
    declare_parameter<int>("initial_morph_kernel", 0);
    declare_parameter<int>("retry_morph_kernel", 0);

    declare_parameter<double>("min_obstacle_clearance_after_m", 0.42);
    declare_parameter<int>("max_swap_deferral_laps", 3);
  }

  void readParameters()
  {
    obstacle_map_topic_ = get_parameter("obstacle_map_topic").as_string();
    global_waypoints_topic_ = get_parameter("global_waypoints_topic").as_string();
    lap_count_topic_ = get_parameter("lap_count_topic").as_string();
    reload_service_ = get_parameter("reload_service").as_string();
    disable_avoid_after_swap_ = get_parameter("disable_avoid_after_swap").as_bool();
    state_machine_node_name_ = get_parameter("state_machine_node_name").as_string();
    control_node_name_ = get_parameter("control_node_name").as_string();
    swap_max_speed_mps_ = get_parameter("swap_max_speed_mps").as_double();

    trigger_lap_count_ = static_cast<int>(get_parameter("trigger_lap_count").as_int());
    ledger_.setMatchThresholds(
      get_parameter("match_max_ds_m").as_double(),
      get_parameter("match_max_dd_m").as_double());

    ego_lookback_m_ = get_parameter("ego_lookback_m").as_double();
    reference_alignment_tolerance_m_ =
      get_parameter("reference_alignment_tolerance_m").as_double();
    max_obstacle_projection_d_m_ =
      get_parameter("max_obstacle_projection_d_m").as_double();

    // decision.* 전체가 plan()이 읽는 파라미터다 (선언부 주석 참고).
    decision_params_.detection_lookahead_m =
      get_parameter("decision.detection_lookahead_m").as_double();
    decision_params_.obstacle_cluster_gap_m =
      get_parameter("decision.obstacle_cluster_gap_m").as_double();
    decision_params_.obstacle_longitudinal_padding_m =
      get_parameter("decision.obstacle_longitudinal_padding_m").as_double();
    decision_params_.vehicle_length_m =
      get_parameter("decision.vehicle_length_m").as_double();
    decision_params_.vehicle_half_width_m =
      get_parameter("decision.vehicle_half_width_m").as_double();
    decision_params_.safety_margin_m =
      get_parameter("decision.safety_margin_m").as_double();
    decision_params_.tracking_error_reserve_m =
      get_parameter("decision.tracking_error_reserve_m").as_double();
    decision_params_.localization_reserve_m =
      get_parameter("decision.localization_reserve_m").as_double();
    decision_params_.wall_safety_margin_m =
      get_parameter("decision.wall_safety_margin_m").as_double();
    decision_params_.fallback_track_half_width_m =
      get_parameter("decision.fallback_track_half_width_m").as_double();
    decision_params_.avoidance_minimum_speed_mps =
      get_parameter("decision.avoidance_minimum_speed_mps").as_double();
    decision_params_.margin_pass_speed_cap_mps =
      get_parameter("decision.margin_pass_speed_cap_mps").as_double();
    decision_params_.approach_feasibility_decel_mps2 =
      get_parameter("decision.approach_feasibility_decel_mps2").as_double();
    decision_params_.approach_feasibility_decel_max_mps2 =
      get_parameter("decision.approach_feasibility_decel_max_mps2").as_double();
    decision_params_.profile_feasibility_decel_mps2 =
      get_parameter("decision.profile_feasibility_decel_mps2").as_double();
    decision_params_.outside_line_transition_scale =
      get_parameter("decision.outside_line_transition_scale").as_double();
    decision_params_.maximum_exit_length_m =
      get_parameter("decision.maximum_exit_length_m").as_double();
    decision_params_.post_merge_lookahead_m =
      get_parameter("decision.post_merge_lookahead_m").as_double();
    decision_params_.post_merge_min_time_sec =
      get_parameter("decision.post_merge_min_time_sec").as_double();
    decision_params_.minimum_target_offset_m =
      get_parameter("decision.minimum_target_offset_m").as_double();
    decision_params_.maximum_target_offset_m =
      get_parameter("decision.maximum_target_offset_m").as_double();
    decision_params_.maximum_lateral_slope =
      get_parameter("decision.maximum_lateral_slope").as_double();
    decision_params_.maximum_curvature_radpm =
      get_parameter("decision.maximum_curvature_radpm").as_double();
    decision_params_.maximum_curvature_rate_radpm2 =
      get_parameter("decision.maximum_curvature_rate_radpm2").as_double();
    decision_params_.safe_stop_buffer_m =
      get_parameter("decision.safe_stop_buffer_m").as_double();
    decision_params_.safe_stop_deceleration_mps2 =
      get_parameter("decision.safe_stop_deceleration_mps2").as_double();
    decision_params_.safe_stop_escape_retreat_step_m =
      get_parameter("decision.safe_stop_escape_retreat_step_m").as_double();
    decision_params_.tracking_error_lut_speed_bins_mps =
      get_parameter("decision.tracking_error_lut_speed_bins_mps").as_double_array();
    decision_params_.tracking_error_lut_curvature_bins_radpm =
      get_parameter("decision.tracking_error_lut_curvature_bins_radpm").as_double_array();
    decision_params_.tracking_error_lut_values_m =
      get_parameter("decision.tracking_error_lut_values_m").as_double_array();
    decision_params_.avoidance_velocity_limit_speed_bins_mps =
      get_parameter("decision.avoidance_velocity_limit_speed_bins_mps").as_double_array();
    decision_params_.avoidance_velocity_limit_lateral_accel_mps2 =
      get_parameter("decision.avoidance_velocity_limit_lateral_accel_mps2").as_double_array();
    decision_params_.pre_apex_distances_m =
      get_parameter("decision.pre_apex_distances_m").as_double_array();
    decision_params_.post_apex_distances_m =
      get_parameter("decision.post_apex_distances_m").as_double_array();
    decision_params_.entry_transition_fractions =
      get_parameter("decision.entry_transition_fractions").as_double_array();
    decision_params_.transition_distance_scales =
      get_parameter("decision.transition_distance_scales").as_double_array();
    decision_params_.target_d_candidate_count =
      static_cast<int>(get_parameter("decision.target_d_candidate_count").as_int());
    decision_params_.minimum_path_points =
      static_cast<int>(get_parameter("decision.minimum_path_points").as_int());
    decision_params_.safe_stop_escape_max_retreats =
      static_cast<int>(get_parameter("decision.safe_stop_escape_max_retreats").as_int());
    decision_params_.safe_stop_escape_check_enable =
      get_parameter("decision.safe_stop_escape_check_enable").as_bool();

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
    initial_smooth_sigma_ = get_parameter("initial_smooth_sigma").as_double();
    retry_safety_width_ = get_parameter("retry_safety_width").as_double();
    retry_smooth_sigma_ = get_parameter("retry_smooth_sigma").as_double();
    initial_morph_kernel_ =
      static_cast<int>(get_parameter("initial_morph_kernel").as_int());
    retry_morph_kernel_ =
      static_cast<int>(get_parameter("retry_morph_kernel").as_int());

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
    // Atomic capture: adapter (side decision + painting) and CLCS (obstacle
    // projection) must both come from this message or neither may be kept.
    auto adapter = std::make_unique<SidePlannerAdapter>(decision_params_);
    std::string error;
    if (!adapter->setReference(*msg, &error)) {
      RCLCPP_WARN(get_logger(), "reference rejected: %s", error.c_str());
      return;
    }

    std::vector<global_planning::ReferenceWaypoint> ref;
    ref.reserve(msg->wpnts.size());
    for (const auto & w : msg->wpnts) {
      ref.push_back({w.x_m, w.y_m, w.s_m});
    }
    global_planning::ClcsFrenetConfig cfg;  // closed_loop=true defaults
    if (max_obstacle_projection_d_m_ > 0.0) {
      cfg.max_projection_distance = max_obstacle_projection_d_m_;
    }
    global_planning::ClcsFrenetConverter::Ptr clcs;
    try {
      clcs = global_planning::ClcsFrenetConverter::create(ref, cfg, 1U);
    } catch (const std::exception & e) {
      // The latched line republishes identical data every 2 s, so a build
      // failure is deterministic: abort instead of silently retrying forever.
      abort(std::string("P0 CLCS build failed: ") + e.what());
      return;
    }
    // CLCS s is geometric arc length; the painter interpolates by waypoint s_m
    // and infers the loop closure from the median spacing. Refuse to mix the
    // two frames when they disagree beyond tolerance.
    const double clcs_length = clcs->stats().track_length;
    const double s_max_error = clcs->stats().waypoint_s_max_error;
    const double length_diff = std::abs(clcs_length - adapter->trackLength());
    if (s_max_error > reference_alignment_tolerance_m_ ||
      length_diff > reference_alignment_tolerance_m_)
    {
      std::ostringstream why;
      why << "P0 reference frames disagree: waypoint_s_max_error=" << s_max_error
          << " m, |clcs_len - painter_len|=" << length_diff
          << " m, tolerance=" << reference_alignment_tolerance_m_ << " m";
      abort(why.str());
      return;
    }

    adapter_ = std::move(adapter);
    clcs_ = std::move(clcs);
    reference_frame_id_ = msg->header.frame_id;
    ledger_.setTrackLength(clcs_length);
    RCLCPP_INFO(get_logger(),
      "baseline reference captured (%zu wpnts, track %.2f m, s alignment %.4f mm)",
      msg->wpnts.size(), clcs_length, s_max_error * 1000.0);
    if (pending_obs_) {
      const auto pending = std::move(pending_obs_);
      ingestObstacles(*pending);
    }
  }

  void lapCallback(const std_msgs::msg::Int32::SharedPtr msg)
  {
    lap_count_ = msg->data;
  }

  void obstaclesCallback(const f110_msgs::msg::ObstacleArray::SharedPtr msg)
  {
    if (!clcs_) {
      pending_obs_ = msg;  // latched arrival order vs /global_waypoints is undefined
      return;
    }
    ingestObstacles(*msg);
  }

  // All-or-nothing: /adaptive_obstacle_map is an authoritative full snapshot,
  // so one bad element must invalidate the whole message. Dropping only the
  // failed obstacle would read as a real disappearance and start the removal
  // hysteresis (ledger contract: transport silence is not absence — and
  // neither is a projection failure). An empty array is a valid snapshot.
  void ingestObstacles(const f110_msgs::msg::ObstacleArray & msg)
  {
    if (!reference_frame_id_.empty() && !msg.header.frame_id.empty() &&
      msg.header.frame_id != reference_frame_id_)
    {
      rejectSnapshot(
        "frame mismatch: snapshot '" + msg.header.frame_id +
        "' vs P0 '" + reference_frame_id_ + "'");
      return;
    }
    std::vector<f110_msgs::msg::Obstacle> projected;
    projected.reserve(msg.obstacles.size());
    for (const auto & ob : msg.obstacles) {
      if (!ob.is_static) {
        continue;  // reclassified-dynamic entries leave the static contract
      }
      if (!ob.has_cartesian) {
        rejectSnapshot(
          "obstacle id=" + std::to_string(ob.id) + " has no Cartesian AABB");
        return;
      }
      const auto bounds = projectCartesianAabb(
        *clcs_, ob.x_min, ob.x_max, ob.y_min, ob.y_max,
        max_obstacle_projection_d_m_);
      if (!bounds) {
        rejectSnapshot(
          "obstacle id=" + std::to_string(ob.id) +
          " P0 projection failed (degenerate AABB, invalid conversion, or |d| > " +
          std::to_string(max_obstacle_projection_d_m_) + " m)");
        return;
      }
      auto out = ob;
      out.s_center = bounds->s_center;
      out.d_center = bounds->d_center;
      out.s_start = bounds->s_start;
      out.s_end = bounds->s_end;
      out.d_left = bounds->d_left;
      out.d_right = bounds->d_right;
      projected.push_back(out);
    }
    snapshot_rejected_ = false;
    ledger_.updateSnapshot(projected, lap_count_);
  }

  void rejectSnapshot(const std::string & why)
  {
    snapshot_rejected_ = true;
    RCLCPP_WARN(get_logger(), "obstacle snapshot rejected (ledger unchanged): %s",
      why.c_str());
    publishStatus("snapshot rejected: " + why);
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
    const fs::path output_dir(outputDir());
    const fs::path png_path = output_dir / (output_map_name_ + ".png");
    const fs::path yaml_path = output_dir / (output_map_name_ + ".yaml");
    std::error_code png_error;
    std::error_code yaml_error;
    const bool png_ready = fs::is_regular_file(png_path, png_error);
    const bool yaml_ready = fs::is_regular_file(yaml_path, yaml_error);
    if (!png_ready || !yaml_ready) {
      std::ostringstream reason;
      reason << "map artifact verification failed: "
             << png_path.string() << "="
             << (png_ready ? "ready" : png_error ? png_error.message() : "missing") << ", "
             << yaml_path.string() << "="
             << (yaml_ready ? "ready" : yaml_error ? yaml_error.message() : "missing");
      abort(reason.str());
      return false;
    }
    RCLCPP_INFO(
      get_logger(), "verified optimizer map inputs: %s, %s",
      png_path.c_str(), yaml_path.c_str());

    json obstacles = json::array();
    for (const auto & entry : baked_) {
      obstacles.push_back(obstacleAabb(entry.obstacle, to_cart));
    }
    std::ofstream out(fs::path(outputDir()) / "obstacles.json");
    out << json{{"obstacles", obstacles}}.dump(2);
    return true;
  }

  std::string buildDriverCommand(
    const std::optional<double> & safety_width,
    const std::optional<double> & smooth_sigma,
    const int morph_kernel) const
  {
    std::ostringstream cmd;
    cmd << "timeout " << static_cast<int>(generation_timeout_sec_) << " ";
    // Empty python_executable runs the driver directly (the C++ binary);
    // a non-empty value keeps supporting script drivers.
    if (!python_executable_.empty()) {
      cmd << python_executable_ << " ";
    }
    cmd << generator_driver_
        << " --map-yaml " << outputDir() << "/" << output_map_name_ << ".yaml"
        << " --gui-params " << gui_params_yaml_
        << " --output-dir " << outputDir()
        << " --obstacles-json " << outputDir() << "/obstacles.json"
        << " --min-clearance " << min_clearance_after_
        << " --preview-png";
    if (safety_width.has_value()) {
      cmd << " --safety-width " << *safety_width;
    }
    if (smooth_sigma.has_value()) {
      cmd << " --smooth-sigma " << *smooth_sigma;
    }
    if (morph_kernel > 0) {
      cmd << " --morph-kernel " << morph_kernel;
    }
    cmd << " >> " << outputDir() << "/regen_log.txt 2>&1";
    return cmd.str();
  }

  void startGeneration(
    const std::optional<double> & safety_width,
    const std::optional<double> & smooth_sigma,
    const int morph_kernel)
  {
    if (!regen_.start(buildDriverCommand(safety_width, smooth_sigma, morph_kernel))) {
      abort("generation already in flight");
      return;
    }
    stage_ = Stage::kGenerating;
    publishStatus(
      safety_width ? "generating (retry overrides)" : "generating");
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
          // Obstacle-line swap: the live global line now clears the obstacles,
          // so the GLOBAL->AVOID entry gate is no longer needed. There is no
          // rollback swap any more, so this gate is never restored -- local
          // avoidance stays disabled for the rest of the run (deliberate).
          scheduleAvoidGate(false);
          scheduleControlMaxSpeed(swap_max_speed_mps_);
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

  // ------------------------------------------ state_machine avoid-gate update
  void scheduleAvoidGate(bool allow)
  {
    if (!disable_avoid_after_swap_) {
      return;
    }
    pending_avoid_gate_ = allow;
    trySendAvoidGate();
  }

  void trySendAvoidGate()
  {
    if (!pending_avoid_gate_.has_value() || !state_machine_param_client_) {
      return;
    }
    if (!state_machine_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "state_machine '%s' parameter service not ready; "
        "retrying allow_avoid_transition update", state_machine_node_name_.c_str());
      return;
    }
    const bool allow = *pending_avoid_gate_;
    pending_avoid_gate_.reset();
    state_machine_param_client_->set_parameters(
      {rclcpp::Parameter("allow_avoid_transition", allow)},
      [this, allow](
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        for (const auto & result : future.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(),
              "allow_avoid_transition=%s rejected by state_machine: %s",
              allow ? "true" : "false", result.reason.c_str());
            publishStatus("avoid gate update rejected");
            return;
          }
        }
        RCLCPP_INFO(get_logger(),
          "state_machine allow_avoid_transition set to %s", allow ? "true" : "false");
        publishStatus(std::string("avoid gate ") +
          (allow ? "re-enabled (baseline rollback)" : "disabled (obstacle line active)"));
      });
  }

  // -------------------------------------------- control max_speed cap update
  void scheduleControlMaxSpeed(double speed_mps)
  {
    if (!control_param_client_ || speed_mps <= 0.0) {
      return;  // disabled for this transition
    }
    pending_control_max_speed_ = speed_mps;
    trySendControlMaxSpeed();
  }

  void trySendControlMaxSpeed()
  {
    if (!pending_control_max_speed_.has_value() || !control_param_client_) {
      return;
    }
    if (!control_param_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "control '%s' parameter service not ready; retrying max_speed update",
        control_node_name_.c_str());
      return;
    }
    const double speed = *pending_control_max_speed_;
    pending_control_max_speed_.reset();
    control_param_client_->set_parameters(
      {rclcpp::Parameter("max_speed", speed)},
      [this, speed](
        std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>> future) {
        for (const auto & result : future.get()) {
          if (!result.successful) {
            RCLCPP_ERROR(get_logger(),
              "max_speed=%.2f rejected by control: %s", speed, result.reason.c_str());
            publishStatus("control max_speed update rejected");
            return;
          }
        }
        RCLCPP_INFO(get_logger(), "control max_speed set to %.2f m/s", speed);
        publishStatus("control max_speed " + std::to_string(speed));
      });
  }

  // ---------------------------------------------------------------- FSM tick
  void tick()
  {
    trySendAvoidGate();
    trySendControlMaxSpeed();
    switch (stage_) {
      case Stage::kIdle:
        if (!fired_ && adapter_ && lap_count_ >= trigger_lap_count_) {
          if (snapshot_rejected_) {
            abort("trigger lap reached but the last obstacle snapshot was "
              "rejected (projection failure); refusing to freeze a stale ledger");
            break;
          }
          fired_ = true;
          frozen_ = ledger_.snapshot();
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
            startGeneration(std::nullopt, initial_smooth_sigma_, initial_morph_kernel_);
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
              "generation gates failed; retrying with safety_width=%.2f, smooth_sigma=%.2f",
              retry_safety_width_, retry_smooth_sigma_);
            startGeneration(retry_safety_width_, retry_smooth_sigma_, retry_morph_kernel_);
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

      case Stage::kMonitoring:
        // Terminal: the swapped line is final for the rest of the run.
        // Obstacles disappearing used to trigger either a regeneration (some gone)
        // or a baseline rollback swap (all gone); both were removed on purpose so
        // nothing re-swaps the global line after the pipeline has committed it.
        // Newly appearing obstacles were never reflected here either -- local
        // avoidance owns everything that shows up after the swap.
        break;

      case Stage::kAborted:
        break;
    }
  }

  // ---------------------------------------------------------------- members
  std::string obstacle_map_topic_, global_waypoints_topic_, lap_count_topic_;
  std::string reload_service_;
  bool disable_avoid_after_swap_{true};
  std::string state_machine_node_name_;
  std::string control_node_name_;
  double swap_max_speed_mps_{0.0};
  int trigger_lap_count_{2};
  double ego_lookback_m_{12.0};
  double reference_alignment_tolerance_m_{0.05};
  double max_obstacle_projection_d_m_{2.0};
  local_planning::RacelineSplineParameters decision_params_;
  std::string base_map_yaml_;
  MapPainterConfig painter_config_;
  std::string generator_driver_, gui_params_yaml_, output_base_dir_;
  std::string output_map_name_, baseline_map_name_, python_executable_;
  bool reseed_on_startup_{true};
  double generation_timeout_sec_{120.0};
  double initial_smooth_sigma_{4.1};
  double retry_safety_width_{0.4};
  double retry_smooth_sigma_{2.5};
  int initial_morph_kernel_{0};
  int retry_morph_kernel_{0};
  double min_clearance_after_{0.42};
  int max_swap_deferral_laps_{3};

  ObstacleLedger ledger_;
  std::unique_ptr<SidePlannerAdapter> adapter_;
  global_planning::ClcsFrenetConverter::Ptr clcs_;
  std::string reference_frame_id_;
  f110_msgs::msg::ObstacleArray::SharedPtr pending_obs_;
  bool snapshot_rejected_{false};
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
  rclcpp::AsyncParametersClient::SharedPtr state_machine_param_client_;
  std::optional<bool> pending_avoid_gate_;
  rclcpp::AsyncParametersClient::SharedPtr control_param_client_;
  std::optional<double> pending_control_max_speed_;
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
