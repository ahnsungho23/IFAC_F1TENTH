// ================================================================================================
// OPPONENT DETECTOR NODE implementation
// ================================================================================================

#include "opponent_detector/opponent_detector_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <numeric>

#include <builtin_interfaces/msg/time.hpp>
#include <f110_msgs/msg/obstacle.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <visualization_msgs/msg/marker.hpp>

namespace opponent_detector
{

namespace
{
double stampToSec(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

double yawFromQuat(const geometry_msgs::msg::Quaternion &q)
{
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

OpponentDetectorNode::OpponentDetectorNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("opponent_detector", options)
{
    declareParameters();
    loadParameters();

    tracker_.configure(tracker_params_, &frenet_);

    // The overtake spline must never cross real structure the raceline CSV bounds do not know
    // about (e.g. obstacle-baked maps): validate every candidate path against the live /map.
    planner_.setCollisionChecker(
        [this](double x, double y) { return pathPointCollides(x, y); });

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // latched (transient_local) QoS for the raceline and the map
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OpponentDetectorNode::scanCallback, this, std::placeholders::_1));
    global_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
        global_wpnts_topic_, latched_qos,
        std::bind(&OpponentDetectorNode::globalWpntsCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, latched_qos,
        std::bind(&OpponentDetectorNode::mapCallback, this, std::placeholders::_1));
    ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ego_odom_topic_, rclcpp::QoS(10),
        std::bind(&OpponentDetectorNode::egoOdomCallback, this, std::placeholders::_1));
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&OpponentDetectorNode::imuCallback, this, std::placeholders::_1));

    obstacles_pub_ = this->create_publisher<f110_msgs::msg::ObstacleArray>(obstacles_topic_, 10);
    if (publish_raw_)
    {
        raw_obstacles_pub_ =
            this->create_publisher<f110_msgs::msg::ObstacleArray>(raw_obstacles_topic_, 10);
    }
    proj_opp_pub_ = this->create_publisher<f110_msgs::msg::ProjOppTraj>(proj_opp_topic_, 10);
    ot_pub_ = this->create_publisher<f110_msgs::msg::OTWpntArray>(ot_topic_, 10);
    avoid_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(avoid_path_topic_, 10);
    opp_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(opp_path_topic_, 10);
    if (publish_markers_)
    {
        markers_pub_ =
            this->create_publisher<visualization_msgs::msg::MarkerArray>(markers_topic_, 10);
    }

    RCLCPP_INFO(this->get_logger(),
                "opponent_detector started (scan=%s, global=%s, map_filter=%s, classifier=%d, "
                "deskew=%s(%s), merge=%s, mahalanobis=%s, simulator=%s)",
                scan_topic_.c_str(), global_wpnts_topic_.c_str(), use_map_filter_ ? "on" : "off",
                static_cast<int>(tracker_params_.classifier_mode),
                deskew_enable_ ? "on" : "off", deskew_source_.c_str(),
                cluster_merge_enable_ ? "on" : "off",
                tracker_params_.assoc_use_mahalanobis ? "on" : "off",
                simulator_ ? "true" : "false");
}

// ------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------
void OpponentDetectorNode::declareParameters()
{
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("ego_odom_topic", "/pf/pose/odom");
    this->declare_parameter<std::string>("imu_topic", "/sensors/imu/raw");
    this->declare_parameter<std::string>("obstacles_topic", "/perception/obstacles");
    this->declare_parameter<std::string>("raw_obstacles_topic", "/perception/detection/raw_obstacles");
    this->declare_parameter<std::string>("proj_opp_traj_topic", "/proj_opponent_trajectory");
    this->declare_parameter<std::string>("markers_topic", "/perception/obstacles/markers");
    this->declare_parameter<std::string>("map_frame", "map");
    this->declare_parameter<bool>("simulator", false);

    this->declare_parameter<double>("max_range", 10.0);

    // clustering
    this->declare_parameter<double>("lambda_deg", 10.0);
    this->declare_parameter<double>("cluster_sigma", 0.03);
    this->declare_parameter<double>("min_2_points_dist", 0.01);
    this->declare_parameter<int>("min_cluster_points", 5);
    // must exceed the diagonal of the largest car / static obstacle: a 0.5x0.5 m box is 0.707 m
    this->declare_parameter<double>("max_obs_size", 0.8);
    // Per-beam motion compensation. LaserScan header.stamp is the first ray; later rays are
    // transformed back to that pose using IMU yaw rate, with odometry as a simulator/fallback.
    this->declare_parameter<bool>("deskew_enable", true);
    this->declare_parameter<std::string>("deskew_source", "auto");
    this->declare_parameter<bool>("deskew_translation_enable", false);
    this->declare_parameter<double>("deskew_sensor_timeout", 0.15);
    this->declare_parameter<double>("imu_angular_scale", M_PI / 180.0);

    // Optional raw-range filtering. Defaults preserve small/far obstacles until tuned in f1sim_C.
    this->declare_parameter<bool>("noise_filter_enable", false);
    this->declare_parameter<double>("noise_eps", 0.05);
    this->declare_parameter<double>("noise_eps_scale", 1.5);
    this->declare_parameter<int>("noise_min_neighbors", 2);
    this->declare_parameter<int>("noise_search_halfwidth", 4);
    this->declare_parameter<int>("scan_median_window", 1);

    // Merge fragments of one physical object before CLCS projection/tracking.
    this->declare_parameter<bool>("cluster_merge_enable", true);
    this->declare_parameter<double>("cluster_merge_distance", 0.12);
    this->declare_parameter<int>("cluster_merge_min_fragment_points", 2);

    // Scale the Kalman measurement covariance for far, sparse, or high-yaw-rate detections.
    this->declare_parameter<double>("meas_range_var_scale", 2.0);
    this->declare_parameter<double>("meas_sparse_var_scale", 1.5);
    this->declare_parameter<double>("meas_yaw_rate_var_scale", 0.25);
    this->declare_parameter<int>("meas_reference_points", 8);

    // filtering
    this->declare_parameter<double>("max_viewing_distance", 9.0);
    this->declare_parameter<double>("view_behind_distance", 1.0);
    this->declare_parameter<double>("boundaries_inflation", 0.1);
    this->declare_parameter<double>("fallback_track_halfwidth", 1.5);
    this->declare_parameter<bool>("use_map_filter", true);
    this->declare_parameter<int>("map_occupied_thresh", 50);
    this->declare_parameter<int>("map_inflation_cells", 1);
    this->declare_parameter<double>("map_point_reject_ratio", 0.6);

    // output
    this->declare_parameter<bool>("publish_raw", true);
    this->declare_parameter<bool>("publish_markers", true);
    this->declare_parameter<int>("proj_traj_max_points", 200);

    // tracker
    this->declare_parameter<double>("meas_var_s", 0.002);
    this->declare_parameter<double>("meas_var_d", 0.002);
    this->declare_parameter<double>("process_var_vs", 2.0);
    this->declare_parameter<double>("process_var_vd", 8.0);
    this->declare_parameter<double>("assoc_gate", 0.5);
    this->declare_parameter<double>("aggro_multi", 2.0);
    this->declare_parameter<bool>("assoc_use_mahalanobis", true);
    this->declare_parameter<double>("assoc_mahalanobis_gate", 9.21);
    this->declare_parameter<int>("ttl_dynamic", 40);
    this->declare_parameter<int>("ttl_static", 3);
    this->declare_parameter<int>("min_hits_confirm", 3);
    this->declare_parameter<std::string>("classifier_mode", "velocity");
    this->declare_parameter<double>("dyn_vel_enter", 0.5);
    this->declare_parameter<double>("dyn_vel_exit", 0.25);
    this->declare_parameter<int>("dyn_min_frames", 3);
    this->declare_parameter<double>("vs_reset", 0.1);
    this->declare_parameter<double>("static_ref_gate", 0.3);
    this->declare_parameter<int>("std_window", 30);
    this->declare_parameter<int>("min_nb_meas", 5);
    this->declare_parameter<double>("min_std", 0.16);
    this->declare_parameter<double>("max_std", 0.20);
    this->declare_parameter<double>("dt_max", 0.5);

    // overtake planner (committed spline overtaking line) — all tunables live in the single YAML
    this->declare_parameter<std::string>("avoidance_ot_topic", "/overtake_waypoints");
    this->declare_parameter<std::string>("avoidance_path_topic", "/planner/avoidance/path");
    this->declare_parameter<std::string>("opponent_path_topic", "/perception/opponent/path");
    this->declare_parameter<bool>("avoidance_enabled", true);
    this->declare_parameter<double>("avoid_trigger_min_ds", 0.5);
    this->declare_parameter<double>("avoid_trigger_max_ds", 8.0);
    this->declare_parameter<double>("avoid_block_margin", 0.15);
    // catchability + corner gates
    this->declare_parameter<double>("ot_min_rel_vel", 0.5);
    this->declare_parameter<double>("ot_abort_rel_vel", 0.2);
    this->declare_parameter<double>("ot_max_catch_time", 6.0);
    this->declare_parameter<double>("ot_max_kappa", 0.6);
    // spline shape
    this->declare_parameter<double>("avoid_pre_distance", 2.0);
    this->declare_parameter<double>("avoid_post_distance", 2.0);
    this->declare_parameter<double>("ot_pass_clearance_s", 1.0);
    this->declare_parameter<double>("avoid_lateral_clearance", 0.30);
    this->declare_parameter<double>("avoid_boundary_margin", 0.20);
    this->declare_parameter<double>("ot_max_d_slope", 0.40);
    this->declare_parameter<double>("ot_max_path_kappa", 0.5);
    this->declare_parameter<double>("ego_half_width", 0.15);
    this->declare_parameter<double>("opponent_half_width", 0.25);
    this->declare_parameter<double>("avoid_spline_resolution", 0.15);
    this->declare_parameter<std::string>("avoid_side_mode", "auto");
    this->declare_parameter<int>("avoid_min_points", 5);
    // speed profile
    this->declare_parameter<double>("avoid_max_lat_accel", 6.0);
    this->declare_parameter<double>("ot_max_long_accel", 4.0);
    this->declare_parameter<double>("avoid_speed_scale", 1.0);
    this->declare_parameter<double>("avoid_v_floor", 1.0);
    this->declare_parameter<double>("avoid_v_ceiling", 9.0);
    this->declare_parameter<double>("ot_speed_blend_s", 1.5);
    // committed-path validity monitoring
    this->declare_parameter<double>("path_intrusion_margin", 0.5);
    this->declare_parameter<double>("replan_ds_threshold", 0.5);
    this->declare_parameter<double>("replan_dd_threshold", 0.2);
    this->declare_parameter<double>("ot_ego_dev_replan", 0.4);
    this->declare_parameter<double>("ego_pose_grace_s", 0.3);
    this->declare_parameter<double>("ot_completion_margin", 1.0);
    this->declare_parameter<double>("ot_max_duration_s", 10.0);
    this->declare_parameter<double>("ot_cooldown_s", 2.0);
    this->declare_parameter<double>("ot_map_clearance", 0.20);
    this->declare_parameter<bool>("ot_trail_enabled", true);
    this->declare_parameter<double>("ot_trail_gap", 1.5);
}

void OpponentDetectorNode::loadParameters()
{
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    global_wpnts_topic_ = this->get_parameter("global_waypoints_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    ego_odom_topic_ = this->get_parameter("ego_odom_topic").as_string();
    imu_topic_ = this->get_parameter("imu_topic").as_string();
    obstacles_topic_ = this->get_parameter("obstacles_topic").as_string();
    raw_obstacles_topic_ = this->get_parameter("raw_obstacles_topic").as_string();
    proj_opp_topic_ = this->get_parameter("proj_opp_traj_topic").as_string();
    markers_topic_ = this->get_parameter("markers_topic").as_string();
    ot_topic_ = this->get_parameter("avoidance_ot_topic").as_string();
    avoid_path_topic_ = this->get_parameter("avoidance_path_topic").as_string();
    opp_path_topic_ = this->get_parameter("opponent_path_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    simulator_ = this->get_parameter("simulator").as_bool();

    max_range_ = this->get_parameter("max_range").as_double();

    lambda_rad_ = this->get_parameter("lambda_deg").as_double() * M_PI / 180.0;
    cluster_sigma_ = this->get_parameter("cluster_sigma").as_double();
    min_2_points_dist_ = this->get_parameter("min_2_points_dist").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    max_obs_size_ = this->get_parameter("max_obs_size").as_double();
    deskew_enable_ = this->get_parameter("deskew_enable").as_bool();
    deskew_source_ = this->get_parameter("deskew_source").as_string();
    deskew_translation_enable_ =
        this->get_parameter("deskew_translation_enable").as_bool();
    deskew_sensor_timeout_ =
        std::max(0.0, this->get_parameter("deskew_sensor_timeout").as_double());
    imu_angular_scale_ = this->get_parameter("imu_angular_scale").as_double();
    noise_filter_enable_ = this->get_parameter("noise_filter_enable").as_bool();
    noise_eps_ = std::max(0.0, this->get_parameter("noise_eps").as_double());
    noise_eps_scale_ = std::max(0.0, this->get_parameter("noise_eps_scale").as_double());
    noise_min_neighbors_ = std::max(
        0, static_cast<int>(this->get_parameter("noise_min_neighbors").as_int()));
    noise_search_halfwidth_ =
        std::max(1, static_cast<int>(
            this->get_parameter("noise_search_halfwidth").as_int()));
    scan_median_window_ = std::max(
        1, static_cast<int>(this->get_parameter("scan_median_window").as_int()));
    if (scan_median_window_ % 2 == 0)
    {
        ++scan_median_window_;
        RCLCPP_WARN(this->get_logger(),
                    "scan_median_window must be odd; using %d", scan_median_window_);
    }
    cluster_merge_enable_ = this->get_parameter("cluster_merge_enable").as_bool();
    cluster_merge_distance_ =
        std::max(0.0, this->get_parameter("cluster_merge_distance").as_double());
    cluster_merge_min_fragment_points_ =
        std::max(1, static_cast<int>(
            this->get_parameter("cluster_merge_min_fragment_points").as_int()));
    meas_range_var_scale_ =
        std::max(0.0, this->get_parameter("meas_range_var_scale").as_double());
    meas_sparse_var_scale_ =
        std::max(0.0, this->get_parameter("meas_sparse_var_scale").as_double());
    meas_yaw_rate_var_scale_ =
        std::max(0.0, this->get_parameter("meas_yaw_rate_var_scale").as_double());
    meas_reference_points_ =
        std::max(1, static_cast<int>(
            this->get_parameter("meas_reference_points").as_int()));

    max_viewing_distance_ = this->get_parameter("max_viewing_distance").as_double();
    view_behind_distance_ = this->get_parameter("view_behind_distance").as_double();
    boundaries_inflation_ = this->get_parameter("boundaries_inflation").as_double();
    use_map_filter_ = this->get_parameter("use_map_filter").as_bool();
    map_occupied_thresh_ = this->get_parameter("map_occupied_thresh").as_int();
    map_inflation_cells_ = this->get_parameter("map_inflation_cells").as_int();
    map_point_reject_ratio_ = this->get_parameter("map_point_reject_ratio").as_double();

    publish_raw_ = this->get_parameter("publish_raw").as_bool();
    publish_markers_ = this->get_parameter("publish_markers").as_bool();
    proj_traj_max_points_ = this->get_parameter("proj_traj_max_points").as_int();

    tracker_params_.meas_var_s = this->get_parameter("meas_var_s").as_double();
    tracker_params_.meas_var_d = this->get_parameter("meas_var_d").as_double();
    tracker_params_.process_var_vs = this->get_parameter("process_var_vs").as_double();
    tracker_params_.process_var_vd = this->get_parameter("process_var_vd").as_double();
    tracker_params_.assoc_gate = this->get_parameter("assoc_gate").as_double();
    tracker_params_.aggro_multi = this->get_parameter("aggro_multi").as_double();
    tracker_params_.assoc_use_mahalanobis =
        this->get_parameter("assoc_use_mahalanobis").as_bool();
    tracker_params_.assoc_mahalanobis_gate =
        std::max(0.1, this->get_parameter("assoc_mahalanobis_gate").as_double());
    tracker_params_.ttl_dynamic = this->get_parameter("ttl_dynamic").as_int();
    tracker_params_.ttl_static = this->get_parameter("ttl_static").as_int();
    tracker_params_.min_hits_confirm = this->get_parameter("min_hits_confirm").as_int();
    tracker_params_.dyn_vel_enter = this->get_parameter("dyn_vel_enter").as_double();
    tracker_params_.dyn_vel_exit = this->get_parameter("dyn_vel_exit").as_double();
    tracker_params_.dyn_min_frames = this->get_parameter("dyn_min_frames").as_int();
    tracker_params_.vs_reset = this->get_parameter("vs_reset").as_double();
    tracker_params_.static_ref_gate = this->get_parameter("static_ref_gate").as_double();
    tracker_params_.std_window = this->get_parameter("std_window").as_int();
    tracker_params_.min_nb_meas = this->get_parameter("min_nb_meas").as_int();
    tracker_params_.min_std = this->get_parameter("min_std").as_double();
    tracker_params_.max_std = this->get_parameter("max_std").as_double();
    tracker_params_.dt_max = this->get_parameter("dt_max").as_double();

    const std::string cm = this->get_parameter("classifier_mode").as_string();
    if (cm == "std")
    {
        tracker_params_.classifier_mode = ClassifierMode::Std;
    }
    else if (cm == "both")
    {
        tracker_params_.classifier_mode = ClassifierMode::Both;
    }
    else
    {
        tracker_params_.classifier_mode = ClassifierMode::Velocity;
    }

    ot_params_.enabled = this->get_parameter("avoidance_enabled").as_bool();
    ot_params_.trigger_min_ds = this->get_parameter("avoid_trigger_min_ds").as_double();
    ot_params_.trigger_max_ds = this->get_parameter("avoid_trigger_max_ds").as_double();
    ot_params_.block_margin = this->get_parameter("avoid_block_margin").as_double();
    ot_params_.min_rel_vel = this->get_parameter("ot_min_rel_vel").as_double();
    ot_params_.abort_rel_vel = this->get_parameter("ot_abort_rel_vel").as_double();
    ot_params_.max_catch_time = this->get_parameter("ot_max_catch_time").as_double();
    ot_params_.max_kappa = this->get_parameter("ot_max_kappa").as_double();
    ot_params_.pre_distance = this->get_parameter("avoid_pre_distance").as_double();
    ot_params_.post_distance = this->get_parameter("avoid_post_distance").as_double();
    ot_params_.pass_clearance_s = this->get_parameter("ot_pass_clearance_s").as_double();
    ot_params_.lateral_clearance = this->get_parameter("avoid_lateral_clearance").as_double();
    ot_params_.boundary_margin = this->get_parameter("avoid_boundary_margin").as_double();
    ot_params_.max_d_slope = this->get_parameter("ot_max_d_slope").as_double();
    ot_params_.max_path_kappa = this->get_parameter("ot_max_path_kappa").as_double();
    ot_params_.ego_half_width = this->get_parameter("ego_half_width").as_double();
    ot_params_.opponent_half_width = this->get_parameter("opponent_half_width").as_double();
    ot_params_.spline_resolution = this->get_parameter("avoid_spline_resolution").as_double();
    ot_params_.min_points = this->get_parameter("avoid_min_points").as_int();
    ot_params_.max_lat_accel = this->get_parameter("avoid_max_lat_accel").as_double();
    ot_params_.max_long_accel = this->get_parameter("ot_max_long_accel").as_double();
    ot_params_.speed_scale = this->get_parameter("avoid_speed_scale").as_double();
    ot_params_.v_floor = this->get_parameter("avoid_v_floor").as_double();
    ot_params_.v_ceiling = this->get_parameter("avoid_v_ceiling").as_double();
    ot_params_.speed_blend_s = this->get_parameter("ot_speed_blend_s").as_double();
    const std::string sm = this->get_parameter("avoid_side_mode").as_string();
    ot_params_.side_mode = (sm == "left") ? 1 : (sm == "right") ? 2 : 0;
    ot_params_.intrusion_margin = this->get_parameter("path_intrusion_margin").as_double();
    ot_params_.replan_ds_threshold = this->get_parameter("replan_ds_threshold").as_double();
    ot_params_.replan_dd_threshold = this->get_parameter("replan_dd_threshold").as_double();
    ot_params_.ego_dev_replan = this->get_parameter("ot_ego_dev_replan").as_double();
    ego_pose_grace_s_ = this->get_parameter("ego_pose_grace_s").as_double();
    ot_params_.completion_margin = this->get_parameter("ot_completion_margin").as_double();
    ot_params_.max_duration = this->get_parameter("ot_max_duration_s").as_double();
    ot_params_.cooldown = this->get_parameter("ot_cooldown_s").as_double();
    ot_params_.trail_enabled = this->get_parameter("ot_trail_enabled").as_bool();
    ot_params_.trail_gap = this->get_parameter("ot_trail_gap").as_double();
    ot_map_clearance_ = this->get_parameter("ot_map_clearance").as_double();
}

// ------------------------------------------------------------------------------------------------
// Input callbacks (raceline / map / ego pose)
// ------------------------------------------------------------------------------------------------
void OpponentDetectorNode::globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
    if (msg->wpnts.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Received /global_waypoints with < 3 points; CLCS needs >= 3. Ignoring.");
        return;
    }

    // lightweight projector: kept only for track-boundary (d_left/d_right) lookup and s-wrap
    std::vector<FrenetProjector::Waypoint> wpnts;
    wpnts.reserve(msg->wpnts.size());
    for (const auto &w : msg->wpnts)
    {
        FrenetProjector::Waypoint fw;
        fw.x = w.x_m;
        fw.y = w.y_m;
        fw.s = w.s_m;
        fw.psi = w.psi_rad;
        fw.d_left = w.d_left;
        fw.d_right = w.d_right;
        fw.vx = w.vx_mps;
        wpnts.push_back(fw);
    }
    frenet_.build(std::move(wpnts), true);

    // CLCS converter: the accurate (x,y) -> (s,d) projection used for clusters and ego.
    std::vector<global_planning::ReferenceWaypoint> ref;
    ref.reserve(msg->wpnts.size());
    for (const auto &w : msg->wpnts)
    {
        global_planning::ReferenceWaypoint rw;
        rw.x = w.x_m;
        rw.y = w.y_m;
        rw.s = w.s_m;
        ref.push_back(rw);
    }
    try
    {
        global_planning::ClcsFrenetConfig cfg;  // closed_loop=true + sane projection-domain defaults
        auto conv = global_planning::ClcsFrenetConverter::create(ref, cfg, ++clcs_version_);
        converter_ = conv;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "CLCS converter built from %zu waypoints (track length %.2f m).",
                         ref.size(), conv->stats().track_length);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "CLCS converter build failed (keeping previous, if any): %s", e.what());
    }

    // overtake planner reference: keep the raw raceline for Frenet(s,d) -> map(x,y)
    // reconstruction. Unset waypoint bounds fall back to fallback_track_halfwidth (same rule as
    // the detection corridor gate) so the planner does not see a zero-width track.
    global_wpnts_ = *msg;
    planner_.setReference(global_wpnts_, frenet_.raceline_length(),
                          this->get_parameter("fallback_track_halfwidth").as_double());
}

void OpponentDetectorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_msg_ = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Occupancy map received (%u x %u @ %.3f m).",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void OpponentDetectorNode::egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    ego_x_ = msg->pose.pose.position.x;
    ego_y_ = msg->pose.pose.position.y;
    // planar speed (twist is body-frame; the magnitude is what the catchability gate needs)
    ego_v_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    odom_linear_x_ = msg->twist.twist.linear.x;
    odom_linear_y_ = msg->twist.twist.linear.y;
    odom_yaw_rate_ = msg->twist.twist.angular.z;
    odom_motion_stamp_ = stampToSec(msg->header.stamp);
    have_ego_pose_ = true;
    if (converter_)
    {
        global_planning::ClcsConversionInput ci;
        ci.x = ego_x_;
        ci.y = ego_y_;
        const auto cr = converter_->convert(ci);
        if (cr.valid)
        {
            ego_s_ = cr.s;
            ego_d_ = cr.d;
            // Freshness stamp: a FAILED projection (ego outside the CLCS domain, e.g. pushed
            // off-track by a crash) keeps the old (s,d) but does NOT refresh this stamp, so the
            // planner stops planning from the stale pose after ego_pose_grace_s.
            ego_frenet_stamp_ = stampToSec(msg->header.stamp);
        }
    }
}

void OpponentDetectorNode::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    const double scaled_yaw_rate = msg->angular_velocity.z * imu_angular_scale_;
    if (!std::isfinite(scaled_yaw_rate))
    {
        return;
    }
    imu_yaw_rate_ = scaled_yaw_rate;
    imu_stamp_ = stampToSec(msg->header.stamp);
}

OpponentDetectorNode::DeskewMotion
OpponentDetectorNode::selectDeskewMotion(double scan_stamp) const
{
    DeskewMotion motion;
    if (!deskew_enable_)
    {
        return motion;
    }

    const auto fresh = [this, scan_stamp](double stamp) {
        return stamp >= 0.0 && scan_stamp >= 0.0 &&
               std::abs(scan_stamp - stamp) <= deskew_sensor_timeout_;
    };
    const bool allow_imu = deskew_source_ == "auto" || deskew_source_ == "imu";
    const bool allow_odom = deskew_source_ == "auto" || deskew_source_ == "odom";
    if (allow_imu && fresh(imu_stamp_))
    {
        motion.enabled = true;
        motion.yaw_rate = imu_yaw_rate_;
        motion.source = "imu";
    }
    else if (allow_odom && fresh(odom_motion_stamp_))
    {
        motion.enabled = true;
        motion.yaw_rate = odom_yaw_rate_;
        motion.source = "odom";
    }
    if (motion.enabled && deskew_translation_enable_ && fresh(odom_motion_stamp_))
    {
        motion.linear_x = odom_linear_x_;
        motion.linear_y = odom_linear_y_;
    }
    return motion;
}

// ------------------------------------------------------------------------------------------------
// TF: scan frame -> map frame
// ------------------------------------------------------------------------------------------------
bool OpponentDetectorNode::lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx,
                                           double &ty, double &yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        tf = tf_buffer_->lookupTransform(map_frame_, scan_header.frame_id, scan_header.stamp,
                                         tf2::durationFromSec(0.05));
    }
    catch (const tf2::TransformException &)
    {
        try
        {
            tf = tf_buffer_->lookupTransform(map_frame_, scan_header.frame_id, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &e)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "TF %s->%s unavailable: %s", map_frame_.c_str(),
                                 scan_header.frame_id.c_str(), e.what());
            return false;
        }
    }
    tx = tf.transform.translation.x;
    ty = tf.transform.translation.y;
    yaw = yawFromQuat(tf.transform.rotation);
    return true;
}

// ------------------------------------------------------------------------------------------------
// Motion-compensated adaptive-breakpoint clustering (points already in map frame)
// ------------------------------------------------------------------------------------------------
std::vector<std::vector<OpponentDetectorNode::ScanPoint>>
OpponentDetectorNode::clusterScan(const sensor_msgs::msg::LaserScan &scan, double tx, double ty,
                                  double yaw, const DeskewMotion &motion,
                                  ScanProcessingStats &stats) const
{
    std::vector<std::vector<ScanPoint>> clusters;
    std::vector<ScanPoint> current;
    const double dphi = scan.angle_increment;
    const double denom = std::sin(lambda_rad_ - dphi);
    const double cyaw = std::cos(yaw);
    const double syaw = std::sin(yaw);
    const std::size_t beam_count = scan.ranges.size();
    std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());

    // A small median window removes isolated range spikes without inventing returns where the
    // sensor reported invalid data. Window 1 preserves the original scan.
    if (scan_median_window_ > 1 && beam_count > 0)
    {
        const int half = scan_median_window_ / 2;
        std::vector<double> filtered(ranges);
        std::vector<double> window;
        window.reserve(static_cast<std::size_t>(scan_median_window_));
        for (std::size_t i = 0; i < beam_count; ++i)
        {
            if (!std::isfinite(ranges[i]))
            {
                continue;
            }
            window.clear();
            const std::size_t lo =
                i > static_cast<std::size_t>(half) ? i - static_cast<std::size_t>(half) : 0;
            const std::size_t hi =
                std::min(beam_count - 1, i + static_cast<std::size_t>(half));
            for (std::size_t j = lo; j <= hi; ++j)
            {
                if (std::isfinite(ranges[j]))
                {
                    window.push_back(ranges[j]);
                }
            }
            if (static_cast<int>(window.size()) > half)
            {
                auto middle = window.begin() +
                    static_cast<std::ptrdiff_t>(window.size() / 2);
                std::nth_element(window.begin(), middle, window.end());
                filtered[i] = *middle;
            }
        }
        ranges.swap(filtered);
    }

    // Ordered-scan DBSCAN core-point test. The adaptive radius grows with range so legitimate
    // far-object returns are not removed merely because adjacent laser rays spread apart.
    std::vector<std::uint8_t> keep(beam_count, 1);
    if (noise_filter_enable_ && noise_min_neighbors_ > 0)
    {
        for (std::size_t i = 0; i < beam_count; ++i)
        {
            const double ri = ranges[i];
            if (!std::isfinite(ri) || ri < scan.range_min || ri >= max_range_)
            {
                continue;
            }
            const double ai = scan.angle_min + static_cast<double>(i) * dphi;
            const double xi = ri * std::cos(ai);
            const double yi = ri * std::sin(ai);
            double eps = 3.0 * cluster_sigma_;
            if (denom > 1e-6)
            {
                eps += ri * std::sin(dphi) / denom;
            }
            eps = std::max(noise_eps_, noise_eps_scale_ * eps);
            const double eps2 = eps * eps;
            const std::size_t halfwidth =
                static_cast<std::size_t>(noise_search_halfwidth_);
            const std::size_t lo = i > halfwidth ? i - halfwidth : 0;
            const std::size_t hi = std::min(beam_count - 1, i + halfwidth);
            int neighbors = 0;
            for (std::size_t j = lo; j <= hi && neighbors < noise_min_neighbors_; ++j)
            {
                if (j == i)
                {
                    continue;
                }
                const double rj = ranges[j];
                if (!std::isfinite(rj) || rj < scan.range_min || rj >= max_range_)
                {
                    continue;
                }
                const double aj = scan.angle_min + static_cast<double>(j) * dphi;
                const double dx = rj * std::cos(aj) - xi;
                const double dy = rj * std::sin(aj) - yi;
                if (dx * dx + dy * dy <= eps2)
                {
                    ++neighbors;
                }
            }
            if (neighbors < noise_min_neighbors_)
            {
                keep[i] = 0;
                ++stats.noise_rejected;
            }
        }
    }

    bool have_prev = false;
    ScanPoint prev{};
    int prev_index = -1000;

    auto flush = [&]() {
        const int required_points =
            cluster_merge_enable_ ? cluster_merge_min_fragment_points_ : min_cluster_points_;
        if (static_cast<int>(current.size()) >= required_points)
        {
            clusters.push_back(current);
        }
        current.clear();
    };

    double time_increment = static_cast<double>(scan.time_increment);
    if (time_increment <= 0.0 && beam_count > 1 && scan.scan_time > 0.0)
    {
        time_increment = static_cast<double>(scan.scan_time) /
            static_cast<double>(beam_count - 1);
    }
    for (std::size_t i = 0; i < beam_count; ++i)
    {
        const double r = ranges[i];
        if (!std::isfinite(r) || r < scan.range_min || r >= max_range_ || keep[i] == 0)
        {
            continue;  // invalid beam breaks contiguity (handled by index gap below)
        }
        ++stats.valid_beams;
        const double ang = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        const double lx = r * std::cos(ang);
        const double ly = r * std::sin(ang);
        const double dt = motion.enabled ? static_cast<double>(i) * time_increment : 0.0;
        const double delta_yaw = motion.yaw_rate * dt;
        const double cdelta = std::cos(delta_yaw);
        const double sdelta = std::sin(delta_yaw);
        // Transform a point measured at t_i back into the laser pose at the first ray. Body-frame
        // translation is optional because yaw-rate deskew is the dominant, better-observed term.
        const double compensated_x =
            cdelta * lx - sdelta * ly + motion.linear_x * dt;
        const double compensated_y =
            sdelta * lx + cdelta * ly + motion.linear_y * dt;
        ScanPoint pt;
        pt.x = tx + cyaw * compensated_x - syaw * compensated_y;
        pt.y = ty + syaw * compensated_x + cyaw * compensated_y;
        pt.range = r;

        bool same_cluster = false;
        if (have_prev && static_cast<int>(i) == prev_index + 1)
        {
            // adaptive breakpoint threshold (Borges/Aldon)
            double d_max = 3.0 * cluster_sigma_;
            if (denom > 1e-6)
            {
                d_max += r * std::sin(dphi) / denom;
            }
            const double dist = std::hypot(pt.x - prev.x, pt.y - prev.y);
            same_cluster = (dist <= std::max(d_max, min_2_points_dist_));
        }

        if (!same_cluster)
        {
            flush();
        }
        current.push_back(pt);
        prev = pt;
        prev_index = static_cast<int>(i);
        have_prev = true;
    }
    flush();
    stats.clusters_before_merge = clusters.size();
    return mergeClusters(std::move(clusters), stats);
}

std::vector<std::vector<OpponentDetectorNode::ScanPoint>>
OpponentDetectorNode::mergeClusters(
    std::vector<std::vector<ScanPoint>> clusters, ScanProcessingStats &stats) const
{
    if (cluster_merge_enable_ && clusters.size() > 1)
    {
        const auto bounds = [](const std::vector<ScanPoint> &cluster) {
            std::array<double, 4> result{
                cluster.front().x, cluster.front().x, cluster.front().y, cluster.front().y};
            for (const auto &point : cluster)
            {
                result[0] = std::min(result[0], point.x);
                result[1] = std::max(result[1], point.x);
                result[2] = std::min(result[2], point.y);
                result[3] = std::max(result[3], point.y);
            }
            return result;
        };
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (std::size_t i = 0; i < clusters.size() && !changed; ++i)
            {
                const auto a = bounds(clusters[i]);
                for (std::size_t j = i + 1; j < clusters.size(); ++j)
                {
                    const auto b = bounds(clusters[j]);
                    const double gap_x =
                        std::max({0.0, a[0] - b[1], b[0] - a[1]});
                    const double gap_y =
                        std::max({0.0, a[2] - b[3], b[2] - a[3]});
                    if (std::hypot(gap_x, gap_y) > cluster_merge_distance_)
                    {
                        continue;
                    }
                    const double merged_min_x = std::min(a[0], b[0]);
                    const double merged_max_x = std::max(a[1], b[1]);
                    const double merged_min_y = std::min(a[2], b[2]);
                    const double merged_max_y = std::max(a[3], b[3]);
                    if (std::hypot(merged_max_x - merged_min_x,
                                   merged_max_y - merged_min_y) > max_obs_size_)
                    {
                        continue;
                    }
                    clusters[i].insert(
                        clusters[i].end(), clusters[j].begin(), clusters[j].end());
                    clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(j));
                    changed = true;
                    break;
                }
            }
        }
    }
    clusters.erase(
        std::remove_if(
            clusters.begin(), clusters.end(),
            [this](const auto &cluster) {
                return static_cast<int>(cluster.size()) < min_cluster_points_;
            }),
        clusters.end());
    stats.clusters_after_merge = clusters.size();
    return clusters;
}

// ------------------------------------------------------------------------------------------------
// Occupancy-grid lookup for the map-based static filter
// ------------------------------------------------------------------------------------------------
bool OpponentDetectorNode::occupiedInMap(double x, double y) const
{
    if (!map_msg_)
    {
        return false;
    }
    const auto &info = map_msg_->info;
    if (info.resolution <= 0.0)
    {
        return false;
    }
    const int gx = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
    const int gy = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
    const int w = static_cast<int>(info.width);
    const int h = static_cast<int>(info.height);
    for (int dy = -map_inflation_cells_; dy <= map_inflation_cells_; ++dy)
    {
        for (int dx = -map_inflation_cells_; dx <= map_inflation_cells_; ++dx)
        {
            const int cx = gx + dx;
            const int cy = gy + dy;
            if (cx < 0 || cy < 0 || cx >= w || cy >= h)
            {
                continue;
            }
            const int8_t v = map_msg_->data[static_cast<std::size_t>(cy) * w + cx];
            if (v >= map_occupied_thresh_)
            {
                return true;
            }
        }
    }
    return false;
}

bool OpponentDetectorNode::pathPointCollides(double x, double y) const
{
    // Overtake-path sample check: the point itself plus a ring of ot_map_clearance radius must be
    // free in the live /map, so the planned line keeps body clearance from structure the raceline
    // CSV bounds may not know about (e.g. obstacle-baked maps).
    if (occupiedInMap(x, y))
    {
        return true;
    }
    const double r = ot_map_clearance_;
    if (r <= 1e-3)
    {
        return false;
    }
    for (int k = 0; k < 6; ++k)
    {
        const double a = static_cast<double>(k) * M_PI / 3.0;
        if (occupiedInMap(x + r * std::cos(a), y + r * std::sin(a)))
        {
            return true;
        }
    }
    return false;
}

// ------------------------------------------------------------------------------------------------
// Main pipeline
// ------------------------------------------------------------------------------------------------
void OpponentDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!frenet_.ready() || !converter_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for /global_waypoints before detecting opponents...");
        return;
    }

    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!lookupScanToMap(msg->header, tx, ty, yaw))
    {
        return;
    }

    const double stamp = stampToSec(msg->header.stamp);
    const DeskewMotion deskew_motion = selectDeskewMotion(stamp);
    ScanProcessingStats scan_stats;
    const auto clusters =
        clusterScan(*msg, tx, ty, yaw, deskew_motion, scan_stats);

    std::vector<Detection> detections;
    f110_msgs::msg::ObstacleArray raw_array;
    raw_array.header = msg->header;
    raw_array.header.frame_id = map_frame_;

    int raw_id = 0;
    for (const auto &cluster : clusters)
    {
        // centroid + axis-aligned box size
        double cx = 0.0;
        double cy = 0.0;
        double minx = cluster.front().x;
        double maxx = cluster.front().x;
        double miny = cluster.front().y;
        double maxy = cluster.front().y;
        for (const auto &p : cluster)
        {
            cx += p.x;
            cy += p.y;
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }
        cx /= static_cast<double>(cluster.size());
        cy /= static_cast<double>(cluster.size());
        const double size = std::hypot(maxx - minx, maxy - miny);
        if (size > max_obs_size_)
        {
            ++scan_stats.clusters_size_rejected;
            continue;  // too big to be an F1TENTH car
        }

        // accurate CLCS projection of the cluster centre; invalid => outside the projection domain
        // (not on/near the track) => drop.
        global_planning::ClcsConversionInput ci;
        ci.x = cx;
        ci.y = cy;
        const auto cr = converter_->convert(ci);
        if (!cr.valid)
        {
            ++scan_stats.clusters_projection_rejected;
            continue;
        }
        const FrenetPoint fp{cr.s, cr.d};

        // viewing-distance gate (ahead of ego)
        if (ego_s_ >= 0.0)
        {
            const double ds = frenet_.wrapDelta(fp.s, ego_s_);
            if (ds > max_viewing_distance_ || ds < -view_behind_distance_)
            {
                continue;
            }
        }

        // track-boundary corridor gate (falls back to a default half-width if bounds are unset)
        double dl = 0.0;
        double dr = 0.0;
        double vx = 0.0;
        frenet_.boundsAtS(fp.s, dl, dr, vx);
        const double fallback = this->get_parameter("fallback_track_halfwidth").as_double();
        const double left_bound = (dl > 0.05 ? dl : fallback) - boundaries_inflation_;
        const double right_bound = (dr > 0.05 ? dr : fallback) - boundaries_inflation_;
        if (fp.d > left_bound || fp.d < -right_bound)
        {
            ++scan_stats.clusters_corridor_rejected;
            continue;
        }

        // map-based static filter: drop clusters that sit on known static structure
        if (use_map_filter_ && map_msg_)
        {
            int occ = 0;
            for (const auto &p : cluster)
            {
                if (occupiedInMap(p.x, p.y))
                {
                    occ++;
                }
            }
            const double ratio = static_cast<double>(occ) / static_cast<double>(cluster.size());
            if (ratio >= map_point_reject_ratio_)
            {
                ++scan_stats.clusters_map_rejected;
                continue;
            }
        }

        Detection det;
        det.s = fp.s;
        det.d = fp.d;
        det.size = size;
        det.x = cx;
        det.y = cy;
        const double mean_range = std::accumulate(
            cluster.begin(), cluster.end(), 0.0,
            [](double sum, const ScanPoint &point) { return sum + point.range; }) /
            static_cast<double>(cluster.size());
        const double range_ratio = max_range_ > 1e-6 ?
            std::clamp(mean_range / max_range_, 0.0, 1.0) : 0.0;
        const double sparse_ratio = std::clamp(
            (static_cast<double>(meas_reference_points_) -
             static_cast<double>(cluster.size())) /
                static_cast<double>(meas_reference_points_),
            0.0, 1.0);
        det.variance_scale =
            1.0 +
            meas_range_var_scale_ * range_ratio * range_ratio +
            meas_sparse_var_scale_ * sparse_ratio +
            meas_yaw_rate_var_scale_ * std::abs(deskew_motion.yaw_rate);
        detections.push_back(det);

        if (publish_raw_)
        {
            f110_msgs::msg::Obstacle ob;
            ob.id = raw_id++;
            ob.s_center = fp.s;
            ob.d_center = fp.d;
            ob.s_start = fp.s - size / 2.0;
            ob.s_end = fp.s + size / 2.0;
            ob.d_left = fp.d + size / 2.0;
            ob.d_right = fp.d - size / 2.0;
            ob.size = size;
            ob.is_static = true;
            ob.is_visible = true;
            raw_array.obstacles.push_back(ob);
        }
    }

    if (publish_raw_ && raw_obstacles_pub_)
    {
        raw_obstacles_pub_->publish(raw_array);
    }

    // ---- tracking ----
    tracker_.update(detections, stamp);
    const auto &tracker_stats = tracker_.lastStats();
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "DIAG perception: beams=%zu noise_drop=%zu clusters=%zu->%zu detections=%zu "
        "reject(size=%zu clcs=%zu corridor=%zu map=%zu) deskew=%s yaw_rate=%.3f "
        "tracks=%zu assoc(match=%d spawn=%d retire=%d euclid_reject=%d maha_reject=%d)",
        scan_stats.valid_beams, scan_stats.noise_rejected,
        scan_stats.clusters_before_merge, scan_stats.clusters_after_merge,
        detections.size(), scan_stats.clusters_size_rejected,
        scan_stats.clusters_projection_rejected, scan_stats.clusters_corridor_rejected,
        scan_stats.clusters_map_rejected, deskew_motion.source, deskew_motion.yaw_rate,
        tracker_.tracks().size(), tracker_stats.matched, tracker_stats.spawned,
        tracker_stats.retired, tracker_stats.euclidean_rejected,
        tracker_stats.mahalanobis_rejected);

    // ---- publish tracked obstacles ----
    f110_msgs::msg::ObstacleArray obs_array;
    obs_array.header = msg->header;
    obs_array.header.frame_id = map_frame_;
    for (const auto &t : tracker_.tracks())
    {
        if (t.hits < tracker_params_.min_hits_confirm)
        {
            continue;
        }
        f110_msgs::msg::Obstacle ob;
        ob.id = t.id;
        ob.s_center = t.s();
        ob.d_center = t.d();
        ob.s_start = t.s() - t.size / 2.0;
        ob.s_end = t.s() + t.size / 2.0;
        ob.d_left = t.d() + t.size / 2.0;
        ob.d_right = t.d() - t.size / 2.0;
        ob.size = t.size;
        ob.vs = t.vs();
        ob.vd = t.vd();
        ob.s_var = t.P(0, 0);
        ob.vs_var = t.P(1, 1);
        ob.d_var = t.P(2, 2);
        ob.vd_var = t.P(3, 3);
        ob.is_static = t.is_static;
        ob.is_visible = t.is_visible;
        ob.is_actually_a_gap = false;
        obs_array.obstacles.push_back(ob);
    }
    obstacles_pub_->publish(obs_array);

    // ---- opponent -> projected Frenet trajectory ----
    const int opp = tracker_.opponentIndex(ego_s_);
    if (opp >= 0)
    {
        const Track &t = tracker_.tracks()[opp];
        f110_msgs::msg::ProjOppPoint pp;
        pp.s = t.s();
        pp.d = t.d();
        pp.vs = t.vs();
        pp.vd = t.vd();
        pp.is_static = false;
        pp.is_visible = t.is_visible;
        pp.time = stamp;
        pp.s_var = t.P(0, 0);
        pp.d_var = t.P(2, 2);
        pp.vs_var = t.P(1, 1);
        pp.vd_var = t.P(3, 3);
        proj_buffer_.push_back(pp);
        while (static_cast<int>(proj_buffer_.size()) > proj_traj_max_points_)
        {
            proj_buffer_.erase(proj_buffer_.begin());
        }
    }

    f110_msgs::msg::ProjOppTraj proj;
    proj.lapcount = proj_lapcount_;
    proj.nrofpoints = static_cast<double>(proj_buffer_.size());
    proj.opp_is_on_trajectory = (opp >= 0);
    proj.detections = proj_buffer_;
    proj_opp_pub_->publish(proj);

    // ---- overtake planner: committed spline overtaking line (OTWpntArray) ----
    // The state machine decides whether a local path exists at all:
    //   Publish -> the committed overtaking segment (wpnt_publisher overrides global with it)
    //   Clear   -> ONE empty OT so wpnt_publisher falls back to the global raceline
    //   None    -> stay silent (no local path; ego follows global)
    // Commit requires: opponent blocks the corridor, relative speed is sufficient to pass, no
    // sharp corner on the maneuver, and lateral room at the PREDICTED pass location. While
    // committed, validity is re-judged every cycle because the opponent keeps moving.
    // Ego-Frenet freshness gate: plan only from a pose that projected successfully recently.
    // With a stale projection (ego outside the CLCS domain — typically after a wall crash) the
    // planner would keep publishing paths anchored at the frozen pre-crash pose; ego.s = -1
    // makes it Clear an active path ONCE and go silent, so downstream falls back to global.
    const bool ego_fresh =
        ego_frenet_stamp_ >= 0.0 && (stamp - ego_frenet_stamp_) <= ego_pose_grace_s_;
    if (!ego_fresh && have_ego_pose_)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "ego Frenet projection stale (%.2fs > grace %.2fs) — overtake "
                             "planner paused (ego off the projection domain?)",
                             ego_frenet_stamp_ >= 0.0 ? stamp - ego_frenet_stamp_ : -1.0,
                             ego_pose_grace_s_);
    }
    EgoState ego;
    ego.s = ego_fresh ? ego_s_ : -1.0;
    ego.d = ego_d_;
    ego.v = ego_v_;

    OpponentState opp_state;
    if (opp >= 0)
    {
        const Track &t = tracker_.tracks()[opp];
        opp_state.valid = true;
        opp_state.s = t.s();
        opp_state.d = t.d();
        opp_state.vs = t.vs();
        opp_state.vd = t.vd();
    }

    const OvertakeDecision dec = planner_.update(ego, opp_state, ot_params_, stamp);
    const bool publishing = (dec.action == OvertakeDecision::Action::Publish);

    if (!dec.reason.empty())
    {
        RCLCPP_INFO(this->get_logger(), "overtake: %s (side=%s, wpnts=%zu)", dec.reason.c_str(),
                    dec.side.empty() ? "-" : dec.side.c_str(), dec.wpnts.wpnts.size());
    }

    // Commit-gate rejection debug: only meaningful while an opponent is tracked (throttled).
    if (!dec.debug.empty() && opp_state.valid)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "overtake gate reject: %s [ego s=%.2f d=%.2f v=%.2f | opp s=%.2f "
                             "d=%.2f vs=%.2f]",
                             dec.debug.c_str(), ego.s, ego.d, ego.v, opp_state.s, opp_state.d,
                             opp_state.vs);
    }

    if (dec.action != OvertakeDecision::Action::None)
    {
        f110_msgs::msg::OTWpntArray ot;
        ot.header.stamp = msg->header.stamp;
        ot.header.frame_id = map_frame_;
        ot.side_switch = false;
        ot.ot_side = dec.side;
        ot.ot_line = publishing ? (dec.side == "trail" ? "trail" : "overtake") : "";
        if (publishing)
        {
            ot.wpnts = dec.wpnts.wpnts;
        }
        ot_pub_->publish(ot);  // empty on Clear -> wpnt_publisher falls back to global

        // RViz mirror of the overtaking line (empty Path clears it)
        nav_msgs::msg::Path avoid_path;
        avoid_path.header.stamp = msg->header.stamp;
        avoid_path.header.frame_id = map_frame_;
        for (const auto &w : ot.wpnts)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = avoid_path.header;
            ps.pose.position.x = w.x_m;
            ps.pose.position.y = w.y_m;
            ps.pose.orientation.w = 1.0;
            avoid_path.poses.push_back(ps);
        }
        avoid_path_pub_->publish(avoid_path);
    }

    // opponent trajectory: convert the Frenet ProjOpp buffer to map (x,y) for a viewable Path
    nav_msgs::msg::Path opp_path;
    opp_path.header.stamp = msg->header.stamp;
    opp_path.header.frame_id = map_frame_;
    if (planner_.ready())
    {
        for (const auto &pt : proj_buffer_)
        {
            geometry_msgs::msg::PoseStamped ps;
            ps.header = opp_path.header;
            planner_.toCartesian(pt.s, pt.d, ps.pose.position.x, ps.pose.position.y);
            ps.pose.orientation.w = 1.0;
            opp_path.poses.push_back(ps);
        }
    }
    opp_path_pub_->publish(opp_path);

    // ---- markers ----
    if (publish_markers_ && markers_pub_)
    {
        visualization_msgs::msg::MarkerArray ma;
        visualization_msgs::msg::Marker del;
        del.header.frame_id = map_frame_;
        del.header.stamp = msg->header.stamp;
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        ma.markers.push_back(del);
        for (const auto &t : tracker_.tracks())
        {
            if (t.hits < tracker_params_.min_hits_confirm)
            {
                continue;
            }
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp = msg->header.stamp;
            m.ns = "opponent_detector";
            m.id = t.id;
            m.type = visualization_msgs::msg::Marker::CYLINDER;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = t.x_map;
            m.pose.position.y = t.y_map;
            m.pose.position.z = 0.1;
            m.pose.orientation.w = 1.0;
            m.scale.x = std::max(0.2, t.size);
            m.scale.y = std::max(0.2, t.size);
            m.scale.z = 0.2;
            m.color.a = 0.8f;
            if (t.is_static)
            {
                m.color.r = 0.2f;
                m.color.g = 0.6f;
                m.color.b = 1.0f;  // blue: static
            }
            else
            {
                m.color.r = 1.0f;
                m.color.g = 0.2f;
                m.color.b = 0.2f;  // red: dynamic opponent
            }
            ma.markers.push_back(m);
        }
        // avoidance line (green): the committed spline overtaking path when active
        if (publishing && !dec.wpnts.wpnts.empty())
        {
            visualization_msgs::msg::Marker line;
            line.header.frame_id = map_frame_;
            line.header.stamp = msg->header.stamp;
            line.ns = "avoidance_line";
            line.id = 0;
            line.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line.action = visualization_msgs::msg::Marker::ADD;
            line.scale.x = 0.08;
            line.color.a = 0.9f;
            line.color.r = 0.1f;
            line.color.g = 1.0f;
            line.color.b = 0.2f;  // green: overtaking path
            line.pose.orientation.w = 1.0;
            for (const auto &w : dec.wpnts.wpnts)
            {
                geometry_msgs::msg::Point pt;
                pt.x = w.x_m;
                pt.y = w.y_m;
                pt.z = 0.05;
                line.points.push_back(pt);
            }
            ma.markers.push_back(line);
        }
        markers_pub_->publish(ma);
    }
}

}  // namespace opponent_detector

// ------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<opponent_detector::OpponentDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
