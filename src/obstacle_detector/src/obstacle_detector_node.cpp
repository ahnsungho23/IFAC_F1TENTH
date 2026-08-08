// ================================================================================================
// OBSTACLE DETECTOR NODE implementation (layered LiDAR obstacle detection)
// ================================================================================================

#include "obstacle_detector/obstacle_detector_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <builtin_interfaces/msg/time.hpp>
#include <f110_msgs/msg/obstacle.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>

#include "obstacle_detector/aabb_frenet_projector.hpp"
#include "obstacle_detector/frenet_marker_builder.hpp"

namespace obstacle_detector
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

ObstacleDetectorNode::ObstacleDetectorNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("obstacle_detector", options)
{
    declareParameters();
    loadParameters();

    tracker_.configure(tracker_params_, &frenet_);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // latched (transient_local) QoS for the raceline and the map
    auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ObstacleDetectorNode::scanCallback, this, std::placeholders::_1));
    global_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
        global_wpnts_topic_, latched_qos,
        std::bind(&ObstacleDetectorNode::globalWpntsCallback, this, std::placeholders::_1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, latched_qos,
        std::bind(&ObstacleDetectorNode::mapCallback, this, std::placeholders::_1));
    ego_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        ego_odom_topic_, rclcpp::QoS(10),
        std::bind(&ObstacleDetectorNode::egoOdomCallback, this, std::placeholders::_1));

    static_obs_pub_ = this->create_publisher<f110_msgs::msg::ObstacleArray>(static_obs_topic_, 10);
    confirmed_static_obs_pub_ =
        this->create_publisher<f110_msgs::msg::ObstacleArray>(confirmed_static_obs_topic_, 10);
    opp_obs_pub_ = this->create_publisher<f110_msgs::msg::ObstacleArray>(opp_obs_topic_, 10);
    if (publish_markers_)
    {
        static_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            static_markers_topic_, 10);
        opp_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            opp_markers_topic_, 10);
    }

    RCLCPP_INFO(
        this->get_logger(),
        "obstacle_detector started (scan=%s, global=%s, map_filter=%s, "
        "static_obs=%s, confirmed_static_obs=%s, opp_obs=%s, "
        "cluster_merge=%s[dist=%.2f, min_frag=%d], "
        "layer_merge=%s[gap_s=%.2f, gap_d=%.2f], mahalanobis=%s[gate=%.2f], "
        "motion_chi2[static<%.2f dynamic>%.2f], diagnostics=%s[period=%.2fs])",
        scan_topic_.c_str(), global_wpnts_topic_.c_str(), use_map_filter_ ? "on" : "off",
        static_obs_topic_.c_str(), confirmed_static_obs_topic_.c_str(), opp_obs_topic_.c_str(),
        cluster_merge_enable_ ? "on" : "off", cluster_merge_distance_,
        cluster_merge_min_fragment_points_, layer_merge_enable_ ? "on" : "off",
        layer_merge_gap_s_, layer_merge_gap_d_,
        tracker_params_.assoc_use_mahalanobis ? "on" : "off",
        tracker_params_.assoc_mahalanobis_gate, tracker_params_.static_chi2_threshold,
        tracker_params_.dynamic_chi2_threshold,
        diagnostics_enable_ ? "on" : "off", diagnostics_period_sec_);
}

// ------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::declareParameters()
{
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("ego_odom_topic", "/pf/pose/odom");
    this->declare_parameter<std::string>("static_obs_topic", "/static_obs");
    this->declare_parameter<std::string>(
        "confirmed_static_obs_topic", "/confirmed_static_obs");
    this->declare_parameter<std::string>("opp_obs_topic", "/opp_obs");
    this->declare_parameter<std::string>("static_markers_topic", "/static_obs/markers");
    this->declare_parameter<std::string>("opp_markers_topic", "/opp_obs/markers");
    this->declare_parameter<std::string>("map_frame", "map");

    this->declare_parameter<double>("max_range", 14.0);

    // clustering
    this->declare_parameter<double>("lambda_deg", 10.0);
    this->declare_parameter<double>("cluster_sigma", 0.03);
    this->declare_parameter<double>("min_2_points_dist", 0.01);
    this->declare_parameter<int>("min_cluster_points", 5);
    // must exceed the diagonal of the largest car / static obstacle: a 0.5x0.5 m box is 0.707 m
    this->declare_parameter<double>("max_obs_size", 0.8);
    // Rejoin close fragments before CLCS projection/tracking. Final clusters must still satisfy
    // min_cluster_points, and their merged AABB must stay within max_obs_size.
    this->declare_parameter<bool>("cluster_merge_enable", true);
    this->declare_parameter<double>("cluster_merge_distance", 0.12);
    this->declare_parameter<int>("cluster_merge_min_fragment_points", 2);

    // Detection-specific Kalman measurement covariance.
    this->declare_parameter<double>("meas_range_var_scale", 2.0);
    this->declare_parameter<double>("meas_sparse_var_scale", 1.5);
    this->declare_parameter<double>("meas_yaw_rate_var_scale", 0.25);
    this->declare_parameter<int>("meas_reference_points", 8);
    this->declare_parameter<double>("meas_variance_scale_max", 10.0);
    this->declare_parameter<double>("meas_motion_timeout", 0.1);

    // Layer-1 filtering
    this->declare_parameter<double>("max_viewing_distance", 13.0);
    this->declare_parameter<double>("view_behind_distance", 1.0);
    this->declare_parameter<double>("boundaries_inflation", 0.1);
    this->declare_parameter<double>("fallback_track_halfwidth", 1.5);
    this->declare_parameter<bool>("use_map_filter", true);
    this->declare_parameter<int>("map_occupied_thresh", 50);
    this->declare_parameter<int>("map_inflation_cells", 1);
    this->declare_parameter<double>("map_point_reject_ratio", 0.6);

    // per-layer 2nd-stage merge (object-level output)
    this->declare_parameter<bool>("layer_merge_enable", true);
    this->declare_parameter<double>("layer_merge_gap_s", 0.4);
    this->declare_parameter<double>("layer_merge_gap_d", 0.3);

    // output
    this->declare_parameter<bool>("publish_markers", true);
    this->declare_parameter<bool>("diagnostics_enable", true);
    this->declare_parameter<double>("diagnostics_period_sec", 1.0);

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
    this->declare_parameter<int>("ttl_static", 25);
    this->declare_parameter<int>("min_hits_confirm", 3);
    this->declare_parameter<int>("confirmation_window", 5);
    this->declare_parameter<double>(
        "motion_classification.dynamic_chi2_threshold", 9.21);
    this->declare_parameter<double>(
        "motion_classification.static_chi2_threshold", 5.99);
    this->declare_parameter<int>("motion_classification.dynamic_vote_window", 5);
    this->declare_parameter<int>("motion_classification.dynamic_vote_required", 3);
    this->declare_parameter<int>("motion_classification.static_vote_window", 15);
    this->declare_parameter<int>("motion_classification.static_vote_required", 10);
    this->declare_parameter<int>("motion_classification.position_history_size", 15);
    this->declare_parameter<int>("motion_classification.static_min_observations", 10);
    this->declare_parameter<double>(
        "motion_classification.static_max_position_rms", 0.10);
    this->declare_parameter<int>(
        "motion_classification.dynamic_to_static_min_observations", 20);
    this->declare_parameter<int>(
        "motion_classification.dynamic_to_static_vote_required", 15);
    this->declare_parameter<double>(
        "motion_classification.dynamic_to_static_max_position_rms", 0.08);
    this->declare_parameter<double>(
        "motion_classification.covariance_regularization_epsilon", 1.0e-6);
    this->declare_parameter<double>(
        "motion_classification.minimum_velocity_covariance", 1.0e-5);
    this->declare_parameter<double>(
        "motion_classification.static_score_forgetting_factor", 0.95);
    this->declare_parameter<bool>("motion_classification.debug_enable", false);
    this->declare_parameter<double>("motion_classification.debug_period_sec", 1.0);
    this->declare_parameter<double>("dt_max", 0.5);
    this->declare_parameter<double>("extent_shrink_alpha", 0.25);
    this->declare_parameter<double>("envelope_stability_tolerance_m", 0.10);
    this->declare_parameter<int>("envelope_stability_frames", 2);
}

void ObstacleDetectorNode::loadParameters()
{
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    global_wpnts_topic_ = this->get_parameter("global_waypoints_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    ego_odom_topic_ = this->get_parameter("ego_odom_topic").as_string();
    static_obs_topic_ = this->get_parameter("static_obs_topic").as_string();
    confirmed_static_obs_topic_ =
        this->get_parameter("confirmed_static_obs_topic").as_string();
    opp_obs_topic_ = this->get_parameter("opp_obs_topic").as_string();
    static_markers_topic_ = this->get_parameter("static_markers_topic").as_string();
    opp_markers_topic_ = this->get_parameter("opp_markers_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    max_range_ = this->get_parameter("max_range").as_double();

    lambda_rad_ = this->get_parameter("lambda_deg").as_double() * M_PI / 180.0;
    cluster_sigma_ = this->get_parameter("cluster_sigma").as_double();
    min_2_points_dist_ = this->get_parameter("min_2_points_dist").as_double();
    min_cluster_points_ = this->get_parameter("min_cluster_points").as_int();
    max_obs_size_ = this->get_parameter("max_obs_size").as_double();
    cluster_merge_enable_ = this->get_parameter("cluster_merge_enable").as_bool();
    cluster_merge_distance_ =
        std::max(0.0, this->get_parameter("cluster_merge_distance").as_double());
    cluster_merge_min_fragment_points_ =
        std::clamp(static_cast<int>(
                       this->get_parameter("cluster_merge_min_fragment_points").as_int()),
                   1, std::max(1, min_cluster_points_));
    meas_range_var_scale_ =
        std::max(0.0, this->get_parameter("meas_range_var_scale").as_double());
    meas_sparse_var_scale_ =
        std::max(0.0, this->get_parameter("meas_sparse_var_scale").as_double());
    meas_yaw_rate_var_scale_ =
        std::max(0.0, this->get_parameter("meas_yaw_rate_var_scale").as_double());
    meas_reference_points_ =
        std::max(1, static_cast<int>(this->get_parameter("meas_reference_points").as_int()));
    meas_variance_scale_max_ =
        std::max(1.0, this->get_parameter("meas_variance_scale_max").as_double());
    meas_motion_timeout_ =
        std::max(0.0, this->get_parameter("meas_motion_timeout").as_double());

    max_viewing_distance_ = this->get_parameter("max_viewing_distance").as_double();
    view_behind_distance_ = this->get_parameter("view_behind_distance").as_double();
    boundaries_inflation_ = this->get_parameter("boundaries_inflation").as_double();
    fallback_track_halfwidth_ = this->get_parameter("fallback_track_halfwidth").as_double();
    use_map_filter_ = this->get_parameter("use_map_filter").as_bool();
    map_occupied_thresh_ = this->get_parameter("map_occupied_thresh").as_int();
    map_inflation_cells_ = this->get_parameter("map_inflation_cells").as_int();
    map_point_reject_ratio_ = this->get_parameter("map_point_reject_ratio").as_double();

    layer_merge_enable_ = this->get_parameter("layer_merge_enable").as_bool();
    layer_merge_gap_s_ = this->get_parameter("layer_merge_gap_s").as_double();
    layer_merge_gap_d_ = this->get_parameter("layer_merge_gap_d").as_double();

    publish_markers_ = this->get_parameter("publish_markers").as_bool();
    diagnostics_enable_ = this->get_parameter("diagnostics_enable").as_bool();
    diagnostics_period_sec_ =
        std::max(0.1, this->get_parameter("diagnostics_period_sec").as_double());

    tracker_params_.meas_var_s = this->get_parameter("meas_var_s").as_double();
    tracker_params_.meas_var_d = this->get_parameter("meas_var_d").as_double();
    tracker_params_.process_var_vs = this->get_parameter("process_var_vs").as_double();
    tracker_params_.process_var_vd = this->get_parameter("process_var_vd").as_double();
    tracker_params_.assoc_gate = this->get_parameter("assoc_gate").as_double();
    tracker_params_.aggro_multi = this->get_parameter("aggro_multi").as_double();
    tracker_params_.assoc_use_mahalanobis =
        this->get_parameter("assoc_use_mahalanobis").as_bool();
    tracker_params_.assoc_mahalanobis_gate =
        std::max(0.0, this->get_parameter("assoc_mahalanobis_gate").as_double());
    tracker_params_.ttl_dynamic = this->get_parameter("ttl_dynamic").as_int();
    tracker_params_.ttl_static = this->get_parameter("ttl_static").as_int();
    tracker_params_.min_hits_confirm = this->get_parameter("min_hits_confirm").as_int();
    tracker_params_.confirmation_window =
        this->get_parameter("confirmation_window").as_int();
    tracker_params_.dynamic_chi2_threshold =
        this->get_parameter(
        "motion_classification.dynamic_chi2_threshold").as_double();
    tracker_params_.static_chi2_threshold =
        this->get_parameter(
        "motion_classification.static_chi2_threshold").as_double();
    tracker_params_.dynamic_vote_window =
        this->get_parameter("motion_classification.dynamic_vote_window").as_int();
    tracker_params_.dynamic_vote_required =
        this->get_parameter("motion_classification.dynamic_vote_required").as_int();
    tracker_params_.static_vote_window =
        this->get_parameter("motion_classification.static_vote_window").as_int();
    tracker_params_.static_vote_required =
        this->get_parameter("motion_classification.static_vote_required").as_int();
    tracker_params_.position_history_size =
        this->get_parameter("motion_classification.position_history_size").as_int();
    tracker_params_.static_min_observations =
        this->get_parameter("motion_classification.static_min_observations").as_int();
    tracker_params_.static_max_position_rms =
        this->get_parameter("motion_classification.static_max_position_rms").as_double();
    tracker_params_.dynamic_to_static_min_observations =
        this->get_parameter(
        "motion_classification.dynamic_to_static_min_observations").as_int();
    tracker_params_.dynamic_to_static_vote_required =
        this->get_parameter(
        "motion_classification.dynamic_to_static_vote_required").as_int();
    tracker_params_.dynamic_to_static_max_position_rms =
        this->get_parameter(
        "motion_classification.dynamic_to_static_max_position_rms").as_double();
    tracker_params_.covariance_regularization_epsilon =
        this->get_parameter(
        "motion_classification.covariance_regularization_epsilon").as_double();
    tracker_params_.minimum_velocity_covariance =
        this->get_parameter(
        "motion_classification.minimum_velocity_covariance").as_double();
    tracker_params_.static_score_forgetting_factor =
        this->get_parameter(
        "motion_classification.static_score_forgetting_factor").as_double();
    motion_debug_enable_ =
        this->get_parameter("motion_classification.debug_enable").as_bool();
    motion_debug_period_sec_ =
        this->get_parameter("motion_classification.debug_period_sec").as_double();
    if (!(motion_debug_period_sec_ > 0.0))
    {
        throw std::invalid_argument(
            "motion_classification.debug_period_sec must be positive");
    }
    tracker_params_.dt_max = this->get_parameter("dt_max").as_double();
    tracker_params_.extent_shrink_alpha =
        std::clamp(this->get_parameter("extent_shrink_alpha").as_double(), 0.0, 1.0);
    tracker_params_.envelope_stability_tolerance_m =
        std::max(0.0, this->get_parameter("envelope_stability_tolerance_m").as_double());
    tracker_params_.envelope_stability_frames =
        std::max(0, static_cast<int>(
            this->get_parameter("envelope_stability_frames").as_int()));

}

// ------------------------------------------------------------------------------------------------
// Input callbacks (raceline / map / ego pose)
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
    if (msg->wpnts.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Received /global_waypoints with < 3 points; CLCS needs >= 3. Ignoring.");
        return;
    }

    // Lightweight helper for track boundaries, s-wrap, and final Frenet-marker interpolation.
    std::vector<FrenetProjector::Waypoint> wpnts;
    std::vector<global_planning::ReferenceWaypoint> ref;
    wpnts.reserve(msg->wpnts.size());
    ref.reserve(msg->wpnts.size());
    for (const auto &w : msg->wpnts)
    {
        FrenetProjector::Waypoint fw;
        fw.x = w.x_m;
        fw.y = w.y_m;
        fw.s = w.s_m;
        fw.d_left = w.d_left;
        fw.d_right = w.d_right;
        wpnts.push_back(fw);

        global_planning::ReferenceWaypoint rw;
        rw.x = w.x_m;
        rw.y = w.y_m;
        rw.s = w.s_m;
        ref.push_back(rw);
    }
    // CLCS converter: the accurate (x,y) -> (s,d) projection used for clusters and ego.
    try
    {
        global_planning::ClcsFrenetConfig cfg;  // closed_loop=true + sane projection-domain defaults
        auto conv = global_planning::ClcsFrenetConverter::create(ref, cfg, ++clcs_version_);
        FrenetProjector next_frenet;
        next_frenet.build(std::move(wpnts), true, conv->stats().track_length);
        frenet_ = std::move(next_frenet);
        converter_ = conv;
        tracker_.clear();  // old tracks live in the previous reference's s-domain
        ego_s_ = -1.0;  // wait for an odometry sample projected against the new CLCS reference
        ego_s_stamp_ = -1.0;
        RCLCPP_INFO_ONCE(this->get_logger(),
                         "CLCS converter built from %zu waypoints (track length %.2f m).",
                         ref.size(), conv->stats().track_length);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(),
                     "CLCS converter build failed (keeping previous, if any): %s", e.what());
    }
}

void ObstacleDetectorNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    map_msg_ = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Occupancy map received (%u x %u @ %.3f m).",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void ObstacleDetectorNode::egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    const double yaw_rate = msg->twist.twist.angular.z;
    if (std::isfinite(yaw_rate))
    {
        odom_yaw_rate_ = yaw_rate;
        odom_motion_stamp_ = stampToSec(msg->header.stamp);
    }
    else
    {
        odom_motion_stamp_ = -1.0;
    }

    if (!converter_)
    {
        return;
    }
    global_planning::ClcsConversionInput ci;
    ci.x = msg->pose.pose.position.x;
    ci.y = msg->pose.pose.position.y;
    const auto cr = converter_->convert(ci);
    if (cr.valid)
    {
        ego_s_ = cr.s;
        ego_s_stamp_ = stampToSec(msg->header.stamp);
    }
}

// ------------------------------------------------------------------------------------------------
// TF: scan frame -> map frame
// ------------------------------------------------------------------------------------------------
bool ObstacleDetectorNode::lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx,
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
// Adaptive-breakpoint clustering (points already in map frame)
// ------------------------------------------------------------------------------------------------
std::vector<std::vector<ObstacleDetectorNode::ScanPoint>>
ObstacleDetectorNode::clusterScan(const sensor_msgs::msg::LaserScan &scan, double tx, double ty,
                                  double yaw, ScanProcessingStats &stats) const
{
    std::vector<std::vector<ScanPoint>> clusters;
    std::vector<ScanPoint> current;
    const double dphi = scan.angle_increment;
    const double denom = std::sin(lambda_rad_ - dphi);
    const double cyaw = std::cos(yaw);
    const double syaw = std::sin(yaw);

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

    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
        const double r = scan.ranges[i];
        ++stats.total_beams;
        if (!std::isfinite(r))
        {
            ++stats.nonfinite_rejected;
            continue;  // invalid beam breaks contiguity (handled by index gap below)
        }
        if (r < scan.range_min)
        {
            ++stats.below_min_range_rejected;
            continue;
        }
        if (r >= max_range_)
        {
            ++stats.at_or_above_max_range_rejected;
            continue;  // invalid beam breaks contiguity (handled by index gap below)
        }
        ++stats.valid_beams;
        const double ang = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
        const double lx = r * std::cos(ang);
        const double ly = r * std::sin(ang);
        ScanPoint pt;
        pt.x = tx + cyaw * lx - syaw * ly;
        pt.y = ty + syaw * lx + cyaw * ly;
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

// ------------------------------------------------------------------------------------------------
// Pre-tracking fragment merge: scan clusters -> complete detections
// ------------------------------------------------------------------------------------------------
std::vector<std::vector<ObstacleDetectorNode::ScanPoint>>
ObstacleDetectorNode::mergeClusters(std::vector<std::vector<ScanPoint>> clusters,
                                    ScanProcessingStats &stats) const
{
    using Bounds = std::array<double, 4>;  // min_x, max_x, min_y, max_y

    const auto bounds = [](const std::vector<ScanPoint> &cluster) {
        Bounds result{
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

    const double merge_distance_sq = cluster_merge_distance_ * cluster_merge_distance_;
    const auto hasClosePointPair =
        [merge_distance_sq](const std::vector<ScanPoint> &a,
                            const std::vector<ScanPoint> &b) {
            for (const auto &pa : a)
            {
                for (const auto &pb : b)
                {
                    const double dx = pa.x - pb.x;
                    const double dy = pa.y - pb.y;
                    if (dx * dx + dy * dy <= merge_distance_sq)
                    {
                        return true;
                    }
                }
            }
            return false;
        };

    if (cluster_merge_enable_ && clusters.size() > 1)
    {
        // Cache bounds once and update only the accepted component. The loop intentionally permits
        // multiple fragments of one surface to join, while max_obs_size bounds any transitive
        // chain.
        std::vector<Bounds> cluster_bounds;
        cluster_bounds.reserve(clusters.size());
        for (const auto &cluster : clusters)
        {
            cluster_bounds.push_back(bounds(cluster));
        }

        bool changed = true;
        while (changed)
        {
            changed = false;
            for (std::size_t i = 0; i < clusters.size() && !changed; ++i)
            {
                const Bounds &a = cluster_bounds[i];
                for (std::size_t j = i + 1; j < clusters.size(); ++j)
                {
                    const Bounds &b = cluster_bounds[j];
                    const double gap_x =
                        std::max({0.0, a[0] - b[1], b[0] - a[1]});
                    const double gap_y =
                        std::max({0.0, a[2] - b[3], b[2] - a[3]});
                    if (gap_x * gap_x + gap_y * gap_y > merge_distance_sq)
                    {
                        continue;
                    }
                    // Overlapping/nearby axis-aligned boxes are only a broad-phase candidate.
                    // Requiring a close real point pair prevents diagonal/concave AABB overlap
                    // from merging physically separated fragments.
                    if (!hasClosePointPair(clusters[i], clusters[j]))
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

                    clusters[i].reserve(clusters[i].size() + clusters[j].size());
                    clusters[i].insert(clusters[i].end(), clusters[j].begin(), clusters[j].end());
                    cluster_bounds[i] =
                        Bounds{merged_min_x, merged_max_x, merged_min_y, merged_max_y};
                    clusters.erase(clusters.begin() + static_cast<std::ptrdiff_t>(j));
                    cluster_bounds.erase(
                        cluster_bounds.begin() + static_cast<std::ptrdiff_t>(j));
                    changed = true;
                    break;
                }
            }
        }
    }

    // Small fragments exist only as merge candidates. They never become detections alone.
    const std::size_t clusters_before_final_filter = clusters.size();
    clusters.erase(
        std::remove_if(
            clusters.begin(), clusters.end(),
            [this](const auto &cluster) {
                return static_cast<int>(cluster.size()) < min_cluster_points_;
            }),
        clusters.end());
    stats.fragments_rejected = clusters_before_final_filter - clusters.size();
    stats.clusters_after_merge = clusters.size();
    return clusters;
}

// ------------------------------------------------------------------------------------------------
// Occupancy-grid lookup for the Layer-1 map filter
// ------------------------------------------------------------------------------------------------
bool ObstacleDetectorNode::occupiedInMap(double x, double y) const
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

// ------------------------------------------------------------------------------------------------
// Per-layer 2nd-stage clustering: tracks of ONE layer -> object-level obstacles
// ------------------------------------------------------------------------------------------------
std::vector<ObstacleDetectorNode::MergedObstacle>
ObstacleDetectorNode::mergeLayer(const std::vector<const Track *> &members,
                                 bool is_static_layer) const
{
    std::vector<MergedObstacle> out;
    const std::size_t n = members.size();
    if (n == 0)
    {
        return out;
    }
    out.reserve(n);

    // Union-find: link two track boxes when BOTH Frenet edge-to-edge gaps are within the merge
    // gaps. Each box retains the independently projected AABB extents instead of expanding the
    // Cartesian diagonal equally along s and d.
    std::vector<std::size_t> parent(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        parent[i] = i;
    }
    auto find = [&parent](std::size_t i) {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];  // path halving
            i = parent[i];
        }
        return i;
    };
    if (layer_merge_enable_)
    {
        for (std::size_t i = 0; i < n; ++i)
        {
            for (std::size_t j = i + 1; j < n; ++j)
            {
                const Track &a = *members[i];
                const Track &b = *members[j];
                const double gap_s = std::max(
                    0.0,
                    std::abs(frenet_.wrapDelta(a.s(), b.s())) -
                    a.s_half_extent - b.s_half_extent);
                const double a_right = a.d() + a.d_right_offset;
                const double a_left = a.d() + a.d_left_offset;
                const double b_right = b.d() + b.d_right_offset;
                const double b_left = b.d() + b.d_left_offset;
                const double gap_d = std::max(
                    {0.0, a_right - b_left, b_right - a_left});
                if (gap_s <= layer_merge_gap_s_ && gap_d <= layer_merge_gap_d_)
                {
                    parent[find(i)] = find(j);
                }
            }
        }
    }

    std::vector<std::vector<const Track *>> comps(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        comps[find(i)].push_back(members[i]);
    }

    const double L = frenet_.raceline_length();
    auto wrapS = [L](double s) {
        if (L <= 0.0)
        {
            return s;
        }
        s = std::fmod(s, L);
        return s < 0.0 ? s + L : s;
    };

    for (const auto &comp : comps)
    {
        if (comp.empty())
        {
            continue;
        }
        // envelope in s relative to the first member (wrap-safe: all members sit within the
        // viewing window, far from half a lap apart), envelope in d directly
        const double s0 = comp.front()->s();
        double lo = std::numeric_limits<double>::max();
        double hi = std::numeric_limits<double>::lowest();
        double d_left = std::numeric_limits<double>::lowest();
        double d_right = std::numeric_limits<double>::max();
        double w_sum = 0.0;
        double vs = 0.0;
        double vd = 0.0;
        double s_var = 0.0;
        double vs_var = 0.0;
        double d_var = 0.0;
        double vd_var = 0.0;
        int id = std::numeric_limits<int>::max();
        bool visible = false;
        bool has_cartesian = false;
        double x_min = std::numeric_limits<double>::max();
        double x_max = std::numeric_limits<double>::lowest();
        double y_min = std::numeric_limits<double>::max();
        double y_max = std::numeric_limits<double>::lowest();
        for (const Track *t : comp)
        {
            const double rel = frenet_.wrapDelta(t->s(), s0);
            lo = std::min(lo, rel - t->s_half_extent);
            hi = std::max(hi, rel + t->s_half_extent);
            d_left = std::max(d_left, t->d() + t->d_left_offset);
            d_right = std::min(d_right, t->d() + t->d_right_offset);
            const double w = std::max(t->size, 1e-3);  // size-weighted merged velocity
            w_sum += w;
            vs += w * t->vs();
            vd += w * t->vd();
            s_var = std::max(s_var, t->P(0, 0));  // conservative: worst member uncertainty
            vs_var = std::max(vs_var, t->P(1, 1));
            d_var = std::max(d_var, t->P(2, 2));
            vd_var = std::max(vd_var, t->P(3, 3));
            id = std::min(id, t->id);  // oldest member id -> stable across frames
            visible = visible || t->is_visible;

            // Frenet prediction does not move the last raw Cartesian scan box. Only a track
            // measured in this scan contributes Cartesian geometry, preventing a dynamic track
            // retained by TTL from publishing a stale map-frame collision footprint.
            const bool valid_aabb =
                t->is_visible &&
                std::isfinite(t->x_min_map) && std::isfinite(t->x_max_map) &&
                std::isfinite(t->y_min_map) && std::isfinite(t->y_max_map) &&
                t->x_min_map <= t->x_max_map && t->y_min_map <= t->y_max_map;
            if (valid_aabb)
            {
                has_cartesian = true;
                x_min = std::min(x_min, t->x_min_map);
                x_max = std::max(x_max, t->x_max_map);
                y_min = std::min(y_min, t->y_min_map);
                y_max = std::max(y_max, t->y_max_map);
            }
        }

        MergedObstacle m;
        m.ob.id = id;
        m.ob.s_start = wrapS(s0 + lo);
        m.ob.s_end = wrapS(s0 + hi);
        m.ob.s_center = wrapS(s0 + 0.5 * (lo + hi));
        m.ob.d_left = d_left;
        m.ob.d_right = d_right;
        m.ob.d_center = 0.5 * (d_left + d_right);
        m.ob.size = std::hypot(hi - lo, d_left - d_right);
        m.ob.vs = vs / w_sum;
        m.ob.vd = vd / w_sum;
        m.ob.s_var = s_var;
        m.ob.vs_var = vs_var;
        m.ob.d_var = d_var;
        m.ob.vd_var = vd_var;
        m.ob.is_static = is_static_layer;
        m.ob.is_visible = visible;
        m.ob.is_actually_a_gap = false;
        m.ob.has_cartesian = has_cartesian;
        if (has_cartesian)
        {
            const double width = x_max - x_min;
            const double height = y_max - y_min;
            m.ob.x_min = x_min;
            m.ob.x_max = x_max;
            m.ob.y_min = y_min;
            m.ob.y_max = y_max;
            m.ob.x_center = 0.5 * (x_min + x_max);
            m.ob.y_center = 0.5 * (y_min + y_max);
            m.ob.radius = 0.5 * std::hypot(width, height);
            // A full Frenet-to-Cartesian covariance rotation needs the local CLCS tangent.
            // Until then, use the worst positional variance on both Cartesian axes.
            m.ob.x_var = std::max(s_var, d_var);
            m.ob.y_var = m.ob.x_var;

            // The current visible Cartesian union is the authoritative footprint. Reproject it
            // once after layer merging so /static_obs and /opp_obs expose one geometry contract:
            // their Frenet bounds describe the same AABB shown by the RViz marker.
            const auto projected = projectCartesianAabb(
                *converter_, x_min, x_max, y_min, y_max);
            if (projected.has_value())
            {
                m.ob.s_start = projected->s_start;
                m.ob.s_end = projected->s_end;
                m.ob.s_center = projected->s_center;
                m.ob.d_right = projected->d_right;
                m.ob.d_left = projected->d_left;
                m.ob.d_center = projected->d_center;
                m.ob.size = projected->diagonal;
            }
        }
        // A component with no currently-measured member is pure prediction. Fan-shaped scatter
        // merging into one ghost blob can grow its envelope past the physical object gate until
        // it spans the corridor and false-blocks planners; drop those oversized ghost blobs.
        if (!visible && m.ob.size > max_obs_size_)
        {
            continue;
        }
        out.push_back(m);
    }
    return out;
}

int ObstacleDetectorNode::selectOpponent(const std::vector<MergedObstacle> &dynamic_objs) const
{
    int best = -1;
    double best_key = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < dynamic_objs.size(); ++i)
    {
        const auto &ob = dynamic_objs[i].ob;
        double key;
        if (ego_s_ >= 0.0)
        {
            double ahead = frenet_.wrapDelta(ob.s_center, ego_s_);
            if (ahead < 0.0)
            {
                ahead += frenet_.raceline_length();  // wrap-aware forward distance from the ego
            }
            // Only the forward half-lap counts as "ahead": the closed-track wrap would otherwise
            // rank an opponent just behind the ego as L - eps "ahead".
            if (ahead <= 0.0 || ahead >= 0.5 * frenet_.raceline_length())
            {
                continue;
            }
            key = ahead;
        }
        else
        {
            key = ob.s_var + ob.d_var;  // lowest positional uncertainty
        }
        if (key < best_key)
        {
            best_key = key;
            best = static_cast<int>(i);
        }
    }
    return best;
}

void ObstacleDetectorNode::logMotionDebug()
{
    if (!motion_debug_enable_)
    {
        return;
    }
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3);
    bool has_confirmed_track = false;
    for (const Track &track : tracker_.tracks())
    {
        if (track.track_status != TrackStatus::Confirmed)
        {
            continue;
        }
        has_confirmed_track = true;
        stream << "\nID=" << track.id
               << " " << trackStatusName(track.track_status)
               << "/" << motionStatusName(track.motion_status)
               << " v_map=(" << track.mapVx() << "," << track.mapVy() << ")"
               << " Tv=" << track.velocity_statistic
               << " votes(S/D)=" << track.static_vote_count
               << "/" << track.dynamic_vote_count
               << " RMS=" << track.map_position_rms
               << " pos_n=" << track.map_position_history.size()
               << " static_conf=" << track.static_confidence
               << " since_meas=" << track.time_since_last_measurement << "s";
    }
    if (!has_confirmed_track)
    {
        stream << "\n(no confirmed tracks)";
    }
    const int throttle_ms =
        std::max(1, static_cast<int>(std::lround(1000.0 * motion_debug_period_sec_)));
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), throttle_ms,
        "MOTION DEBUG%s", stream.str().c_str());
}

// ------------------------------------------------------------------------------------------------
// Passive diagnostics. Event counters are accumulated; live track/classification counts remain a
// current snapshot from the most recent tracker update.
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::updateDiagnostics(const ScanProcessingStats &scan_stats,
                                             const TrackerUpdateStats *tracker_stats,
                                             double measurement_yaw_rate,
                                             bool yaw_rate_fresh)
{
    if (!diagnostics_enable_)
    {
        return;
    }

    auto &total = diagnostics_scan_totals_;
    total.scans_received += scan_stats.scans_received;
    total.scans_processed += scan_stats.scans_processed;
    total.clcs_unavailable += scan_stats.clcs_unavailable;
    total.tf_unavailable += scan_stats.tf_unavailable;
    total.total_beams += scan_stats.total_beams;
    total.valid_beams += scan_stats.valid_beams;
    total.nonfinite_rejected += scan_stats.nonfinite_rejected;
    total.below_min_range_rejected += scan_stats.below_min_range_rejected;
    total.at_or_above_max_range_rejected +=
        scan_stats.at_or_above_max_range_rejected;
    total.clusters_before_merge += scan_stats.clusters_before_merge;
    total.clusters_after_merge += scan_stats.clusters_after_merge;
    total.fragments_rejected += scan_stats.fragments_rejected;
    total.size_rejected += scan_stats.size_rejected;
    total.projection_rejected += scan_stats.projection_rejected;
    total.viewing_window_rejected += scan_stats.viewing_window_rejected;
    total.track_boundary_rejected += scan_stats.track_boundary_rejected;
    total.map_rejected += scan_stats.map_rejected;
    total.detections += scan_stats.detections;

    if (tracker_stats != nullptr)
    {
        auto &events = diagnostics_tracker_event_totals_;
        events.candidate_pairs += tracker_stats->candidate_pairs;
        events.matched += tracker_stats->matched;
        events.spawned += tracker_stats->spawned;
        events.retired += tracker_stats->retired;
        events.euclidean_pair_rejected += tracker_stats->euclidean_pair_rejected;
        events.mahalanobis_pair_rejected += tracker_stats->mahalanobis_pair_rejected;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!diagnostics_window_started_)
    {
        diagnostics_window_start_ = now;
        diagnostics_window_started_ = true;
        return;
    }
    const double elapsed =
        std::chrono::duration<double>(now - diagnostics_window_start_).count();
    if (elapsed < diagnostics_period_sec_)
    {
        return;
    }

    const auto &snapshot = tracker_.lastStats();
    const auto &events = diagnostics_tracker_event_totals_;
    RCLCPP_INFO(
        this->get_logger(),
        "DIAG perception [%.2fs scans=%zu/%zu drop(clcs=%zu tf=%zu)] "
        "beam(valid=%zu/%zu nonfinite=%zu below_min=%zu at_or_above_max=%zu) "
        "cluster=%zu->%zu fragment_drop=%zu detections=%zu "
        "reject(size=%zu clcs_projection=%zu view=%zu boundary=%zu map=%zu) "
        "track(total=%zu visible=%zu raw=%zu tentative=%zu "
        "unknown=%zu static=%zu dynamic=%zu predicted=%zu invalid_Pv=%zu) "
        "assoc(pairs=%zu match=%zu spawn=%zu retire=%zu euclid_reject=%zu maha_reject=%zu) "
        "motion(yaw_used=%.3f fresh=%s)",
        elapsed, total.scans_processed, total.scans_received, total.clcs_unavailable,
        total.tf_unavailable, total.valid_beams, total.total_beams,
        total.nonfinite_rejected, total.below_min_range_rejected,
        total.at_or_above_max_range_rejected, total.clusters_before_merge,
        total.clusters_after_merge, total.fragments_rejected, total.detections,
        total.size_rejected, total.projection_rejected, total.viewing_window_rejected,
        total.track_boundary_rejected, total.map_rejected, snapshot.total_tracks,
        snapshot.visible_tracks, snapshot.raw_tracks, snapshot.tentative_tracks,
        snapshot.motion_unknown, snapshot.motion_static, snapshot.motion_dynamic,
        snapshot.predicted_only, snapshot.invalid_velocity_covariance,
        events.candidate_pairs, events.matched, events.spawned,
        events.retired, events.euclidean_pair_rejected,
        events.mahalanobis_pair_rejected, measurement_yaw_rate,
        yaw_rate_fresh ? "true" : "false");

    diagnostics_scan_totals_ = ScanProcessingStats{};
    diagnostics_tracker_event_totals_ = TrackerUpdateStats{};
    diagnostics_window_start_ = now;
}

// ------------------------------------------------------------------------------------------------
// Main pipeline (scan-driven)
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    ScanProcessingStats stats;
    stats.scans_received = 1;

    if (!frenet_.ready() || !converter_)
    {
        stats.clcs_unavailable = 1;
        updateDiagnostics(stats, nullptr, 0.0, false);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "Waiting for /global_waypoints before detecting obstacles...");
        return;
    }

    double tx = 0.0;
    double ty = 0.0;
    double yaw = 0.0;
    if (!lookupScanToMap(msg->header, tx, ty, yaw))
    {
        stats.tf_unavailable = 1;
        updateDiagnostics(stats, nullptr, 0.0, false);
        return;
    }

    const double stamp = stampToSec(msg->header.stamp);
    double measurement_yaw_rate = 0.0;
    const bool yaw_rate_fresh =
        odom_motion_stamp_ >= 0.0 &&
        std::abs(stamp - odom_motion_stamp_) <= meas_motion_timeout_;
    if (yaw_rate_fresh)
    {
        measurement_yaw_rate = odom_yaw_rate_;
    }

    // ============================================================================================
    // LAYER 1 [map]: cluster the scan, then reject everything that IS the map (walls / known
    // static structure) plus everything outside the drivable corridor / viewing window. What
    // survives is a candidate obstacle NOT part of the map. The map layer is a filter only.
    // ============================================================================================
    const auto clusters = clusterScan(*msg, tx, ty, yaw, stats);
    std::vector<Detection> detections;
    detections.reserve(clusters.size());
    for (const auto &cluster : clusters)
    {
        // Map-frame axis-aligned box. Its centre and all four corners are projected together so
        // the published Frenet footprint preserves independent longitudinal/lateral extents.
        double minx = cluster.front().x;
        double maxx = cluster.front().x;
        double miny = cluster.front().y;
        double maxy = cluster.front().y;
        double range_sum = 0.0;
        for (const auto &p : cluster)
        {
            range_sum += p.range;
            minx = std::min(minx, p.x);
            maxx = std::max(maxx, p.x);
            miny = std::min(miny, p.y);
            maxy = std::max(maxy, p.y);
        }
        const double size = std::hypot(maxx - minx, maxy - miny);
        if (size > max_obs_size_)
        {
            ++stats.size_rejected;
            continue;  // too big to be an F1TENTH car / cone
        }

        const auto bounds = projectCartesianAabb(
            *converter_, minx, maxx, miny, maxy);
        if (!bounds.has_value())
        {
            ++stats.projection_rejected;
            continue;
        }
        // viewing-distance gate (ahead of ego)
        if (ego_s_ >= 0.0)
        {
            const double ds = frenet_.wrapDelta(bounds->s_center, ego_s_);
            if (ds > max_viewing_distance_ || ds < -view_behind_distance_)
            {
                ++stats.viewing_window_rejected;
                continue;
            }
        }

        // track-boundary corridor gate (falls back to a default half-width if bounds are unset).
        // Test the inflated envelope EDGES, not just the centre: wall-hugging fan scatter keeps
        // its centre inside the corridor while its AABB edge already pokes into the wall.
        double dl = 0.0;
        double dr = 0.0;
        frenet_.boundsAtS(bounds->s_center, dl, dr);
        const double left_bound = (dl > 0.05 ? dl : fallback_track_halfwidth_) - boundaries_inflation_;
        const double right_bound =
            (dr > 0.05 ? dr : fallback_track_halfwidth_) - boundaries_inflation_;
        if (bounds->d_left > left_bound || bounds->d_right < -right_bound)
        {
            ++stats.track_boundary_rejected;
            continue;
        }

        // map-based filter: drop clusters that sit on known static structure (the map itself)
        if (use_map_filter_ && map_msg_)
        {
            int occ = 0;
            bool reject_as_map = 0.0 >= map_point_reject_ratio_;
            for (const auto &p : cluster)
            {
                if (occupiedInMap(p.x, p.y))
                {
                    ++occ;
                    const double ratio =
                        static_cast<double>(occ) / static_cast<double>(cluster.size());
                    if (ratio >= map_point_reject_ratio_)
                    {
                        reject_as_map = true;
                        break;
                    }
                }
            }
            if (reject_as_map)
            {
                ++stats.map_rejected;
                continue;
            }
        }

        Detection det;
        det.s = bounds->s_center;
        det.d = bounds->d_center;
        det.s_half_extent = bounds->longitudinal_half_extent;
        det.d_right_offset = bounds->d_right - bounds->d_center;
        det.d_left_offset = bounds->d_left - bounds->d_center;
        det.size = size;
        det.x_min = minx;
        det.x_max = maxx;
        det.y_min = miny;
        det.y_max = maxy;
        const double mean_range = range_sum / static_cast<double>(cluster.size());
        const double range_ratio = max_range_ > 1e-6 ?
            std::clamp(mean_range / max_range_, 0.0, 1.0) : 0.0;
        const double reference_points = static_cast<double>(meas_reference_points_);
        const double sparse_ratio =
            std::clamp((reference_points - static_cast<double>(cluster.size())) /
                           reference_points,
                       0.0, 1.0);
        const double variance_scale =
            1.0 +
            meas_range_var_scale_ * range_ratio * range_ratio +
            meas_sparse_var_scale_ * sparse_ratio +
            meas_yaw_rate_var_scale_ * std::abs(measurement_yaw_rate);
        det.variance_scale = std::clamp(variance_scale, 1.0, meas_variance_scale_max_);
        detections.push_back(det);
        ++stats.detections;
    }

    // ---- tracking: Frenet association/output geometry plus a supplemental map-frame Kalman
    //      velocity/significance and position-persistence motion classifier ----
    tracker_.update(detections, stamp, measurement_yaw_rate, yaw_rate_fresh);
    logMotionDebug();
    stats.scans_processed = 1;
    updateDiagnostics(stats, &tracker_.lastStats(), measurement_yaw_rate, yaw_rate_fresh);

    // ============================================================================================
    // Layer assembly: gather each layer's publishable tracks, merge same-object fragments inside the
    // layer (2nd-stage clustering, mergeLayer), then publish object-level obstacles.
    // LAYER 2 [static]    -> /static_obs           : every merged static object.
    // LAYER 2 [confirmed] -> /confirmed_static_obs : confirmed merged static objects only.
    // LAYER 3 [dynamic]   -> /opp_obs              : nearest-ahead merged opponent.
    // All are published EVERY scan (empty when a layer is void) so consumers tick at scan rate.
    // ============================================================================================
    std::vector<const Track *> static_members;
    std::vector<const Track *> confirmed_static_members;
    std::vector<const Track *> dynamic_members;
    static_members.reserve(tracker_.tracks().size());
    confirmed_static_members.reserve(tracker_.tracks().size());
    dynamic_members.reserve(tracker_.tracks().size());
    for (const Track &t : tracker_.tracks())
    {
        if (t.track_status != TrackStatus::Confirmed)
        {
            continue;
        }
        if (t.motion_status == MotionStatus::Dynamic)
        {
            dynamic_members.push_back(&t);
        }
        else if (t.envelope_stable_streak >= tracker_params_.envelope_stability_frames)
        {
            // Provisional and confirmed static share the same track and ID. Promotion therefore
            // never creates a one-scan gap in /static_obs. Fan-shaped morphing clusters never
            // reach the stability streak, so they stay out of the published layer entirely.
            static_members.push_back(&t);
            if (t.motion_status == MotionStatus::Static)
            {
                confirmed_static_members.push_back(&t);
            }
        }
    }
    const auto static_objs = mergeLayer(static_members, true);
    const auto confirmed_static_objs = mergeLayer(confirmed_static_members, true);
    const auto dynamic_objs = mergeLayer(dynamic_members, false);
    const int opp = selectOpponent(dynamic_objs);
    // Layer 3 ranks opponents by forward distance from ego_s_. A stale ego odometry sample would
    // misplace that ranking, so /opp_obs is suppressed until fresh odometry arrives.
    const bool ego_s_fresh =
        ego_s_ < 0.0 ||
        (ego_s_stamp_ >= 0.0 && std::abs(stamp - ego_s_stamp_) <= meas_motion_timeout_);

    f110_msgs::msg::ObstacleArray static_arr;
    static_arr.header = msg->header;
    static_arr.header.frame_id = map_frame_;
    static_arr.obstacles.reserve(static_objs.size());
    for (const auto &m : static_objs)
    {
        static_arr.obstacles.push_back(m.ob);
    }
    static_obs_pub_->publish(static_arr);

    f110_msgs::msg::ObstacleArray confirmed_static_arr;
    confirmed_static_arr.header = static_arr.header;
    confirmed_static_arr.obstacles.reserve(confirmed_static_objs.size());
    for (const auto &m : confirmed_static_objs)
    {
        confirmed_static_arr.obstacles.push_back(m.ob);
    }
    confirmed_static_obs_pub_->publish(confirmed_static_arr);

    f110_msgs::msg::ObstacleArray opp_arr;
    opp_arr.header = msg->header;
    opp_arr.header.frame_id = map_frame_;
    if (opp >= 0)
    {
        opp_arr.obstacles.push_back(dynamic_objs[opp].ob);
    }
    if (ego_s_fresh)
    {
        opp_obs_pub_->publish(opp_arr);
    }
    else
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "Ego odometry stale beyond meas_motion_timeout; suppressing /opp_obs");
    }

    // RViz mirrors are built from the final published Frenet arrays. They therefore visualize the
    // exact s/d envelopes consumed by downstream planners, including predicted-only objects.
    if (publish_markers_ && static_markers_pub_ && opp_markers_pub_)
    {
        static_markers_pub_->publish(
            buildFrenetObstacleMarkers(
                static_arr, frenet_, "static_obs_frenet", 0.2F, 0.6F, 1.0F));
        if (ego_s_fresh)
        {
            opp_markers_pub_->publish(
                buildFrenetObstacleMarkers(
                    opp_arr, frenet_, "opp_obs_frenet", 1.0F, 0.2F, 0.2F));
        }
    }
}

}  // namespace obstacle_detector

// ------------------------------------------------------------------------------------------------
// main
// ------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_detector::ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
