// ================================================================================================
// 계층형 LiDAR 장애물 검출 노드 구현
// ================================================================================================

#include "obstacle_detector/obstacle_detector_node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

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
// ROS 메시지 시각을 추적기에서 쓰는 초 단위 실수로 변환한다.
double stampToSec(const builtin_interfaces::msg::Time &t)
{
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

// 정규화된 quaternion에서 평면 회전 yaw만 추출한다.
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

    // 늦게 참여한 노드도 마지막 raceline과 map을 받을 수 있도록 latched QoS를 사용한다.
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
        "obstacle_detector started (scan=%s, global=%s, map_filter=%s, classifier=%d, "
        "static_obs=%s, opp_obs=%s, cluster_merge=%s[dist=%.2f, min_frag=%d], "
        "layer_merge=%s[gap_s=%.2f, gap_d=%.2f], mahalanobis=%s[gate=%.2f], "
        "diagnostics=%s[period=%.2fs])",
        scan_topic_.c_str(), global_wpnts_topic_.c_str(), use_map_filter_ ? "on" : "off",
        static_cast<int>(tracker_params_.classifier_mode), static_obs_topic_.c_str(),
        opp_obs_topic_.c_str(), cluster_merge_enable_ ? "on" : "off", cluster_merge_distance_,
        cluster_merge_min_fragment_points_, layer_merge_enable_ ? "on" : "off",
        layer_merge_gap_s_, layer_merge_gap_d_,
        tracker_params_.assoc_use_mahalanobis ? "on" : "off",
        tracker_params_.assoc_mahalanobis_gate, diagnostics_enable_ ? "on" : "off",
        diagnostics_period_sec_);
}

// ------------------------------------------------------------------------------------------------
// 파라미터 선언
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::declareParameters()
{
    this->declare_parameter<std::string>("scan_topic", "/scan");
    this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    this->declare_parameter<std::string>("map_topic", "/map");
    this->declare_parameter<std::string>("ego_odom_topic", "/pf/pose/odom");
    this->declare_parameter<std::string>("static_obs_topic", "/static_obs");
    this->declare_parameter<std::string>("opp_obs_topic", "/opp_obs");
    this->declare_parameter<std::string>("static_markers_topic", "/static_obs/markers");
    this->declare_parameter<std::string>("opp_markers_topic", "/opp_obs/markers");
    this->declare_parameter<std::string>("map_frame", "map");

    this->declare_parameter<double>("max_range", 10.0);

    // 적응형 breakpoint 군집화
    this->declare_parameter<double>("lambda_deg", 10.0);
    this->declare_parameter<double>("cluster_sigma", 0.03);
    this->declare_parameter<double>("min_2_points_dist", 0.01);
    this->declare_parameter<int>("min_cluster_points", 5);
    // 가장 큰 차량/정적 장애물의 대각선보다 커야 한다. 예: 0.5 x 0.5 m 상자는 0.707 m이다.
    this->declare_parameter<double>("max_obs_size", 0.8);
    // CLCS 투영과 추적 전에 가까운 파편을 다시 연결한다. 최종 군집은 min_cluster_points를
    // 만족해야 하며 병합 AABB도 max_obs_size 안에 있어야 한다.
    this->declare_parameter<bool>("cluster_merge_enable", true);
    this->declare_parameter<double>("cluster_merge_distance", 0.12);
    this->declare_parameter<int>("cluster_merge_min_fragment_points", 2);

    // 거리·점 밀도·자차 회전에 따라 검출별로 조절하는 Kalman 측정 공분산
    this->declare_parameter<double>("meas_range_var_scale", 2.0);
    this->declare_parameter<double>("meas_sparse_var_scale", 1.5);
    this->declare_parameter<double>("meas_yaw_rate_var_scale", 0.25);
    this->declare_parameter<int>("meas_reference_points", 8);
    this->declare_parameter<double>("meas_variance_scale_max", 10.0);
    this->declare_parameter<double>("meas_motion_timeout", 0.1);

    // 계층 1 가시 거리·트랙 경계·점유 지도 필터
    this->declare_parameter<double>("max_viewing_distance", 9.0);
    this->declare_parameter<double>("view_behind_distance", 1.0);
    this->declare_parameter<double>("boundaries_inflation", 0.1);
    this->declare_parameter<double>("fallback_track_halfwidth", 1.5);
    this->declare_parameter<bool>("use_map_filter", true);
    this->declare_parameter<int>("map_occupied_thresh", 50);
    this->declare_parameter<int>("map_inflation_cells", 1);
    this->declare_parameter<double>("map_point_reject_ratio", 0.6);

    // 계층별 2차 병합: 여러 트랙 파편을 물체 단위 출력으로 묶는다.
    this->declare_parameter<bool>("layer_merge_enable", true);
    this->declare_parameter<double>("layer_merge_gap_s", 0.4);
    this->declare_parameter<double>("layer_merge_gap_d", 0.3);

    // 출력과 주기 진단
    this->declare_parameter<bool>("publish_markers", true);
    this->declare_parameter<bool>("diagnostics_enable", true);
    this->declare_parameter<double>("diagnostics_period_sec", 1.0);

    // 등속 Kalman 추적 및 정적/동적 분류
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
    this->declare_parameter<std::string>("classifier_mode", "velocity");
    this->declare_parameter<double>("dyn_vel_enter", 0.5);
    this->declare_parameter<double>("dyn_vel_exit", 0.25);
    this->declare_parameter<int>("static_confirm_frames", 3);
    this->declare_parameter<int>("dynamic_confirm_frames", 25);
    this->declare_parameter<double>("dyn_velocity_mahalanobis_gate", 9.21);
    this->declare_parameter<double>("dyn_max_abs_yaw_rate", 1.5);
    this->declare_parameter<double>("static_ref_gate", 0.3);
    this->declare_parameter<int>("std_window", 30);
    this->declare_parameter<int>("min_nb_meas", 5);
    this->declare_parameter<double>("min_std", 0.16);
    this->declare_parameter<double>("max_std", 0.20);
    this->declare_parameter<double>("dt_max", 0.5);
}

void ObstacleDetectorNode::loadParameters()
{
    // ROS 인터페이스 이름과 기준 좌표계를 먼저 읽는다.
    scan_topic_ = this->get_parameter("scan_topic").as_string();
    global_wpnts_topic_ = this->get_parameter("global_waypoints_topic").as_string();
    map_topic_ = this->get_parameter("map_topic").as_string();
    ego_odom_topic_ = this->get_parameter("ego_odom_topic").as_string();
    static_obs_topic_ = this->get_parameter("static_obs_topic").as_string();
    opp_obs_topic_ = this->get_parameter("opp_obs_topic").as_string();
    static_markers_topic_ = this->get_parameter("static_markers_topic").as_string();
    opp_markers_topic_ = this->get_parameter("opp_markers_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();

    // 스캔 전처리와 군집 파라미터
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
    // 잘못된 음수 배율이 공분산을 줄이지 못하도록 품질 배율을 안전 범위로 제한한다.
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

    // 계층 1 필터와 계층 내 병합 파라미터
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

    // 추적기 설정은 별도 구조체로 모아 configure()에 한 번 전달한다.
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
    tracker_params_.dyn_vel_enter = this->get_parameter("dyn_vel_enter").as_double();
    tracker_params_.dyn_vel_exit = this->get_parameter("dyn_vel_exit").as_double();
    tracker_params_.static_confirm_frames =
        std::max(1, static_cast<int>(
            this->get_parameter("static_confirm_frames").as_int()));
    tracker_params_.dynamic_confirm_frames =
        std::max(1, static_cast<int>(
            this->get_parameter("dynamic_confirm_frames").as_int()));
    tracker_params_.dyn_velocity_mahalanobis_gate =
        std::max(0.0, this->get_parameter("dyn_velocity_mahalanobis_gate").as_double());
    tracker_params_.dyn_max_abs_yaw_rate =
        std::max(0.0, this->get_parameter("dyn_max_abs_yaw_rate").as_double());
    tracker_params_.static_ref_gate = this->get_parameter("static_ref_gate").as_double();
    tracker_params_.std_window = this->get_parameter("std_window").as_int();
    tracker_params_.min_nb_meas = this->get_parameter("min_nb_meas").as_int();
    tracker_params_.min_std = this->get_parameter("min_std").as_double();
    tracker_params_.max_std = this->get_parameter("max_std").as_double();
    tracker_params_.dt_max = this->get_parameter("dt_max").as_double();

    // 문자열 설정을 내부 enum으로 변환한다. 알 수 없는 값은 기본 velocity 방식으로 되돌린다.
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
}

// ------------------------------------------------------------------------------------------------
// 입력 콜백: raceline / 점유 지도 / 자차 자세
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
    if (msg->wpnts.size() < 3)
    {
        RCLCPP_WARN(this->get_logger(),
                    "Received /global_waypoints with < 3 points; CLCS needs >= 3. Ignoring.");
        return;
    }

    // 트랙 경계, 폐루프 s 연산, 최종 Frenet marker 보간에 쓸 경량 waypoint를 만든다.
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
    // 군집과 자차의 정확한 (x, y) -> (s, d) 변환에 쓸 CLCS 변환기를 새로 만든다.
    try
    {
        // 기본 설정은 폐루프와 안전한 투영 탐색 범위를 제공한다.
        global_planning::ClcsFrenetConfig cfg;
        auto conv = global_planning::ClcsFrenetConverter::create(ref, cfg, ++clcs_version_);
        FrenetProjector next_frenet;
        next_frenet.build(std::move(wpnts), true, conv->stats().track_length);
        frenet_ = std::move(next_frenet);
        converter_ = conv;
        // 기준 경로가 바뀌었으므로 새 CLCS에 대해 odometry가 다시 투영될 때까지 기다린다.
        ego_s_ = -1.0;
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
    // transient_local 구독이므로 map server가 먼저 실행됐어도 마지막 지도를 받을 수 있다.
    map_msg_ = msg;
    RCLCPP_INFO_ONCE(this->get_logger(), "Occupancy map received (%u x %u @ %.3f m).",
                     msg->info.width, msg->info.height, msg->info.resolution);
}

void ObstacleDetectorNode::egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 최신 yaw rate는 급회전 중 잘못된 동적 판정을 억제하고 측정 공분산을 키우는 데 사용한다.
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
    }
}

// ------------------------------------------------------------------------------------------------
// TF 조회: scan 좌표계 -> map 좌표계
// ------------------------------------------------------------------------------------------------
bool ObstacleDetectorNode::lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx,
                                           double &ty, double &yaw)
{
    geometry_msgs::msg::TransformStamped tf;
    try
    {
        // 우선 스캔 취득 시각의 정확한 변환을 짧게 기다린다.
        tf = tf_buffer_->lookupTransform(map_frame_, scan_header.frame_id, scan_header.stamp,
                                         tf2::durationFromSec(0.05));
    }
    catch (const tf2::TransformException &)
    {
        // 과거 시각 TF가 버퍼에 없으면 최신 변환으로 한 번 더 시도해 스캔 유실을 줄인다.
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
// 적응형 breakpoint 군집화: 입력 점은 이미 map 좌표로 변환되어 있다.
// ------------------------------------------------------------------------------------------------
std::vector<std::vector<ObstacleDetectorNode::ScanPoint>>
ObstacleDetectorNode::clusterScan(const sensor_msgs::msg::LaserScan &scan, double tx, double ty,
                                  double yaw, ScanProcessingStats &stats) const
{
    std::vector<std::vector<ScanPoint>> clusters;
    std::vector<ScanPoint> current;
    const double dphi = scan.angle_increment;
    // Borges/Aldon 임계식에서 반복되는 항을 스캔당 한 번만 계산한다.
    const double denom = std::sin(lambda_rad_ - dphi);
    const double cyaw = std::cos(yaw);
    const double syaw = std::sin(yaw);

    bool have_prev = false;
    ScanPoint prev{};
    int prev_index = -1000;

    // 현재 파편을 저장한다. 병합 기능을 켰다면 작은 파편도 일단 후보로 보존한다.
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
            continue;  // 유효하지 않은 beam은 아래 index gap 검사에서 연속성을 끊는다.
        }
        if (r < scan.range_min)
        {
            ++stats.below_min_range_rejected;
            continue;
        }
        if (r >= max_range_)
        {
            ++stats.at_or_above_max_range_rejected;
            continue;  // 유효하지 않은 beam은 아래 index gap 검사에서 연속성을 끊는다.
        }
        ++stats.valid_beams;
        // scan 극좌표 점을 센서 좌표에서 map 좌표로 회전·이동한다.
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
            // 거리가 멀수록 인접 beam 간 간격이 커지는 Borges/Aldon 적응 임계값
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
// 추적 전 파편 병합: 스캔 군집을 완전한 검출 단위로 복원한다.
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
    // AABB만 겹치는 오목/대각 형상을 잘못 합치지 않도록 실제 점 간 거리를 확인한다.
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
        // AABB는 한 번 계산해 캐시하고 병합된 연결 요소만 갱신한다. 반복 루프가 같은 표면의
        // 여러 파편을 연쇄적으로 합칠 수 있게 하되, max_obs_size로 과도한 전이 병합을 막는다.
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
                    // 겹치거나 가까운 AABB는 broad-phase 후보일 뿐이다. 실제 점 쌍도 가까워야
                    // 대각선/오목 AABB 중첩 때문에 물리적으로 떨어진 파편이 합쳐지는 일을 막는다.
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

    // 작은 파편은 병합 후보로만 허용했으므로, 끝까지 단독으로 남으면 검출로 만들지 않는다.
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
// 계층 1 점유 지도 필터를 위한 좌표 조회
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
    // map 좌표를 occupancy grid의 정수 셀 좌표로 변환한다.
    const int gx = static_cast<int>(std::floor((x - info.origin.position.x) / info.resolution));
    const int gy = static_cast<int>(std::floor((y - info.origin.position.y) / info.resolution));
    const int w = static_cast<int>(info.width);
    const int h = static_cast<int>(info.height);
    // 지도 해상도 오차를 고려해 주변 inflation 셀 중 하나라도 점유되면 참으로 본다.
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
// 계층별 2차 군집화: 같은 계층의 트랙을 물체 단위 장애물로 병합한다.
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

    // Union-find로 두 Frenet 상자의 s와 d 모서리 간격이 모두 기준 이내일 때 연결한다.
    // Cartesian 대각선을 s/d 양쪽에 똑같이 늘리지 않고, 각 AABB에서 독립적으로 투영한
    // 종·횡방향 범위를 그대로 사용한다.
    std::vector<std::size_t> parent(n);
    for (std::size_t i = 0; i < n; ++i)
    {
        parent[i] = i;
    }
    auto find = [&parent](std::size_t i) {
        while (parent[i] != i)
        {
            parent[i] = parent[parent[i]];  // 경로 절반 줄이기로 후속 탐색을 빠르게 한다.
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
        // s 외곽은 첫 멤버에 대한 상대 거리로 계산해 시작/끝 경계 wrap을 안전하게 처리한다.
        // 모든 멤버가 짧은 가시 구간 안에 있으므로 반 바퀴 이상 떨어진 모호한 경우는 없다.
        // d 외곽은 폐루프가 없으므로 절대값 범위에서 바로 계산한다.
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
            // 더 큰 관측 형상에 더 큰 가중치를 주어 병합 물체의 속도를 계산한다.
            const double w = std::max(t->size, 1e-3);
            w_sum += w;
            vs += w * t->vs();
            vd += w * t->vd();
            // 병합 후 불확실성을 과소평가하지 않도록 멤버 중 가장 큰 분산을 사용한다.
            s_var = std::max(s_var, t->P(0, 0));
            vs_var = std::max(vs_var, t->P(1, 1));
            d_var = std::max(d_var, t->P(2, 2));
            vd_var = std::max(vd_var, t->P(3, 3));
            id = std::min(id, t->id);  // 가장 오래된 ID를 써서 프레임 간 식별자를 안정화한다.
            visible = visible || t->is_visible;

            // Frenet 예측은 마지막 원시 Cartesian 스캔 상자를 이동시키지 않는다. 이번 스캔에서
            // 실제 측정된 트랙만 Cartesian 형상에 포함해, TTL로 살아 있는 동적 트랙이 오래된
            // map 좌표 충돌 영역을 발행하지 않게 한다.
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
            // 정확한 Frenet->Cartesian 공분산 회전에는 국소 CLCS 접선이 필요하다. 현재는
            // 위치 분산 중 큰 값을 두 Cartesian 축에 사용해 불확실성을 과소평가하지 않는다.
            m.ob.x_var = std::max(s_var, d_var);
            m.ob.y_var = m.ob.x_var;

            // 현재 보이는 Cartesian 합집합을 최종 형상으로 간주한다. 계층 병합 후 한 번 다시
            // 투영하여 /static_obs와 /opp_obs의 Frenet 경계가 RViz에 보이는 동일 AABB를
            // 설명하도록 출력 형상 계약을 일치시킨다.
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
                ahead += frenet_.raceline_length();  // 뒤쪽 음수 거리를 다음 바퀴의 전방 거리로 바꾼다.
            }
            key = ahead;
        }
        else
        {
            key = ob.s_var + ob.d_var;  // 자차 s가 없으면 위치 불확실성이 가장 작은 물체를 고른다.
        }
        if (key < best_key)
        {
            best_key = key;
            best = static_cast<int>(i);
        }
    }
    return best;
}

// ------------------------------------------------------------------------------------------------
// 수동형 주기 진단: 사건 수는 주기 동안 누적하고, 트랙/분류 수는 최신 갱신의 snapshot을 쓴다.
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
        "track(total=%zu visible=%zu hit_pending=%zu class_pending=%zu "
        "provisional=%zu static=%zu dynamic=%zu motion_gated=%zu) "
        "assoc(pairs=%zu match=%zu spawn=%zu retire=%zu euclid_reject=%zu maha_reject=%zu) "
        "motion(yaw_used=%.3f fresh=%s ref_vs=%.3f ref_vd=%.3f)",
        elapsed, total.scans_processed, total.scans_received, total.clcs_unavailable,
        total.tf_unavailable, total.valid_beams, total.total_beams,
        total.nonfinite_rejected, total.below_min_range_rejected,
        total.at_or_above_max_range_rejected, total.clusters_before_merge,
        total.clusters_after_merge, total.fragments_rejected, total.detections,
        total.size_rejected, total.projection_rejected, total.viewing_window_rejected,
        total.track_boundary_rejected, total.map_rejected, snapshot.total_tracks,
        snapshot.visible_tracks, snapshot.hit_confirmation_pending,
        snapshot.classification_pending, snapshot.provisional_static,
        snapshot.confirmed_static, snapshot.confirmed_dynamic,
        snapshot.dynamic_motion_gated, events.candidate_pairs, events.matched, events.spawned,
        events.retired, events.euclidean_pair_rejected,
        events.mahalanobis_pair_rejected, measurement_yaw_rate,
        yaw_rate_fresh ? "true" : "false", tracker_.staticRefVs(), tracker_.staticRefVd());

    diagnostics_scan_totals_ = ScanProcessingStats{};
    diagnostics_tracker_event_totals_ = TrackerUpdateStats{};
    diagnostics_window_start_ = now;
}

// ------------------------------------------------------------------------------------------------
// 메인 인지 파이프라인: LaserScan이 들어올 때마다 한 주기를 실행한다.
// ------------------------------------------------------------------------------------------------
void ObstacleDetectorNode::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    ScanProcessingStats stats;
    stats.scans_received = 1;

    // 기준 경로 없이 Cartesian 점을 일관된 Frenet 좌표로 바꿀 수 없으므로 스캔을 보류한다.
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

    // 측정 시각과 가까운 odometry만 사용해 오래된 회전률이 공분산/분류를 오염시키지 않게 한다.
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
    // 계층 1 [map]: 스캔을 군집화한 뒤 벽·알려진 정적 구조물처럼 지도 자체인 군집과
    // 주행 경계/가시 구간 밖 군집을 제거한다. 남은 군집만 지도에 속하지 않는 장애물 후보다.
    // 지도 계층은 필터 전용이며 별도 장애물 출력으로 발행하지 않는다.
    // ============================================================================================
    const auto clusters = clusterScan(*msg, tx, ty, yaw, stats);
    std::vector<Detection> detections;
    detections.reserve(clusters.size());
    for (const auto &cluster : clusters)
    {
        // map 좌표 축 정렬 상자를 계산한다. 중심과 네 모서리를 함께 투영해야 발행할 Frenet
        // 형상에서 종방향·횡방향 크기가 서로 독립적으로 보존된다.
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
            continue;  // F1TENTH 차량이나 cone으로 보기에는 지나치게 큰 군집이다.
        }

        const auto bounds = projectCartesianAabb(
            *converter_, minx, maxx, miny, maxy);
        if (!bounds.has_value())
        {
            ++stats.projection_rejected;
            continue;
        }
        // 자차 기준 전방/후방 가시 거리 게이트
        if (ego_s_ >= 0.0)
        {
            const double ds = frenet_.wrapDelta(bounds->s_center, ego_s_);
            if (ds > max_viewing_distance_ || ds < -view_behind_distance_)
            {
                ++stats.viewing_window_rejected;
                continue;
            }
        }

        // 주행 가능 트랙 경계 게이트: waypoint 경계가 없으면 기본 반폭을 사용한다.
        double dl = 0.0;
        double dr = 0.0;
        frenet_.boundsAtS(bounds->s_center, dl, dr);
        const double left_bound = (dl > 0.05 ? dl : fallback_track_halfwidth_) - boundaries_inflation_;
        const double right_bound =
            (dr > 0.05 ? dr : fallback_track_halfwidth_) - boundaries_inflation_;
        if (bounds->d_center > left_bound || bounds->d_center < -right_bound)
        {
            ++stats.track_boundary_rejected;
            continue;
        }

        // 점유 지도 필터: 군집 점 중 설정 비율 이상이 알려진 구조물 위에 있으면 제거한다.
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
        // 먼 거리, 성긴 군집, 빠른 자차 회전일수록 측정 신뢰도가 낮다고 보고 공분산을 키운다.
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

    // 추적 단계: 각 생존 군집에 Frenet 흐름 속도와 map-flow 대비 정적/동적 라벨을 부여한다.
    // 자세한 분류 조건은 obstacle_tracker.hpp의 계층 설명을 참고한다.
    tracker_.update(detections, stamp, measurement_yaw_rate, yaw_rate_fresh);
    stats.scans_processed = 1;
    updateDiagnostics(stats, &tracker_.lastStats(), measurement_yaw_rate, yaw_rate_fresh);

    // ============================================================================================
    // 계층 조립: 발행 가능한 트랙을 분류별로 모은 뒤 같은 물체의 파편을 계층 안에서 2차
    // 병합하고 물체 단위로 발행한다.
    // 계층 2 [static]  -> /static_obs : 병합된 모든 정적 물체
    // 계층 3 [dynamic] -> /opp_obs    : 병합 후 자차 전방에 가장 가까운 상대 차량 하나
    // 계층이 비어도 매 스캔 빈 배열을 발행하므로 소비 노드의 갱신 주기가 끊기지 않는다.
    // ============================================================================================
    std::vector<const Track *> static_members;
    std::vector<const Track *> dynamic_members;
    static_members.reserve(tracker_.tracks().size());
    dynamic_members.reserve(tracker_.tracks().size());
    for (const Track &t : tracker_.tracks())
    {
        if (!t.classified || t.motion_class == MotionClass::Pending)
        {
            continue;
        }
        if (t.motion_class == MotionClass::Dynamic)
        {
            dynamic_members.push_back(&t);
        }
        else
        {
            // 임시 정적과 확정 정적은 같은 트랙과 ID를 공유한다. 상태 승격 순간에도
            // /static_obs에서 한 스캔 동안 장애물이 사라지는 현상이 생기지 않는다.
            static_members.push_back(&t);
        }
    }
    const auto static_objs = mergeLayer(static_members, true);
    const auto dynamic_objs = mergeLayer(dynamic_members, false);
    const int opp = selectOpponent(dynamic_objs);

    f110_msgs::msg::ObstacleArray static_arr;
    static_arr.header = msg->header;
    static_arr.header.frame_id = map_frame_;
    static_arr.obstacles.reserve(static_objs.size());
    for (const auto &m : static_objs)
    {
        static_arr.obstacles.push_back(m.ob);
    }
    static_obs_pub_->publish(static_arr);

    f110_msgs::msg::ObstacleArray opp_arr;
    opp_arr.header = msg->header;
    opp_arr.header.frame_id = map_frame_;
    if (opp >= 0)
    {
        opp_arr.obstacles.push_back(dynamic_objs[opp].ob);
    }
    opp_obs_pub_->publish(opp_arr);

    // RViz marker는 내부 중간값이 아니라 최종 발행 Frenet 배열에서 만든다. 따라서 예측 전용
    // 물체까지 포함해 하위 planner가 실제로 받는 s/d 외곽을 그대로 시각화한다.
    if (publish_markers_ && static_markers_pub_ && opp_markers_pub_)
    {
        static_markers_pub_->publish(
            buildFrenetObstacleMarkers(
                static_arr, frenet_, "static_obs_frenet", 0.2F, 0.6F, 1.0F));
        opp_markers_pub_->publish(
            buildFrenetObstacleMarkers(
                opp_arr, frenet_, "opp_obs_frenet", 1.0F, 0.2F, 0.2F));
    }
}

}  // namespace obstacle_detector

// ------------------------------------------------------------------------------------------------
// ROS 2 프로세스 진입점
// ------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<obstacle_detector::ObstacleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
