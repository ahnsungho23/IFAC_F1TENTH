// Kinematic-ICP based map localization node (2026_IFAC).
//
// Drop-in replacement for the MCL (particle_filter_cpp) localization output:
// subscribes to /scan, aligns each deskewed scan against a frozen voxel map
// built from an existing occupancy map (.kissmap), and publishes the estimated
// pose on /pf/pose/odom plus the map -> odom TF (same semantics as MCL).
//
// Uses the vendored kinematic-icp core (third_party/kinematic-icp, MIT) with
// Config::freeze_local_map = true so incoming scans never modify the map.
//
// slam_mode = true turns the same pipeline into online SLAM (freeze_local_map
// = false): no frozen map is loaded, the pose auto-initializes to identity on
// the first scan, the registered (downsampled) frames are accumulated in the
// map frame, and the accumulated map can be saved to a .kissmap file via the
// ~/save_map service or automatically on shutdown. The accumulated map is
// also published on ~/map_points for RViz.
//
// Output smoothing (smoothing_enable): raw ICP poses are noisy (cm-level
// steps at speed, see the real-car bag measurements in docs). A complementary
// filter predicts the pose with the wheel-odometry delta and pulls it toward
// the ICP estimate with a velocity-adaptive gain (same idea as the MCL
// smoother):  T_pred = T_last_out * delta_odom,
//  T_out = T_pred * exp(diag(alpha, alpha_rot) * log(T_pred^-1 * T_icp)).
// Rotation uses its own fixed gain (smoothing_alpha_rot): the ICP yaw
// measurement is ~10x noisier than the wheel-odom yaw prediction while
// cornering, so pulling yaw at the (velocity-boosted) translation alpha turns
// corners into yaw jitter downstream.
// The filtered pose drives both /pf/pose/odom and the map->odom TF; the SLAM
// map accumulation keeps using the raw ICP pose (see ProcessScan).
//
// Built-in map server: the (frozen or accumulated) map points are rasterized
// to a nav_msgs/OccupancyGrid and published on map_topic (transient_local)
// so RViz tools like 2D Pose Estimate work without a separate map_server.
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <deque>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <kiss_icp/core/Preprocessing.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <memory>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sophus/se3.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

#include "kinematic_icp/pipeline/KinematicICP.hpp"
#include "utils.hpp"

namespace kinematic_localization {

class LocalizationNode : public rclcpp::Node {
public:
    LocalizationNode() : Node("kinematic_localization") {
        // Topics / frames
        lidar_topic_ = declare_parameter<std::string>("lidar_topic", "/scan");
        odom_topic_ = declare_parameter<std::string>("odom_topic", "/odom");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/pf/pose/odom");
        initial_pose_topic_ =
            declare_parameter<std::string>("initial_pose_topic", "/initialpose");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
        publish_map_odom_tf_ = declare_parameter<bool>("publish_map_odom_tf", true);
        map_name_ = declare_parameter<std::string>("map_name", "");

        // MCL(particle_filter_cpp) 호환: /global_waypoints의 첫 웨이포인트(스타트라인
        // 자세)로 자동 초기화한다. MCL의 `auto_init_from_waypoints`와 같은 규약이라,
        // 이 노드로 갈아타도 "RViz 2D Pose Estimate를 매번 찍어야 하는" 운영 변화가
        // 없다. `/initialpose`가 먼저 오면 그쪽이 이긴다(사람이 찍은 값 우선).
        // ⚠️ 차가 스타트라인 근처(수렴 베이슨 ≈ voxel_size 1.0 m)에 있을 때만 맞는다.
        //    엉뚱한 곳에서 켜면 잘못된 포즈를 자신 있게 발행하므로, 그런 운용이면
        //    false로 두고 /initialpose를 쓸 것(그때는 발행 자체를 안 해 컨트롤러
        //    odom 워치독이 차를 세운다 = fail-safe).
        auto_init_from_waypoints_ =
            declare_parameter<bool>("auto_init_from_waypoints", true);
        auto_init_topic_ =
            declare_parameter<std::string>("auto_init_topic", "/global_waypoints");
        // 자동 초기화 검증(수동 /initialpose에는 적용하지 않는다 — 사람이 본 것이므로).
        // 🔴 이 게이트가 없으면 auto-init은 위험하다: 스타트라인에서 4.7 m 떨어진
        //    자세로 초기화한 재생에서 ICP가 **끝내 회복하지 못하고**(60초 내내 9~16 m
        //    이탈) 그 틀린 포즈를 40 Hz로 조용히 계속 발행했다. 수렴 베이슨이
        //    voxel_size(1.0 m) 수준이라 구조적으로 못 돌아온다.
        // 그래서 초기화 직후 `auto_init_validate_frames` 프레임 동안 **발행을 보류**하고
        // 정합 품질만 본다. run_0818_182531 재생 실측 residual_rms 중앙값(40프레임):
        //   정상 초기화 0.19 (관측 최악 0.275) / 4.7 m 오초기화 0.45~0.54
        // 분리비가 2.3배로 일정해 warmup 없이도 갈린다. 임계 0.35는 관측 최악 정상의
        // 1.27배이자 오초기화 중앙값의 0.78배 — 딱 중간이다. 오탐(false reject)의 대가는
        // "사람이 2D Pose Estimate를 한 번 찍는 것"(= 포팅 전과 같은 절차)이고,
        // 미탐(false accept)의 대가는 벽이므로 **애매하면 거부하는 쪽으로 잡는다.**
        // 실패하면 미초기화로 되돌리고 /initialpose를 기다린다(= 발행 없음 →
        // 컨트롤러 odom 워치독이 차를 세운다, fail-safe).
        auto_init_validate_frames_ =
            declare_parameter<int>("auto_init_validate_frames", 40);
        auto_init_max_residual_ =
            declare_parameter<double>("auto_init_max_residual", 0.35);

        // Online SLAM mode: same pipeline, but the KISS local map is updated by
        // scans (freeze_local_map = false), no frozen map is loaded, and the
        // accumulated map can be saved as a new .kissmap.
        slam_mode_ = declare_parameter<bool>("slam_mode", false);
        map_output_file_ = declare_parameter<std::string>("map_output_file", "slam_map.kissmap");
        map_publish_period_sec_ = declare_parameter<double>("map_publish_period_sec", 2.0);

        // Output smoothing (complementary filter: wheel-odom prediction +
        // ICP correction, velocity-adaptive alpha). These are starting values
        // for real-car tuning, not final tuned constants.
        smoothing_enable_ = declare_parameter<bool>("smoothing_enable", true);
        smoothing_alpha_ = declare_parameter<double>("smoothing_alpha", 0.2);
        smoothing_alpha_gain_ = declare_parameter<double>("smoothing_alpha_gain", 0.3);
        smoothing_velocity_full_ =
            declare_parameter<double>("smoothing_velocity_full_mps", 3.0);
        smoothing_alpha_max_ = declare_parameter<double>("smoothing_alpha_max", 0.8);
        // < 0: rotation follows the translation alpha (legacy uniform filter)
        smoothing_alpha_rot_ = declare_parameter<double>("smoothing_alpha_rot", -1.0);

        // Built-in /map occupancy server (RViz 2D Pose Estimate support)
        map_topic_ = declare_parameter<std::string>("map_topic", "/map");
        map_grid_resolution_ = declare_parameter<double>("map_grid_resolution", 0.05);
        map_point_dilation_m_ = declare_parameter<double>("map_point_dilation_m", 0.15);

        // Robustness plan §2: watchdog — during scan gaps keep publishing the
        // wheel-odometry extrapolation so /pf/pose/odom and map->odom never
        // freeze (same failure MCL covers with its 40 Hz odom-only timer).
        watchdog_enable_ = declare_parameter<bool>("watchdog_enable", true);
        watchdog_rate_hz_ = declare_parameter<double>("watchdog_rate_hz", 40.0);
        scan_timeout_sec_ = declare_parameter<double>("scan_timeout_sec", 0.15);
        max_dead_reckoning_sec_ = declare_parameter<double>("max_dead_reckoning_sec", 2.0);
        watchdog_trans_error_rate_ =
            declare_parameter<double>("watchdog_trans_error_rate", 0.03);
        watchdog_rot_error_rate_ = declare_parameter<double>("watchdog_rot_error_rate", 0.10);

        // Robustness plan §3: registration quality diagnostics
        diagnostics_enable_ = declare_parameter<bool>("diagnostics_enable", true);
        diagnostics_topic_ =
            declare_parameter<std::string>("diagnostics_topic", "~/diagnostics");
        min_inlier_ratio_warn_ = declare_parameter<double>("min_inlier_ratio_warn", 0.3);

        // Robustness plan §4: map-based pose validity check (log/diagnostic +
        // gate force-accept blocker only — never rejects poses on its own)
        pose_check_enable_ = declare_parameter<bool>("pose_check_enable", true);
        permissible_radius_m_ = declare_parameter<double>("permissible_radius_m", 0.15);

        // Robustness plan §5: Mahalanobis gate. Default OFF — enable only
        // after the §3 diagnostics distributions were measured on real bags.
        gate_enable_ = declare_parameter<bool>("gate_enable", false);
        gate_chi2_ = declare_parameter<double>("gate_chi2", 9.21);
        gate_trans_error_rate_ = declare_parameter<double>("gate_trans_error_rate", 0.03);
        gate_rot_error_rate_ = declare_parameter<double>("gate_rot_error_rate", 0.10);
        gate_meas_std_floor_ = declare_parameter<double>("gate_meas_std_floor", 0.02);
        gate_force_accept_ = declare_parameter<int>("gate_force_accept", 20);

        // Kinematic-ICP configuration (values verified on run_0803_210100)
        kinematic_icp::pipeline::Config config;
        config.max_range = declare_parameter<double>("max_range", 30.0);
        config.min_range = declare_parameter<double>("min_range", 0.1);
        config.voxel_size = declare_parameter<double>("voxel_size", 1.0);
        // <= 0: source downsample follows voxel_size (upstream behavior)
        config.source_voxel_size =
            declare_parameter<double>("source_voxel_size", config.source_voxel_size);
        config.max_points_per_voxel =
            declare_parameter<int>("max_points_per_voxel", config.max_points_per_voxel);
        config.use_adaptive_threshold =
            declare_parameter<bool>("use_adaptive_threshold", config.use_adaptive_threshold);
        config.fixed_threshold = declare_parameter<double>("fixed_threshold", 1.0);
        config.max_num_iterations = declare_parameter<int>("max_num_iterations", 30);
        config.convergence_criterion =
            declare_parameter<double>("convergence_criterion", config.convergence_criterion);
        config.max_num_threads =
            declare_parameter<int>("max_num_threads", config.max_num_threads);
        config.use_adaptive_odometry_regularization = declare_parameter<bool>(
            "use_adaptive_odometry_regularization", config.use_adaptive_odometry_regularization);
        config.fixed_regularization = declare_parameter<double>("fixed_regularization", 0.0);
        config.deskew = declare_parameter<bool>("deskew", true);
        // Robustness plan §6: soft lateral DoF (core patch). Default OFF —
        // requires the §5 gate safety net and measured symptoms before use.
        config.lateral_dof_enable = declare_parameter<bool>("lateral_dof_enable", false);
        config.lateral_regularization_scale =
            declare_parameter<double>("lateral_regularization_scale", 1.0);
        config.lateral_regularization_floor_tau2 =
            declare_parameter<double>("lateral_regularization_floor_tau2", 1.0);
        config.freeze_local_map = !slam_mode_;  // SLAM mode: scans update the local map
        if (slam_mode_) {
            // The map-premised robustness features assume a frozen map;
            // in SLAM the map is still growing and the lateral constraint is
            // what suppresses accumulated drift — force them off (plan §4-§6;
            // the watchdog would corrupt the accumulated map by reseeding).
            if (watchdog_enable_ || pose_check_enable_ || gate_enable_ ||
                config.lateral_dof_enable) {
                RCLCPP_INFO(get_logger(),
                            "slam_mode: watchdog/pose_check/gate/lateral_dof forced off");
            }
            watchdog_enable_ = false;
            pose_check_enable_ = false;
            gate_enable_ = false;
            config.lateral_dof_enable = false;
        }
        if (config.max_range < config.min_range) {
            RCLCPP_WARN(get_logger(), "max_range < min_range, setting min_range to 0.0");
            config.min_range = 0.0;
        }
        voxel_size_ = config.voxel_size;
        max_range_ = config.max_range;

        position_covariance_ = declare_parameter<double>("position_covariance", 0.1);
        orientation_covariance_ = declare_parameter<double>("orientation_covariance", 0.1);

        icp_ = std::make_unique<kinematic_icp::pipeline::KinematicICP>(config);

        if (slam_mode_) {
            if (!map_name_.empty()) {
                RCLCPP_WARN(get_logger(), "slam_mode: map_name '%s' is ignored (no frozen map)",
                            map_name_.c_str());
            }
        } else {
            LoadFrozenMap();
        }

        // Publishers: same interface as MCL
        pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(pose_topic_,
                                                              rclcpp::QoS(10).reliable());
        occ_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_topic_, rclcpp::QoS(1).transient_local().reliable());
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        // Frozen mode: publish the rasterized frozen map once (latched).
        if (!slam_mode_ && !frozen_points_.empty()) PublishOccupancyMap(frozen_points_);

        odom_msg_.header.frame_id = map_frame_;
        odom_msg_.child_frame_id = base_frame_;
        odom_msg_.pose.covariance.fill(0.0);
        odom_msg_.twist.covariance.fill(0.0);
        odom_msg_.twist.covariance[0] = position_covariance_;
        odom_msg_.twist.covariance[7] = position_covariance_;
        odom_msg_.twist.covariance[35] = orientation_covariance_;
        SetOutputCovariance(0.0);

        // Large queues: bag playback publishes scans/odom in bursts, and a
        // shallow best-effort queue drops them (uniform 40 Hz input is fine
        // either way — the callback takes only ~2 ms).
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            lidar_topic_, rclcpp::SensorDataQoS().keep_last(100),
            [this](const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) { OnScan(msg); });
        // Wheel odometry topic (same source MCL uses). The odom->base TF in the
        // bag/real system has multi-hundred-ms gaps, so deltas are computed
        // from this history instead of TF lookups.
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS().keep_last(100),
            [this](const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
                const rclcpp::Time stamp(msg->header.stamp);
                // Out-of-order guard: a stale/foreign publisher (e.g. another bag
                // on the same DDS domain) corrupts the sorted history and the
                // interpolated prior — drop anything not newer than the newest.
                if (!odom_history_.empty() && stamp <= odom_history_.back().first) return;
                odom_history_.emplace_back(stamp, utils::PoseToSophus(msg->pose.pose));
                while (odom_history_.size() > 400) odom_history_.pop_front();
                // Wheel twist is forwarded verbatim on /pf/pose/odom (MCL convention):
                // a pose-delta velocity is far noisier than the wheel signal and
                // dithered the controller's L1 lookahead / speed PI (run_0818_173938:
                // vx noise std 0.285 m/s vs 0.086 for wheel-sourced MCL twist).
                latest_odom_twist_ = msg->twist.twist;
            });
        // MCL 호환 자동 초기화 입력. 발행자가 transient_local이라 늦게 떠도 받는다.
        if (auto_init_from_waypoints_ && !slam_mode_) {
            auto_init_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
                auto_init_topic_, rclcpp::QoS(1).transient_local(),
                [this](const f110_msgs::msg::WpntArray::ConstSharedPtr msg) {
                    OnGlobalWaypoints(msg);
                });
        }

        initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            initial_pose_topic_, rclcpp::QoS(10),
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
                OnInitialPose(msg);
            });

        if (diagnostics_enable_) {
            diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
                diagnostics_topic_, rclcpp::QoS(10));
        }
        // Watchdog timer (§2). Uses the node clock so bag replay (sim time)
        // and the real car behave the same.
        if (watchdog_enable_ && watchdog_rate_hz_ > 0.0) {
            watchdog_timer_ = rclcpp::create_timer(
                this, get_clock(),
                rclcpp::Duration::from_seconds(1.0 / watchdog_rate_hz_),
                [this]() { OnWatchdog(); });
        }

        if (slam_mode_) {
            save_map_srv_ = create_service<std_srvs::srv::Trigger>(
                "~/save_map",
                [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                    size_t n_points = 0;
                    response->success = SaveMap(&n_points);
                    response->message = response->success
                                            ? "Saved " + std::to_string(n_points) + " points to " +
                                                  map_output_file_
                                            : "Failed to save map to " + map_output_file_ +
                                                  " (accumulated: " +
                                                  std::to_string(accumulated_points_.size()) +
                                                  " raw points)";
                    RCLCPP_INFO(get_logger(), "save_map: %s", response->message.c_str());
                });
            if (map_publish_period_sec_ > 0.0) {
                map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
                    "~/map_points", rclcpp::QoS(1).transient_local().reliable());
            }
            RCLCPP_INFO(get_logger(),
                        "kinematic_localization started in SLAM mode (map frame = start pose). "
                        "Auto-initializing on first scan; save with 'ros2 service call "
                        "%s/save_map' or on shutdown -> %s",
                        get_name(), map_output_file_.c_str());
        } else {
            RCLCPP_INFO(get_logger(),
                        "kinematic_localization started. Waiting for /initialpose in frame '%s' "
                        "(frozen map: %zu points)",
                        map_frame_.c_str(), frozen_points_.size());
        }

        // Built-in map server refresh (both modes): SLAM republishes the
        // growing map; frozen mode re-publishes the cached grid so a late or
        // restarted RViz/DDS participant always converges on /map even if the
        // initial latched sample was missed.
        if (map_publish_period_sec_ > 0.0) {
            map_pub_timer_ = create_wall_timer(
                std::chrono::duration<double>(map_publish_period_sec_), [this]() {
                    if (slam_mode_) {
                        if (accumulated_points_.empty()) return;
                        const auto downsampled = kiss_icp::VoxelDownsample(accumulated_points_,
                                                                           voxel_size_ * 0.5);
                        PublishMapPoints(downsampled);
                        PublishOccupancyMap(downsampled);
                    } else if (occ_grid_.has_value()) {
                        occ_grid_->header.stamp = now();
                        occ_map_pub_->publish(*occ_grid_);
                    }
                });
        }
    }

    // Normal shutdown (Ctrl-C): persist the SLAM map like the save_map service.
    ~LocalizationNode() override {
        if (slam_mode_ && !accumulated_points_.empty()) {
            size_t n_points = 0;
            if (SaveMap(&n_points)) {
                RCLCPP_INFO(get_logger(), "Shutdown: saved %zu points to %s", n_points,
                            map_output_file_.c_str());
            } else {
                RCLCPP_ERROR(get_logger(), "Shutdown: failed to save map to %s",
                             map_output_file_.c_str());
            }
        }
    }

private:
    void LoadFrozenMap() {
        if (map_name_.empty()) {
            RCLCPP_WARN(get_logger(),
                        "map_name is empty: running as pure odometry (no frozen map)");
            return;
        }
        const std::string path =
            (map_name_.front() == '/')
                ? map_name_
                : ament_index_cpp::get_package_share_directory("kinematic_localization") +
                      "/maps/" + map_name_ + ".kissmap";
        const auto map = utils::ReadKissMap(path);
        if (!map.has_value()) {
            // Fail fast: a mistyped/uninstalled map used to silently degrade to
            // pure odometry with no /map served, which looks like "the map
            // server never launched" downstream. Empty map_name is still the
            // documented pure-odometry mode.
            RCLCPP_FATAL(get_logger(),
                         "Failed to load frozen map '%s' (map_name '%s'): file missing or not a "
                         "KISSMAP1 file. Use map_name:='' for pure odometry.",
                         path.c_str(), map_name_.c_str());
            throw std::runtime_error("kinematic_localization: failed to load frozen map " + path);
        }
        if (std::abs(map->voxel_size - voxel_size_) > 1e-9) {
            RCLCPP_WARN(get_logger(),
                        "Frozen map voxel_size (%.3f) != node voxel_size (%.3f)", map->voxel_size,
                        voxel_size_);
        }
        if (std::abs(map->max_range - max_range_) > 1e-9) {
            RCLCPP_WARN(get_logger(), "Frozen map max_range (%.3f) != node max_range (%.3f)",
                        map->max_range, max_range_);
        }
        frozen_points_ = map->points;
        RCLCPP_INFO(get_logger(), "Loaded frozen map '%s' (%zu points)", path.c_str(),
                    frozen_points_.size());
    }

    // MCL `waypointsCB`의 자동 초기화와 같은 규약: 첫 웨이포인트(스타트라인) 자세로
    // 한 번만 초기화하고, 사람이 /initialpose를 찍었으면 양보한다.
    void OnGlobalWaypoints(const f110_msgs::msg::WpntArray::ConstSharedPtr &msg) {
        if (auto_init_done_ || manual_init_done_ || msg->wpnts.empty()) return;
        const auto &w = msg->wpnts.front();
        Sophus::SE3d T_map_base(
            Sophus::SO3d::rotZ(w.psi_rad),
            Eigen::Vector3d(w.x_m, w.y_m, 0.0));
        RCLCPP_INFO(get_logger(),
                    "Auto-initializing from %s start pose: [%.3f, %.3f, %.3f rad (%.1f deg)] "
                    "(MCL auto_init_from_waypoints 호환 — /initialpose로 언제든 덮어쓸 수 있다)",
                    auto_init_topic_.c_str(), w.x_m, w.y_m, w.psi_rad,
                    w.psi_rad * 180.0 / M_PI);
        ApplyInitialPose(T_map_base);
        auto_init_done_ = true;
        validating_auto_init_ = (auto_init_validate_frames_ > 0);
    }

    // 자동 초기화 검증 창. true를 돌려주면 이번 프레임은 발행하지 않는다.
    // 창이 끝나면 정합 품질로 채택/기각을 결정한다.
    bool HoldForAutoInitValidation(double residual_rms) {
        if (!validating_auto_init_) return false;
        if (residual_rms > 0.0 && std::isfinite(residual_rms))
            validate_residuals_.push_back(residual_rms);
        if (static_cast<int>(validate_residuals_.size()) <
            static_cast<size_t>(auto_init_validate_frames_)) {
            return true;   // 아직 판단 못 함 — 보류
        }
        auto v = validate_residuals_;
        std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
        const double med = v[v.size() / 2];
        validating_auto_init_ = false;
        validate_residuals_.clear();
        if (med > auto_init_max_residual_) {
            RCLCPP_ERROR(get_logger(),
                         "Auto-init REJECTED: residual_rms median %.3f > %.3f over %d frames. "
                         "차가 스타트라인 근처가 아니었을 가능성이 높다 — ICP는 이 상태에서 "
                         "회복하지 못한다. RViz 2D Pose Estimate(%s)로 초기 포즈를 직접 줄 것. "
                         "(그때까지 포즈를 발행하지 않는다)",
                         med, auto_init_max_residual_, auto_init_validate_frames_,
                         initial_pose_topic_.c_str());
            initialized_ = false;      // /initialpose를 다시 기다린다
            has_last_out_ = false;     // 워치독도 같이 멈춘다
            auto_init_done_ = true;    // 같은 자세로 자동 재시도하지 않는다
            return true;
        }
        RCLCPP_INFO(get_logger(),
                    "Auto-init accepted (residual_rms median %.3f <= %.3f over %d frames)",
                    med, auto_init_max_residual_, auto_init_validate_frames_);
        return false;
    }

    void OnInitialPose(
        const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr &msg) {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_) {
            RCLCPP_WARN(get_logger(), "Initial pose frame '%s' != map_frame '%s', using anyway",
                        msg->header.frame_id.c_str(), map_frame_.c_str());
        }
        const Sophus::SE3d T_map_base = utils::PoseToSophus(msg->pose.pose);
        // 사람이 찍은 값이 우선 — 이후 /global_waypoints 재발행이 덮어쓰지 못하게 한다.
        manual_init_done_ = true;
        ApplyInitialPose(T_map_base);
        RCLCPP_INFO_STREAM(get_logger(), "Initial pose set:\n" << T_map_base.matrix());
    }

    // /initialpose와 자동 초기화가 공유하는 리셋 경로.
    void ApplyInitialPose(const Sophus::SE3d &T_map_base) {
        // SetPose clears the local map and resets the adaptive threshold,
        // so re-inject the frozen map points right after.
        icp_->SetPose(T_map_base);
        if (!frozen_points_.empty()) icp_->VoxelMap().AddPoints(frozen_points_);
        initialized_ = true;
        last_valid_icp_pose_ = T_map_base;
        pending_scan_.reset();
        // Reset the output filter: the first output after (re-)initialization
        // is the raw ICP pose.
        last_out_ = T_map_base;
        has_last_out_ = false;
        // Robustness state resets: watchdog idles until the first registered
        // scan, the gate starts with a clean rejection streak.
        last_scan_processed_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
        dead_reckoning_sec_ = 0.0;
        gate_reject_streak_ = 0;
        low_inlier_frames_ = 0;
        validate_residuals_.clear();
    }

    void OnScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
        if (!initialized_) {
            if (slam_mode_) {
                // SLAM: the map frame is the start pose, so no /initialpose is
                // needed. If one arrived earlier it already set the pose.
                icp_->SetPose(Sophus::SE3d());
                initialized_ = true;
                last_valid_icp_pose_ = Sophus::SE3d();
                last_out_ = Sophus::SE3d();
                has_last_out_ = false;
                RCLCPP_INFO(get_logger(), "SLAM mode: initialized at identity (first scan)");
            } else {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Waiting for initial pose on %s, dropping scans",
                                     initial_pose_topic_.c_str());
                return;
            }
        }
        // Process with one scan of lag: the odom TF bracketing the scan end
        // stamp is published right after the scan, so the lookup only succeeds
        // once the *next* scan arrives (same effect as upstream's tf_timeout).
        if (pending_scan_) ProcessScan(pending_scan_);
        pending_scan_ = msg;
    }

    void ProcessScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr &msg) {
        // Laser extrinsic (static transform base -> laser), looked up once
        if (!lidar_to_base_.has_value()) {
            const auto extrinsic =
                utils::LookupTransform(base_frame_, msg->header.frame_id, *tf_buffer_);
            if (!extrinsic.has_value()) {
                RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "Waiting for TF %s -> %s", base_frame_.c_str(),
                                     msg->header.frame_id.c_str());
                return;
            }
            lidar_to_base_ = extrinsic;
        }

        // 2D scan -> deskewable point cloud with per-point timestamps
        auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
        laser_projector_.projectLaser(*msg, *cloud, -1.0,
                                      laser_geometry::channel_option::Timestamp);
        const auto points = utils::PointCloud2ToEigen(*cloud);
        // The handler's end stamp (msg stamp + scan duration) is the deskew
        // reference time of KISS, so the *output* is stamped with it. The
        // odometry prior uses the raw msg stamps: lookups at the end stamp
        // extrapolate beyond the newest odom message during live playback.
        const auto &processed = timestamps_handler_.ProcessTimestamps(*cloud);
        const auto &timestamps = std::get<2>(processed);
        const rclcpp::Time out_stamp = std::get<1>(processed);  // scan end (deskew reference)
        const rclcpp::Time stamp(msg->header.stamp);
        const rclcpp::Time begin_stamp =
            last_scan_stamp_.nanoseconds() > 0 ? last_scan_stamp_ : stamp;
        last_scan_stamp_ = stamp;

        // Wheel odometry prior from the /odom history, interpolated to the
        // window boundaries (nearest-message matching quantizes the prior by
        // up to half an odom period, which matters at 5 m/s)
        const auto T_begin = OdomAt(begin_stamp);
        const auto T_end = OdomAt(stamp);
        const double dt = (stamp - begin_stamp).seconds();
        Sophus::SE3d delta_odom;  // identity when no odometry is available
        double speed = 0.0;
        bool gate_rejected = false;
        double gate_d2 = 0.0;
        bool pose_ok = true;
        bool registered = false;
        if (T_begin.has_value() && T_end.has_value()) {
            delta_odom = T_begin->inverse() * (*T_end);
            if (dt > 1e-6) speed = delta_odom.translation().norm() / dt;
            // Always register, even when (nearly) stationary: the frozen map
            // is never polluted by scans, and ICP can pull the pose back onto
            // the map while the car stands still.
            const auto &result =
                icp_->RegisterFrame(points, timestamps, *lidar_to_base_, delta_odom);
            registered = true;
            // Deep defense against the core NaN paths (see the Registration.cpp
            // patch): never let a non-finite pose reach the output or poison
            // last_pose_ permanently. Roll back through the non-const pose()
            // accessor — SetPose would clear the frozen map.
            if (!icp_->pose().matrix().allFinite()) {
                RCLCPP_ERROR(get_logger(),
                             "ICP produced a non-finite pose, rolling back (frame dropped)");
                if (last_valid_icp_pose_.has_value()) icp_->pose() = *last_valid_icp_pose_;
                return;
            }
            last_valid_icp_pose_ = icp_->pose();
            // Mahalanobis gate (§5): compare the ICP correction against the
            // odom prediction under the measurement covariance derived from
            // the §3 diagnostics (R = rms^2 * JTJ^-1). Rejected frames publish
            // the prediction and roll the core back through the non-const
            // pose() accessor (SetPose would clear the frozen map).
            if (gate_enable_ && has_last_out_) {
                gate_rejected = ApplyGate(delta_odom, &gate_d2);
            }
            // Pose validity (§4): never rejects on its own (a wide veto blocks
            // recovery on tight wall-hugging tracks — measured with MCL), only
            // logs, feeds diagnostics and blocks the gate force-accept.
            if (pose_check_enable_) {
                pose_ok = IsPosePermissible(icp_->pose());
                if (!pose_ok) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Pose is inside a wall (no non-occupied cell within "
                                         "%.2f m)",
                                         permissible_radius_m_);
                }
            }
            // SLAM: accumulate the downsampled registration frame in the map
            // frame (same scheme as mapping_node). Stationary frames are
            // skipped to keep the raw accumulation bounded while standing.
            // The raw ICP pose is used on purpose: the accumulated map must
            // stay consistent with the ICP estimate, not with the lagged
            // smoothing-filter output.
            if (slam_mode_ && delta_odom.log().norm() > 1e-3) {
                const Sophus::SE3d &pose = icp_->pose();
                for (const auto &p : std::get<1>(result)) accumulated_points_.push_back(pose * p);
            }
        } else {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "No wheel odometry available, keeping last pose");
        }

        // Output smoothing: complementary filter between the wheel-odometry
        // prediction and the raw ICP pose (see the file header). The first
        // output after (re-)initialization is the raw ICP pose.
        const Sophus::SE3d &T_icp = icp_->pose();
        Sophus::SE3d T_out = T_icp;
        double alpha_used = 1.0;
        double alpha_rot_used = 1.0;
        if (smoothing_enable_ && has_last_out_) {
            const Sophus::SE3d T_pred = last_out_ * delta_odom;
            const Sophus::SE3d err = T_pred.inverse() * T_icp;
            double alpha = smoothing_alpha_;
            if (smoothing_velocity_full_ > 0.0) {
                alpha += smoothing_alpha_gain_ *
                         std::min(speed / smoothing_velocity_full_, 1.0);
            }
            alpha = std::clamp(alpha, 0.0, smoothing_alpha_max_);
            alpha_used = alpha;
            const double alpha_rot = smoothing_alpha_rot_ >= 0.0
                                         ? std::min(smoothing_alpha_rot_, 1.0)
                                         : alpha;
            alpha_rot_used = alpha_rot;
            Sophus::SE3d::Tangent xi = err.log();
            xi.head<3>() *= alpha;
            xi.tail<3>() *= alpha_rot;
            T_out = T_pred * Sophus::SE3d::exp(xi);
        }
        // Same deep defense on the published pose (smoothing math included).
        if (!T_out.matrix().allFinite()) {
            RCLCPP_ERROR(get_logger(), "Non-finite output pose, dropping this frame");
            return;
        }
        // 자동 초기화 검증 창: 판정 전에는 한 프레임도 내보내지 않는다.
        if (registered && HoldForAutoInitValidation(icp_->registrationDiagnostics().residual_rms))
            return;

        last_out_ = T_out;
        has_last_out_ = true;

        // Latest wheel odom pose (MCL convention) for the map->odom TF
        const std::optional<Sophus::SE3d> T_odom_base =
            odom_history_.empty() ? std::nullopt
                                  : std::make_optional(odom_history_.back().second);
        SetOutputCovariance(0.0);  // fresh registration: base covariance
        PublishOdometry(T_out, out_stamp, T_odom_base);

        // Watchdog anchor (§2): a real scan was registered and published.
        last_scan_processed_time_ = now();
        dead_reckoning_sec_ = 0.0;

        // §3: low-inlier streak WARN + diagnostics publish. Only for frames
        // that actually registered — otherwise the core snapshot is stale.
        if (registered) {
            const auto &diag = icp_->registrationDiagnostics();
            const double inlier_ratio =
                diag.num_source_points > 0
                    ? static_cast<double>(diag.num_correspondences) / diag.num_source_points
                    : 0.0;
            if (inlier_ratio < min_inlier_ratio_warn_) {
                // 10 consecutive frames =~ 0.25 s at scan rate: long enough to
                // skip single-frame glitches, short enough to fire before a
                // real loss
                if (++low_inlier_frames_ >= 10) {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                         "Low inlier ratio %.2f < %.2f for %d frames",
                                         inlier_ratio, min_inlier_ratio_warn_,
                                         low_inlier_frames_);
                }
            } else {
                low_inlier_frames_ = 0;
            }
            PublishDiagnostics(out_stamp, speed, alpha_used, alpha_rot_used, gate_rejected,
                               gate_d2, pose_ok);
        }
    }

    // Odom pose interpolated to the requested stamp. Falls back to the
    // nearest message (within 100 ms) at the history edges.
    std::optional<Sophus::SE3d> OdomAt(const rclcpp::Time &stamp) const {
        if (odom_history_.empty()) return std::nullopt;
        if (stamp <= odom_history_.front().first) {
            if ((odom_history_.front().first - stamp).seconds() > 0.1) return std::nullopt;
            return odom_history_.front().second;
        }
        if (stamp >= odom_history_.back().first) {
            if ((stamp - odom_history_.back().first).seconds() > 0.1) return std::nullopt;
            return odom_history_.back().second;
        }
        const auto upper =
            std::lower_bound(odom_history_.begin(), odom_history_.end(), stamp,
                             [](const auto &entry, const rclcpp::Time &t) {
                                 return entry.first < t;
                             });
        const auto lower = upper - 1;
        const double span = (upper->first - lower->first).seconds();
        if (span <= 0.0) return upper->second;
        const double alpha = (stamp - lower->first).seconds() / span;
        return lower->second *
               Sophus::SE3d::exp(alpha * (lower->second.inverse() * upper->second).log());
    }

    // §5 Mahalanobis gate. Returns true when the ICP estimate was rejected
    // (the core is then already rolled back to the odom prediction). d2_out
    // always receives the computed distance (0 when the gate could not run).
    bool ApplyGate(const Sophus::SE3d &delta_odom, double *d2_out) {
        *d2_out = 0.0;
        const auto &diag = icp_->registrationDiagnostics();
        const long dims = diag.JTJ.rows();
        // Needs a solved frame: JTJ present and a usable residual. Frames that
        // broke before the first solve (0 correspondences) are already the
        // odom prediction — nothing to gate.
        if (dims < 2 || diag.residual_rms <= 0.0 || !diag.JTJ.allFinite()) return false;

        const Sophus::SE3d T_pred = last_out_ * delta_odom;
        const Sophus::SE3d::Tangent err = (T_pred.inverse() * icp_->pose()).log();
        Eigen::VectorXd nu(dims);
        if (dims == 2) {
            nu << err[0], err[5];  // longitudinal, yaw (§6 off)
        } else {
            nu << err[0], err[1], err[5];  // + lateral (§6 on)
        }

        // R = rms^2 * JTJ^-1, floored (over-confidence guard) and inflated on
        // non-converged frames (the recorded rms is one iteration stale and
        // exactly those frames deserve less trust — plan §3).
        Eigen::MatrixXd R = diag.residual_rms * diag.residual_rms *
                            diag.JTJ.inverse();
        if (!diag.converged) {
            const double inflation = 1.0 + diag.final_dx_norm / gate_meas_std_floor_;
            R *= inflation * inflation;
        }
        const double floor2 = gate_meas_std_floor_ * gate_meas_std_floor_;
        for (long i = 0; i < dims; ++i) R(i, i) = std::max(R(i, i), floor2);

        // P: odom prediction covariance, distance/rotation proportional with a
        // small absolute floor so P + R stays invertible while stationary.
        const double d_trans = delta_odom.translation().norm();
        const double d_yaw = std::abs(delta_odom.so3().log().z());
        const double sigma_trans = std::max(gate_trans_error_rate_ * d_trans, 1e-3);
        const double sigma_yaw = std::max(gate_rot_error_rate_ * d_yaw, 1e-3);
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(dims, dims);
        P(0, 0) = sigma_trans * sigma_trans;
        if (dims == 2) {
            P(1, 1) = sigma_yaw * sigma_yaw;
        } else {
            P(1, 1) = sigma_trans * sigma_trans;
            P(2, 2) = sigma_yaw * sigma_yaw;
        }

        const Eigen::MatrixXd S = P + R;
        if (!S.allFinite()) return false;
        const double d2 = nu.dot(S.ldlt().solve(nu));
        if (!std::isfinite(d2)) return false;
        *d2_out = d2;
        if (d2 <= gate_chi2_) {
            gate_reject_streak_ = 0;
            return false;
        }
        ++gate_reject_streak_;
        // Escape hatch: after gate_force_accept consecutive rejections accept
        // the ICP estimate — but never re-anchor into a wall (§4 blocker).
        if (gate_reject_streak_ >= gate_force_accept_ &&
            (!pose_check_enable_ || IsPosePermissible(icp_->pose()))) {
            RCLCPP_WARN(get_logger(),
                        "Gate force-accept after %d consecutive rejections (d2=%.1f)",
                        gate_reject_streak_, d2);
            gate_reject_streak_ = 0;
            return false;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Gate rejected ICP estimate (d2=%.1f > %.1f, streak %d) — "
                             "publishing odom prediction",
                             d2, gate_chi2_, gate_reject_streak_);
        icp_->pose() = T_pred;  // core rollback, local map preserved
        last_valid_icp_pose_ = T_pred;
        return true;
    }

    // §4: MCL is_pose_permissible equivalent on the rasterized map. The grid
    // only paints occupied disks (everything else is unknown), so "permissible"
    // means: at least one non-occupied cell within permissible_radius_m —
    // i.e. reject only poses buried deeper than the radius inside a wall blob.
    // Cells outside the grid count as unknown (not wall).
    bool IsPosePermissible(const Sophus::SE3d &pose) const {
        if (!occ_grid_.has_value()) return true;
        const auto &grid = *occ_grid_;
        const double res = grid.info.resolution;
        if (res <= 0.0 || grid.data.empty()) return true;
        const int width = static_cast<int>(grid.info.width);
        const int height = static_cast<int>(grid.info.height);
        const int cx = static_cast<int>(
            std::floor((pose.translation().x() - grid.info.origin.position.x) / res));
        const int cy = static_cast<int>(
            std::floor((pose.translation().y() - grid.info.origin.position.y) / res));
        const int r = static_cast<int>(std::ceil(permissible_radius_m_ / res));
        const double r2 = permissible_radius_m_ * permissible_radius_m_;
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                const double ddx = dx * res, ddy = dy * res;
                if (ddx * ddx + ddy * ddy > r2) continue;
                const int x = cx + dx, y = cy + dy;
                if (x < 0 || y < 0 || x >= width || y >= height) return true;
                if (grid.data[static_cast<size_t>(y) * width + x] != 100) return true;
            }
        }
        return false;
    }

    // §2 watchdog tick: when scans stall but wheel odometry still flows,
    // publish the odom-extrapolated pose so downstream never sees a frozen
    // /pf/pose/odom or map->odom TF.
    void OnWatchdog() {
        if (!initialized_ || !has_last_out_) return;
        if (last_scan_processed_time_.nanoseconds() == 0) return;
        const rclcpp::Time now_time = now();
        const double gap = (now_time - last_scan_processed_time_).seconds();
        if (gap < 0.0) {
            // Clock jumped backwards (bag restart) — re-anchor, do not publish.
            last_scan_processed_time_ = now_time;
            return;
        }
        if (gap <= scan_timeout_sec_) return;
        if (gap > max_dead_reckoning_sec_) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                                  "Scan gap %.2f s > max_dead_reckoning_sec %.2f — pose output "
                                  "suspended (dead reckoning would mislead downstream)",
                                  gap, max_dead_reckoning_sec_);
            return;
        }
        if (odom_history_.empty()) return;
        const rclcpp::Time odom_stamp = odom_history_.back().first;
        if (odom_stamp <= last_scan_stamp_) return;  // no new odometry yet
        const auto T_anchor = OdomAt(last_scan_stamp_);
        if (!T_anchor.has_value()) return;
        const Sophus::SE3d &T_now = odom_history_.back().second;
        const Sophus::SE3d delta_odom = T_anchor->inverse() * T_now;
        const Sophus::SE3d T_out = last_out_ * delta_odom;
        if (!T_out.matrix().allFinite()) return;
        const double dt = (odom_stamp - last_scan_stamp_).seconds();
        // Reseed trio (plan §2): output pose, odom anchor (= last_scan_stamp_)
        // and the ICP seed advance TOGETHER. Skipping any of them either
        // double-counts the gap motion in the return frame's prior/prediction,
        // lets the anchor fall out of the 400-entry odom ring buffer, or makes
        // ICP restart from a seconds-old pose when scans return.
        last_out_ = T_out;
        last_scan_stamp_ = odom_stamp;
        icp_->pose() = T_out;  // non-const accessor: frozen map preserved
        last_valid_icp_pose_ = T_out;
        // The pre-gap pending scan predates the advanced anchor: registering
        // it now would apply a backwards odometry prior spanning the whole
        // gap. Drop it — the first post-gap scan re-fills the pipeline.
        pending_scan_.reset();
        dead_reckoning_sec_ = gap;
        SetOutputCovariance(gap);
        PublishOdometry(T_out, odom_stamp, std::make_optional(T_now));
        const double speed = dt > 1e-6 ? delta_odom.translation().norm() / dt : 0.0;
        PublishDiagnostics(odom_stamp, speed, 0.0, 0.0, false, 0.0, true);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Scan gap %.2f s: publishing wheel-odometry dead reckoning", gap);
    }

    // Published pose covariance: base value, inflated while dead reckoning
    // (MCL ekf_*_error_rate style — std grows with the un-corrected span).
    void SetOutputCovariance(double dead_reckoning_sec) {
        const double trans_std_add = watchdog_trans_error_rate_ * dead_reckoning_sec;
        const double yaw_std_add = watchdog_rot_error_rate_ * dead_reckoning_sec;
        odom_msg_.pose.covariance[0] = position_covariance_ + trans_std_add * trans_std_add;
        odom_msg_.pose.covariance[7] = odom_msg_.pose.covariance[0];
        odom_msg_.pose.covariance[35] =
            orientation_covariance_ + yaw_std_add * yaw_std_add;
    }

    // §3: registration quality on ~/diagnostics (diagnostic_msgs, rqt-friendly)
    void PublishDiagnostics(const rclcpp::Time &stamp, double speed, double alpha,
                            double alpha_rot, bool gate_rejected, double gate_d2, bool pose_ok) {
        if (!diag_pub_) return;
        const auto &d = icp_->registrationDiagnostics();
        const double inlier_ratio =
            d.num_source_points > 0
                ? static_cast<double>(d.num_correspondences) / d.num_source_points
                : 0.0;
        diagnostic_msgs::msg::DiagnosticArray array;
        array.header.stamp = stamp;
        diagnostic_msgs::msg::DiagnosticStatus status;
        status.name = std::string(get_name()) + ": registration";
        status.hardware_id = map_name_;
        const bool degraded = inlier_ratio < min_inlier_ratio_warn_ || !d.converged ||
                              gate_rejected || !pose_ok || dead_reckoning_sec_ > 0.0;
        status.level = degraded ? diagnostic_msgs::msg::DiagnosticStatus::WARN
                                : diagnostic_msgs::msg::DiagnosticStatus::OK;
        status.message = dead_reckoning_sec_ > 0.0 ? "dead reckoning (scan gap)"
                         : gate_rejected           ? "gate rejected"
                         : !pose_ok                ? "pose impermissible"
                         : degraded                ? "low inlier ratio / not converged"
                                                   : "ok";
        auto add = [&status](const std::string &key, const std::string &value) {
            diagnostic_msgs::msg::KeyValue kv;
            kv.key = key;
            kv.value = value;
            status.values.push_back(kv);
        };
        char buf[32];
        auto fmt = [&buf](double v) {
            std::snprintf(buf, sizeof(buf), "%.6g", v);
            return std::string(buf);
        };
        add("inlier_ratio", fmt(inlier_ratio));
        add("num_correspondences", std::to_string(d.num_correspondences));
        add("num_source_points", std::to_string(d.num_source_points));
        add("residual_rms", fmt(d.residual_rms));
        add("tau", fmt(d.threshold_tau));
        add("beta", fmt(d.beta));
        add("iterations", std::to_string(d.iterations));
        add("converged", d.converged ? "true" : "false");
        add("final_dx_norm", fmt(d.final_dx_norm));
        add("speed", fmt(speed));
        add("alpha", fmt(alpha));
        add("alpha_rot", fmt(alpha_rot));
        add("dead_reckoning_sec", fmt(dead_reckoning_sec_));
        add("gate_rejected", gate_rejected ? "true" : "false");
        add("gate_d2", fmt(gate_d2));
        add("gate_reject_streak", std::to_string(gate_reject_streak_));
        add("pose_impermissible", pose_ok ? "false" : "true");
        array.status.push_back(status);
        diag_pub_->publish(array);
    }

    void PublishOdometry(const Sophus::SE3d &pose, const rclcpp::Time &stamp,
                         const std::optional<Sophus::SE3d> &T_odom_base) {
        // map -> odom TF, same convention as MCL: T_map_odom = T_map_base * T_odom_base^-1
        if (publish_map_odom_tf_ && T_odom_base.has_value()) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = stamp;
            tf_msg.header.frame_id = map_frame_;
            tf_msg.child_frame_id = odom_frame_;
            tf_msg.transform = utils::SophusToTransform(pose * T_odom_base->inverse());
            tf_broadcaster_->sendTransform(tf_msg);
        }

        // Twist: forward the latest wheel odometry (MCL convention), not a
        // pose-delta velocity — see the odom callback comment.
        odom_msg_.pose.pose = utils::SophusToPose(pose);
        odom_msg_.twist.twist.linear.x = latest_odom_twist_.linear.x;
        odom_msg_.twist.twist.linear.y = 0.0;
        odom_msg_.twist.twist.angular.z = latest_odom_twist_.angular.z;
        odom_msg_.header.stamp = stamp;
        pose_pub_->publish(odom_msg_);
    }

    // Voxel-downsample the accumulated SLAM map (same resolution as
    // mapping_node) and write it as a .kissmap. Returns false when there is
    // nothing to save or the write fails.
    bool SaveMap(size_t *n_saved) {
        if (accumulated_points_.empty()) return false;
        utils::KissMap map;
        map.voxel_size = voxel_size_;
        map.max_range = max_range_;
        map.points = kiss_icp::VoxelDownsample(accumulated_points_, voxel_size_ * 0.5);
        if (!utils::WriteKissMap(map_output_file_, map)) return false;
        if (n_saved) *n_saved = map.points.size();
        return true;
    }

    // Built-in /map server: rasterize map points to an occupancy grid and
    // publish it (transient_local, so late-joining RViz still receives it).
    void PublishOccupancyMap(const std::vector<Eigen::Vector3d> &points) {
        if (points.empty()) return;
        // Cache the grid: the refresh timer re-publishes it and the pose
        // validity check (is_pose_permissible) samples it.
        occ_grid_ = utils::RasterizeOccupancyGrid(points, map_grid_resolution_,
                                                  map_point_dilation_m_, 1.0, map_frame_, now());
        occ_map_pub_->publish(*occ_grid_);
        RCLCPP_INFO_ONCE(get_logger(), "Published occupancy map on %s (%zu points)",
                         map_topic_.c_str(), points.size());
    }

    // RViz visualization of the accumulated SLAM map (downsampled copy).
    void PublishMapPoints(const std::vector<Eigen::Vector3d> &downsampled) {
        if (downsampled.empty()) return;
        sensor_msgs::msg::PointCloud2 cloud;
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(downsampled.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        for (const auto &p : downsampled) {
            *iter_x = static_cast<float>(p.x());
            *iter_y = static_cast<float>(p.y());
            *iter_z = static_cast<float>(p.z());
            ++iter_x;
            ++iter_y;
            ++iter_z;
        }
        cloud.header.frame_id = map_frame_;
        cloud.header.stamp = now();
        map_pub_->publish(cloud);
    }

    std::string lidar_topic_, odom_topic_, pose_topic_, initial_pose_topic_;
    std::string map_frame_, odom_frame_, base_frame_, map_name_;
    // MCL 호환 자동 초기화 상태
    bool auto_init_from_waypoints_ = true;
    std::string auto_init_topic_ = "/global_waypoints";
    bool auto_init_done_ = false;
    bool manual_init_done_ = false;
    bool validating_auto_init_ = false;
    int auto_init_validate_frames_ = 40;
    double auto_init_max_residual_ = 0.35;
    std::vector<double> validate_residuals_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr auto_init_sub_;
    bool publish_map_odom_tf_ = true;
    double voxel_size_ = 1.0, max_range_ = 30.0;

    // Online SLAM mode
    bool slam_mode_ = false;
    std::string map_output_file_;
    double map_publish_period_sec_ = 2.0;
    std::vector<Eigen::Vector3d> accumulated_points_;

    // Output smoothing (complementary filter)
    bool smoothing_enable_ = true;
    double smoothing_alpha_ = 0.2;
    double smoothing_alpha_gain_ = 0.3;
    double smoothing_velocity_full_ = 3.0;
    double smoothing_alpha_max_ = 0.8;
    double smoothing_alpha_rot_ = -1.0;
    Sophus::SE3d last_out_;
    bool has_last_out_ = false;

    // Built-in /map occupancy server
    std::string map_topic_;
    double map_grid_resolution_ = 0.05;
    double map_point_dilation_m_ = 0.15;
    std::optional<nav_msgs::msg::OccupancyGrid> occ_grid_;

    // 단일 스레드 executor 전제(main의 rclcpp::spin, 콜백 그룹 미지정 = 노드
    // 기본 MutuallyExclusive) — 아래 상태는 락 없이 콜백(스캔/odom/initialpose/
    // 워치독·맵 타이머) 간 공유된다. MultiThreadedExecutor나 Reentrant 콜백
    // 그룹으로 바꾸면 last_out_/last_scan_stamp_/odom_history_/pending_scan_/
    // last_valid_icp_pose_/last_scan_processed_time_/gate_reject_streak_에
    // 락이 필요해진다.
    std::unique_ptr<kinematic_icp::pipeline::KinematicICP> icp_;
    std::vector<Eigen::Vector3d> frozen_points_;
    std::optional<Sophus::SE3d> last_valid_icp_pose_;
    bool initialized_ = false;
    std::optional<Sophus::SE3d> lidar_to_base_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr pending_scan_;
    rclcpp::Time last_scan_stamp_{0, 0, RCL_ROS_TIME};
    std::deque<std::pair<rclcpp::Time, Sophus::SE3d>> odom_history_;
    geometry_msgs::msg::Twist latest_odom_twist_{};  // forwarded on /pf/pose/odom

    // §2 watchdog
    bool watchdog_enable_ = true;
    double watchdog_rate_hz_ = 40.0;
    double scan_timeout_sec_ = 0.15;
    double max_dead_reckoning_sec_ = 2.0;
    double watchdog_trans_error_rate_ = 0.03;
    double watchdog_rot_error_rate_ = 0.10;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    rclcpp::Time last_scan_processed_time_{0, 0, RCL_ROS_TIME};
    double dead_reckoning_sec_ = 0.0;

    // §3 diagnostics
    bool diagnostics_enable_ = true;
    std::string diagnostics_topic_;
    double min_inlier_ratio_warn_ = 0.3;
    int low_inlier_frames_ = 0;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

    // §4 pose validity
    bool pose_check_enable_ = true;
    double permissible_radius_m_ = 0.15;

    // §5 Mahalanobis gate
    bool gate_enable_ = false;
    double gate_chi2_ = 9.21;
    double gate_trans_error_rate_ = 0.03;
    double gate_rot_error_rate_ = 0.10;
    double gate_meas_std_floor_ = 0.02;
    int gate_force_accept_ = 20;
    int gate_reject_streak_ = 0;

    double position_covariance_ = 0.1;
    double orientation_covariance_ = 0.1;

    laser_geometry::LaserProjection laser_projector_;
    utils::TimeStampHandler timestamps_handler_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_map_srv_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    nav_msgs::msg::Odometry odom_msg_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace kinematic_localization

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        // Scope the node so its destructor (SLAM map auto-save on Ctrl-C) runs
        // while rclcpp is still initialized.
        auto node = std::make_shared<kinematic_localization::LocalizationNode>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        // Constructor failures (e.g. frozen map missing) end up here with a
        // readable FATAL instead of an unhandled-exception abort.
        RCLCPP_FATAL(rclcpp::get_logger("kinematic_localization"), "%s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
