// ================================================================================================
// OBSTACLE DETECTOR NODE - LiDAR-only layered obstacle detection (Frenet output)
// ================================================================================================
// A single scan-driven perception node that separates LiDAR returns into three layers and
// publishes the two obstacle layers in the Frenet frame:
//
//   LAYER 1 [map]     : /scan -> map-frame points (TF2) -> structural wall filter (per-beam:
//                       points near LINEAR wall components extracted from /map are dropped
//                       before clustering) -> adaptive-breakpoint clustering -> pre-tracking
//                       fragment merge -> box fit -> viewing + track-boundary gates.
//                       Points that ARE the map (walls / known static structure) are removed here;
//                       the map layer is a filter, it is not published. (2026-08-13: the per-cell
//                       occupancy vote was replaced by WallDistanceFilter — the SLAM map and the
//                       scan diverge on the real car, which both erased real obstacles on mapped
//                       cells and kept wall returns slightly off the mapped wall.)
//   (tracking)        : Frenet KF [s,vs,d,vd] preserves association/output geometry. A parallel
//                       map KF [x,vx,y,vy] provides velocity covariance significance and measured
//                       map-position persistence for motion classification.
//   LAYER 2 [static]  : existence-confirmed UNKNOWN or STATIC objects. Published as an
//                       f110_msgs/ObstacleArray on `static_obs_topic` (/static_obs).
//   LAYER 3 [dynamic] : the nearest-ahead existence-confirmed DYNAMIC opponent. Published as an
//                       f110_msgs/ObstacleArray with a single element on `opp_obs_topic` (/opp_obs).
//   (layer merge)     : before publishing, tracks inside the SAME layer whose Frenet boxes are
//                       within layer_merge_gap_s/_d of each other are merged into ONE object-level
//                       obstacle (occlusion/corner fragments of one physical object otherwise show
//                       up as several array entries). The opponent is selected among the MERGED
//                       dynamic objects (nearest ahead of ego). Currently visible member AABBs are
//                       unioned into the Cartesian output; predicted-only outputs remain Frenet-only.
//
// Both layer topics are published EVERY scan (empty when a layer has no obstacle) so downstream
// consumers receive a deterministic scan-rate perception update.
// ================================================================================================

#ifndef OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
#define OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include "global_planning/clcs_frenet_converter.hpp"

#include "obstacle_detector/frenet_projector.hpp"
#include "obstacle_detector/obstacle_tracker.hpp"
#include "obstacle_detector/wall_distance_filter.hpp"

namespace obstacle_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
  public:
    explicit ObstacleDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    // "이 held 봉투가 통째로 스캐너 FOV 밖(후방 사각)인가". 마지막 실측 map AABB 의 네
    // 꼭짓점을 스캔 프레임으로 옮겨 bearing 을 재고, **넷 다** [angle_min, angle_max] 밖일
    // 때만 true 다. 하나라도 보이면 차폐일 수 있으므로 hold 를 그대로 둔다.
    //
    // 🔑 FOV 는 상수로 두지 않고 스캔 헤더의 angle_min/angle_max 를 그대로 쓴다 — 라이다를
    //    바꾸면 판정도 따라간다(이번 실차는 ±135.0°, 1081 빔). 360° 스캐너는 사각이 없으므로
    //    항상 false 를 낸다.
    // ⚠️ tx/ty/yaw 는 map 기준 **스캔 프레임 원점**이다(lookupScanToMap 이 map->scan 을
    //    조회한다). base_link 가 아니므로 라이다 전방 오프셋을 따로 더하지 말 것.
    // 순수 기하라 노드 상태에 의존하지 않는다 — static 으로 두어 단위 시험이 노드를 띄우지
    // 않고 직접 호출한다.
    static bool envelopeInRearBlindCone(const Track &track,
                                        const sensor_msgs::msg::LaserScan &scan,
                                        double tx, double ty, double yaw, double margin_rad);

  private:
    // ---- a Cartesian scan point (map frame) plus its raw range for the breakpoint threshold ----
    struct ScanPoint
    {
        double x;
        double y;
        double range;
    };

    // ---- one object-level obstacle: same-layer Frenet envelope + visible Cartesian AABB union ----
    struct MergedObstacle
    {
        f110_msgs::msg::Obstacle ob;
    };

    // Per-scan pipeline counters. The node sums these over diagnostics_period_sec before logging.
    // No scan-noise or deskew field exists here because those operations are not performed by this
    // node; an upstream preprocessing node must report its own filtering statistics.
    struct ScanProcessingStats
    {
        std::size_t scans_received{0};
        std::size_t scans_processed{0};
        std::size_t clcs_unavailable{0};
        std::size_t tf_unavailable{0};

        std::size_t total_beams{0};
        std::size_t valid_beams{0};
        std::size_t nonfinite_rejected{0};
        std::size_t below_min_range_rejected{0};
        std::size_t at_or_above_max_range_rejected{0};

        std::size_t clusters_before_merge{0};
        std::size_t clusters_after_merge{0};
        std::size_t fragments_rejected{0};
        std::size_t size_rejected{0};
        std::size_t projection_rejected{0};
        std::size_t viewing_window_rejected{0};
        std::size_t track_boundary_rejected{0};
        std::size_t map_rejected{0};
        std::size_t detections{0};
    };

    // ---- callbacks ----
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void applyEgoOdometry(const nav_msgs::msg::Odometry & msg);

    // ---- pipeline helpers ----
    bool lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx, double &ty,
                         double &yaw);
    std::vector<std::vector<ScanPoint>> clusterScan(const sensor_msgs::msg::LaserScan &scan,
                                                    double tx, double ty, double yaw,
                                                    ScanProcessingStats &stats) const;
    // Free-space refutation of one held track's last measured map AABB against the current scan.
    // A beam refutes only when it traverses the box's shrunk core and returns from something at
    // least `static_hold_freespace_refute_margin_m` beyond the far face: that is a return from
    // BEHIND the box, which is possible only if the box's space is empty. A beam that stops short
    // (the object itself, or whatever occludes it) proves nothing, and no-return beams are never
    // counted because a dark or out-of-range surface produces the same reading. Requires
    // `static_hold_freespace_refute_min_beams` such beams so a single edge grazing cannot refute.
    bool scanRefutesHeldEnvelope(const Track &track, const sensor_msgs::msg::LaserScan &scan,
                                 double tx, double ty, double yaw) const;
    // Rejoin small scan fragments before one Detection/Track is created. The AABB gap is only a
    // broad-phase check: at least one real point pair must also be within the configured distance,
    // and the merged AABB must remain no larger than max_obs_size.
    std::vector<std::vector<ScanPoint>> mergeClusters(
        std::vector<std::vector<ScanPoint>> clusters, ScanProcessingStats &stats) const;
    // 2nd-stage clustering inside one layer: union tracks whose boxes are within the merge gaps
    // (wrap-aware in s) and emit one envelope obstacle per component. With layer_merge_enable
    // false every track stays a singleton (identical to per-track publishing).
    std::vector<MergedObstacle> mergeLayer(const std::vector<const Track *> &members,
                                           bool is_static_layer) const;
    // The opponent among the merged dynamic objects: nearest ahead of ego (ego_s_ < 0 falls back
    // to lowest positional uncertainty). Returns -1 when the layer is empty.
    int selectOpponent(const std::vector<MergedObstacle> &dynamic_objs) const;
    // Check whether the selected opponent overlaps the ego corridor and is within the current or
    // constant-velocity predicted longitudinal safety distance.
    bool isOpponentInterfering(const f110_msgs::msg::Obstacle &opponent);
    void updateDiagnostics(const ScanProcessingStats &scan_stats,
                           const TrackerUpdateStats *tracker_stats,
                           double measurement_yaw_rate, bool yaw_rate_fresh);
    void publishReplayDiagnostics(
        const std_msgs::msg::Header &scan_header,
        const std::vector<Detection> &detections,
        const std::vector<MergedObstacle> &static_objects,
        const std::vector<MergedObstacle> &confirmed_static_objects,
        double measurement_yaw_rate, bool yaw_rate_fresh);
    void logMotionDebug();

    void declareParameters();
    void loadParameters();

    // ---- parameters ----
    std::string scan_topic_;
    std::string global_wpnts_topic_;
    std::string map_topic_;
    std::string ego_odom_topic_;
    std::string static_obs_topic_;   // Layer 2 output (/static_obs)
    std::string confirmed_static_obs_topic_;  // confirmed-only Layer 2 output
    std::string opp_obs_topic_;      // Layer 3 output (/opp_obs)
    std::string static_markers_topic_;
    std::string opp_markers_topic_;
    std::string map_frame_;

    double max_range_;
    // clustering
    double lambda_rad_;
    double cluster_sigma_;
    double min_2_points_dist_;
    int min_cluster_points_;
    double max_obs_size_;
    // pre-tracking scan-fragment merge
    bool cluster_merge_enable_;
    double cluster_merge_distance_;
    int cluster_merge_min_fragment_points_;
    // detection-specific Kalman measurement uncertainty
    double meas_range_var_scale_;
    double meas_sparse_var_scale_;
    double meas_yaw_rate_var_scale_;
    int meas_reference_points_;
    double meas_variance_scale_max_;
    double meas_motion_timeout_;
    // Max age accepted from the latest-TF fallback in lookupScanToMap [s]. <=0 disables the
    // fallback entirely (scan-stamp lookup only).
    double tf_fallback_max_age_sec_;
    // Layer-1 filtering
    double max_viewing_distance_;
    double view_behind_distance_;
    double boundaries_inflation_;
    double fallback_track_halfwidth_;
    bool use_map_filter_;
    int map_occupied_thresh_;
    // 구조적 벽 필터(WallDistanceFilter) 파라미터 — 팀 edge_test 계열 포팅 (2026-08-13)
    double wall_assoc_distance_m_;
    double wall_min_length_m_;
    // per-layer 2nd-stage merge
    bool layer_merge_enable_;
    double layer_merge_gap_s_;
    double layer_merge_gap_d_;
    // output
    bool publish_markers_;
    bool diagnostics_enable_;
    double diagnostics_period_sec_;
    bool interference_check_enable_;
    double interference_distance_m_;
    double interference_distance_margin_ratio_;
    double interference_time_horizon_sec_;
    double interference_min_closing_speed_mps_;
    double interference_lateral_margin_m_;
    double interference_ego_half_width_m_;
    double interference_ego_front_offset_m_;
    // Withhold prediction-only static tracks. The tracker may retain them for ID continuity, but
    // they are not authoritative current obstacle geometry for local planning.
    bool static_publish_requires_visible_{true};
    bool freespace_refute_enable_{true};
    int freespace_refute_min_beams_{3};
    double freespace_refute_margin_m_{0.15};
    double freespace_refute_box_shrink_m_{0.05};
    bool rear_blind_retire_enable_{true};
    double rear_blind_retire_margin_rad_{0.0349};   // 2.0 deg
    bool motion_debug_enable_;
    double motion_debug_period_sec_;
    bool replay_diagnostics_enable_;
    std::string replay_diagnostics_topic_;
    bool lockstep_mode_{false};
    double lockstep_scan_offset_x_m_{0.275};

    TrackerParams tracker_params_;

    // ---- state ----
    // CLCS does the accurate (x,y) -> (s,d) projection. FrenetProjector provides track-boundary
    // lookup, s-wrap, and map interpolation for visualization of the final Frenet envelopes.
    global_planning::ClcsFrenetConverter::Ptr converter_;
    std::uint64_t clcs_version_{0};
    // Geometry that produced the active CLCS. The global planner republishes the same latched
    // route periodically; identical messages must not reset tracks or physical-ID memory.
    std::vector<FrenetProjector::Waypoint> active_reference_waypoints_;
    FrenetProjector frenet_;
    ObstacleTracker tracker_;
    // 맵에서 선형 벽 성분을 추출해 빔 단위 Layer-1 판정을 O(1)로 제공 (맵 수신 시 1회 빌드)
    WallDistanceFilter wall_filter_;
    double ego_s_{-1.0};   // ego arc-length; < 0 disables the ahead-preference until first proj
    double ego_d_{0.0};
    double ego_vs_{0.0};
    double ego_s_stamp_{-1.0};  // odometry stamp of the last ego_s_ update (freshness check)
    bool opponent_interference_latched_{false};
    int opponent_interference_id_{-1};
    global_planning::ClcsContinuityState ego_continuity_;
    double odom_yaw_rate_{0.0};
    double odom_motion_stamp_{-1.0};
    // Ego longitudinal-acceleration transient tracking (odometry twist finite difference,
    // exponentially smoothed). While |accel| exceeds the suppress threshold, dynamic motion
    // votes in the tracker are withheld for dynamic_vote_suppress_hold_sec after the spike.
    double dynamic_vote_ego_accel_suppress_mps2_{2.0};
    double dynamic_vote_suppress_hold_sec_{0.5};
    double ego_accel_smoothing_sec_{0.15};
    double ego_last_speed_{std::numeric_limits<double>::quiet_NaN()};
    double ego_last_speed_stamp_{-1.0};
    double ego_accel_smoothed_{0.0};
    double ego_motion_transient_until_{-1.0};
    nav_msgs::msg::Odometry::SharedPtr lockstep_odom_msg_;
    sensor_msgs::msg::LaserScan::SharedPtr lockstep_pending_scan_;
    ScanProcessingStats diagnostics_scan_totals_;
    TrackerUpdateStats diagnostics_tracker_event_totals_;
    std::chrono::steady_clock::time_point diagnostics_window_start_;
    bool diagnostics_window_started_{false};

    // ---- ROS interfaces ----
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;

    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr static_obs_pub_;  // Layer 2
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr
        confirmed_static_obs_pub_;  // confirmed-only Layer 2
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr opp_obs_pub_;     // Layer 3
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr opp_markers_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr replay_diagnostics_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
