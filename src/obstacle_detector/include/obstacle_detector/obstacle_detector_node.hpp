// ================================================================================================
// OBSTACLE DETECTOR NODE - LiDAR-only layered obstacle detection (Frenet output)
// ================================================================================================
// A single scan-driven perception node that separates LiDAR returns into three layers and
// publishes the two obstacle layers in the Frenet frame:
//
//   LAYER 1 [map]     : /scan -> map-frame points (TF2) -> adaptive-breakpoint clustering ->
//                       pre-tracking fragment merge -> box fit -> viewing + track-boundary gates
//                       -> /map occupancy filter.
//                       Points that ARE the map (walls / known static structure) are removed here;
//                       the map layer is a filter, it is not published.
//   (tracking)        : the surviving clusters are tracked with a constant-velocity Kalman filter
//                       [s, vs, d, vd], which gives each cluster its Frenet flow velocity. The flow
//                       is classified against the "map-flow" reference (mean flow of the slow
//                       clusters) — see obstacle_tracker.hpp.
//   LAYER 2 [static]  : clusters that flow WITH the map (|flow - ref| small) -> STATIC obstacles.
//                       Published as an f110_msgs/ObstacleArray on `static_obs_topic` (/static_obs).
//   LAYER 3 [dynamic] : the one cluster that clearly flows slower than the map (the same-direction
//                       opponent) -> the DYNAMIC opponent. Published as an f110_msgs/ObstacleArray
//                       with a single element on `opp_obs_topic` (/opp_obs).
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
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include "global_planning/clcs_frenet_converter.hpp"

#include "obstacle_detector/frenet_projector.hpp"
#include "obstacle_detector/obstacle_tracker.hpp"

namespace obstacle_detector
{

class ObstacleDetectorNode : public rclcpp::Node
{
  public:
    explicit ObstacleDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

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

    // ---- pipeline helpers ----
    bool lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx, double &ty,
                         double &yaw);
    std::vector<std::vector<ScanPoint>> clusterScan(const sensor_msgs::msg::LaserScan &scan,
                                                    double tx, double ty, double yaw,
                                                    ScanProcessingStats &stats) const;
    // Rejoin small scan fragments before one Detection/Track is created. The AABB gap is only a
    // broad-phase check: at least one real point pair must also be within the configured distance,
    // and the merged AABB must remain no larger than max_obs_size.
    std::vector<std::vector<ScanPoint>> mergeClusters(
        std::vector<std::vector<ScanPoint>> clusters, ScanProcessingStats &stats) const;
    bool occupiedInMap(double x, double y) const;
    // 2nd-stage clustering inside one layer: union tracks whose boxes are within the merge gaps
    // (wrap-aware in s) and emit one envelope obstacle per component. With layer_merge_enable
    // false every track stays a singleton (identical to per-track publishing).
    std::vector<MergedObstacle> mergeLayer(const std::vector<const Track *> &members,
                                           bool is_static_layer) const;
    // The opponent among the merged dynamic objects: nearest ahead of ego (ego_s_ < 0 falls back
    // to lowest positional uncertainty). Returns -1 when the layer is empty.
    int selectOpponent(const std::vector<MergedObstacle> &dynamic_objs) const;
    void updateDiagnostics(const ScanProcessingStats &scan_stats,
                           const TrackerUpdateStats *tracker_stats,
                           double measurement_yaw_rate, bool yaw_rate_fresh);

    void declareParameters();
    void loadParameters();

    // ---- parameters ----
    std::string scan_topic_;
    std::string global_wpnts_topic_;
    std::string map_topic_;
    std::string ego_odom_topic_;
    std::string static_obs_topic_;   // Layer 2 output (/static_obs)
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
    // Layer-1 filtering
    double max_viewing_distance_;
    double view_behind_distance_;
    double boundaries_inflation_;
    double fallback_track_halfwidth_;
    bool use_map_filter_;
    int map_occupied_thresh_;
    int map_inflation_cells_;
    double map_point_reject_ratio_;
    // per-layer 2nd-stage merge
    bool layer_merge_enable_;
    double layer_merge_gap_s_;
    double layer_merge_gap_d_;
    // output
    bool publish_markers_;
    bool diagnostics_enable_;
    double diagnostics_period_sec_;

    TrackerParams tracker_params_;

    // ---- state ----
    // CLCS does the accurate (x,y) -> (s,d) projection. FrenetProjector provides track-boundary
    // lookup, s-wrap, and map interpolation for visualization of the final Frenet envelopes.
    global_planning::ClcsFrenetConverter::Ptr converter_;
    std::uint64_t clcs_version_{0};
    FrenetProjector frenet_;
    ObstacleTracker tracker_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    double ego_s_{-1.0};   // ego arc-length; < 0 disables the ahead-preference until first proj
    double ego_s_stamp_{-1.0};  // odometry stamp of the last ego_s_ update (freshness check)
    double odom_yaw_rate_{0.0};
    double odom_motion_stamp_{-1.0};
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
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr opp_obs_pub_;     // Layer 3
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr opp_markers_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
