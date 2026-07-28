// ================================================================================================
// OPPONENT DETECTOR NODE - LiDAR obstacle detection & tracking with optional motion compensation
// ================================================================================================
// Pipeline: /scan + IMU/odom deskew -> adaptive-breakpoint clustering + fragment merge ->
// map-frame points (TF2) -> map + track-boundary filter -> Frenet projection ->
// covariance-aware association + CV Kalman tracking -> velocity-based static/dynamic
// classification -> publish f110_msgs ObstacleArray + ProjOppTraj.
// ================================================================================================

#ifndef OPPONENT_DETECTOR__OPPONENT_DETECTOR_NODE_HPP_
#define OPPONENT_DETECTOR__OPPONENT_DETECTOR_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/proj_opp_traj.hpp>
#include <f110_msgs/msg/proj_opp_point.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <f110_msgs/msg/ot_wpnt_array.hpp>

#include "global_planning/clcs_frenet_converter.hpp"

#include "opponent_detector/frenet_projector.hpp"
#include "opponent_detector/obstacle_tracker.hpp"
#include "opponent_detector/overtake_planner.hpp"

namespace opponent_detector
{

class OpponentDetectorNode : public rclcpp::Node
{
  public:
    explicit OpponentDetectorNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    // ---- a Cartesian scan point (map frame) plus its raw range for the breakpoint threshold ----
    struct ScanPoint
    {
        double x;
        double y;
        double range;
    };

    struct DeskewMotion
    {
        bool enabled{false};
        double yaw_rate{0.0};
        double linear_x{0.0};
        double linear_y{0.0};
        const char *source{"off"};
    };

    struct ScanProcessingStats
    {
        std::size_t valid_beams{0};
        std::size_t noise_rejected{0};
        std::size_t clusters_before_merge{0};
        std::size_t clusters_after_merge{0};
        std::size_t clusters_size_rejected{0};
        std::size_t clusters_projection_rejected{0};
        std::size_t clusters_corridor_rejected{0};
        std::size_t clusters_map_rejected{0};
    };

    // ---- callbacks ----
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

    // ---- pipeline helpers ----
    bool lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx, double &ty,
                         double &yaw);
    std::vector<std::vector<ScanPoint>> clusterScan(const sensor_msgs::msg::LaserScan &scan,
                                                    double tx, double ty, double yaw,
                                                    const DeskewMotion &motion,
                                                    ScanProcessingStats &stats) const;
    std::vector<std::vector<ScanPoint>> mergeClusters(
        std::vector<std::vector<ScanPoint>> clusters, ScanProcessingStats &stats) const;
    DeskewMotion selectDeskewMotion(double scan_stamp) const;
    bool occupiedInMap(double x, double y) const;
    bool pathPointCollides(double x, double y) const;  // occupiedInMap + ot_map_clearance ring

    void declareParameters();
    void loadParameters();

    // ---- parameters ----
    std::string scan_topic_;
    std::string global_wpnts_topic_;
    std::string map_topic_;
    std::string ego_odom_topic_;
    std::string imu_topic_;
    std::string obstacles_topic_;
    std::string static_obstacles_topic_;
    std::string raw_obstacles_topic_;
    std::string proj_opp_topic_;
    std::string markers_topic_;
    std::string ot_topic_;
    std::string avoid_path_topic_;
    std::string opp_path_topic_;
    std::string map_frame_;

    double max_range_;
    // clustering
    double lambda_rad_;
    double cluster_sigma_;
    double min_2_points_dist_;
    int min_cluster_points_;
    double max_obs_size_;
    // scan motion compensation
    bool deskew_enable_;
    std::string deskew_source_;
    bool deskew_translation_enable_;
    double deskew_sensor_timeout_;
    double imu_angular_scale_;
    // optional raw scan noise filter
    bool noise_filter_enable_;
    double noise_eps_;
    double noise_eps_scale_;
    int noise_min_neighbors_;
    int noise_search_halfwidth_;
    int scan_median_window_;
    // fragmented-cluster merge
    bool cluster_merge_enable_;
    double cluster_merge_distance_;
    int cluster_merge_min_fragment_points_;
    // adaptive measurement covariance
    double meas_range_var_scale_;
    double meas_sparse_var_scale_;
    double meas_yaw_rate_var_scale_;
    int meas_reference_points_;
    // filtering
    double max_viewing_distance_;
    double view_behind_distance_;
    double boundaries_inflation_;
    bool use_map_filter_;
    int map_occupied_thresh_;
    int map_inflation_cells_;
    double map_point_reject_ratio_;
    // output
    bool publish_raw_;
    bool publish_markers_;
    int proj_traj_max_points_;
    bool simulator_;

    TrackerParams tracker_params_;
    double ot_map_clearance_{0.20};  // free-space ring radius around each overtake-path sample [m]

    // ---- state ----
    // CLCS-based converter does the accurate (x,y) -> (s,d) projection; the lightweight
    // FrenetProjector is kept only for track-boundary lookup (d_left/d_right) and s-wrap, which
    // CLCS does not provide.
    global_planning::ClcsFrenetConverter::Ptr converter_;
    std::uint64_t clcs_version_{0};
    FrenetProjector frenet_;
    ObstacleTracker tracker_;
    // merged detect+overtake: the state-machine planner turns the detected opponent into a
    // committed OTWpntArray overtaking line (or stays silent)
    OvertakePlanner planner_;
    OvertakeParams ot_params_;
    f110_msgs::msg::WpntArray global_wpnts_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    bool have_ego_pose_{false};
    double ego_x_{0.0};
    double ego_y_{0.0};
    double ego_s_{-1.0};
    double ego_d_{0.0};
    double ego_v_{0.0};
    double odom_linear_x_{0.0};
    double odom_linear_y_{0.0};
    double odom_yaw_rate_{0.0};
    double odom_motion_stamp_{-1.0};
    double imu_yaw_rate_{0.0};
    double imu_stamp_{-1.0};
    // stamp (odom header, sec) of the last SUCCESSFUL ego CLCS projection. A failed projection
    // (ego outside the domain, e.g. pushed off-track by a crash) must not silently reuse the
    // stale (s,d) forever — the planner gets ego.s = -1 once this is older than ego_pose_grace_s.
    double ego_frenet_stamp_{-1.0};
    double ego_pose_grace_s_{0.3};
    std::vector<f110_msgs::msg::ProjOppPoint> proj_buffer_;
    double proj_lapcount_{0.0};

    // ---- ROS interfaces ----
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacles_pub_;
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr static_obstacles_pub_;
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr raw_obstacles_pub_;
    rclcpp::Publisher<f110_msgs::msg::ProjOppTraj>::SharedPtr proj_opp_pub_;
    rclcpp::Publisher<f110_msgs::msg::OTWpntArray>::SharedPtr ot_pub_;
    // RViz-native nav_msgs/Path mirrors (custom msgs are not directly viewable in RViz)
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr avoid_path_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opp_path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace opponent_detector

#endif  // OPPONENT_DETECTOR__OPPONENT_DETECTOR_NODE_HPP_
