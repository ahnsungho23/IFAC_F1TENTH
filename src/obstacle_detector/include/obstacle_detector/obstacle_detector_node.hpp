// ================================================================================================
// OBSTACLE DETECTOR NODE - LiDAR 기반 계층형 장애물 검출(Frenet 출력)
// ================================================================================================
// 하나의 LiDAR 스캔을 기준으로 전체 인지 파이프라인을 실행한다. 스캔 반사점을 세 계층으로
// 나누어 처리하고, 그중 장애물에 해당하는 두 계층을 Frenet 좌표계로 발행한다.
//
//   계층 1 [map]      : /scan -> TF2로 map 좌표 변환 -> 적응형 breakpoint 군집화 ->
//                       추적 전 파편 병합 -> AABB 생성 -> 가시 거리·트랙 경계 게이트 ->
//                       /map 점유 필터 순으로 처리한다.
//                       벽이나 이미 알려진 구조물처럼 지도 자체에 속한 점은 여기서 제거한다.
//                       이 계층은 필터로만 사용하며 별도 토픽으로 발행하지 않는다.
//   추적 단계         : 남은 군집을 등속 Kalman 상태 [s, vs, d, vd]로 추적하여 각 군집의
//                       Frenet 흐름 속도를 구한다. 이후 느린 군집의 평균 흐름인 map-flow
//                       기준과 비교해 정적/동적으로 분류한다(obstacle_tracker.hpp 참고).
//   계층 2 [static]   : map-flow와 비슷하게 움직이는 군집(|flow - ref|가 작음)을 정적
//                       장애물로 보고 `static_obs_topic`(/static_obs)에
//                       f110_msgs/ObstacleArray로 발행한다.
//   계층 3 [dynamic]  : map-flow에서 명확히 벗어나는 군집을 동적 상대 차량으로 보고,
//                       그중 차량 전방에 가장 가까운 하나를 `opp_obs_topic`(/opp_obs)에
//                       단일 원소 f110_msgs/ObstacleArray로 발행한다.
//   계층 내 병합      : 발행 전 같은 계층의 Frenet 상자 사이 간격이
//                       layer_merge_gap_s/_d 이내이면 하나의 물체로 병합한다. 가림이나
//                       모서리 반사로 물체 하나가 여러 항목이 되는 것을 막기 위함이다.
//                       현재 스캔에서 관측된 멤버의 Cartesian AABB만 합집합에 포함하고,
//                       예측만 남은 출력은 오래된 상자를 쓰지 않고 Frenet 형상만 유지한다.
//
// 두 출력 토픽은 해당 계층이 비었을 때도 빈 배열로 매 스캔마다 발행한다. 따라서 하위 노드는
// 장애물 유무와 무관하게 일정한 스캔 주기의 인지 갱신을 받을 수 있다.
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
    // map 좌표의 스캔 점과 breakpoint 임계값 계산에 필요한 원본 거리
    struct ScanPoint
    {
        double x;
        double y;
        double range;
    };

    // 같은 계층의 Frenet 외곽과 현재 보이는 Cartesian AABB 합집합으로 만든 물체 단위 출력
    struct MergedObstacle
    {
        f110_msgs::msg::Obstacle ob;
    };

    // 스캔 한 회의 단계별 통계이다. diagnostics_period_sec 동안 누적한 뒤 로그로 출력한다.
    // 이 노드는 노이즈 제거와 deskew를 수행하지 않으므로 관련 필드는 두지 않는다. 해당 통계는
    // 그 작업을 실제로 수행하는 상위 전처리 노드가 보고해야 한다.
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

    // ROS 입력 콜백
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void globalWpntsCallback(const f110_msgs::msg::WpntArray::SharedPtr msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void egoOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // 인지 파이프라인 보조 함수
    bool lookupScanToMap(const std_msgs::msg::Header &scan_header, double &tx, double &ty,
                         double &yaw);
    std::vector<std::vector<ScanPoint>> clusterScan(const sensor_msgs::msg::LaserScan &scan,
                                                    double tx, double ty, double yaw,
                                                    ScanProcessingStats &stats) const;
    // Detection/Track을 만들기 전에 가까운 스캔 파편을 다시 합친다. AABB 간격은 빠른 후보
    // 검사에만 사용하며, 실제 점 쌍 하나 이상도 설정 거리 안에 있어야 한다. 합친 AABB 크기는
    // max_obs_size를 넘을 수 없다.
    std::vector<std::vector<ScanPoint>> mergeClusters(
        std::vector<std::vector<ScanPoint>> clusters, ScanProcessingStats &stats) const;
    bool occupiedInMap(double x, double y) const;
    // 한 계층 안에서 수행하는 2차 군집화이다. s의 폐루프 경계를 고려하면서 상자 간격이 병합
    // 기준 이내인 트랙을 연결하고, 연결 요소마다 하나의 외곽 장애물을 만든다.
    // layer_merge_enable이 false이면 모든 트랙을 개별 물체로 유지한다.
    std::vector<MergedObstacle> mergeLayer(const std::vector<const Track *> &members,
                                           bool is_static_layer) const;
    // 병합된 동적 물체 중 자차 전방에서 가장 가까운 상대를 고른다. ego_s_를 아직 모르면 위치
    // 불확실성이 가장 작은 물체를 대신 고르며, 동적 계층이 비었으면 -1을 반환한다.
    int selectOpponent(const std::vector<MergedObstacle> &dynamic_objs) const;
    void updateDiagnostics(const ScanProcessingStats &scan_stats,
                           const TrackerUpdateStats *tracker_stats,
                           double measurement_yaw_rate, bool yaw_rate_fresh);

    void declareParameters();
    void loadParameters();

    // 토픽·좌표계 파라미터
    std::string scan_topic_;
    std::string global_wpnts_topic_;
    std::string map_topic_;
    std::string ego_odom_topic_;
    std::string static_obs_topic_;   // 계층 2 정적 장애물 출력(/static_obs)
    std::string opp_obs_topic_;      // 계층 3 동적 상대 차량 출력(/opp_obs)
    std::string static_markers_topic_;
    std::string opp_markers_topic_;
    std::string map_frame_;

    double max_range_;
    // 적응형 breakpoint 군집화
    double lambda_rad_;
    double cluster_sigma_;
    double min_2_points_dist_;
    int min_cluster_points_;
    double max_obs_size_;
    // 추적 전 스캔 파편 병합
    bool cluster_merge_enable_;
    double cluster_merge_distance_;
    int cluster_merge_min_fragment_points_;
    // 검출 품질에 따라 조절하는 Kalman 측정 불확실성
    double meas_range_var_scale_;
    double meas_sparse_var_scale_;
    double meas_yaw_rate_var_scale_;
    int meas_reference_points_;
    double meas_variance_scale_max_;
    double meas_motion_timeout_;
    // 계층 1 지도·주행 영역 필터
    double max_viewing_distance_;
    double view_behind_distance_;
    double boundaries_inflation_;
    double fallback_track_halfwidth_;
    bool use_map_filter_;
    int map_occupied_thresh_;
    int map_inflation_cells_;
    double map_point_reject_ratio_;
    // 계층별 2차 물체 병합
    bool layer_merge_enable_;
    double layer_merge_gap_s_;
    double layer_merge_gap_d_;
    // 출력과 진단
    bool publish_markers_;
    bool diagnostics_enable_;
    double diagnostics_period_sec_;

    TrackerParams tracker_params_;

    // 실행 상태
    // CLCS는 정확한 (x, y) -> (s, d) 투영을 담당한다. FrenetProjector는 트랙 경계 조회,
    // 폐루프 s 연산, 최종 Frenet 외곽 시각화를 위한 map 좌표 보간만 담당한다.
    global_planning::ClcsFrenetConverter::Ptr converter_;
    std::uint64_t clcs_version_{0};
    FrenetProjector frenet_;
    ObstacleTracker tracker_;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    double ego_s_{-1.0};   // 자차 호 길이이며, 첫 투영 전의 음수 값은 전방 우선 선택을 끈다.
    double odom_yaw_rate_{0.0};
    double odom_motion_stamp_{-1.0};
    ScanProcessingStats diagnostics_scan_totals_;
    TrackerUpdateStats diagnostics_tracker_event_totals_;
    std::chrono::steady_clock::time_point diagnostics_window_start_;
    bool diagnostics_window_started_{false};

    // ROS 구독·발행 및 TF 인터페이스
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr global_wpnts_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odom_sub_;

    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr static_obs_pub_;  // 정적 계층
    rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr opp_obs_pub_;     // 동적 계층
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr static_markers_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr opp_markers_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace obstacle_detector

#endif  // OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_NODE_HPP_
