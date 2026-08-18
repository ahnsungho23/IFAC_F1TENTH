// ================================================================================================
// PARTICLE FILTER HEADER - Monte Carlo Localization (MCL) Class Definition
// ================================================================================================
// Features: Multinomial resampling, velocity motion model, beam sensor model, ray casting
// ================================================================================================

#ifndef PARTICLE_FILTER_CPP__PARTICLE_FILTER_HPP_
#define PARTICLE_FILTER_CPP__PARTICLE_FILTER_HPP_

#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/srv/get_map.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <Eigen/Dense>
#include <memory>
#include <mutex>
#include <random>
#include <vector>

#include "particle_filter_cpp/utils.hpp"

namespace particle_filter_cpp
{

// Motion command structure for optimized motion model
struct MotionCommand
{
    double velocity;          // Linear velocity (m/s)
    double angular_velocity;  // Angular velocity (rad/s)
    double dt;               // Time interval (s)
    
    // Default constructor
    MotionCommand() : velocity(0.0), angular_velocity(0.0), dt(0.0) {}
    
    // Constructor with values
    MotionCommand(double v, double w, double time) : velocity(v), angular_velocity(w), dt(time) {}
    
    // Constructor from legacy action vector (displacement-based)
    static MotionCommand from_displacement(const Eigen::Vector3d& action, double time_interval)
    {
        double v = (std::abs(action[0]) > 0.001) ? action[0] / time_interval : 0.0;
        double w = (std::abs(action[2]) > 0.001) ? action[2] / time_interval : 0.0;
        return MotionCommand(v, w, time_interval);
    }
};

class ParticleFilter : public rclcpp::Node
{
  public:
    explicit ParticleFilter(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  private:
    // --------------------------------- CORE MCL ALGORITHM ---------------------------------
    void MCL(const MotionCommand &motion_cmd, const std::vector<float> &observation);
    void motion_model(Eigen::MatrixXd &proposal_dist, const MotionCommand &motion_cmd);
    void sensor_model(const Eigen::MatrixXd &proposal_dist, const std::vector<float> &obs,
                      std::vector<double> &weights);

    // Sensor model helper functions
    void initialize_sensor_arrays(int num_rays, int total_queries);
    void generate_ray_queries(const Eigen::MatrixXd &proposal_dist, int num_rays);
    void calculate_particle_weights(const std::vector<float> &obs, int num_rays,
                                   std::vector<double> &weights);

    // weights 인자를 명시로 받는 버전 — 리샘플 여부와 무관하게 "센서로 실제 informed된"
    // 가중치로 계산하고 싶을 때 쓴다(예: 리샘플 직전 캐시). 무인자 오버로드는 현재 멤버
    // weights_ 기준(하위 호환용, 리샘플 직후 호출하면 균등가중치라 무의미해진다 — 아래
    // pre_resample_pose_ 캐시를 우선 쓸 것).
    Eigen::Vector3d expected_pose(const std::vector<double> &weights);
    Eigen::Vector3d expected_pose();
    Eigen::Vector3d smooth_pose(const Eigen::Vector3d &raw_pose);

    // --------------------------------- INITIALIZATION ---------------------------------
    void initialize_global();
    void initialize_particles_pose(const Eigen::Vector3d &pose);
    void precompute_sensor_model();

    // --------------------------------- ROS2 CALLBACKS ---------------------------------
    void lidarCB(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void odomCB(const nav_msgs::msg::Odometry::SharedPtr msg);
    void clicked_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void clicked_point(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void waypointsCB(const f110_msgs::msg::WpntArray::ConstSharedPtr msg);

    // --------------------------------- MAP MANAGEMENT ---------------------------------
    void get_omap();

    // --------------------------------- OUTPUT & VISUALIZATION ---------------------------------
    void publish_tf(const Eigen::Vector3d &pose, const rclcpp::Time &stamp);
    void visualize(const rclcpp::Time &stamp = rclcpp::Time(0));
    void publish_particles(const Eigen::MatrixXd &particles_to_pub, const rclcpp::Time &stamp = rclcpp::Time(0));
    
    // --------------------------------- POSE MANAGEMENT ---------------------------------
    Eigen::Vector3d get_current_pose();
    bool is_pose_valid(const Eigen::Vector3d& pose);

    // --------------------------------- TF UTILITIES ---------------------------------
    Eigen::Vector3d apply_tf_offset(const Eigen::Vector3d& pose_in_laser_frame);
    
    // --------------------------------- ODOMETRY-BASED TRACKING ---------------------------------
    void initialize_odom_tracking(const Eigen::Vector3d& initial_pose, bool from_rviz = true);
    void update_odom_pose(const nav_msgs::msg::Odometry::SharedPtr& msg);


    // --------------------------------- RAY CASTING ---------------------------------
    std::vector<float> calc_range_many(const Eigen::MatrixXd &queries);
    float cast_ray(double x, double y, double angle);

    // --------------------------------- ALGORITHM PARAMETERS ---------------------------------
    int ANGLE_STEP;
    int MAX_PARTICLES;
    int MAX_VIZ_PARTICLES;
    double INV_SQUASH_FACTOR;
    bool USE_ADAPTIVE_SQUASH;                 // true면 아래 두 상황별 squash 조정 활성 (false=구 거동)
    double SQUASH_FACTOR_HIGH_SPEED_CURVE;    // 고속 커브에서 squash 지수 = 1/이값 (base보다 크게 → 더 눌림)
    double SQUASH_FACTOR_FAST_CONVERGENCE;    // 초기 수렴 시 squash 지수 = 1/이값 (base보다 작게 → 더 뾰족)
    double MAX_RANGE_METERS;
    bool PUBLISH_ODOM;
    // /pf/pose/odom 발행 시 병진 전방 외삽 시간 [s] (0=off). 출력 지연 보상 — cpp 선언부 주석 참고.
    double PUBLISH_EXTRAPOLATION_SEC{0.0};
    bool PUBLISH_MAP_ODOM_TF;
    bool PUBLISH_ODOM_BASE_TF;
    bool DO_VIZ;
    double TIMER_FREQUENCY;
    bool USE_PARALLEL_RAYCASTING;
    int NUM_THREADS;
    double MAX_POSE_RANGE;
    double SMOOTHING_ALPHA;
    double SMOOTHING_VELOCITY_FULL_MPS;   // 속도 적응 alpha가 최대 보정에 도달하는 속도
    double SMOOTHING_ALPHA_GAIN;          // 최대 속도에서 base alpha에 더해지는 폭
    double SMOOTHING_ALPHA_MAX;           // 속도 적응 alpha 상한

    // ------------------------- ESS & CLUSTER PARAMETERS (T5, T6) -------------------------
    double ESS_THRESHOLD;                 // ESS/N < threshold 일 때만 리샘플링
    double CLUSTER_RADIUS;                // 최고 가중치 주변 클러스터 포즈 추정 반경 (m)
    double CLUSTER_YAW_THRES;             // 최고 가중치 주변 클러스터 포즈 추정 각도 범위 (rad)

    // ------------------------- POSE FUSION EKF (odom 예측 + MCL 보정) -------------------------
    // 복도처럼 진행방향 관측성이 없는 구간에서 파티클 기대값이 종방향으로 표류하는 문제를
    // 출력단에서 해결한다: odom(주행거리 대비 ~0.3% 오차)으로 매 주기 예측하고, MCL 기대
    // 포즈를 측정으로 보정하되 측정 노이즈 R에 "파티클 가중 공분산"을 그대로 사용 —
    // 복도에선 종방향 분산이 커져 자동으로 odom을 더 믿고, 코너에선 분산이 줄어 MCL을 믿는다.
    bool USE_POSE_EKF;
    double EKF_TRANS_ERROR_RATE;      // 주행거리 대비 병진 오차율 (실차 실측 ~0.003)
    double EKF_TRANS_FLOOR_MPS;       // 병진 프로세스 노이즈 시간 하한 [m/s]
    double EKF_LAT_ERROR_RATIO;       // 종방향 시그마 대비 횡방향 비율
    double EKF_ROT_ERROR_RATE;        // 회전량 대비 요 오차율
    double EKF_ROT_FLOOR_RADPS;       // 요 프로세스 노이즈 시간 하한 [rad/s]
    double EKF_MEAS_VAR_INFLATION;    // 파티클 공분산 → R 배율
    double EKF_MEAS_LONG_INFLATION;   // 차체 종방향(진행방향) R 추가 배율 — 복도 표류 차단
    double EKF_MEAS_POS_STD_FLOOR;    // 측정 위치 표준편차 하한 [m]
    double EKF_MEAS_YAW_STD_FLOOR;    // 측정 요 표준편차 하한 [rad]
    double EKF_GATE_CHI2;             // 마할라노비스 게이트 (0=비활성)
    int EKF_GATE_FORCE_ACCEPT;        // 연속 기각 이 횟수 도달 시 강제 수용(재고정)
    double EKF_GATE_FORCE_ACCEPT_DIST;  // 기각 중 누적 주행거리가 이 값[m] 도달 시에도 재고정 (0=거리 조건 해제)

    bool ekf_initialized_;
    Eigen::Vector3d ekf_state_;
    Eigen::Matrix3d ekf_cov_;
    int ekf_reject_count_;
    double ekf_reject_dist_ = 0.0;    // 기각 중 누적 주행거리 [m] (고속에서의 장시간 묵보정 방지)
    bool ekf_prev_odom_valid_;
    Eigen::Vector3d ekf_prev_odom_;   // 직전 예측 시점의 원시 odom 포즈 (델타 계산용)
    double lidar_offset_x_ = 0.27;    // base_link→laser 오프셋 (TF에서 갱신, apply_tf_offset 공유)
    double lidar_offset_y_ = 0.0;

    void ekf_reset(const Eigen::Vector3d &pose);
    void ekf_predict_from_odom(const Eigen::Vector3d &odom_now);
    void ekf_update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R);
    Eigen::Matrix3d particle_covariance(const Eigen::Vector3d &mean);
    // 포즈가 맵의 free 공간에 있는지 (반경 3셀≈15 cm 내 free 1개 이상이면 허용 — 벽 스침/맵 오차 허용)
    bool is_pose_permissible(const Eigen::Vector3d& pose) const;

    // --------------------------------- SENSOR MODEL PARAMETERS ---------------------------------
    double Z_SHORT, Z_MAX, Z_RAND, Z_HIT, SIGMA_HIT;

    // --------------------------------- SCAN ROBUSTNESS & HEALTH ---------------------------------
    double RAY_LIKELIHOOD_FLOOR_RATIO;      // per-ray likelihood 하한 (열 최댓값 대비, 0=비활성)
    std::vector<double> sensor_model_col_max_;  // 센서 모델 열(기대 거리)별 최댓값
    bool USE_SCAN_QUALITY_R;                // 스캔 품질 연동 측정 노이즈 부풀림
    double SCAN_QUALITY_OUTLIER_GAIN;       // K_OUTLIER
    double SCAN_QUALITY_OUTLIER_START;      // q0
    double SCAN_QUALITY_ESS_GAIN;           // K_ESS
    double SCAN_QUALITY_ESS_START;          // ess0
    double outlier_fraction_ = 0.0;         // 최대 가중치 파티클 기준 outlier 레이 비율
    double ess_ratio_ = 1.0;                // ESS / N
    bool was_resampled_in_last_step_ = true;// 가중치 누적 vs 리셋 추적용
    double bimodality_ratio_ = 0.0;         // 클러스터 외 파티클 가중치 비율 (다봉성 지표)
    // 리샘플 직전(= 센서로 실제 informed된 가중치)에 계산해 캐시한 포즈. 리샘플이 일어나면
    // weights_가 전부 1/N으로 균등화되는데, 그 상태에서 expected_pose()를 부르면 "최고
    // 가중치 파티클"이 실질적으로 discrete_distribution이 뽑은 순서상 첫 슬롯(입자 0번 자리)
    // 으로 고정돼 매 사이클 무작위 표본 하나를 추정 포즈로 쓰게 된다(대칭 복도·다봉 분포에서
    // 발행 포즈가 사이클마다 모드 사이를 무작위로 점프하는 원인). MCL()이 리샘플 전에 채우고,
    // 외부 호출자(및 긴급복구 리시드)는 반드시 이 값을 쓴다.
    Eigen::Vector3d pre_resample_pose_ = Eigen::Vector3d::Zero();
    double last_publish_gap_ms_ = 0.0;      // 포즈 발행 마지막 공백 (ms)
    rclcpp::Time last_pose_pub_stamp_{0};   // 직전 포즈 발행 타임스탬프

    // --------------------------------- MOTION MODEL PARAMETERS ---------------------------------
    double MOTION_DISPERSION_X, MOTION_DISPERSION_Y, MOTION_DISPERSION_THETA;

    // --------------------------------- SENSOR FRAME PARAMETERS ---------------------------------
    double WHEELBASE;

    // --------------------------------- STARTUP PERFORMANCE CONTROLS ---------------------------------
    bool startup_mode_;              // Flag to track startup throttling mode
    int startup_thread_count_;       // Reduced thread count during startup
    int startup_timer_interval_;     // Timer interval during startup (ms)
    int full_timer_interval_;        // Full speed timer interval (ms)

    // --------------------------------- TF FRAME NAMES ---------------------------------
    std::string MAP_FRAME;
    std::string ODOM_FRAME;
    std::string BASE_FRAME;
    std::string LASER_FRAME;

    // --------------------------------- PARTICLE FILTER STATE ---------------------------------
    Eigen::MatrixXd particles_;
    std::vector<double> weights_;
    Eigen::Vector3d inferred_pose_;
    Eigen::Vector3d smoothed_pose_;          // Smoothed pose for output
    Eigen::Vector3d odometry_data_;
    Eigen::Vector3d last_pose_;
    bool pose_smoothing_initialized_;
    
    // --------------------------------- ODOMETRY-BASED TRACKING ---------------------------------
    Eigen::Vector3d odom_pose_;              // Current odometry-based pose estimate (rear axle)
    Eigen::Vector3d odom_reference_pose_;    // Reference pose from last MCL correction
    Eigen::Vector3d odom_reference_odom_;    // Odometry reading at last MCL correction
    bool pose_initialized_from_rviz_;        // Flag to track if pose was set via 2D Pose Estimate
    bool odom_tracking_active_;              // Flag to track if odometry tracking is active

    // Fast convergence after RViz initialization
    bool fast_convergence_mode_;             // Enable aggressive convergence mode
    int fast_convergence_remaining_;         // Iterations remaining for fast convergence

    // --------------------------------- SENSOR DATA ---------------------------------
    std::vector<float> laser_angles_;
    std::vector<float> downsampled_angles_;
    std::vector<float> downsampled_ranges_;
    rclcpp::Time last_lidar_time_;
    bool has_new_lidar_data_;
    double mcl_processing_time_;  // Store actual MCL processing time for timestamp compensation

    // --------------------------------- MAP DATA ---------------------------------
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg_;
    Eigen::MatrixXi permissible_region_;
    bool map_initialized_;
    bool lidar_initialized_;
    bool odom_initialized_;
    bool first_sensor_update_;

    // --------------------------------- SENSOR MODEL OPTIMIZATION ---------------------------------
    Eigen::MatrixXd sensor_model_table_;
    int MAX_RANGE_PX;
    double map_resolution_;
    Eigen::Vector3d map_origin_;

    // --------------------------------- PERFORMANCE CACHES ---------------------------------
    Eigen::MatrixXd local_deltas_;
    Eigen::MatrixXd queries_;
    Eigen::MatrixXd proposal_distribution_;  // Pre-allocated for MCL resampling
    std::vector<float> ranges_;
    std::vector<float> tiled_angles_;
    std::vector<float> obs_px_;              // Pre-allocated for sensor model
    std::vector<float> ranges_px_;           // Pre-allocated for sensor model

    // --------------------------------- CALLBACK GROUPS ---------------------------------
    rclcpp::CallbackGroup::SharedPtr update_cb_group_;
    rclcpp::CallbackGroup::SharedPtr map_viz_cb_group_;

    // --------------------------------- ROS2 INTERFACES ---------------------------------
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr click_sub_;
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;

    // Auto-initialization from global path
    bool auto_init_from_waypoints_;
    bool auto_init_done_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_pub_;

    // Services and TF
    rclcpp::Client<nav_msgs::srv::GetMap>::SharedPtr map_client_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> pub_tf_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr map_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;

    // --------------------------------- THREADING ---------------------------------
    std::mutex state_lock_;

    // --------------------------------- RANDOM NUMBER GENERATION ---------------------------------
    std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_dist_;
    std::normal_distribution<double> normal_dist_;

    // --------------------------------- TIMING & STATISTICS ---------------------------------
    rclcpp::Time last_stamp_;
    int iters_;
    
    // --------------------------------- VELOCITY TRACKING ---------------------------------
    double current_velocity_;          // Current linear velocity (m/s)
    double current_angular_vel_;       // Current angular velocity (rad/s)
    
    // Performance profiling
    utils::performance::TimingStats timing_stats_;
    

    // --------------------------------- ALGORITHM INTERNALS ---------------------------------
    std::vector<int> particle_indices_;

    // --------------------------------- UPDATE CONTROL ---------------------------------
    void timer_update();
    void publish_map_periodically();
    void publish_health();
};

} // namespace particle_filter_cpp

#endif // PARTICLE_FILTER_CPP__PARTICLE_FILTER_HPP_
