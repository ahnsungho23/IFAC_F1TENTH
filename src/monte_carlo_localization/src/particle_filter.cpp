// ================================================================================================
// PARTICLE FILTER IMPLEMENTATION - Monte Carlo Localization (MCL)
// ================================================================================================
// Features: Multinomial resampling, velocity motion model, beam sensor model, ray casting
// ================================================================================================

#include "particle_filter_cpp/particle_filter.hpp"
#include "particle_filter_cpp/utils.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numeric>
#include <omp.h>

namespace particle_filter_cpp
{

// ================================================================================================
// CONSTRUCTOR & INITIALIZATION
// ================================================================================================

ParticleFilter::ParticleFilter(const rclcpp::NodeOptions &options)
    : Node("particle_filter", options), rng_(std::random_device{}()), uniform_dist_(0.0, 1.0), normal_dist_(0.0, 1.0)
{
    // === PARAMETER DECLARATIONS ===
    // Core algorithm parameters
    this->declare_parameter("angle_step", 18);
    this->declare_parameter("max_particles", 2000);
    this->declare_parameter("max_viz_particles", 60);
    this->declare_parameter("squash_factor", 2.2);
    this->declare_parameter("use_adaptive_squash", true);
    this->declare_parameter("squash_factor_high_speed_curve", 3.5);
    this->declare_parameter("squash_factor_fast_convergence", 1.2);
    this->declare_parameter("max_range", 12.0);
    this->declare_parameter("max_pose_range", 10000.0);
    this->declare_parameter("smoothing_alpha", 0.3);
    this->declare_parameter("smoothing_velocity_full_mps", 2.0);
    this->declare_parameter("smoothing_alpha_gain", 0.4);
    this->declare_parameter("smoothing_alpha_max", 0.8);

    // Pose fusion EKF (odom 예측 + MCL 보정) — 헤더 주석 참고
    this->declare_parameter("use_pose_ekf", true);
    this->declare_parameter("ekf_trans_error_rate", 0.003);
    this->declare_parameter("ekf_trans_floor_mps", 0.01);
    this->declare_parameter("ekf_lat_error_ratio", 0.3);
    this->declare_parameter("ekf_rot_error_rate", 0.05);
    this->declare_parameter("ekf_rot_floor_radps", 0.01);
    this->declare_parameter("ekf_meas_var_inflation", 1.0);
    this->declare_parameter("ekf_meas_long_inflation", 25.0);
    this->declare_parameter("ekf_meas_pos_std_floor", 0.02);
    this->declare_parameter("ekf_meas_yaw_std_floor", 0.02);
    this->declare_parameter("ekf_gate_chi2", 16.0);
    this->declare_parameter("ekf_gate_force_accept", 60);
    this->declare_parameter("ekf_gate_force_accept_dist", 0.0);
    
    // Sensor model parameters
    this->declare_parameter("z_short", 0.01);
    this->declare_parameter("z_max", 0.07);
    this->declare_parameter("z_rand", 0.12);
    this->declare_parameter("z_hit", 0.80);
    this->declare_parameter("sigma_hit", 8.0);

    // Scan robustness (다이낯믹 환경 대응)
    this->declare_parameter("ray_likelihood_floor_ratio", 0.0);
    this->declare_parameter("use_scan_quality_r", false);
    this->declare_parameter("scan_quality_outlier_gain", 4.0);
    this->declare_parameter("scan_quality_outlier_start", 0.15);
    this->declare_parameter("scan_quality_ess_gain", 2.0);
    this->declare_parameter("scan_quality_ess_start", 0.4);
    
    // Motion model parameters
    this->declare_parameter("motion_dispersion_x", 0.05);
    this->declare_parameter("motion_dispersion_y", 0.025);
    this->declare_parameter("motion_dispersion_theta", 0.25);
    
    // Robot geometry
    this->declare_parameter("wheelbase", 0.325);
    
    // ROS interface
    this->declare_parameter("scan_topic", "/scan");
    this->declare_parameter("odom_topic", "/odom");
    this->declare_parameter("publish_odom", true);
    this->declare_parameter("viz", true);
    this->declare_parameter("timer_frequency", 100.0);
    
    // Performance
    this->declare_parameter("use_parallel_raycasting", true);
    this->declare_parameter("num_threads", 0); // 0 = auto-detect
    
    // TF frames
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("odom_frame", "odom");
    this->declare_parameter("base_frame", "base_link"); 
    this->declare_parameter("laser_frame", "laser");
    
    // TF publishing control
    this->declare_parameter("publish_map_odom_tf", true);
    this->declare_parameter("publish_odom_base_tf", true);

    // Auto-initialization from global path
    this->declare_parameter("auto_init_from_waypoints", true);

    // === PARAMETER RETRIEVAL ===
    // Core algorithm parameters
    ANGLE_STEP = this->get_parameter("angle_step").as_int();
    MAX_PARTICLES = this->get_parameter("max_particles").as_int();
    MAX_VIZ_PARTICLES = this->get_parameter("max_viz_particles").as_int();
    INV_SQUASH_FACTOR = 1.0 / this->get_parameter("squash_factor").as_double();
    USE_ADAPTIVE_SQUASH = this->get_parameter("use_adaptive_squash").as_bool();
    SQUASH_FACTOR_HIGH_SPEED_CURVE = this->get_parameter("squash_factor_high_speed_curve").as_double();
    SQUASH_FACTOR_FAST_CONVERGENCE = this->get_parameter("squash_factor_fast_convergence").as_double();
    MAX_RANGE_METERS = this->get_parameter("max_range").as_double();
    MAX_POSE_RANGE = this->get_parameter("max_pose_range").as_double();
    SMOOTHING_ALPHA = this->get_parameter("smoothing_alpha").as_double();
    SMOOTHING_VELOCITY_FULL_MPS = this->get_parameter("smoothing_velocity_full_mps").as_double();
    SMOOTHING_ALPHA_GAIN = this->get_parameter("smoothing_alpha_gain").as_double();
    SMOOTHING_ALPHA_MAX = this->get_parameter("smoothing_alpha_max").as_double();

    USE_POSE_EKF = this->get_parameter("use_pose_ekf").as_bool();
    EKF_TRANS_ERROR_RATE = this->get_parameter("ekf_trans_error_rate").as_double();
    EKF_TRANS_FLOOR_MPS = this->get_parameter("ekf_trans_floor_mps").as_double();
    EKF_LAT_ERROR_RATIO = this->get_parameter("ekf_lat_error_ratio").as_double();
    EKF_ROT_ERROR_RATE = this->get_parameter("ekf_rot_error_rate").as_double();
    EKF_ROT_FLOOR_RADPS = this->get_parameter("ekf_rot_floor_radps").as_double();
    EKF_MEAS_VAR_INFLATION = this->get_parameter("ekf_meas_var_inflation").as_double();
    EKF_MEAS_LONG_INFLATION = this->get_parameter("ekf_meas_long_inflation").as_double();
    EKF_MEAS_POS_STD_FLOOR = this->get_parameter("ekf_meas_pos_std_floor").as_double();
    EKF_MEAS_YAW_STD_FLOOR = this->get_parameter("ekf_meas_yaw_std_floor").as_double();
    EKF_GATE_CHI2 = this->get_parameter("ekf_gate_chi2").as_double();
    EKF_GATE_FORCE_ACCEPT = static_cast<int>(this->get_parameter("ekf_gate_force_accept").as_int());
    EKF_GATE_FORCE_ACCEPT_DIST = this->get_parameter("ekf_gate_force_accept_dist").as_double();

    // Sensor model parameters
    Z_SHORT = this->get_parameter("z_short").as_double();
    Z_MAX = this->get_parameter("z_max").as_double();
    Z_RAND = this->get_parameter("z_rand").as_double();
    Z_HIT = this->get_parameter("z_hit").as_double();
    SIGMA_HIT = this->get_parameter("sigma_hit").as_double();

    // Scan robustness parameters
    RAY_LIKELIHOOD_FLOOR_RATIO = this->get_parameter("ray_likelihood_floor_ratio").as_double();
    USE_SCAN_QUALITY_R = this->get_parameter("use_scan_quality_r").as_bool();
    SCAN_QUALITY_OUTLIER_GAIN = this->get_parameter("scan_quality_outlier_gain").as_double();
    SCAN_QUALITY_OUTLIER_START = this->get_parameter("scan_quality_outlier_start").as_double();
    SCAN_QUALITY_ESS_GAIN = this->get_parameter("scan_quality_ess_gain").as_double();
    SCAN_QUALITY_ESS_START = this->get_parameter("scan_quality_ess_start").as_double();

    // Motion model parameters
    MOTION_DISPERSION_X = this->get_parameter("motion_dispersion_x").as_double();
    MOTION_DISPERSION_Y = this->get_parameter("motion_dispersion_y").as_double();
    MOTION_DISPERSION_THETA = this->get_parameter("motion_dispersion_theta").as_double();

    // Robot geometry
    WHEELBASE = this->get_parameter("wheelbase").as_double();

    // ROS interface
    PUBLISH_ODOM = this->get_parameter("publish_odom").as_bool();
    DO_VIZ = this->get_parameter("viz").as_bool();
    TIMER_FREQUENCY = this->get_parameter("timer_frequency").as_double();

    // Performance
    USE_PARALLEL_RAYCASTING = this->get_parameter("use_parallel_raycasting").as_bool();
    NUM_THREADS = this->get_parameter("num_threads").as_int();

    // TF frames
    MAP_FRAME = this->get_parameter("map_frame").as_string();
    ODOM_FRAME = this->get_parameter("odom_frame").as_string();
    BASE_FRAME = this->get_parameter("base_frame").as_string();
    LASER_FRAME = this->get_parameter("laser_frame").as_string();

    // TF publishing control
    PUBLISH_MAP_ODOM_TF = this->get_parameter("publish_map_odom_tf").as_bool();
    PUBLISH_ODOM_BASE_TF = this->get_parameter("publish_odom_base_tf").as_bool();

    // Auto-initialization from global path
    auto_init_from_waypoints_ = this->get_parameter("auto_init_from_waypoints").as_bool();
    auto_init_done_ = false;

    // State initialization
    MAX_RANGE_PX = 0;
    iters_ = 0;
    map_initialized_ = false;
    lidar_initialized_ = false;
    odom_initialized_ = false;
    first_sensor_update_ = true;
    current_velocity_ = 0.0;
    ekf_initialized_ = false;
    ekf_state_ = Eigen::Vector3d::Zero();
    ekf_cov_ = Eigen::Matrix3d::Identity();
    ekf_reject_count_ = 0;
    ekf_prev_odom_valid_ = false;
    ekf_prev_odom_ = Eigen::Vector3d::Zero();
    current_angular_vel_ = 0.0;
    has_new_lidar_data_ = false;
    last_lidar_time_ = rclcpp::Time(0);
    mcl_processing_time_ = 0.0;
    
    // Odometry tracking
    odom_pose_ = Eigen::Vector3d::Zero();
    odom_reference_pose_ = Eigen::Vector3d::Zero();
    odom_reference_odom_ = Eigen::Vector3d::Zero();
    pose_initialized_from_rviz_ = false;
    odom_tracking_active_ = false;

    // Fast convergence initialization
    fast_convergence_mode_ = false;
    fast_convergence_remaining_ = 0;
    smoothed_pose_ = Eigen::Vector3d::Zero();
    pose_smoothing_initialized_ = false;

    // Startup performance controls
    startup_mode_ = false;
    startup_thread_count_ = 1;
    startup_timer_interval_ = 67;  // Default 15Hz
    full_timer_interval_ = 33;     // Default 30Hz
    
    // Threading setup with startup throttling
    if (USE_PARALLEL_RAYCASTING) {
        if (NUM_THREADS == 0) {
            NUM_THREADS = omp_get_max_threads();
        }
        // Reduce thread count during startup to prevent resource contention
        startup_thread_count_ = std::max(1, NUM_THREADS / 2);
        omp_set_num_threads(startup_thread_count_);
        startup_mode_ = true;
    }

    // Particle initialization
    particles_ = Eigen::MatrixXd::Zero(MAX_PARTICLES, 3);
    weights_.resize(MAX_PARTICLES, 1.0 / MAX_PARTICLES);
    particle_indices_.resize(MAX_PARTICLES);
    std::iota(particle_indices_.begin(), particle_indices_.end(), 0);

    // Motion cache and performance optimizations
    local_deltas_ = Eigen::MatrixXd::Zero(MAX_PARTICLES, 3);
    proposal_distribution_ = Eigen::MatrixXd::Zero(MAX_PARTICLES, 3);

    // Publishers
    if (DO_VIZ)
    {
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/pf/viz/inferred_pose", 1);
        particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/pf/viz/particles", 1);
    }

    if (PUBLISH_ODOM)
    {
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/pf/pose/odom", 1);
    }

    // Map publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::QoS(1).transient_local());

    // TF broadcaster and listener
    pub_tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscribers
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        this->get_parameter("scan_topic").as_string(), 1,
        std::bind(&ParticleFilter::lidarCB, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        this->get_parameter("odom_topic").as_string(), 1,
        std::bind(&ParticleFilter::odomCB, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 1, std::bind(&ParticleFilter::clicked_pose, this, std::placeholders::_1));

    click_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/clicked_point", 1, std::bind(&ParticleFilter::clicked_point, this, std::placeholders::_1));

    if (auto_init_from_waypoints_)
    {
        waypoints_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
            "/global_waypoints", rclcpp::QoS(1).transient_local(),
            std::bind(&ParticleFilter::waypointsCB, this, std::placeholders::_1));
    }

    // Map service client
    map_client_ = this->create_client<nav_msgs::srv::GetMap>("/map_server/map");

    // Load map
    get_omap();
    initialize_global();

    // Update timer - use slower frequency during startup to reduce resource contention
    double startup_frequency = std::min(TIMER_FREQUENCY, 15.0);  // Cap at 15Hz during startup
    int timer_interval_ms = static_cast<int>(1000.0 / startup_frequency);
    update_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(timer_interval_ms),
        std::bind(&ParticleFilter::timer_update, this)
    );
    startup_timer_interval_ = timer_interval_ms;
    full_timer_interval_ = static_cast<int>(1000.0 / TIMER_FREQUENCY);

    // Map publisher timer
    map_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200),
        std::bind(&ParticleFilter::publish_map_periodically, this)
    );


    RCLCPP_INFO(this->get_logger(), "Particle filter initialized - %.1fHz, %s threading (%d threads)", 
        TIMER_FREQUENCY, USE_PARALLEL_RAYCASTING ? "parallel" : "sequential", 
        USE_PARALLEL_RAYCASTING ? NUM_THREADS : 1);
}

// ================================================================================================
// MAP LOADING & PREPROCESSING
// ================================================================================================
void ParticleFilter::get_omap()
{
    RCLCPP_INFO(this->get_logger(), "Requesting map from map server...");

    while (!map_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
            return;
        RCLCPP_INFO(this->get_logger(), "Get map service not available, waiting...");
    }

    auto request = std::make_shared<nav_msgs::srv::GetMap::Request>();
    auto future = map_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        map_msg_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(future.get()->map);
        map_resolution_ = map_msg_->info.resolution;
        map_origin_ = Eigen::Vector3d(map_msg_->info.origin.position.x, map_msg_->info.origin.position.y,
                                      utils::geometry::quaternion_to_yaw(map_msg_->info.origin.orientation));

        MAX_RANGE_PX = static_cast<int>(MAX_RANGE_METERS / map_resolution_);

        // Extract free space for particle initialization
        int height = map_msg_->info.height;
        int width = map_msg_->info.width;
        permissible_region_ = Eigen::MatrixXi::Zero(height, width);

        for (int i = 0; i < height; ++i)
        {
            for (int j = 0; j < width; ++j)
            {
                int idx = i * width + j;
                if (idx < static_cast<int>(map_msg_->data.size()) && map_msg_->data[idx] == 0)
                {
                    permissible_region_(i, j) = 1; // permissible
                }
            }
        }

        map_initialized_ = true;
        RCLCPP_INFO(this->get_logger(), "Map loaded and published");

        // Publish map
        if (map_pub_) {
            map_pub_->publish(*map_msg_);
        }

        // Generate sensor model lookup table
        precompute_sensor_model();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to get map from map server");
    }
}

// ================================================================================================
// SENSOR MODEL PRECOMPUTATION
// ================================================================================================
void ParticleFilter::precompute_sensor_model()
{
    if (map_resolution_ <= 0.0)
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid map resolution: %.6f", map_resolution_);
        return;
    }

    int table_width = MAX_RANGE_PX + 1;
    sensor_model_table_ = Eigen::MatrixXd::Zero(table_width, table_width);

    auto start_time = std::chrono::high_resolution_clock::now();

    // Build lookup table
    for (int d = 0; d < table_width; ++d)  // d = expected range
    {
        double norm = 0.0;

        for (int r = 0; r < table_width; ++r)  // r = observed range
        {
            double prob = 0.0;
            double z = static_cast<double>(r - d);

            // Z_HIT: Gaussian around expected range
            prob += Z_HIT * std::exp(-(z * z) / (2.0 * SIGMA_HIT * SIGMA_HIT)) / (SIGMA_HIT * std::sqrt(2.0 * M_PI));

            // Z_SHORT: Exponential for early obstacles
            if (r < d)
            {
                prob += 2.0 * Z_SHORT * (d - r) / static_cast<double>(d);
            }

            // Z_MAX: Delta function at maximum range
            if (r == MAX_RANGE_PX)
            {
                prob += Z_MAX;
            }

            // Z_RAND: Uniform distribution
            if (r < MAX_RANGE_PX)
            {
                prob += Z_RAND * 1.0 / static_cast<double>(MAX_RANGE_PX);
            }

            norm += prob;
            sensor_model_table_(r, d) = prob;
        }

        // Normalize
        if (norm > 0)
        {
            sensor_model_table_.col(d) /= norm;
        }
    }

    // per-ray likelihood floor용 열(기대 거리)별 최댓값 — 틀린 레이 하나의 벌점을
    // peak/floor 비율(약 66배)에서 floor_ratio 배로 제한해, 맵과 일부 다른 레이가
    // 있어도 정답 가설이 급사하지 않게 한다 (다이낯믹 환경 대응)
    sensor_model_col_max_.resize(table_width);
    for (int d = 0; d < table_width; ++d)
        sensor_model_col_max_[d] = sensor_model_table_.col(d).maxCoeff();

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    RCLCPP_INFO(this->get_logger(), "Sensor model ready (%ld ms)", duration.count());
}

// ================================================================================================
// SENSOR CALLBACKS
// ================================================================================================
void ParticleFilter::lidarCB(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (laser_angles_.empty())
    {
        // Extract scan parameters and downsample
        laser_angles_.resize(msg->ranges.size());
        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            laser_angles_[i] = msg->angle_min + i * msg->angle_increment;
        }

        // Create downsampled angles
        for (size_t i = 0; i < laser_angles_.size(); i += ANGLE_STEP)
        {
            downsampled_angles_.push_back(laser_angles_[i]);
        }

        RCLCPP_INFO(this->get_logger(), "LiDAR initialized - %zu angles", downsampled_angles_.size());
    }

    // Extract downsampled measurements
    downsampled_ranges_.clear();
    for (size_t i = 0; i < msg->ranges.size(); i += ANGLE_STEP)
    {
        downsampled_ranges_.push_back(msg->ranges[i]);
    }

    // Mark new lidar data available - protected by mutex
    {
        std::lock_guard<std::mutex> lock(state_lock_);
        last_lidar_time_ = msg->header.stamp;
        has_new_lidar_data_ = true;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "LiDAR callback: new data received, timestamp: %d.%09u",
            msg->header.stamp.sec, msg->header.stamp.nanosec);
    }
    lidar_initialized_ = true;
}

void ParticleFilter::odomCB(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // Store pose data
    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y,
                             utils::geometry::quaternion_to_yaw(msg->pose.pose.orientation));

    // 공유 상태(속도/odom 추적/포즈) 쓰기는 전부 락 안에서 수행
    {
        std::lock_guard<std::mutex> lock(state_lock_);

        // Store velocity information
        current_velocity_ = msg->twist.twist.linear.x;
        current_angular_vel_ = msg->twist.twist.angular.z;

        // Update odometry tracking if active
        bool can_use_odom_tracking = pose_initialized_from_rviz_ ||
                                   (map_initialized_ && iters_ > 0 && is_pose_valid(inferred_pose_));

        if (can_use_odom_tracking && odom_tracking_active_) {
            update_odom_pose(msg);
        }

        // Store pose and timestamp
        if (last_pose_.norm() <= 0)
        {
            RCLCPP_INFO_ONCE(this->get_logger(), "Odometry initialized");
        }
        last_pose_ = position;
        last_stamp_ = msg->header.stamp;
    }
    odom_initialized_ = true;
}

// ================================================================================================
// INTERACTIVE INITIALIZATION
// ================================================================================================
void ParticleFilter::clicked_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    Eigen::Vector3d pose(msg->pose.pose.position.x, msg->pose.pose.position.y,
                         utils::geometry::quaternion_to_yaw(msg->pose.pose.orientation));
    
    // Initialize particle filter around clicked pose
    initialize_particles_pose(pose);

    // Initialize odometry-based tracking from this pose
    initialize_odom_tracking(pose);

    // Pose EKF는 다음 MCL 측정(laser frame)에서 재시드 — RViz 포즈(base_link)를 그대로
    // 시드하면 lidar 오프셋만큼 종방향 편차로 시작해 종방향 저게인 탓에 오래 남는다.
    ekf_initialized_ = false;

    // Set inferred pose immediately for visualization
    inferred_pose_ = pose;

    // Enable aggressive convergence mode for next 20 iterations
    fast_convergence_mode_ = true;
    fast_convergence_remaining_ = 20;

    RCLCPP_INFO(this->get_logger(), "Pose initialized from RViz at [%.3f, %.3f, %.3f] - fast convergence enabled",
                pose[0], pose[1], pose[2]);

    // Trigger immediate visualization update
    visualize(this->get_clock()->now());
}

void ParticleFilter::clicked_point(const geometry_msgs::msg::PointStamped::SharedPtr /*msg*/)
{
    initialize_global();
}

void ParticleFilter::waypointsCB(const f110_msgs::msg::WpntArray::ConstSharedPtr msg)
{
    if (!auto_init_from_waypoints_ || auto_init_done_ || pose_initialized_from_rviz_)
    {
        return;
    }

    if (msg->wpnts.empty())
    {
        return;
    }

    // Extract initial pose from first waypoint (start line pose)
    const auto &start_wp = msg->wpnts[0];
    Eigen::Vector3d start_pose(start_wp.x_m, start_wp.y_m, start_wp.psi_rad);

    RCLCPP_INFO(this->get_logger(),
                "Auto-initializing particles from global waypoints start pose: [%.3f, %.3f, %.3f rad (%.1f deg)]",
                start_pose[0], start_pose[1], start_pose[2], start_pose[2] * 180.0 / M_PI);

    initialize_particles_pose(start_pose);
    initialize_odom_tracking(start_pose, false);
    ekf_initialized_ = false;   // 다음 MCL 측정(laser frame)에서 시드

    inferred_pose_ = start_pose;
    fast_convergence_mode_ = true;
    fast_convergence_remaining_ = 30;
    auto_init_done_ = true;

    visualize(this->get_clock()->now());
}

// ================================================================================================
// PARTICLE INITIALIZATION
// ================================================================================================
void ParticleFilter::initialize_particles_pose(const Eigen::Vector3d &pose)
{
    RCLCPP_INFO(this->get_logger(), "Initializing particles at [%.3f, %.3f, %.3f]", 
                pose[0], pose[1], pose[2]);

    std::lock_guard<std::mutex> lock(state_lock_);
    std::fill(weights_.begin(), weights_.end(), 1.0 / MAX_PARTICLES);

    // Use tighter distribution for faster convergence after manual pose setting
    double pos_std = 0.1;   // Reduced from 0.5m to 0.1m (±10cm)
    double angle_std = 0.1; // Reduced from 0.4rad to 0.1rad (±5.7°)

    for (int i = 0; i < MAX_PARTICLES; ++i)
    {
        particles_(i, 0) = pose[0] + normal_dist_(rng_) * pos_std;
        particles_(i, 1) = pose[1] + normal_dist_(rng_) * pos_std;
        particles_(i, 2) = pose[2] + normal_dist_(rng_) * angle_std;

        // Normalize angle
        particles_(i, 2) = utils::geometry::normalize_angle(particles_(i, 2));
    }
}

void ParticleFilter::initialize_global()
{
    if (!map_initialized_)
        return;

    RCLCPP_INFO(this->get_logger(), "Global initialization started");

    std::lock_guard<std::mutex> lock(state_lock_);

    // Extract free space cells
    std::vector<std::pair<int, int>> permissible_positions;
    for (int i = 0; i < permissible_region_.rows(); ++i)
    {
        for (int j = 0; j < permissible_region_.cols(); ++j)
        {
            if (permissible_region_(i, j) == 1)
            {
                permissible_positions.emplace_back(i, j);
            }
        }
    }

    if (permissible_positions.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "No free space found in map!");
        return;
    }

    // Sample particles uniformly over free space
    std::uniform_int_distribution<int> pos_dist(0, permissible_positions.size() - 1);
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);

    for (int i = 0; i < MAX_PARTICLES; ++i)
    {
        int idx = pos_dist(rng_);
        auto pos = permissible_positions[idx];

        particles_(i, 0) = pos.second * map_resolution_ + map_origin_[0];
        particles_(i, 1) = pos.first * map_resolution_ + map_origin_[1];
        particles_(i, 2) = angle_dist(rng_);
    }

    std::fill(weights_.begin(), weights_.end(), 1.0 / MAX_PARTICLES);

    // 전역 초기화는 어디에 수렴할지 모르므로 EKF는 다음 MCL 보정에서 재시드
    ekf_initialized_ = false;

    RCLCPP_INFO(this->get_logger(), "Initialized %d particles globally", MAX_PARTICLES);
}

// ================================================================================================
// MCL ALGORITHM CORE
// ================================================================================================

/**
 * @brief Applies motion model to particles using bicycle kinematics with adaptive noise
 *
 * Features:
 * - Bicycle model with straight-line and curved motion handling
 * - Multi-step integration for high-speed accuracy (>3 m/s)
 * - Curve-aware noise scaling to prevent particle divergence
 * - Velocity-dependent noise adaptation
 *
 * @param proposal_dist Matrix of particle poses to update (in-place)
 * @param motion_cmd Motion command containing velocity, angular velocity, and time step
 */
void ParticleFilter::motion_model(Eigen::MatrixXd &proposal_dist, const MotionCommand &motion_cmd)
{
    // Extract motion parameters with safety bounds
    double dt = std::max(0.001, std::min(motion_cmd.dt, 0.5));  // Bounds: 1ms to 500ms
    
    // Apply velocity limits (prevent unrealistic speeds)
    double velocity = std::copysign(std::min(std::abs(motion_cmd.velocity), 15.0), motion_cmd.velocity);  // 15 m/s max
    double angular_velocity = std::copysign(std::min(std::abs(motion_cmd.angular_velocity), 10.0), motion_cmd.angular_velocity);  // 10 rad/s max

    // Pre-compute common values for optimization
    const double speed = std::abs(velocity);
    const double angular_speed = std::abs(angular_velocity);
    const bool is_straight_motion = angular_speed < 1e-6;
    const bool use_high_speed_integration = speed > 3.0 && !is_straight_motion;

    const double delta_theta = angular_velocity * dt;
    const double radius = is_straight_motion ? 0.0 : velocity / angular_velocity;
    const double linear_displacement = velocity * dt;

    // Pre-compute noise scaling factors
    const double speed_factor = 1.0 + (speed / 8.0);  // Linear scaling: 1.0x at 0 m/s, 1.625x at 5 m/s
    double curve_factor = 1.0;
    if (speed > 3.0 && angular_speed > 0.5) {
        // In high-speed curves, reduce noise to prevent particle divergence
        curve_factor = std::max(0.4, 1.0 - (speed * angular_speed / 10.0));
    }
    const double noise_factor = std::min(speed_factor * curve_factor, 2.0);

    // Apply bicycle model kinematics
    for (int i = 0; i < MAX_PARTICLES; ++i)
    {
        const double x = proposal_dist(i, 0);
        const double y = proposal_dist(i, 1);
        const double theta = proposal_dist(i, 2);

        if (is_straight_motion) {
            // Straight line motion - optimized with pre-computed displacement
            const double cos_theta = std::cos(theta);
            const double sin_theta = std::sin(theta);
            proposal_dist(i, 0) = x + linear_displacement * cos_theta;
            proposal_dist(i, 1) = y + linear_displacement * sin_theta;
            proposal_dist(i, 2) = theta;
        } else if (use_high_speed_integration) {
            // Multi-step integration for high-speed curved motion
            const int substeps = std::max(2, static_cast<int>(speed * dt * 2));
            const double sub_dt = dt / substeps;
            const double sub_delta_theta = angular_velocity * sub_dt;
            const double sub_displacement = velocity * sub_dt;

            double current_x = x, current_y = y, current_theta = theta;
            for (int step = 0; step < substeps; ++step) {
                current_theta += sub_delta_theta;
                current_x += sub_displacement * std::cos(current_theta);
                current_y += sub_displacement * std::sin(current_theta);
            }

            proposal_dist(i, 0) = current_x;
            proposal_dist(i, 1) = current_y;
            proposal_dist(i, 2) = current_theta;
        } else {
            // Standard bicycle model for normal curved motion
            const double new_theta = theta + delta_theta;
            const double sin_theta = std::sin(theta);
            const double cos_theta = std::cos(theta);
            const double sin_new_theta = std::sin(new_theta);
            const double cos_new_theta = std::cos(new_theta);

            proposal_dist(i, 0) = x + radius * (sin_new_theta - sin_theta);
            proposal_dist(i, 1) = y - radius * (cos_new_theta - cos_theta);
            proposal_dist(i, 2) = new_theta;
        }

        // Add adaptive motion noise — 차체 프레임(종/횡)에서 생성 후 갱신된 헤딩으로 회전.
        // (이전에는 map 좌표축에 직접 더해져 헤딩에 따라 종/횡 의도가 뒤집혔다)
        const double n_long = normal_dist_(rng_) * MOTION_DISPERSION_X * noise_factor;
        const double n_lat  = normal_dist_(rng_) * MOTION_DISPERSION_Y * noise_factor;
        const double th_new = proposal_dist(i, 2);
        const double cn = std::cos(th_new), sn = std::sin(th_new);

        proposal_dist(i, 0) += cn * n_long - sn * n_lat;
        proposal_dist(i, 1) += sn * n_long + cn * n_lat;
        proposal_dist(i, 2) += normal_dist_(rng_) * MOTION_DISPERSION_THETA * noise_factor;

        // Normalize angle
        proposal_dist(i, 2) = utils::geometry::normalize_angle(proposal_dist(i, 2));
    }
}


/**
 * @brief Evaluates sensor model likelihood for all particles using beam model
 *
 * Features:
 * - Efficient batch ray casting with parallel processing
 * - Pre-computed sensor model lookup table for fast evaluation
 * - Adaptive weight squashing for high-speed curve stability
 * - Memory-optimized query generation and range conversion
 *
 * @param proposal_dist Matrix of particle poses to evaluate
 * @param obs Vector of observed laser range measurements
 * @param weights Output vector of particle weights (normalized)
 */
void ParticleFilter::sensor_model(const Eigen::MatrixXd &proposal_dist, const std::vector<float> &obs,
                                  std::vector<double> &weights)
{
    const int num_rays = downsampled_angles_.size();
    const int total_queries = num_rays * MAX_PARTICLES;

    // === INITIALIZATION: First-time memory allocation ===
    initialize_sensor_arrays(num_rays, total_queries);

    // === RAY QUERY GENERATION ===
    auto query_start = std::chrono::high_resolution_clock::now();
    generate_ray_queries(proposal_dist, num_rays);
    timing_stats_.query_prep_time += std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - query_start).count();

    // === RAY CASTING ===
    ranges_ = calc_range_many(queries_);

    // === WEIGHT CALCULATION ===
    auto sensor_eval_start = std::chrono::high_resolution_clock::now();
    calculate_particle_weights(obs, num_rays, weights);
    timing_stats_.sensor_model_time += std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - sensor_eval_start).count();

    // === SCAN QUALITY METRIC ===
    // 최대 가중치 파티클의 기대 거리와 관측이 3·sigma_hit 이상 어긋나는 레이 비율.
    // 맵과 환경이 다른(다이낯믹) 구간에서 급등 → 측정 노이즈 R 부풀림의 입력이 된다.
    if (!weights.empty()) {
        const int best = static_cast<int>(std::distance(
            weights.begin(), std::max_element(weights.begin(), weights.end())));
        const double outlier_thresh = 3.0 * SIGMA_HIT * map_resolution_;
        int outliers = 0, valid = 0;
        for (int j = 0; j < num_rays; ++j) {
            const float o = obs[j];
            if (!std::isfinite(o) || o <= 0.0f) continue;
            ++valid;
            if (std::abs(static_cast<double>(o) - ranges_[best * num_rays + j]) > outlier_thresh)
                ++outliers;
        }
        outlier_fraction_ = valid > 0 ? static_cast<double>(outliers) / valid : 0.0;
    }
}

void ParticleFilter::initialize_sensor_arrays(int num_rays, int total_queries)
{
    if (first_sensor_update_) {
        queries_ = Eigen::MatrixXd::Zero(total_queries, 3);
        ranges_.resize(total_queries);

        // Pre-compute tiled angles for efficiency
        tiled_angles_.resize(total_queries);
        for (int i = 0; i < MAX_PARTICLES; ++i) {
            std::copy(downsampled_angles_.begin(), downsampled_angles_.end(),
                     tiled_angles_.begin() + i * num_rays);
        }
        first_sensor_update_ = false;
    }
}

void ParticleFilter::generate_ray_queries(const Eigen::MatrixXd &proposal_dist, int num_rays)
{
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        const int base_idx = i * num_rays;
        const double x = proposal_dist(i, 0);
        const double y = proposal_dist(i, 1);
        const double theta = proposal_dist(i, 2);

        for (int j = 0; j < num_rays; ++j) {
            const int idx = base_idx + j;
            queries_(idx, 0) = x;
            queries_(idx, 1) = y;
            queries_(idx, 2) = theta + downsampled_angles_[j];
        }
    }
}

void ParticleFilter::calculate_particle_weights(const std::vector<float> &obs, int num_rays,
                                               std::vector<double> &weights)
{
    // Convert observations to pixel units
    obs_px_.resize(obs.size());
    for (size_t i = 0; i < obs.size(); ++i) {
        // 무효 레이(NaN/inf/0 등)는 max range로 취급 — 무효값이 "매우 가까운 벽"으로
        // 해석돼 전체 파티클 가중치를 붕괴시키는 것을 방지
        const float r = obs[i];
        if (!std::isfinite(r) || r <= 0.0f) {
            obs_px_[i] = static_cast<double>(MAX_RANGE_PX);
        } else {
            obs_px_[i] = std::min(static_cast<double>(MAX_RANGE_PX), r / map_resolution_);
        }
    }

    // Convert expected ranges to pixel units
    ranges_px_.resize(ranges_.size());
    for (size_t i = 0; i < ranges_.size(); ++i) {
        ranges_px_[i] = std::min(static_cast<double>(MAX_RANGE_PX), ranges_[i] / map_resolution_);
    }

    // 상황별 squash 조정 — use_adaptive_squash=true일 때만 적용 (false면 구 거동 그대로).
    // 지수가 작을수록 가중치 대비가 눌리고(소프트), 클수록 뾰족해진다.
    // ※ 과거엔 std::min(INV, 1/1.8)처럼 INV보다 큰 상수와 min을 해서 두 분기 모두
    //    no-op이었다 — min은 INV보다 작은 값과, max는 INV보다 큰 값과 쌍을 이뤄야 한다.
    double squash_factor = INV_SQUASH_FACTOR;
    if (USE_ADAPTIVE_SQUASH) {
        // 고속 커브: 이례적 소수 파티클의 지배를 막기 위해 가중치를 더 눌러준다 (지수↓)
        const bool is_high_speed_curve = (current_velocity_ > 4.0 && std::abs(current_angular_vel_) > 0.3);
        if (is_high_speed_curve) {
            squash_factor = std::min(squash_factor, 1.0 / SQUASH_FACTOR_HIGH_SPEED_CURVE);
        }

        // 초기 수렴(RViz/자동 초기화 직후 N회): 가중치를 더 뾰족하게 해 정답 가설을 빨리 선택
        if (fast_convergence_mode_ && fast_convergence_remaining_ > 0) {
            squash_factor = std::max(squash_factor, 1.0 / SQUASH_FACTOR_FAST_CONVERGENCE);
        }
    }

    // fast convergence 카운터는 토글과 무관하게 소진 (모드 종료 로그 유지)
    if (fast_convergence_mode_ && fast_convergence_remaining_ > 0) {
        --fast_convergence_remaining_;
        if (fast_convergence_remaining_ <= 0) {
            fast_convergence_mode_ = false;
            RCLCPP_INFO(this->get_logger(), "Fast convergence mode completed");
        }
    }

    // Compute particle weights using pre-computed sensor model lookup table
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        double weight = 1.0;
        const int base_idx = i * num_rays;

        for (int j = 0; j < num_rays; ++j) {
            const int obs_idx = std::max(0, std::min(static_cast<int>(std::round(obs_px_[j])), MAX_RANGE_PX));
            const int range_idx = std::max(0, std::min(static_cast<int>(std::round(ranges_px_[base_idx + j])), MAX_RANGE_PX));

            const double p = sensor_model_table_(obs_idx, range_idx);
            if (RAY_LIKELIHOOD_FLOOR_RATIO > 0.0) {
                weight *= std::max(p, RAY_LIKELIHOOD_FLOOR_RATIO * sensor_model_col_max_[range_idx]);
            } else {
                weight *= p;
            }
        }

        weights[i] = std::pow(weight, squash_factor);
    }
}

// ================================================================================================
// RAY CASTING
// ================================================================================================
std::vector<float> ParticleFilter::calc_range_many(const Eigen::MatrixXd &queries)
{
    auto raycast_start = std::chrono::high_resolution_clock::now();
    
    std::vector<float> results(queries.rows());

    if (USE_PARALLEL_RAYCASTING) {
        #pragma omp parallel for schedule(dynamic)
        for (int i = 0; i < queries.rows(); ++i)
        {
            results[i] = cast_ray(queries(i, 0), queries(i, 1), queries(i, 2));
        }
    } else {
        for (int i = 0; i < queries.rows(); ++i)
        {
            results[i] = cast_ray(queries(i, 0), queries(i, 1), queries(i, 2));
        }
    }

    auto raycast_end = std::chrono::high_resolution_clock::now();
    timing_stats_.ray_casting_time += std::chrono::duration<double, std::milli>(raycast_end - raycast_start).count();
    
    return results;
}

float ParticleFilter::cast_ray(double x, double y, double angle)
{
    if (!map_initialized_)
        return MAX_RANGE_METERS;

    double dx = std::cos(angle) * map_resolution_;
    double dy = std::sin(angle) * map_resolution_;

    double current_x = x;
    double current_y = y;

    for (int step = 0; step < MAX_RANGE_PX; ++step)
    {
        current_x += dx;
        current_y += dy;

        // World to grid coordinate transformation
        int grid_x = static_cast<int>((current_x - map_origin_[0]) / map_resolution_);
        int grid_y = static_cast<int>((current_y - map_origin_[1]) / map_resolution_);

        // Map boundary collision
        if (grid_x < 0 || grid_x >= static_cast<int>(map_msg_->info.width) || grid_y < 0 ||
            grid_y >= static_cast<int>(map_msg_->info.height))
        {
            return step * map_resolution_;
        }

        // Check for obstacles
        int map_idx = grid_y * map_msg_->info.width + grid_x;
        if (map_idx >= 0 && map_idx < static_cast<int>(map_msg_->data.size()))
        {
            if (map_msg_->data[map_idx] > 50)
            {
                return step * map_resolution_;
            }
        }
    }

    return MAX_RANGE_METERS;
}

/**
 * @brief Main Monte Carlo Localization algorithm implementation
 *
 * Implements the complete MCL cycle:
 * 1. Particle resampling based on previous weights
 * 2. Motion model prediction with adaptive noise
 * 3. Sensor model likelihood evaluation
 * 4. Weight normalization with diversity monitoring
 * 5. Emergency recovery for high-speed scenarios
 *
 * @param motion_cmd Motion command for particle prediction
 * @param observation Laser scan measurements for likelihood evaluation
 */
void ParticleFilter::MCL(const MotionCommand &motion_cmd, const std::vector<float> &observation)
{
    auto mcl_start = std::chrono::high_resolution_clock::now();
    
    // 1. Multinomial resampling - using pre-allocated memory
    auto resample_start = std::chrono::high_resolution_clock::now();
    std::discrete_distribution<int> particle_dist(weights_.begin(), weights_.end());

    for (int i = 0; i < MAX_PARTICLES; ++i)
    {
        int idx = particle_dist(rng_);
        proposal_distribution_.row(i) = particles_.row(idx);
    }
    auto resample_end = std::chrono::high_resolution_clock::now();
    timing_stats_.resampling_time += std::chrono::duration<double, std::milli>(resample_end - resample_start).count();

    // 2. Motion prediction
    auto motion_start = std::chrono::high_resolution_clock::now();
    motion_model(proposal_distribution_, motion_cmd);
    auto motion_end = std::chrono::high_resolution_clock::now();
    timing_stats_.motion_model_time += std::chrono::duration<double, std::milli>(motion_end - motion_start).count();

    // 3. Sensor likelihood evaluation
    sensor_model(proposal_distribution_, observation, weights_);

    // 4. Weight normalization with particle diversity check
    double sum_weights = std::accumulate(weights_.begin(), weights_.end(), 0.0);
    if (sum_weights > 0)
    {
        for (double &w : weights_)
        {
            w /= sum_weights;
        }

        // Check effective sample size for high-speed recovery
        double effective_particles = 0.0;
        for (const double &w : weights_) {
            effective_particles += w * w;
        }
        effective_particles = 1.0 / effective_particles;
        ess_ratio_ = effective_particles / MAX_PARTICLES;   // 스캔 품질 연동 R 입력용으로 보존

        // Emergency recovery: inject random particles during high-speed maneuvers if diversity is too low
        if (effective_particles < MAX_PARTICLES * 0.15 && std::abs(current_velocity_) > 4.0) {
            // Replace 10% of particles with random samples around current estimate
            int recovery_count = MAX_PARTICLES / 10;
            Eigen::Vector3d current_pose = expected_pose();

            std::uniform_int_distribution<int> particle_idx_dist(0, MAX_PARTICLES - 1);
            for (int i = 0; i < recovery_count; ++i) {
                int idx = particle_idx_dist(rng_);
                // Add particles around current pose with moderate spread
                proposal_distribution_(idx, 0) = current_pose[0] + normal_dist_(rng_) * 1.0;
                proposal_distribution_(idx, 1) = current_pose[1] + normal_dist_(rng_) * 1.0;
                proposal_distribution_(idx, 2) = current_pose[2] + normal_dist_(rng_) * 0.5;
                proposal_distribution_(idx, 2) = utils::geometry::normalize_angle(proposal_distribution_(idx, 2));
                weights_[idx] = 1.0 / MAX_PARTICLES;
            }
        }
    }

    // 5. Update particle set - using efficient swap
    particles_.swap(proposal_distribution_);
    
    auto mcl_end = std::chrono::high_resolution_clock::now();
    timing_stats_.total_mcl_time += std::chrono::duration<double, std::milli>(mcl_end - mcl_start).count();
    timing_stats_.measurement_count++;
}

Eigen::Vector3d ParticleFilter::expected_pose()
{
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();
    double sum_sin = 0.0, sum_cos = 0.0;
    
    // Weighted mean for x, y
    for (int i = 0; i < MAX_PARTICLES; ++i)
    {
        pose[0] += weights_[i] * particles_(i, 0);  // x
        pose[1] += weights_[i] * particles_(i, 1);  // y
        
        // Circular mean for angles
        sum_sin += weights_[i] * std::sin(particles_(i, 2));
        sum_cos += weights_[i] * std::cos(particles_(i, 2));
    }
    
    // Final angle calculation
    pose[2] = std::atan2(sum_sin, sum_cos);
    
    return pose;
}

// ================================================================================================
// POSE SMOOTHING
// ================================================================================================
Eigen::Vector3d ParticleFilter::smooth_pose(const Eigen::Vector3d &raw_pose)
{
    if (!pose_smoothing_initialized_) {
        smoothed_pose_ = raw_pose;
        pose_smoothing_initialized_ = true;
        return smoothed_pose_;
    }
    
    // EMA filter with adaptive alpha based on velocity.
    // 출력 지연 시정수 τ ≈ T·(1-α)/α (T = 1/timer_frequency). 저속에서 α가 작을수록
    // 노이즈는 줄지만 pose가 실제를 뒤따르는 지연이 커진다 — base alpha 0.05는 30 Hz에서
    // τ ≈ 0.63 s로 실차 저속 체감 지연의 주범이었다(2026-07-29 sim 실측 근거는 README §6).
    double base_alpha = SMOOTHING_ALPHA;  // Base smoothing factor from config
    double velocity_factor =
        std::min(1.0, std::abs(current_velocity_) / SMOOTHING_VELOCITY_FULL_MPS);
    double alpha = base_alpha + velocity_factor * SMOOTHING_ALPHA_GAIN;
    alpha = std::min(SMOOTHING_ALPHA_MAX, alpha);
    
    // Smooth x, y
    smoothed_pose_[0] = alpha * raw_pose[0] + (1.0 - alpha) * smoothed_pose_[0];
    smoothed_pose_[1] = alpha * raw_pose[1] + (1.0 - alpha) * smoothed_pose_[1];
    
    // Smooth angle with circular interpolation
    double angle_diff = utils::geometry::normalize_angle(raw_pose[2] - smoothed_pose_[2]);
    smoothed_pose_[2] = utils::geometry::normalize_angle(smoothed_pose_[2] + alpha * angle_diff);
    
    return smoothed_pose_;
}


// ================================================================================================
// POSE FUSION EKF — odom 예측 + MCL 보정 (설계 배경은 헤더 주석 참고)
// ================================================================================================
void ParticleFilter::ekf_reset(const Eigen::Vector3d &pose)
{
    ekf_state_ = pose;
    ekf_cov_ = Eigen::Matrix3d::Zero();
    ekf_cov_(0, 0) = ekf_cov_(1, 1) = 0.05 * 0.05;
    ekf_cov_(2, 2) = 0.05 * 0.05;
    ekf_initialized_ = true;
    ekf_reject_count_ = 0;
    ekf_reject_dist_ = 0.0;
    ekf_prev_odom_valid_ = false;   // 다음 예측에서 원시 odom 기준점 재설정
}

void ParticleFilter::ekf_predict_from_odom(const Eigen::Vector3d &odom_now)
{
    if (!ekf_initialized_)
        return;

    // twist 적분(30Hz 희소 샘플) 대신 소스가 전속으로 적분해 둔 odom "포즈 델타"를 쓴다 —
    // 실효 오차가 휠 odom 고유 오차율(~0.3%/거리)에 수렴한다.
    if (!ekf_prev_odom_valid_) {
        ekf_prev_odom_ = odom_now;
        ekf_prev_odom_valid_ = true;
        return;
    }

    const double prev_theta = ekf_prev_odom_[2];
    const double dx_map = odom_now[0] - ekf_prev_odom_[0];
    const double dy_map = odom_now[1] - ekf_prev_odom_[1];
    // odom 프레임 델타를 직전 odom 헤딩 기준 body 프레임으로 회전
    const double cp = std::cos(prev_theta), sp = std::sin(prev_theta);
    double dx_body = cp * dx_map + sp * dy_map;
    double dy_body = -sp * dx_map + cp * dy_map;
    const double dtheta = utils::geometry::normalize_angle(odom_now[2] - prev_theta);
    ekf_prev_odom_ = odom_now;

    // 텔레포트(재초기화 등) 감지 — 한 주기 1 m 이상 점프는 델타로 쓰지 않는다
    double ds = std::hypot(dx_body, dy_body);
    if (ds > 1.0) {
        return;
    }

    // EKF 상태는 MCL과 같은 laser 프레임이므로 base_link 델타를 laser 델타로 변환:
    // Δ_laser = T(L)^-1 · Δ_base · T(L), 병진 = Δt + (R(dθ)-I)·L. 회전 중 laser가
    // 오프셋 L만큼 추가 호를 그리는 효과 — 빼먹으면 코너마다 ~L·Δθ 만큼 계통 오차가 쌓인다.
    {
        const double cd = std::cos(dtheta), sd = std::sin(dtheta);
        dx_body += (cd - 1.0) * lidar_offset_x_ - sd * lidar_offset_y_;
        dy_body += sd * lidar_offset_x_ + (cd - 1.0) * lidar_offset_y_;
        ds = std::hypot(dx_body, dy_body);
    }

    const double theta = ekf_state_[2];
    const double c = std::cos(theta), s = std::sin(theta);
    ekf_state_[0] += c * dx_body - s * dy_body;
    ekf_state_[1] += s * dx_body + c * dy_body;
    ekf_state_[2] = utils::geometry::normalize_angle(theta + dtheta);

    Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
    F(0, 2) = -(s * dx_body + c * dy_body);
    F(1, 2) = (c * dx_body - s * dy_body);

    // 프로세스 노이즈: 이동량 비례(휠 odom 오차율) + 미소 하한(1주기분), body → map 회전
    const double dt_nom = 1.0 / TIMER_FREQUENCY;
    const double sigma_long = EKF_TRANS_ERROR_RATE * ds + EKF_TRANS_FLOOR_MPS * dt_nom;
    const double sigma_lat = EKF_LAT_ERROR_RATIO * sigma_long + 0.5 * EKF_TRANS_FLOOR_MPS * dt_nom;
    const double sigma_yaw = EKF_ROT_ERROR_RATE * std::abs(dtheta) + EKF_ROT_FLOOR_RADPS * dt_nom;
    Eigen::Matrix2d rot;
    rot << c, -s, s, c;
    Eigen::Matrix2d q_body = Eigen::Matrix2d::Zero();
    q_body(0, 0) = sigma_long * sigma_long;
    q_body(1, 1) = sigma_lat * sigma_lat;
    Eigen::Matrix3d Q = Eigen::Matrix3d::Zero();
    Q.topLeftCorner<2, 2>() = rot * q_body * rot.transpose();
    Q(2, 2) = sigma_yaw * sigma_yaw;

    ekf_cov_ = F * ekf_cov_ * F.transpose() + Q;
}

void ParticleFilter::ekf_update(const Eigen::Vector3d &z, const Eigen::Matrix3d &R)
{
    if (!ekf_initialized_) {
        ekf_reset(z);
        return;
    }

    Eigen::Vector3d innov = z - ekf_state_;
    innov[2] = utils::geometry::normalize_angle(innov[2]);
    const Eigen::Matrix3d S = ekf_cov_ + R;   // H = I
    const Eigen::Matrix3d S_inv = S.inverse();

    if (EKF_GATE_CHI2 > 0.0) {
        const double maha2 = innov.dot(S_inv * innov);
        if (maha2 > EKF_GATE_CHI2) {
            // 기각 중 누적 주행거리 추적 — 횟수 기준(40회@30Hz=1.3 s)만으로는
            // 고속에서 너무 오래 묵보정 상태가 된다 (5 m/s면 ~7 m).
            ekf_reject_dist_ += std::abs(current_velocity_) / TIMER_FREQUENCY;
            const bool dist_exceeded = EKF_GATE_FORCE_ACCEPT_DIST > 0.0 &&
                                       ekf_reject_dist_ >= EKF_GATE_FORCE_ACCEPT_DIST;
            if (ekf_reject_count_ < EKF_GATE_FORCE_ACCEPT && !dist_exceeded) {
                ++ekf_reject_count_;
                return;   // MCL 순간 글리치로 판단하고 이번 보정은 건너뜀
            }
            // 연속 기각이 길어지면 EKF 자신이 틀렸다고 보고 측정에 재고정.
            // 단, 재고정 대상이 물리적으로 불가능한 위치(벽/미지 깊숙이)면 재고정하지 않는다 —
            // 갭/맵 불일치 후 MCL이 벽 속 모드로 수렴했을 때 추정 전체를 벽 속에 박는
            // 최악 경로(run_0803_210100 t≈24-25, 점유율 0.8 셀에 34회 보고)를 차단.
            if (!is_pose_permissible(z)) {
                RCLCPP_WARN(this->get_logger(),
                            "Pose EKF: force-accept target in occupied/unknown cell - NOT re-anchoring");
                ekf_reject_count_ = 0;   // 일정 주기 후 재시도
                ekf_reject_dist_ = 0.0;
                return;
            }
            RCLCPP_WARN(this->get_logger(),
                        "Pose EKF: %d consecutive gate rejections (%.2f m rejected) - re-anchoring to MCL pose",
                        ekf_reject_count_, ekf_reject_dist_);
            ekf_reset(z);
            return;
        }
    }
    ekf_reject_count_ = 0;
    ekf_reject_dist_ = 0.0;

    const Eigen::Matrix3d K = ekf_cov_ * S_inv;
    ekf_state_ += K * innov;
    ekf_state_[2] = utils::geometry::normalize_angle(ekf_state_[2]);
    ekf_cov_ = (Eigen::Matrix3d::Identity() - K) * ekf_cov_;
}

Eigen::Matrix3d ParticleFilter::particle_covariance(const Eigen::Vector3d &mean)
{
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        const double wgt = weights_[i];
        const double dx = particles_(i, 0) - mean[0];
        const double dy = particles_(i, 1) - mean[1];
        const double dth = utils::geometry::normalize_angle(particles_(i, 2) - mean[2]);
        cov(0, 0) += wgt * dx * dx;
        cov(0, 1) += wgt * dx * dy;
        cov(1, 1) += wgt * dy * dy;
        cov(0, 2) += wgt * dx * dth;
        cov(1, 2) += wgt * dy * dth;
        cov(2, 2) += wgt * dth * dth;
    }
    cov(1, 0) = cov(0, 1);
    cov(2, 0) = cov(0, 2);
    cov(2, 1) = cov(1, 2);
    return cov;
}

bool ParticleFilter::is_pose_permissible(const Eigen::Vector3d& pose) const
{
    if (!map_initialized_ || !map_msg_)
        return true;

    const int width = map_msg_->info.width;
    const int height = map_msg_->info.height;
    const int gx = static_cast<int>((pose[0] - map_origin_[0]) / map_resolution_);
    const int gy = static_cast<int>((pose[1] - map_origin_[1]) / map_resolution_);

    // 반경 3셀(약 15 cm) 안에 free가 하나라도 있으면 허용 — 벽에 바짝 붙은 정상 주행
    // (맵 오차/벽 스침)은 용인하고, 벽 깊숙한 곳(불가능)만 판정한다.
    constexpr int TOL = 3;
    for (int dy = -TOL; dy <= TOL; ++dy) {
        for (int dx = -TOL; dx <= TOL; ++dx) {
            const int x = gx + dx, y = gy + dy;
            if (x >= 0 && y >= 0 && x < width && y < height && permissible_region_(y, x) == 1)
                return true;
        }
    }
    return false;
}

// ================================================================================================
// TIMER UPDATE
// ================================================================================================
void ParticleFilter::timer_update()
{
    if (!map_initialized_) {
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();

    // Determine what sensor data is available
    bool has_odom = odom_initialized_;

    // Check if LiDAR data is recent (within last 500ms)
    bool has_recent_lidar = false;
    if (lidar_initialized_ && last_lidar_time_.nanoseconds() != 0) {
        double lidar_age = (current_time - last_lidar_time_).seconds();
        has_recent_lidar = (lidar_age < 0.5); // 500ms threshold - more lenient

        // Debug: Log lidar timing (only in debug builds)
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "LiDAR age: %.3f sec, recent: %d, has_new: %d, has_lidar: %d",
            lidar_age, has_recent_lidar, has_new_lidar_data_, has_recent_lidar && has_new_lidar_data_);
    }

    bool has_lidar = has_recent_lidar && has_new_lidar_data_;

    // Conditional processing based on sensor availability
    if (!has_odom && !has_lidar) {
        // Case 4: Neither available - just return and wait
        return;
    }

    if (!has_odom && has_lidar) {
        // Case 3: Only LiDAR - localization without motion model
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Running MCL with LiDAR only (no odometry)");
    }

    if (has_odom && !has_lidar) {
        // Case 2: Only odometry - use odometry tracking
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                            "Running odometry tracking only (no LiDAR)");
    }

    // Case 1: Both available - full MCL (no special message needed)

    // Use steady clock for simulation compatibility
    static auto steady_start_time = std::chrono::steady_clock::now();
    static auto last_steady_time = steady_start_time;

    auto current_steady_time = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(current_steady_time - last_steady_time).count();

    // First call initialization
    static bool timer_initialized = false;
    if (!timer_initialized) {
        timer_initialized = true;
        last_steady_time = current_steady_time;
        return;
    }

    // Skip excessive time steps — 단, 기준 시각은 갱신해야 한다. 갱신 없이 리턴하면
    // 이후 dt가 영원히 >1.0이라 노드가 조용히 정지한다 (bag 일시정지/CPU 스파이크 후 사고).
    if (dt > 1.0) {
        last_steady_time = current_steady_time;
        return;
    }

    bool apply_motion = (dt >= 0.0001);

    bool mcl_executed = false;  // Track if MCL was executed this cycle

    if (state_lock_.try_lock()) {

        // Pose EKF 예측 — 라이다 유무와 무관하게 원시 odom 포즈 델타로 매 주기 전파
        if (USE_POSE_EKF && has_odom && apply_motion) {
            ekf_predict_from_odom(last_pose_);
        }

        // CASE 1 & 3: Process LiDAR data (with or without odometry)
        if (has_lidar && !downsampled_ranges_.empty()) {
            // Record MCL start time for accurate timestamp calculation
            auto mcl_start_time = std::chrono::steady_clock::now();

            // Run MCL when new lidar data is available
            ++iters_;

            // Exit startup mode after sufficient iterations to allow full threading
            if (startup_mode_ && iters_ >= 50) {
                startup_mode_ = false;
                if (USE_PARALLEL_RAYCASTING) {
                    omp_set_num_threads(NUM_THREADS);
                    RCLCPP_INFO(this->get_logger(), "Exiting startup mode - using full threading (%d threads)", NUM_THREADS);
                }

                // Recreate timer with full frequency
                update_timer_.reset();
                update_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(full_timer_interval_),
                    std::bind(&ParticleFilter::timer_update, this)
                );
                RCLCPP_INFO(this->get_logger(), "Timer frequency increased to %.1f Hz", TIMER_FREQUENCY);
            }

            MotionCommand motion_cmd;
            if (has_odom && apply_motion &&
                (std::abs(current_velocity_) > 0.0001 || std::abs(current_angular_vel_) > 0.0001)) {
                motion_cmd = MotionCommand(current_velocity_, current_angular_vel_, dt);
            } else if (!has_odom && !pose_initialized_from_rviz_ && iters_ < 15) {
                // No odometry: Add small random motion for particle diversity
                double noise_factor = std::max(0.1, 1.0 - (static_cast<double>(iters_) / 15.0));
                double random_velocity = normal_dist_(rng_) * 0.02 * noise_factor / dt;
                double random_angular_velocity = normal_dist_(rng_) * 0.05 * noise_factor / dt;
                motion_cmd = MotionCommand(random_velocity, random_angular_velocity, dt);
            }

            auto observation = downsampled_ranges_;
            // Execute MCL pipeline
            MCL(motion_cmd, observation);
            Eigen::Vector3d raw_pose = expected_pose();
            // 실측 지연(스캔 시각 → 지금). 스캔 스윕+전송+타이머 큐 대기+MCL 연산을 모두 포함하므로
            // 별도 계수 없이 그대로 쓴다 (delay_compensation_factor 은퇴 — 곱하던 base인
            // mcl_processing_time_은 실제 지연과 무관했다).
            const double lidar_age = std::clamp((current_time - last_lidar_time_).seconds(), 0.0, 0.2);
            if (USE_POSE_EKF) {
                // EKF 상태는 "지금"이고 MCL 기대 포즈는 스캔 시점(과거)이므로, 기존 지연 보상을
                // 측정에 먼저 적용해 시점을 맞춘다(속도×지연 계통 편차 → 게이트 상시 기각 방지).
                Eigen::Vector3d z = raw_pose;
                const double comp_t = lidar_age;
                z[0] += current_velocity_ * comp_t * std::cos(raw_pose[2]);
                z[1] += current_velocity_ * comp_t * std::sin(raw_pose[2]);
                z[2] = utils::geometry::normalize_angle(z[2] + current_angular_vel_ * comp_t);

                // 측정 노이즈 = 파티클 가중 공분산 + "종방향 선택 불신". 평행벽 복도에선 MCL이
                // 진행방향 위치를 관측하지 못해 틀린 곳에 좁게 수렴하므로(공분산이 모호성을
                // 과소평가), 차체 종방향 성분만 항상 크게 부풀린다 — 복도 표류는 무시되고,
                // 코너를 돌면 이전 종방향 오차가 횡방향으로 회전되어 고게인으로 보정된다.
                Eigen::Matrix3d meas_cov = particle_covariance(raw_pose) * EKF_MEAS_VAR_INFLATION;
                const double cy = std::cos(raw_pose[2]), sy = std::sin(raw_pose[2]);
                Eigen::Matrix2d to_body;
                to_body << cy, sy, -sy, cy;
                Eigen::Matrix2d cov_body = to_body * meas_cov.topLeftCorner<2, 2>() * to_body.transpose();
                cov_body(0, 0) = std::max(cov_body(0, 0) * EKF_MEAS_LONG_INFLATION,
                                          EKF_MEAS_POS_STD_FLOOR * EKF_MEAS_POS_STD_FLOOR);
                cov_body(1, 1) = std::max(cov_body(1, 1), EKF_MEAS_POS_STD_FLOOR * EKF_MEAS_POS_STD_FLOOR);
                meas_cov.topLeftCorner<2, 2>() = to_body.transpose() * cov_body * to_body;
                meas_cov(0, 2) = meas_cov(2, 0) = meas_cov(1, 2) = meas_cov(2, 1) = 0.0;
                meas_cov(2, 2) = std::max(meas_cov(2, 2), EKF_MEAS_YAW_STD_FLOOR * EKF_MEAS_YAW_STD_FLOOR);

                // 스캔 품질 연동 R 부풀림: 맵과 환경이 다른(다이낯믹) 구간에서 outlier 비율이
                // 급등하고 ESS가 묻히므로, 그때는 MCL 측정을 연속적으로 불신해 odom 우세로
                // 버틴다. 하드 게이트와 달리 부분 신뢰라 복구 불능이 없고, R이 커지면
                // S=P+R도 커져 게이트 과민 기각도 함께 완화된다.
                if (USE_SCAN_QUALITY_R) {
                    const double q_term = std::clamp(
                        (outlier_fraction_ - SCAN_QUALITY_OUTLIER_START) / (1.0 - SCAN_QUALITY_OUTLIER_START),
                        0.0, 1.0);
                    const double ess_term = std::clamp(
                        (SCAN_QUALITY_ESS_START - ess_ratio_) / SCAN_QUALITY_ESS_START, 0.0, 1.0);
                    const double s_scan = 1.0
                        + SCAN_QUALITY_OUTLIER_GAIN * q_term
                        + SCAN_QUALITY_ESS_GAIN * ess_term;
                    meas_cov *= s_scan * s_scan;
                }

                ekf_update(z, meas_cov);
                inferred_pose_ = ekf_state_;
            } else {
                inferred_pose_ = smooth_pose(raw_pose);
            }

            // Calculate MCL processing time for timestamp compensation
            auto mcl_end_time = std::chrono::steady_clock::now();
            mcl_processing_time_ = std::chrono::duration<double>(mcl_end_time - mcl_start_time).count();

            // Update odometry tracking
            bool can_use_odom_tracking = has_odom &&
                (pose_initialized_from_rviz_ || (map_initialized_ && iters_ > 0 && is_pose_valid(inferred_pose_)));

            if (can_use_odom_tracking) {
                if (!odom_tracking_active_ && is_pose_valid(inferred_pose_)) {
                    initialize_odom_tracking(inferred_pose_, false);
                    RCLCPP_INFO(this->get_logger(), "Odometry tracking initialized");
                }

                // Apply delay compensation using measured latency
                // (EKF 모드는 측정 단계에서 이미 시점 보상됨 — 이중 보상 방지 위해 0)
                Eigen::Vector3d compensated_pose = inferred_pose_;
                const double comp_t = USE_POSE_EKF ? 0.0 : lidar_age;
                double longitudinal_displacement = current_velocity_ * comp_t;
                double angular_displacement = current_angular_vel_ * comp_t;

                // Apply compensation in vehicle's forward direction
                compensated_pose[0] += longitudinal_displacement * std::cos(inferred_pose_[2]);
                compensated_pose[1] += longitudinal_displacement * std::sin(inferred_pose_[2]);
                // Apply heading compensation
                compensated_pose[2] += angular_displacement;
                compensated_pose[2] = utils::geometry::normalize_angle(compensated_pose[2]);

                odom_reference_pose_ = compensated_pose;
                odom_reference_odom_ = last_pose_;
                odom_pose_ = compensated_pose;

                // Update inferred pose to compensated pose for consistency
                inferred_pose_ = compensated_pose;
            }

            mcl_executed = true;  // Mark that MCL was executed
            has_new_lidar_data_ = false;    // Mark lidar data as processed
        }
        // CASE 2: Only odometry available - odometry tracking
        else if (has_odom && !has_lidar && odom_tracking_active_) {
            // Update pose based on odometry when no new lidar data is available
            if (apply_motion && (std::abs(current_velocity_) > 0.0001 || std::abs(current_angular_vel_) > 0.0001)) {
                // 라이다 갭 동안 파티클 구름도 odom 기반 모션 모델로 전파한다.
                // 전파하지 않으면 파티클이 갭 시작 시점 위치에 얼어붙어, 갭 종료 후
                // 첫 MCL이 수 초 전 위치로 평가돼 가중치 붕괴 → 포즈 스냅이 발생한다
                // (run_0803_210100: 스캔 갭 13회 중 16회의 점프가 갭 직후에 발생).
                if (iters_ > 0) {
                    MotionCommand gap_motion_cmd(current_velocity_, current_angular_vel_, dt);
                    motion_model(particles_, gap_motion_cmd);
                }

                // Dead reckoning based on current velocities
                Eigen::Vector3d current_pose_estimate = get_current_pose();

                // Apply motion model using current velocities
                if (std::abs(current_angular_vel_) < 1e-6) {
                    // Straight line motion
                    current_pose_estimate[0] += current_velocity_ * dt * std::cos(current_pose_estimate[2]);
                    current_pose_estimate[1] += current_velocity_ * dt * std::sin(current_pose_estimate[2]);
                } else {
                    // Curved motion (bicycle model)
                    double radius = current_velocity_ / current_angular_vel_;
                    double delta_theta = current_angular_vel_ * dt;

                    current_pose_estimate[0] += radius * (std::sin(current_pose_estimate[2] + delta_theta) - std::sin(current_pose_estimate[2]));
                    current_pose_estimate[1] -= radius * (std::cos(current_pose_estimate[2] + delta_theta) - std::cos(current_pose_estimate[2]));
                    current_pose_estimate[2] += delta_theta;
                }

                // Normalize angle
                current_pose_estimate[2] = utils::geometry::normalize_angle(current_pose_estimate[2]);

                // Update odom pose
                odom_pose_ = current_pose_estimate;
            }
        }

        state_lock_.unlock();
    }

    // Always update steady time for next calculation
    last_steady_time = current_steady_time;

    // Publish TF and odometry with appropriate timestamp
    if (map_initialized_) {
        Eigen::Vector3d current_pose = get_current_pose();

        // Safely read timestamps - protected by mutex
        rclcpp::Time timestamp;
        {
            std::lock_guard<std::mutex> lock(state_lock_);
            if (USE_POSE_EKF && has_odom && last_stamp_.nanoseconds() != 0) {
                // EKF 모드: 내용이 odom 최신 시각(≈now)이므로 stamp도 odom 시각으로 맞춘다
                timestamp = last_stamp_;
            } else if (mcl_executed && last_lidar_time_.nanoseconds() != 0) {
                // Use compensated timestamp: LiDAR time + processing time
                int64_t compensation_ns = static_cast<int64_t>(mcl_processing_time_ * 1e9);
                timestamp = last_lidar_time_ + rclcpp::Duration::from_nanoseconds(compensation_ns);
            } else if (has_odom && last_stamp_.nanoseconds() != 0) {
                // Use odometry timestamp for odom-based updates
                timestamp = last_stamp_;
            } else {
                // Fallback to current time
                timestamp = current_time;
            }
        }

        publish_tf(current_pose, timestamp);

        // Update visualization with same timestamp whenever pose changes
        if (mcl_executed || (has_odom && !has_lidar && odom_tracking_active_)) {
            visualize(timestamp);
        }
    }
}

void ParticleFilter::publish_map_periodically()
{
    // Maintain persistent map display in RViz
    if (map_initialized_ && map_pub_ && map_msg_) {
        map_pub_->publish(*map_msg_);
    }
}


// ================================================================================================
// OUTPUT & VISUALIZATION
// ================================================================================================
void ParticleFilter::publish_tf(const Eigen::Vector3d &pose, const rclcpp::Time &stamp)
{
    Eigen::Vector3d base_link_pose = apply_tf_offset(pose);

    // === TF TRANSFORM PUBLISHING ===
    // Real mode: Publish map->odom and odom->base_link
    // Sim mode:  Don't publish TF (simulator handles map->base_link directly)
    
    if (PUBLISH_MAP_ODOM_TF) {
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = (stamp.nanoseconds() != 0) ? stamp : this->get_clock()->now();
        map_to_odom.header.frame_id = MAP_FRAME;
        map_to_odom.child_frame_id = ODOM_FRAME;

        if (odom_initialized_ && last_pose_.norm() > 0) {
            // Calculate map->odom transform: T_map_odom = T_map_base * T_base_odom^(-1)
            double mcl_x = base_link_pose[0];
            double mcl_y = base_link_pose[1];
            double mcl_yaw = base_link_pose[2];
            double odom_x = last_pose_[0], odom_y = last_pose_[1], odom_yaw = last_pose_[2];

            // Inverse odom transform
            double cos_odom_inv = std::cos(-odom_yaw), sin_odom_inv = std::sin(-odom_yaw);
            double inv_odom_x = -(odom_x * cos_odom_inv - odom_y * sin_odom_inv);
            double inv_odom_y = -(odom_x * sin_odom_inv + odom_y * cos_odom_inv);
            double inv_odom_yaw = -odom_yaw;

            // Compose transforms
            double cos_mcl = std::cos(mcl_yaw), sin_mcl = std::sin(mcl_yaw);
            map_to_odom.transform.translation.x = mcl_x + inv_odom_x * cos_mcl - inv_odom_y * sin_mcl;
            map_to_odom.transform.translation.y = mcl_y + inv_odom_x * sin_mcl + inv_odom_y * cos_mcl;
            map_to_odom.transform.translation.z = 0.0;
            map_to_odom.transform.rotation = utils::geometry::yaw_to_quaternion(
                utils::geometry::normalize_angle(mcl_yaw + inv_odom_yaw));
        } else {
            // Identity transform fallback
            map_to_odom.transform.translation.x = 0.0;
            map_to_odom.transform.translation.y = 0.0;
            map_to_odom.transform.translation.z = 0.0;
            map_to_odom.transform.rotation = utils::geometry::yaw_to_quaternion(0.0);
        }
        pub_tf_->sendTransform(map_to_odom);
    }

    if (PUBLISH_ODOM_BASE_TF && odom_initialized_ && last_pose_.norm() > 0) {
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = stamp;
        odom_to_base.header.frame_id = ODOM_FRAME;
        odom_to_base.child_frame_id = BASE_FRAME;
        odom_to_base.transform.translation.x = last_pose_[0];
        odom_to_base.transform.translation.y = last_pose_[1];
        odom_to_base.transform.translation.z = 0.0;
        odom_to_base.transform.rotation = utils::geometry::yaw_to_quaternion(last_pose_[2]);
        pub_tf_->sendTransform(odom_to_base);
    }

    // Optional odometry message
    if (PUBLISH_ODOM && odom_pub_)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = (stamp.nanoseconds() != 0) ? stamp : this->get_clock()->now();
        odom.header.frame_id = MAP_FRAME;
        odom.child_frame_id = BASE_FRAME;
        odom.pose.pose.position.x = base_link_pose[0];
        odom.pose.pose.position.y = base_link_pose[1];
        odom.pose.pose.orientation = utils::geometry::yaw_to_quaternion(pose[2]);
        odom.twist.twist.linear.x = current_velocity_;
        odom_pub_->publish(odom);
    }
}


Eigen::Vector3d ParticleFilter::get_current_pose()
{
    // Priority 1: EKF 융합 결과 (EKF 모드) — 융합 출력이 최종 포즈
    if (USE_POSE_EKF && ekf_initialized_ && is_pose_valid(ekf_state_))
        return ekf_state_;

    // Priority 2: Use odometry-based tracking if active and valid
    if (odom_tracking_active_ && is_pose_valid(odom_pose_))
        return odom_pose_;

    // Priority 3: Use particle filter estimate if valid
    if (is_pose_valid(inferred_pose_))
        return inferred_pose_;
    
    // Priority 3: During initialization without pose estimate, use center of particles
    if (map_initialized_ && particles_.rows() > 0) {
        Eigen::Vector3d particle_center = particles_.colwise().mean();
        if (is_pose_valid(particle_center)) {
            return particle_center;
        }
    }
    
    // Priority 4: Fallback to last known good pose
    if (is_pose_valid(last_pose_))
        return last_pose_;
    
    // Default to origin
    return Eigen::Vector3d::Zero();
}

bool ParticleFilter::is_pose_valid(const Eigen::Vector3d& pose)
{
    return utils::validation::is_pose_valid(pose, MAX_POSE_RANGE);
}

void ParticleFilter::visualize(const rclcpp::Time &stamp)
{
    if (!DO_VIZ)
        return;

    // Use provided timestamp or fallback to current time
    rclcpp::Time viz_stamp = (stamp.nanoseconds() != 0) ? stamp : this->get_clock()->now();

    // RViz pose visualization (with vehicle frame offset)
    if (pose_pub_ && pose_pub_->get_subscription_count() > 0)
    {
        // Apply vehicle frame offset using TF
        Eigen::Vector3d offset_pose = apply_tf_offset(inferred_pose_);
        
        geometry_msgs::msg::PoseStamped ps;
        ps.header.stamp = viz_stamp;
        ps.header.frame_id = "map";
        ps.pose.position.x = offset_pose[0];
        ps.pose.position.y = offset_pose[1];
        ps.pose.orientation = utils::geometry::yaw_to_quaternion(inferred_pose_[2]);
        pose_pub_->publish(ps);
    }

    // RViz particle cloud (downsampled for performance)
    if (particle_pub_ && particle_pub_->get_subscription_count() > 0)
    {
        if (MAX_PARTICLES > MAX_VIZ_PARTICLES)
        {
            // Weighted downsampling
            std::discrete_distribution<int> particle_dist(weights_.begin(), weights_.end());
            Eigen::MatrixXd viz_particles(MAX_VIZ_PARTICLES, 3);

            for (int i = 0; i < MAX_VIZ_PARTICLES; ++i)
            {
                int idx = particle_dist(rng_);
                viz_particles.row(i) = particles_.row(idx);
            }

            publish_particles(viz_particles, viz_stamp);
        }
        else
        {
            publish_particles(particles_, viz_stamp);
        }
    }
}

void ParticleFilter::publish_particles(const Eigen::MatrixXd &particles_to_pub, const rclcpp::Time &stamp)
{
    // Apply vehicle frame offset to all particles
    Eigen::MatrixXd offset_particles = particles_to_pub;

    for (int i = 0; i < offset_particles.rows(); ++i) {
        Eigen::Vector3d particle_pose(offset_particles(i, 0), offset_particles(i, 1), offset_particles(i, 2));
        Eigen::Vector3d offset_pose = apply_tf_offset(particle_pose);
        offset_particles(i, 0) = offset_pose[0];
        offset_particles(i, 1) = offset_pose[1];
    }

    auto pa = utils::particles_to_pose_array(offset_particles);
    pa.header.stamp = (stamp.nanoseconds() != 0) ? stamp : this->get_clock()->now();
    pa.header.frame_id = "map";
    particle_pub_->publish(pa);
}



// ================================================================================================
// ODOMETRY TRACKING
// ================================================================================================
void ParticleFilter::initialize_odom_tracking(const Eigen::Vector3d& initial_pose, bool from_rviz)
{
    RCLCPP_INFO(this->get_logger(), "Odometry tracking init: [%.3f, %.3f, %.3f]", 
                initial_pose[0], initial_pose[1], initial_pose[2]);
    
    odom_pose_ = initial_pose;
    odom_reference_pose_ = initial_pose;
    
    if (last_pose_.norm() > 0) {
        odom_reference_odom_ = last_pose_;
    }
    
    pose_initialized_from_rviz_ = from_rviz;
    odom_tracking_active_ = true;
}

void ParticleFilter::update_odom_pose(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    if (!odom_tracking_active_) return;

    Eigen::Vector3d current_odom(msg->pose.pose.position.x, msg->pose.pose.position.y,
                                 utils::geometry::quaternion_to_yaw(msg->pose.pose.orientation));

    // odom 프레임 델타를 map 프레임으로 회전해 합성 (SE(2) 합성).
    // odom 프레임 → map 프레임 요 차이는 기준(앵커) 시점에 고정된다.
    const double dyaw_frame = utils::geometry::normalize_angle(
        odom_reference_pose_[2] - odom_reference_odom_[2]);
    const double c = std::cos(dyaw_frame), s = std::sin(dyaw_frame);

    const double dx  = current_odom[0] - odom_reference_odom_[0];
    const double dy  = current_odom[1] - odom_reference_odom_[1];
    const double dth = utils::geometry::normalize_angle(current_odom[2] - odom_reference_odom_[2]);

    odom_pose_[0] = odom_reference_pose_[0] + (c * dx - s * dy);
    odom_pose_[1] = odom_reference_pose_[1] + (s * dx + c * dy);
    odom_pose_[2] = utils::geometry::normalize_angle(odom_reference_pose_[2] + dth);
}

// ================================================================================================
// TF UTILITIES
// ================================================================================================
Eigen::Vector3d ParticleFilter::apply_tf_offset(const Eigen::Vector3d& pose_in_laser_frame)
{
    // Get the offset from F1Tenth system's static transform
    // (멤버 lidar_offset_x_/y_는 EKF의 base→laser 델타 변환에서도 공유)
    double &lidar_offset_x = lidar_offset_x_;
    double &lidar_offset_y = lidar_offset_y_;
    static bool offset_read = false;
    static int tf_retry_count = 0;
    static std::chrono::steady_clock::time_point last_tf_attempt = std::chrono::steady_clock::now();

    // Implement backoff strategy to prevent excessive TF lookups during startup
    if (!offset_read && tf_retry_count < 10) {
        auto now = std::chrono::steady_clock::now();
        auto time_since_last_attempt = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_tf_attempt);

        // Exponential backoff: wait longer between attempts
        int backoff_ms = 100 * (1 << std::min(tf_retry_count, 4)); // 100, 200, 400, 800, 1600ms max

        if (time_since_last_attempt.count() >= backoff_ms) {
            try {
                // Use shorter timeout to prevent blocking
                auto transform = tf_buffer_->lookupTransform(
                    BASE_FRAME, LASER_FRAME, tf2::TimePointZero, tf2::Duration(std::chrono::milliseconds(50)));

                lidar_offset_x = transform.transform.translation.x;
                lidar_offset_y = transform.transform.translation.y;
                offset_read = true;

                RCLCPP_INFO(this->get_logger(), "Using F1Tenth TF offset: x=%.3fm, y=%.3fm",
                           lidar_offset_x, lidar_offset_y);
            }
            catch (tf2::TransformException &ex) {
                tf_retry_count++;
                last_tf_attempt = now;

                if (tf_retry_count >= 10) {
                    RCLCPP_WARN(this->get_logger(), "Could not read F1Tenth TF after %d attempts, using default 0.27m: %s",
                               tf_retry_count, ex.what());
                    offset_read = true;  // Stop trying after max attempts
                }
            }
        }
    }

    // Apply offset: laser frame pose → base_link frame pose
    double cos_theta = std::cos(pose_in_laser_frame[2]);
    double sin_theta = std::sin(pose_in_laser_frame[2]);

    Eigen::Vector3d base_link_pose;
    base_link_pose[0] = pose_in_laser_frame[0] - lidar_offset_x * cos_theta + lidar_offset_y * sin_theta;
    base_link_pose[1] = pose_in_laser_frame[1] - lidar_offset_x * sin_theta - lidar_offset_y * cos_theta;
    base_link_pose[2] = pose_in_laser_frame[2];

    return base_link_pose;
}


} // namespace particle_filter_cpp

// ================================================================================================
// PROGRAM ENTRY POINT
// ================================================================================================
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<particle_filter_cpp::ParticleFilter>());
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(particle_filter_cpp::ParticleFilter)
