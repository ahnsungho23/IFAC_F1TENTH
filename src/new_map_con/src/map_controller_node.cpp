#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <f110_msgs/msg/wpnt.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <chrono>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

std::string trim(const std::string & input)
{
  const auto first = input.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = input.find_last_not_of(" \t\r\n");
  return input.substr(first, last - first + 1);
}

std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> out;
  std::stringstream stream(line);
  std::string item;
  while (std::getline(stream, item, ',')) {
    out.push_back(trim(item));
  }
  if (!line.empty() && line.back() == ',') {
    out.emplace_back("");
  }
  return out;
}

bool isAbsolutePath(const std::string & path)
{
  return !path.empty() && path.front() == '/';
}

bool fileExists(const std::string & path)
{
  std::ifstream file(path);
  return file.good();
}

// 선행 `~` 또는 `$HOME`을 $HOME으로 치환한다. ROS 2 파라미터 YAML은 셸이 아니라서
// 이 확장을 해주지 않으므로, 설정 파일에 홈 기준 경로를 쓰려면 노드가 직접 펼쳐야 한다.
std::string expandHome(const std::string & path)
{
  const char * home = std::getenv("HOME");
  if (home == nullptr) {
    return path;
  }
  if (path.rfind("~/", 0) == 0) {
    return std::string(home) + path.substr(1);
  }
  if (path.rfind("$HOME/", 0) == 0) {
    return std::string(home) + path.substr(5);
  }
  return path;
}

std::string joinPath(const std::string & root, const std::string & path)
{
  if (root.empty()) {
    return path;
  }
  if (root.back() == '/') {
    return root + path;
  }
  return root + "/" + path;
}

double normalizeAngle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

double yawFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double distance2d(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::hypot(dx, dy);
}

double clamp(double value, double min_value, double max_value)
{
  return std::max(min_value, std::min(value, max_value));
}

}  // namespace

class MapController : public rclcpp::Node
{
public:
  MapController()
  : Node("map_controller")
  {
    declareParameters();
    readParameters();

    global_pub_ = create_publisher<f110_msgs::msg::WpntArray>(
      global_waypoints_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    steering_pub_ = create_publisher<visualization_msgs::msg::Marker>(steering_marker_topic_, 10);
    lookahead_pub_ = create_publisher<visualization_msgs::msg::Marker>(lookahead_marker_topic_, 10);
    waypoint_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      waypoint_marker_topic_,
      10);
    l1_pub_ = create_publisher<geometry_msgs::msg::Point>(l1_topic_, 10);

    local_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      local_waypoints_topic_, 10,
      [this](const f110_msgs::msg::WpntArray::SharedPtr msg) {onLocalWaypoints(msg);});
    pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onPose(msg);});
    speed_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      speed_topic_, 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onSpeed(msg);});
    state_sub_ = create_subscription<std_msgs::msg::String>(
      state_topic_, 10, [this](const std_msgs::msg::String::SharedPtr msg) {state_ = msg->data;});
    if (!imu_topic_.empty()) {
      imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 10, [this](const sensor_msgs::msg::Imu::SharedPtr msg) {onImu(msg);});
    } else {
      RCLCPP_INFO(get_logger(), "IMU subscription disabled because imu_topic is empty.");
    }

    if (!global_waypoints_csv_.empty()) {
      global_waypoints_ = loadWaypointsFromCsv(resolvePackagePath(global_waypoints_csv_));
      has_global_waypoints_ = !global_waypoints_.wpnts.empty();
      if (has_global_waypoints_) {
        global_pub_->publish(global_waypoints_);
        RCLCPP_INFO(
          get_logger(), "Loaded %zu global waypoints from %s",
          global_waypoints_.wpnts.size(), resolvePackagePath(global_waypoints_csv_).c_str());
      }
    }

    const double control_period = 1.0 / std::max(control_rate_hz_, 1.0);
    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(control_period), [this]() {controlLoop();});

    if (publish_global_waypoints_) {
      const double publish_period = 1.0 / std::max(global_publish_rate_hz_, 0.1);
      global_timer_ = create_wall_timer(
        std::chrono::duration<double>(publish_period), [this]() {publishGlobalWaypoints();});
    }

    RCLCPP_INFO(
      get_logger(), "new_map_con C++ map_controller started (simulator=%s, pose=%s, speed=%s).",
      simulator_ ? "true" : "false", pose_topic_.c_str(), speed_topic_.c_str());
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("global_waypoints_csv", "maps/fuck_f1.csv");
    declare_parameter<std::string>("package_resource_root", "");
    declare_parameter<std::string>("frame_id", "map");
    declare_parameter<std::string>("base_frame_id", "base_link");
    declare_parameter<std::string>("active_state", "GB_TRACK");
    declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
    declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
    declare_parameter<std::string>("drive_topic", "");
    declare_parameter<std::string>("pose_topic", "");
    declare_parameter<std::string>("speed_topic", "");
    declare_parameter<std::string>("state_topic", "/state");
    declare_parameter<std::string>("imu_topic", "");
    declare_parameter<std::string>("steering_marker_topic", "steering");
    declare_parameter<std::string>("lookahead_marker_topic", "lookahead_point");
    declare_parameter<std::string>("waypoint_marker_topic", "my_waypoints");
    declare_parameter<std::string>("l1_topic", "l1_distance");

    declare_parameter<bool>("simulator", false);
    declare_parameter<bool>("publish_global_waypoints", true);
    declare_parameter<bool>("use_local_waypoints", true);
    declare_parameter<bool>("fallback_to_global_waypoints", true);
    declare_parameter<bool>("stop_on_state_mismatch", false);
    declare_parameter<bool>("publish_debug_markers", true);

    declare_parameter<double>("control_rate_hz", 40.0);
    declare_parameter<double>("global_publish_rate_hz", 1.0);
    declare_parameter<double>("min_lookahead_distance", 1.5);
    declare_parameter<double>("max_lookahead_distance", 5.0);
    declare_parameter<double>("lookahead_gain", 0.5);
    declare_parameter<double>("lookahead_speed_gain", 0.3);
    declare_parameter<double>("speed_lookahead_time", 0.15);
    declare_parameter<double>("speed_lookahead_for_steering", 0.0);
    declare_parameter<double>("lateral_error_coeff", 1.0);
    declare_parameter<double>("min_lateral_error", 0.01);
    declare_parameter<double>("max_lateral_error", 0.5);
    declare_parameter<double>("curvature_scale", 0.8);
    declare_parameter<double>("wheelbase", 0.33);
    declare_parameter<double>("max_steering_angle", 0.42);
    declare_parameter<double>("max_steering_delta", 0.4);
    declare_parameter<double>("acceleration_scaler_for_steering", 1.2);
    declare_parameter<double>("deceleration_scaler_for_steering", 0.9);
    declare_parameter<double>("start_scale_speed", 7.0);
    declare_parameter<double>("end_scale_speed", 8.0);
    declare_parameter<double>("downscale_factor", 0.1);
    declare_parameter<double>("default_track_bound", 0.0);  // Fallback when CSV lacks d_left/d_right (should not happen with new CSVs)
    declare_parameter<double>("local_waypoint_timeout_sec", 0.5);
    declare_parameter<double>("waypoint_safety_timeout_sec", 1.0);

    declare_parameter<int>("curvature_window", 20);
    declare_parameter<int>("acceleration_filter_size", 10);
    declare_parameter<int>("marker_stride", 3);

    declare_parameter<std::string>("simulator_drive_topic", "/drive");
    declare_parameter<std::string>("simulator_pose_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("simulator_speed_topic", "/ego_racecar/odom");
    declare_parameter<std::string>("simulator_imu_topic", "");
    declare_parameter<std::string>("vehicle_drive_topic", "/drive");
    declare_parameter<std::string>("vehicle_pose_topic", "/pf/pose/odom");
    declare_parameter<std::string>("vehicle_speed_topic", "/odom");
    declare_parameter<std::string>("vehicle_imu_topic", "/sensors/imu/raw");
  }

  void readParameters()
  {
    global_waypoints_csv_ = get_parameter("global_waypoints_csv").as_string();
    package_resource_root_ = get_parameter("package_resource_root").as_string();
    frame_id_ = get_parameter("frame_id").as_string();
    base_frame_id_ = get_parameter("base_frame_id").as_string();
    active_state_ = get_parameter("active_state").as_string();
    state_ = active_state_;
    global_waypoints_topic_ = get_parameter("global_waypoints_topic").as_string();
    local_waypoints_topic_ = get_parameter("local_waypoints_topic").as_string();
    state_topic_ = get_parameter("state_topic").as_string();
    steering_marker_topic_ = get_parameter("steering_marker_topic").as_string();
    lookahead_marker_topic_ = get_parameter("lookahead_marker_topic").as_string();
    waypoint_marker_topic_ = get_parameter("waypoint_marker_topic").as_string();
    l1_topic_ = get_parameter("l1_topic").as_string();

    simulator_ = get_parameter("simulator").as_bool();
    drive_topic_ =
      selectModeTopic("drive_topic", "simulator_drive_topic", "vehicle_drive_topic");
    pose_topic_ =
      selectModeTopic("pose_topic", "simulator_pose_topic", "vehicle_pose_topic");
    speed_topic_ =
      selectModeTopic("speed_topic", "simulator_speed_topic", "vehicle_speed_topic");
    imu_topic_ =
      selectModeTopic("imu_topic", "simulator_imu_topic", "vehicle_imu_topic");

    publish_global_waypoints_ = get_parameter("publish_global_waypoints").as_bool();
    use_local_waypoints_ = get_parameter("use_local_waypoints").as_bool();
    fallback_to_global_waypoints_ = get_parameter("fallback_to_global_waypoints").as_bool();
    stop_on_state_mismatch_ = get_parameter("stop_on_state_mismatch").as_bool();
    publish_debug_markers_ = get_parameter("publish_debug_markers").as_bool();

    control_rate_hz_ = get_parameter("control_rate_hz").as_double();
    global_publish_rate_hz_ = get_parameter("global_publish_rate_hz").as_double();
    min_lookahead_distance_ = get_parameter("min_lookahead_distance").as_double();
    max_lookahead_distance_ = get_parameter("max_lookahead_distance").as_double();
    lookahead_gain_ = get_parameter("lookahead_gain").as_double();
    lookahead_speed_gain_ = get_parameter("lookahead_speed_gain").as_double();
    speed_lookahead_time_ = get_parameter("speed_lookahead_time").as_double();
    speed_lookahead_for_steering_ = get_parameter("speed_lookahead_for_steering").as_double();
    lateral_error_coeff_ = get_parameter("lateral_error_coeff").as_double();
    min_lateral_error_ = get_parameter("min_lateral_error").as_double();
    max_lateral_error_ = get_parameter("max_lateral_error").as_double();
    curvature_scale_ = get_parameter("curvature_scale").as_double();
    wheelbase_ = get_parameter("wheelbase").as_double();
    max_steering_angle_ = get_parameter("max_steering_angle").as_double();
    max_steering_delta_ = get_parameter("max_steering_delta").as_double();
    acceleration_scaler_for_steering_ =
      get_parameter("acceleration_scaler_for_steering").as_double();
    deceleration_scaler_for_steering_ =
      get_parameter("deceleration_scaler_for_steering").as_double();
    start_scale_speed_ = get_parameter("start_scale_speed").as_double();
    end_scale_speed_ = get_parameter("end_scale_speed").as_double();
    downscale_factor_ = get_parameter("downscale_factor").as_double();
    default_track_bound_ = get_parameter("default_track_bound").as_double();
    local_waypoint_timeout_sec_ = get_parameter("local_waypoint_timeout_sec").as_double();
    waypoint_safety_timeout_sec_ = get_parameter("waypoint_safety_timeout_sec").as_double();

    curvature_window_ = get_parameter("curvature_window").as_int();
    acceleration_filter_size_ = get_parameter("acceleration_filter_size").as_int();
    marker_stride_ = std::max(1, static_cast<int>(get_parameter("marker_stride").as_int()));
    acceleration_samples_.clear();
  }

  std::string selectModeTopic(
    const std::string & override_parameter,
    const std::string & simulator_parameter,
    const std::string & vehicle_parameter) const
  {
    const auto override_topic = get_parameter(override_parameter).as_string();
    if (!override_topic.empty()) {
      return override_topic;
    }
    return get_parameter(simulator_ ? simulator_parameter : vehicle_parameter).as_string();
  }

  std::string resolvePackagePath(const std::string & raw_path) const
  {
    // ROS 2 파라미터 YAML은 `~`/`$HOME`을 펼치지 않으므로 여기서 직접 펼친다.
    const auto path = expandHome(raw_path);
    if (path.empty() || isAbsolutePath(path)) {
      return path;
    }
    if (fileExists(path)) {
      return path;
    }
    if (!package_resource_root_.empty()) {
      const auto source_path = joinPath(package_resource_root_, path);
      if (fileExists(source_path)) {
        return source_path;
      }
    }
    const auto share = ament_index_cpp::get_package_share_directory("new_map_con");
    return share + "/" + path;
  }

  f110_msgs::msg::WpntArray loadWaypointsFromCsv(const std::string & path)
  {
    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open waypoint CSV: " + path);
    }

    std::string header_line;
    if (!std::getline(file, header_line)) {
      throw std::runtime_error("Waypoint CSV is empty: " + path);
    }
    const auto header = splitCsvLine(header_line);
    std::unordered_map<std::string, std::size_t> columns;
    for (std::size_t i = 0; i < header.size(); ++i) {
      columns[header[i]] = i;
    }

    auto has = [&columns](const std::string & name) {
        return columns.find(name) != columns.end();
      };
    auto read = [&columns](const std::vector<std::string> & row, const std::string & name,
        double fallback) {
        const auto it = columns.find(name);
        if (it == columns.end() || it->second >= row.size() || row[it->second].empty()) {
          return fallback;
        }
        return std::stod(row[it->second]);
      };

    if (!has("x_m") || !has("y_m") || !has("psi_rad") || !has("kappa_radpm") ||
      !has("vx_mps") || !has("ax_mps2"))
    {
      throw std::runtime_error(
              "Waypoint CSV must contain x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2");
    }

    f110_msgs::msg::WpntArray out;
    out.header.frame_id = frame_id_;
    out.header.stamp = now();

    std::string line;
    int generated_id = 0;
    double cumulative_s = 0.0;
    f110_msgs::msg::Wpnt previous;
    bool has_previous = false;
    while (std::getline(file, line)) {
      if (trim(line).empty()) {
        continue;
      }
      const auto row = splitCsvLine(line);
      f110_msgs::msg::Wpnt wp;
      wp.id = static_cast<int32_t>(read(row, "id", generated_id));
      wp.x_m = read(row, "x_m", 0.0);
      wp.y_m = read(row, "y_m", 0.0);
      if (has("s")) {
        wp.s_m = read(row, "s", 0.0);
      } else if (has("s_m")) {
        wp.s_m = read(row, "s_m", 0.0);
      } else {
        if (has_previous) {
          cumulative_s += distance2d(previous.x_m, previous.y_m, wp.x_m, wp.y_m);
        }
        wp.s_m = cumulative_s;
      }
      wp.d_m = read(row, "d_m", 0.0);
      wp.d_right = read(row, "d_right", default_track_bound_);
      wp.d_left = read(row, "d_left", default_track_bound_);
      wp.psi_rad = read(row, "psi_rad", 0.0);
      wp.kappa_radpm = read(row, "kappa_radpm", 0.0);
      wp.vx_mps = read(row, "vx_mps", 0.0);
      wp.ax_mps2 = read(row, "ax_mps2", 0.0);
      out.wpnts.push_back(wp);
      previous = wp;
      has_previous = true;
      ++generated_id;
    }

    return out;
  }

  void onLocalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    local_waypoints_ = *msg;
    has_local_waypoints_ = !local_waypoints_.wpnts.empty();
    last_local_waypoints_time_ = now();
  }

  void onPose(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_yaw_ = yawFromOdom(*msg);
    has_pose_ = true;
  }

  void onSpeed(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    speed_now_ = msg->twist.twist.linear.x;
    has_speed_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    acceleration_samples_.push_front(-msg->linear_acceleration.y);
    const auto max_size = static_cast<std::size_t>(std::max(1, acceleration_filter_size_));
    while (acceleration_samples_.size() > max_size) {
      acceleration_samples_.pop_back();
    }
  }

  void publishGlobalWaypoints()
  {
    if (!has_global_waypoints_ || global_waypoints_.wpnts.empty()) {
      return;
    }
    global_waypoints_.header.stamp = now();
    global_pub_->publish(global_waypoints_);
  }

  const std::vector<f110_msgs::msg::Wpnt> * selectWaypoints(bool & circular)
  {
    circular = false;
    if (use_local_waypoints_ && has_local_waypoints_ && !local_waypoints_.wpnts.empty()) {
      const double age = (now() - last_local_waypoints_time_).seconds();
      if (age <= local_waypoint_timeout_sec_) {
        return &local_waypoints_.wpnts;
      }
    }
    if (fallback_to_global_waypoints_ && has_global_waypoints_ &&
      !global_waypoints_.wpnts.empty())
    {
      circular = true;
      return &global_waypoints_.wpnts;
    }
    return nullptr;
  }

  std::size_t nearestWaypoint(
    double x, double y, const std::vector<f110_msgs::msg::Wpnt> & wpnts) const
  {
    std::size_t best = 0;
    double best_dist = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < wpnts.size(); ++i) {
      const double dist = distance2d(x, y, wpnts[i].x_m, wpnts[i].y_m);
      if (dist < best_dist) {
        best_dist = dist;
        best = i;
      }
    }
    return best;
  }

  std::size_t stepIndex(std::size_t index, bool circular, std::size_t size) const
  {
    if (size == 0) {
      return 0;
    }
    if (index + 1 < size) {
      return index + 1;
    }
    return circular ? 0 : index;
  }

  std::size_t waypointAtDistance(
    const std::vector<f110_msgs::msg::Wpnt> & wpnts,
    std::size_t start,
    double target_distance,
    bool circular) const
  {
    if (wpnts.empty()) {
      return 0;
    }
    double accumulated = 0.0;
    std::size_t current = start;
    for (std::size_t steps = 0; steps + 1 < wpnts.size(); ++steps) {
      const std::size_t next = stepIndex(current, circular, wpnts.size());
      if (next == current) {
        return current;
      }
      accumulated += distance2d(
        wpnts[current].x_m, wpnts[current].y_m, wpnts[next].x_m, wpnts[next].y_m);
      current = next;
      if (accumulated >= target_distance) {
        return current;
      }
    }
    return current;
  }

  double meanCurvature(
    const std::vector<f110_msgs::msg::Wpnt> & wpnts,
    std::size_t start,
    bool circular) const
  {
    if (wpnts.empty()) {
      return 0.0;
    }
    const int count = std::max(1, std::min(curvature_window_, static_cast<int>(wpnts.size())));
    double sum = 0.0;
    std::size_t index = start;
    for (int i = 0; i < count; ++i) {
      sum += std::abs(wpnts[index].kappa_radpm);
      index = stepIndex(index, circular, wpnts.size());
    }
    return sum / static_cast<double>(count);
  }

  double signedLateralError(const f110_msgs::msg::Wpnt & wp, double x, double y) const
  {
    const double dx = x - wp.x_m;
    const double dy = y - wp.y_m;
    const double normal_x = -std::sin(wp.psi_rad);
    const double normal_y = std::cos(wp.psi_rad);
    return dx * normal_x + dy * normal_y;
  }

  double lateralErrorNorm(double lateral_error) const
  {
    const double abs_error = std::abs(lateral_error);
    const double clipped = clamp(abs_error, min_lateral_error_, max_lateral_error_);
    const double denom = std::max(1e-6, max_lateral_error_ - min_lateral_error_);
    return 0.5 * ((clipped - min_lateral_error_) / denom);
  }

  double adjustSpeed(double global_speed, double lateral_norm, double curvature) const
  {
    const double lat_coeff = clamp(lateral_error_coeff_, 0.0, 1.0);
    const double curvature_norm = clamp(
      2.0 * (curvature / std::max(
        curvature_scale_,
        1e-6)) - 2.0, 0.0, 1.0);
    return global_speed *
           (1.0 - lat_coeff + lat_coeff * std::exp(-(lateral_norm * 2.0) * curvature_norm));
  }

  double averageAcceleration() const
  {
    if (acceleration_samples_.empty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (const auto value : acceleration_samples_) {
      sum += value;
    }
    return sum / static_cast<double>(acceleration_samples_.size());
  }

  double scaleSteering(double steering, double speed)
  {
    const double acc = averageAcceleration();
    if (acc >= 1.0) {
      steering *= acceleration_scaler_for_steering_;
    } else if (acc <= -1.0) {
      steering *= deceleration_scaler_for_steering_;
    }

    const double speed_diff = std::max(0.1, end_scale_speed_ - start_scale_speed_);
    const double speed_factor =
      1.0 - clamp((speed - start_scale_speed_) / speed_diff, 0.0, 1.0) * downscale_factor_;
    steering *= speed_factor;

    const double lower = previous_steering_angle_ - max_steering_delta_;
    const double upper = previous_steering_angle_ + max_steering_delta_;
    steering = clamp(steering, lower, upper);
    steering = clamp(steering, -max_steering_angle_, max_steering_angle_);
    previous_steering_angle_ = steering;
    return steering;
  }

  double calculateSteering(
    const f110_msgs::msg::Wpnt & target, double lookahead_distance,
    double speed)
  {
    const double dx = target.x_m - pose_x_;
    const double dy = target.y_m - pose_y_;
    const double target_heading = std::atan2(dy, dx);
    const double eta = normalizeAngle(target_heading - pose_yaw_);
    const double safe_l1 = std::max(lookahead_distance, 1e-3);
    const double steering = std::atan2(2.0 * wheelbase_ * std::sin(eta), safe_l1);
    return scaleSteering(steering, speed);
  }

  void publishStop()
  {
    ackermann_msgs::msg::AckermannDriveStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = base_frame_id_;
    msg.drive.speed = 0.0;
    msg.drive.steering_angle = 0.0;
    drive_pub_->publish(msg);
  }

  void controlLoop()
  {
    if (stop_on_state_mismatch_ && !active_state_.empty() && state_ != active_state_) {
      publishStop();
      return;
    }
    if (!has_pose_ || !has_speed_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for pose and speed topics.");
      publishStop();
      return;
    }

    bool circular = false;
    const auto * wpnts = selectWaypoints(circular);
    if (wpnts == nullptr || wpnts->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "No usable waypoints. Waiting for CSV or local waypoints.");
      publishStop();
      return;
    }

    if (use_local_waypoints_ && has_local_waypoints_) {
      const double age = (now() - last_local_waypoints_time_).seconds();
      if (age > waypoint_safety_timeout_sec_ && !fallback_to_global_waypoints_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(), 1000, "Local waypoints timed out. Stopping.");
        publishStop();
        return;
      }
    }

    const std::size_t nearest = nearestWaypoint(pose_x_, pose_y_, *wpnts);
    const auto & nearest_wp = (*wpnts)[nearest];
    const double lateral_error = signedLateralError(nearest_wp, pose_x_, pose_y_);
    const double lateral_norm = lateralErrorNorm(lateral_error);
    const double curvature = meanCurvature(*wpnts, nearest, circular);

    double lookahead = lookahead_gain_ + std::abs(speed_now_) * lookahead_speed_gain_;
    lookahead = clamp(
      lookahead,
      std::max(min_lookahead_distance_, std::sqrt(2.0) * std::abs(lateral_error)),
      max_lookahead_distance_);

    const auto target_idx = waypointAtDistance(*wpnts, nearest, lookahead, circular);
    const auto & target_wp = (*wpnts)[target_idx];

    const double vx = std::cos(pose_yaw_) * speed_now_;
    const double vy = std::sin(pose_yaw_) * speed_now_;
    const double speed_x = pose_x_ + vx * speed_lookahead_time_;
    const double speed_y = pose_y_ + vy * speed_lookahead_time_;
    const std::size_t speed_idx = nearestWaypoint(speed_x, speed_y, *wpnts);
    double speed_command = adjustSpeed((*wpnts)[speed_idx].vx_mps, lateral_norm, curvature);
    speed_command = std::max(0.0, speed_command);

    const double steering = calculateSteering(target_wp, lookahead, speed_command);

    ackermann_msgs::msg::AckermannDriveStamped drive;
    drive.header.stamp = now();
    drive.header.frame_id = base_frame_id_;
    drive.drive.speed = speed_command;
    drive.drive.steering_angle = steering;
    drive_pub_->publish(drive);

    geometry_msgs::msg::Point l1_msg;
    l1_msg.x = static_cast<double>(nearest);
    l1_msg.y = lookahead;
    l1_msg.z = lateral_error;
    l1_pub_->publish(l1_msg);

    if (publish_debug_markers_) {
      publishLookaheadMarker(target_wp);
      publishSteeringMarker(steering);
      publishWaypointMarkers(*wpnts);
    }
  }

  void publishLookaheadMarker(const f110_msgs::msg::Wpnt & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id_;
    marker.header.stamp = now();
    marker.ns = "new_map_con";
    marker.id = 100;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.pose.position.x = target.x_m;
    marker.pose.position.y = target.y_m;
    marker.pose.orientation.w = 1.0;
    lookahead_pub_->publish(marker);
  }

  void publishSteeringMarker(double steering)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = base_frame_id_;
    marker.header.stamp = now();
    marker.ns = "new_map_con";
    marker.id = 50;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.6;
    marker.scale.y = 0.05;
    marker.scale.z = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    marker.pose.orientation.z = std::sin(steering * 0.5);
    marker.pose.orientation.w = std::cos(steering * 0.5);
    steering_pub_->publish(marker);
  }

  void publishWaypointMarkers(const std::vector<f110_msgs::msg::Wpnt> & wpnts)
  {
    visualization_msgs::msg::MarkerArray array;
    int marker_id = 0;
    for (std::size_t i = 0; i < wpnts.size(); i += static_cast<std::size_t>(marker_stride_)) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = frame_id_;
      marker.header.stamp = now();
      marker.ns = "new_map_con_waypoints";
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.08;
      marker.scale.y = 0.08;
      marker.scale.z = 0.08;
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      marker.pose.position.x = wpnts[i].x_m;
      marker.pose.position.y = wpnts[i].y_m;
      marker.pose.orientation.w = 1.0;
      array.markers.push_back(marker);
    }
    waypoint_pub_->publish(array);
  }

  std::string global_waypoints_csv_;
  std::string package_resource_root_;
  std::string frame_id_;
  std::string base_frame_id_;
  std::string active_state_;
  std::string state_;
  std::string global_waypoints_topic_;
  std::string local_waypoints_topic_;
  std::string drive_topic_;
  std::string pose_topic_;
  std::string speed_topic_;
  std::string state_topic_;
  std::string imu_topic_;
  std::string steering_marker_topic_;
  std::string lookahead_marker_topic_;
  std::string waypoint_marker_topic_;
  std::string l1_topic_;

  bool publish_global_waypoints_{true};
  bool use_local_waypoints_{true};
  bool fallback_to_global_waypoints_{true};
  bool stop_on_state_mismatch_{false};
  bool publish_debug_markers_{true};
  bool simulator_{false};

  double control_rate_hz_{40.0};
  double global_publish_rate_hz_{1.0};
  double min_lookahead_distance_{1.5};
  double max_lookahead_distance_{5.0};
  double lookahead_gain_{0.5};
  double lookahead_speed_gain_{0.3};
  double speed_lookahead_time_{0.15};
  double speed_lookahead_for_steering_{0.0};
  double lateral_error_coeff_{1.0};
  double min_lateral_error_{0.01};
  double max_lateral_error_{0.5};
  double curvature_scale_{0.8};
  double wheelbase_{0.33};
  double max_steering_angle_{0.42};
  double max_steering_delta_{0.4};
  double acceleration_scaler_for_steering_{1.2};
  double deceleration_scaler_for_steering_{0.9};
  double start_scale_speed_{7.0};
  double end_scale_speed_{8.0};
  double downscale_factor_{0.1};
  double default_track_bound_{0.0};
  double local_waypoint_timeout_sec_{0.5};
  double waypoint_safety_timeout_sec_{1.0};

  int curvature_window_{20};
  int acceleration_filter_size_{10};
  int marker_stride_{3};

  bool has_global_waypoints_{false};
  bool has_local_waypoints_{false};
  bool has_pose_{false};
  bool has_speed_{false};
  f110_msgs::msg::WpntArray global_waypoints_;
  f110_msgs::msg::WpntArray local_waypoints_;
  rclcpp::Time last_local_waypoints_time_{0, 0, RCL_ROS_TIME};

  double pose_x_{0.0};
  double pose_y_{0.0};
  double pose_yaw_{0.0};
  double speed_now_{0.0};
  double previous_steering_angle_{0.0};
  std::deque<double> acceleration_samples_;

  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr global_pub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steering_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr lookahead_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr l1_pub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr local_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr global_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MapController>());
  } catch (const std::exception & exc) {
    std::cerr << "new_map_con map_controller failed: " << exc.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
