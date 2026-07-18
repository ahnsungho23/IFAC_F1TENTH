#include "local_planning/local_planner_node.hpp"

#include <cmath>
#include <algorithm>
#include <cctype>
#include <chrono>

using std::placeholders::_1;

namespace local_planning
{

LocalPlannerNode::LocalPlannerNode(const rclcpp::NodeOptions & options)
: Node("local_planner_node", options)
{
  initParameters();
  initInterfaces();
  RCLCPP_INFO(this->get_logger(), "LocalPlannerNode 시작됨 (lookahead=%d, degree=%d)",
    lookahead_wpnt_num_, poly_degree_);
}

void LocalPlannerNode::initParameters()
{
  lookahead_wpnt_num_ = this->declare_parameter<int>("lookahead_wpnt_num", 40);
  safety_margin_ = this->declare_parameter<double>("safety_margin", 0.65);
  wall_margin_ = this->declare_parameter<double>("wall_margin", 0.35);
  avoid_offset_ = this->declare_parameter<double>("avoid_offset", 1.0);
  poly_degree_ = this->declare_parameter<int>("poly_degree", 3);
  speed_reduction_ratio_ = this->declare_parameter<double>("speed_reduction_ratio", 0.6);
  publish_standalone_local_ = this->declare_parameter<bool>("publish_standalone_local", false);
  timer_period_ms_ = this->declare_parameter<int>("timer_period_ms", 50);

  last_eval_time_ = this->now();

  // 토픽 및 프레임 파라미터 선언
  global_waypoints_topic_ = this->declare_parameter<std::string>("global_waypoints_topic", "/global_waypoints");
  map_topic_ = this->declare_parameter<std::string>("map_topic", "/map");
  frenet_odom_topic_ = this->declare_parameter<std::string>("frenet_odom_topic", "/car_state/frenet/odom");
  scan_topic_ = this->declare_parameter<std::string>("scan_topic", "/scan");
  ot_waypoints_topic_ = this->declare_parameter<std::string>("ot_waypoints_topic", "/avoid_waypoints");
  local_waypoints_topic_ = this->declare_parameter<std::string>("local_waypoints_topic", "/local_waypoints");
  local_path_topic_ = this->declare_parameter<std::string>("local_path_topic", "/local_planning/path");
  exact_local_path_topic_ = this->declare_parameter<std::string>("exact_local_path_topic", "/local_path");
  marker_topic_ = this->declare_parameter<std::string>("marker_topic", "/local_planning/markers");
  frame_id_ = this->declare_parameter<std::string>("frame_id", "map");
}

void LocalPlannerNode::initInterfaces()
{
  // QoS 설정
  auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto qos_default = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

  // 구독자
  global_wpnts_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
    global_waypoints_topic_, qos_transient,
    std::bind(&LocalPlannerNode::onGlobalWaypoints, this, _1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, qos_transient,
    std::bind(&LocalPlannerNode::onMap, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    frenet_odom_topic_, qos_default,
    std::bind(&LocalPlannerNode::onOdom, this, _1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic_, rclcpp::SensorDataQoS(),
    std::bind(&LocalPlannerNode::onScan, this, _1));

  // 발행자
  ot_pub_ = this->create_publisher<f110_msgs::msg::OTWpntArray>(ot_waypoints_topic_, qos_default);
  local_wpnts_pub_ = this->create_publisher<f110_msgs::msg::WpntArray>(local_waypoints_topic_, qos_default);
  local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(local_path_topic_, qos_default);
  exact_local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(exact_local_path_topic_, qos_default);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, qos_default);

  // 타이머 (500ms = 2Hz 주기 실행, 0.5초마다 갱신)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_period_ms_),
    std::bind(&LocalPlannerNode::onTimer, this));

  // 동적 파라미터 변경 콜백 등록 (GUI 조작 시 RViz 즉각 반영)
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&LocalPlannerNode::onParameterChange, this, _1));
}

rcl_interfaces::msg::SetParametersResult LocalPlannerNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();
    if (name == "lookahead_wpnt_num") {
      lookahead_wpnt_num_ = param.as_int();
    } else if (name == "safety_margin") {
      safety_margin_ = param.as_double();
    } else if (name == "wall_margin") {
      wall_margin_ = param.as_double();
    } else if (name == "avoid_offset") {
      avoid_offset_ = param.as_double();
    } else if (name == "poly_degree") {
      poly_degree_ = param.as_int();
    } else if (name == "speed_reduction_ratio") {
      speed_reduction_ratio_ = param.as_double();
    }
  }

  // 파라미터 변경 직후 즉각 궤적 재생성 및 RViz 마커/경로 갱신
  triggerEventDrivenPlanning();

  return result;
}

void LocalPlannerNode::onGlobalWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  if (!msg || msg->wpnts.empty()) {
    RCLCPP_WARN(this->get_logger(), "수신된 /global_waypoints 가 비어있습니다.");
    return;
  }
  global_wpnts_ = *msg;
  has_global_ = true;
}

void LocalPlannerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!msg) return;
  grid_map_ = *msg;
  has_map_ = true;
  triggerEventDrivenPlanning();
}

void LocalPlannerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!msg) return;
  current_odom_ = *msg;
  has_odom_ = true;
  triggerEventDrivenPlanning();
}

void LocalPlannerNode::onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  if (!msg) return;

  std::string target_frame = frame_id_.empty() ? "map" : frame_id_;
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(
      target_frame, msg->header.frame_id,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "LiDAR TF 변환 실패: %s", ex.what());
    return;
  }

  std::vector<geometry_msgs::msg::Point> scan_points_map;
  scan_points_map.reserve(msg->ranges.size());

  double angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
    double r = msg->ranges[i];
    if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max) {
      continue;
    }
    double x_l = r * std::cos(angle);
    double y_l = r * std::sin(angle);
    double z_l = 0.0;

    const auto & t = tf_stamped.transform.translation;
    const auto & q = tf_stamped.transform.rotation;

    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    double ix =  qw * x_l + qy * z_l - qz * y_l;
    double iy =  qw * y_l + qz * x_l - qx * z_l;
    double iz =  qw * z_l + qx * y_l - qy * x_l;
    double iw = -qx * x_l - qy * y_l - qz * z_l;

    double x_m = ix * qw + iw * -qx + iy * -qz - iz * -qy + t.x;
    double y_m = iy * qw + iw * -qy + iz * -qx - ix * -qz + t.y;
    double z_m = iz * qw + iw * -qz + ix * -qy - iy * -qx + t.z;

    geometry_msgs::msg::Point pt;
    pt.x = x_m;
    pt.y = y_m;
    pt.z = z_m;
    scan_points_map.push_back(pt);
  }

  latest_scan_points_map_ = std::move(scan_points_map);
  has_scan_ = true;
  triggerEventDrivenPlanning();
}

void LocalPlannerNode::triggerEventDrivenPlanning()
{
  if (!has_global_ || !has_odom_ || (!has_map_ && !has_scan_)) {
    return;
  }
  auto now = this->now();
  if ((now - last_eval_time_).seconds() >= min_eval_interval_sec_) {
    last_eval_time_ = now;
    onTimer();
  }
}

std::optional<int> LocalPlannerNode::parseIndex(const std::string & s)
{
  if (s.empty()) return std::nullopt;
  size_t i = 0, j = s.size();
  while (i < j && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
  while (j > i && std::isspace(static_cast<unsigned char>(s[j - 1]))) --j;
  if (i >= j) return std::nullopt;
  std::string t = s.substr(i, j - i);
  for (char c : t) {
    if (!std::isdigit(static_cast<unsigned char>(c))) return std::nullopt;
  }
  try {
    return std::stoi(t);
  } catch (...) {
    return std::nullopt;
  }
}

int LocalPlannerNode::findClosestWaypointIndex(double x, double y)
{
  int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (total == 0) return 0;
  int best_idx = 0;
  double min_dist_sq = 1e9;
  for (int i = 0; i < total; ++i) {
    double dx = global_wpnts_.wpnts[i].x_m - x;
    double dy = global_wpnts_.wpnts[i].y_m - y;
    double dist_sq = dx * dx + dy * dy;
    if (dist_sq < min_dist_sq) {
      min_dist_sq = dist_sq;
      best_idx = i;
    }
  }
  return best_idx;
}

void LocalPlannerNode::detectObstaclesAndDecideDirection(
  int start_idx, int count,
  bool & obs_detected, bool & avoid_left,
  std::vector<int> & collision_indices,
  std::vector<ObstacleBound> & obs_bounds)
{
  obs_detected = false;
  avoid_left = true;
  collision_indices.clear();
  obs_bounds.assign(count, ObstacleBound{});

  if (!has_map_ && !has_scan_) return;

  int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (total == 0) return;

  double res = has_map_ ? grid_map_.info.resolution : 0.05;
  if (has_map_ && res <= 0.0) return;

  double origin_x = has_map_ ? grid_map_.info.origin.position.x : 0.0;
  double origin_y = has_map_ ? grid_map_.info.origin.position.y : 0.0;
  int width = has_map_ ? static_cast<int>(grid_map_.info.width) : 0;
  int height = has_map_ ? static_cast<int>(grid_map_.info.height) : 0;

  double total_room_left = 0.0;
  double total_room_right = 0.0;
  int obs_count = 0;

  for (int k = 0; k < count; ++k) {
    int idx = (start_idx + k) % total;
    const auto & wp = global_wpnts_.wpnts[idx];

    // 웨이포인트 주변 안전 반경(또는 좌우 트랙 폭) 내 셀 및 LiDAR 점 탐색
    double search_radius = std::max({safety_margin_, wp.d_left, wp.d_right});

    bool col_found_at_wp = false;
    double min_d = 1e9;
    double max_d = -1e9;

    if (has_map_ && res > 0.0) {
      int radius_cells = static_cast<int>(std::ceil(search_radius / res));
      int c_center = static_cast<int>((wp.x_m - origin_x) / res);
      int r_center = static_cast<int>((wp.y_m - origin_y) / res);

      for (int dr = -radius_cells; dr <= radius_cells; ++dr) {
        for (int dc = -radius_cells; dc <= radius_cells; ++dc) {
          if (dr * dr + dc * dc > radius_cells * radius_cells) continue;
          int c = c_center + dc;
          int r = r_center + dr;
          if (c < 0 || c >= width || r < 0 || r >= height) continue;

          int grid_idx = r * width + c;
          if (grid_idx >= 0 && grid_idx < static_cast<int>(grid_map_.data.size()) && grid_map_.data[grid_idx] > 50) {
            double cell_x = origin_x + c * res;
            double cell_y = origin_y + r * res;
            double dx = cell_x - wp.x_m;
            double dy = cell_y - wp.y_m;

            double lon_s = dx * std::cos(wp.psi_rad) + dy * std::sin(wp.psi_rad);
            if (std::abs(lon_s) > std::max(res * 3.0, 0.25)) continue;

            double lat_d = -dx * std::sin(wp.psi_rad) + dy * std::cos(wp.psi_rad);

            double safe_margin_left = std::min(wall_margin_, wp.d_left * 0.6);
            double safe_margin_right = std::min(wall_margin_, wp.d_right * 0.6);
            double left_wall_bound = wp.d_left - safe_margin_left;
            double right_wall_bound = -(wp.d_right - safe_margin_right);

            if (lat_d >= left_wall_bound || lat_d <= right_wall_bound) {
              continue;
            }

            col_found_at_wp = true;
            obs_detected = true;
            if (lat_d < min_d) min_d = lat_d;
            if (lat_d > max_d) max_d = lat_d;
          }
        }
      }
    }

    // 실시간 LiDAR 점군 탐색
    if (has_scan_) {
      for (const auto & pt : latest_scan_points_map_) {
        double dx = pt.x - wp.x_m;
        double dy = pt.y - wp.y_m;
        if (dx * dx + dy * dy > search_radius * search_radius) continue;

        double lon_s = dx * std::cos(wp.psi_rad) + dy * std::sin(wp.psi_rad);
        if (std::abs(lon_s) > std::max(res > 0.0 ? res * 3.0 : 0.15, 0.25)) continue;

        double lat_d = -dx * std::sin(wp.psi_rad) + dy * std::cos(wp.psi_rad);

        double safe_margin_left = std::min(wall_margin_, wp.d_left * 0.6);
        double safe_margin_right = std::min(wall_margin_, wp.d_right * 0.6);
        double left_wall_bound = wp.d_left - safe_margin_left;
        double right_wall_bound = -(wp.d_right - safe_margin_right);

        if (lat_d >= left_wall_bound || lat_d <= right_wall_bound) {
          continue;
        }

        col_found_at_wp = true;
        obs_detected = true;
        if (lat_d < min_d) min_d = lat_d;
        if (lat_d > max_d) max_d = lat_d;
      }
    }

    if (col_found_at_wp) {
      collision_indices.push_back(k);
      obs_bounds[k].min_d = min_d;
      obs_bounds[k].max_d = max_d;
      obs_bounds[k].collision = true;

      // 좌측 회피 가용 통로폭(room_left) vs 우측 회피 가용 통로폭(room_right) 계산
      double safe_margin_left = std::min(wall_margin_, wp.d_left * 0.6);
      double safe_margin_right = std::min(wall_margin_, wp.d_right * 0.6);
      double room_left = (wp.d_left - safe_margin_left) - (max_d + safety_margin_);
      double room_right = (min_d - safety_margin_) - (-(wp.d_right - safe_margin_right));
      total_room_left += std::max(0.0, room_left);
      total_room_right += std::max(0.0, room_right);
      obs_count++;
    }
  }

  // 회피 방향 결정: 좌우 가용 공간(Room)을 비교하여 더 안전하고 넓은 쪽으로 회피
  if (obs_detected && obs_count > 0) {
    if (total_room_left >= total_room_right) {
      avoid_left = true;  // 좌측 공간이 더 넓으므로 좌측 회피
    } else {
      avoid_left = false; // 우측 공간이 더 넓으므로 우측 회피
    }
  }
}

Eigen::VectorXd LocalPlannerNode::computeLeastSquaresSpline(
  const std::vector<double> & s_vals,
  const std::vector<double> & d_vals,
  const std::vector<double> & w_vals,
  int degree)
{
  int n = static_cast<int>(s_vals.size());
  if (n == 0) return Eigen::VectorXd::Zero(degree + 1);

  // 데이터 적합(Data fitting) 행렬 + 곡률 정규화(Curvature regularization) 행렬
  int reg_count = n; // 각 지점마다 2차 미분 정규화 행 추가
  Eigen::MatrixXd A(n + reg_count, degree + 1);
  Eigen::VectorXd b(n + reg_count);
  A.setZero();
  b.setZero();

  // 1. 가중 데이터 적합 행 (Weighted Data Fitting: sqrt(w) * (Ac - d) = 0)
  for (int i = 0; i < n; ++i) {
    double sqrt_w = std::sqrt(std::max(1e-4, w_vals[i]));
    b(i) = sqrt_w * d_vals[i];
    double s_pow = 1.0;
    for (int j = 0; j <= degree; ++j) {
      A(i, j) = sqrt_w * s_pow;
      s_pow *= s_vals[i];
    }
  }

  // 2. 곡률 평활화 정규화 행 (Curvature Regularization: sqrt(w_reg) * d''(s) = 0)
  double sqrt_w_reg = std::sqrt(0.5); // 평활화 가중치 0.5
  for (int i = 0; i < reg_count; ++i) {
    int row = n + i;
    double s = s_vals[i];
    b(row) = 0.0;
    if (degree >= 2) {
      A(row, 2) = sqrt_w_reg * 2.0;
    }
    if (degree >= 3) {
      A(row, 3) = sqrt_w_reg * 6.0 * s;
    }
    if (degree >= 4) {
      A(row, 4) = sqrt_w_reg * 12.0 * s * s;
    }
  }

  // QR 분해를 통한 가중 최소자승 해 도출
  Eigen::VectorXd c = A.colPivHouseholderQr().solve(b);
  return c;
}

void LocalPlannerNode::onTimer()
{
  if (!has_global_ || global_wpnts_.wpnts.empty()) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "글로벌 웨이포인트 대기 중...");
    return;
  }

  int total = static_cast<int>(global_wpnts_.wpnts.size());
  if (total == 0) return;

  // 1. 차량 현재 위치 기준 전방 Lookahead 탐색 윈도우 설정
  int start_idx = 0;
  if (has_odom_) {
    auto opt_idx = parseIndex(current_odom_.child_frame_id);
    if (opt_idx.has_value() && opt_idx.value() >= 0 && opt_idx.value() < total) {
      start_idx = opt_idx.value();
    } else {
      start_idx = findClosestWaypointIndex(current_odom_.pose.pose.position.x, current_odom_.pose.pose.position.y);
    }
  }
  int count = std::min(total, lookahead_wpnt_num_);

  // 2. 전방 Lookahead 구간 대상 장애물 간섭 검사 수행 (벽/장애물 분리)
  bool obs_detected = false;
  bool avoid_left = true;
  std::vector<int> collision_indices;
  std::vector<ObstacleBound> obs_bounds;
  detectObstaclesAndDecideDirection(start_idx, count, obs_detected, avoid_left, collision_indices, obs_bounds);

  // 3. 전체 웨이포인트를 글로벌 경로로 초기화 (기본 d = 0)
  f110_msgs::msg::WpntArray local_wpnts = global_wpnts_;
  local_wpnts.header.stamp = this->now();
  local_wpnts.header.frame_id = global_wpnts_.header.frame_id.empty() ? frame_id_ : global_wpnts_.header.frame_id;

  std::vector<geometry_msgs::msg::Point> debug_obs_points;

  // 4. 장애물이 감지된 경우, 해당 구간 주변을 윈도우로 잡아 가중 최소자승법 스플라인 회피 경로 생성 및 엮기 (4f00941 알고리즘 차용)
  if (obs_detected && !collision_indices.empty()) {
    int col_min = collision_indices.front();
    int col_max = collision_indices.back();

    // 장애물 감지 지점 전후로 부드러운 전환을 위한 스플라인 윈도우 구간 설정
    int win_margin = 10;
    int start_win = std::max(0, col_min - win_margin);
    int end_win = std::min(count - 1, col_max + win_margin);
    int win_count = end_win - start_win + 1;

    if (win_count > poly_degree_ + 1) {
      std::vector<double> s_vals(win_count, 0.0);
      std::vector<double> d_vals(win_count, 0.0);
      std::vector<double> w_vals(win_count, 1.0);

      double cum_s = 0.0;
      for (int i = 0; i < win_count; ++i) {
        int k = start_win + i;
        int idx = (start_idx + k) % total;
        if (i > 0) {
          int prev_idx = (start_idx + start_win + i - 1) % total;
          double dx = global_wpnts_.wpnts[idx].x_m - global_wpnts_.wpnts[prev_idx].x_m;
          double dy = global_wpnts_.wpnts[idx].y_m - global_wpnts_.wpnts[prev_idx].y_m;
          cum_s += std::hypot(dx, dy);
        }
        s_vals[i] = cum_s;

        const auto & wp = global_wpnts_.wpnts[idx];
        double max_safe_left = std::max(0.05, wp.d_left - wall_margin_);
        double max_safe_right = std::max(0.05, wp.d_right - wall_margin_);

        // 윈도우 양 끝단은 글로벌 경로($d=0$) 추종, 중앙 장애물 구간은 회피 오프셋 및 가중치($w=15.0$) 부여
        if (k >= col_min - 2 && k <= col_max + 2) {
          double target_d = 0.0;
          if (avoid_left) {
            double obs_max = obs_bounds[k].collision ? obs_bounds[k].max_d : 0.0;
            target_d = std::min(max_safe_left, std::max(obs_max + safety_margin_, avoid_offset_));
          } else {
            double obs_min = obs_bounds[k].collision ? obs_bounds[k].min_d : 0.0;
            target_d = std::max(-max_safe_right, std::min(obs_min - safety_margin_, -avoid_offset_));
          }
          d_vals[i] = target_d;
          w_vals[i] = 15.0; // 장애물 회피에 강한 가중치 부여
        } else {
          d_vals[i] = 0.0;
          w_vals[i] = 1.0;  // 글로벌 경로 추종 가중치
        }
      }

      // 가중 최소자승법(Weighted Least Squares)으로 3차 다항식 계수 계산
      Eigen::VectorXd coeffs = computeLeastSquaresSpline(s_vals, d_vals, w_vals, poly_degree_);
      // 계산된 스플라인을 해당 윈도우 구간의 글로벌 웨이포인트에 부드럽게 엮어 넣기 (Weaving into Closed Loop)
      for (int i = 0; i < win_count; ++i) {
        int k = start_win + i;
        int idx = (start_idx + k) % total;
        f110_msgs::msg::Wpnt & wp = local_wpnts.wpnts[idx];

        double s = s_vals[i];
        double d_spline = 0.0;
        double s_pow = 1.0;
        for (int j = 0; j <= poly_degree_ && j < coeffs.size(); ++j) {
          d_spline += coeffs(j) * s_pow;
          s_pow *= s;
        }

        // 글로벌 경로와의 경계면(윈도우 양단)에서 완벽한 C1 연속성(기울기 0, 오프셋 0)을 보장하기 위한 부드러운 테이퍼링 적용
        double taper = 1.0;
        int taper_len = std::min(6, win_count / 3);
        if (i < taper_len) {
          double angle = (M_PI / 2.0) * (static_cast<double>(i) / taper_len);
          taper = std::sin(angle) * std::sin(angle);
        } else if (i > win_count - 1 - taper_len) {
          double angle = (M_PI / 2.0) * (static_cast<double>(win_count - 1 - i) / taper_len);
          taper = std::sin(angle) * std::sin(angle);
        }
        d_spline *= taper;

        // 최종 벽 안전계수 클램핑: 어떠한 경우에도 트랙 경계선(Wall Boundary) 침범 금지
        double max_safe_left = std::max(0.05, wp.d_left - wall_margin_);
        double max_safe_right = std::max(0.05, wp.d_right - wall_margin_);
        double d_final = std::clamp(d_spline, -max_safe_right, max_safe_left);

        // 회피 오프셋을 지도 Cartesian 좌표(x, y)에 반영하여 글로벌 경로와 엮기
        double cos_psi = std::cos(wp.psi_rad);
        double sin_psi = std::sin(wp.psi_rad);
        wp.x_m -= d_final * sin_psi;
        wp.y_m += d_final * cos_psi;
        wp.d_m = d_final;

        if (std::abs(d_final) > 0.05) {
          wp.vx_mps = std::max(wp.vx_mps * std::max(0.8, speed_reduction_ratio_), 3.0);
        }
      }
    }

    // 디버깅용 충돌 지점 수집
    for (int k : collision_indices) {
      int idx = (start_idx + k) % total;
      geometry_msgs::msg::Point pt;
      pt.x = global_wpnts_.wpnts[idx].x_m;
      pt.y = global_wpnts_.wpnts[idx].y_m;
      pt.z = 0.1;
      debug_obs_points.push_back(pt);
    }
  }

  // 5. OTWpntArray (avoidance 토픽) 발행 -> wpnt_publisher 와 자동 연동
  f110_msgs::msg::OTWpntArray ot_msg;
  ot_msg.header = local_wpnts.header;
  ot_msg.last_switch_time = this->now();
  ot_msg.side_switch = obs_detected;
  ot_msg.ot_side = avoid_left ? "left" : "right";
  ot_msg.ot_line = "least_squares_spline_closed_loop";
  if (obs_detected && !collision_indices.empty()) {
    ot_msg.wpnts = local_wpnts.wpnts;
  } else {
    ot_msg.wpnts.clear(); // 장애물 미감지 시 빈 배열 발행으로 즉각 회피 상태 해제 유도
  }
  ot_pub_->publish(ot_msg);

  // 6. 스탠드얼론 로컬 웨이포인트 발행 (옵션)
  if (publish_standalone_local_) {
    local_wpnts_pub_->publish(local_wpnts);
  }

  // 7. 하나의 닫힌 곡선(Closed Curve)으로 엮인 "local path" 메시지 생성
  nav_msgs::msg::Path path_msg;
  path_msg.header = local_wpnts.header;
  path_msg.poses.reserve(total);
  for (const auto & w : local_wpnts.wpnts) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = local_wpnts.header;
    ps.pose.position.x = w.x_m;
    ps.pose.position.y = w.y_m;
    ps.pose.position.z = 0.05; // 지면 위 약간 부양
    ps.pose.orientation.w = 1.0;
    path_msg.poses.push_back(ps);
  }

  // /local_path ("local path") 및 /local_planning/path로 동시 발행
  exact_local_path_pub_->publish(path_msg);
  local_path_pub_->publish(path_msg);

  // 8. RViz 시각화 마커 발행
  publishDebugVisualization(local_wpnts, debug_obs_points);
}

void LocalPlannerNode::publishDebugVisualization(
  const f110_msgs::msg::WpntArray & local_wpnts,
  const std::vector<geometry_msgs::msg::Point> & obs_points)
{
  visualization_msgs::msg::MarkerArray markers;

  // 마커 1: 장애물 간섭 지점 (적색 구체)
  visualization_msgs::msg::Marker obs_marker;
  obs_marker.header = local_wpnts.header;
  obs_marker.ns = "collision_points";
  obs_marker.id = 0;
  obs_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  obs_marker.action = visualization_msgs::msg::Marker::ADD;
  obs_marker.pose.orientation.w = 1.0;
  obs_marker.scale.x = 0.4;
  obs_marker.scale.y = 0.4;
  obs_marker.scale.z = 0.4;
  obs_marker.color.r = 1.0f;
  obs_marker.color.g = 0.0f;
  obs_marker.color.b = 0.0f;
  obs_marker.color.a = 0.8f;
  obs_marker.points = obs_points;
  markers.markers.push_back(obs_marker);

  // 마커 2: 최소자승법 스플라인 로컬 경로
  visualization_msgs::msg::Marker path_marker;
  path_marker.header = local_wpnts.header;
  path_marker.ns = "least_squares_spline";
  path_marker.id = 1;
  path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::msg::Marker::ADD;
  path_marker.pose.orientation.w = 1.0;
  path_marker.scale.x = 0.1;
  path_marker.color.r = 0.0f;
  path_marker.color.g = 1.0f;
  path_marker.color.b = 0.2f;
  path_marker.color.a = 0.9f;
  for (const auto & w : local_wpnts.wpnts) {
    geometry_msgs::msg::Point pt;
    pt.x = w.x_m;
    pt.y = w.y_m;
    pt.z = 0.1;
    path_marker.points.push_back(pt);
  }
  markers.markers.push_back(path_marker);

  marker_pub_->publish(markers);
}

}  // namespace local_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::LocalPlannerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
