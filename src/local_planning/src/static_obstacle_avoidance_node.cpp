#include "local_planning/static_obstacle_avoidance_node.hpp"

#include <algorithm>
#include <cmath>

using std::placeholders::_1;

namespace local_planning
{

StaticObstacleAvoidanceNode::StaticObstacleAvoidanceNode(const rclcpp::NodeOptions & options)
: Node("static_obstacle_avoidance_node", options)
{
  declareAndLoadParameters();

  auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  global_sub_ = this->create_subscription<f110_msgs::msg::WpntArray>(
    "/global_waypoints", qos_transient,
    std::bind(&StaticObstacleAvoidanceNode::globalWaypointsCallback, this, _1));

  frenet_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/car_state/frenet/odom", qos_reliable,
    std::bind(&StaticObstacleAvoidanceNode::frenetOdomCallback, this, _1));

  obs_sub_ = this->create_subscription<f110_msgs::msg::ObstacleArray>(
    "/obstacles", qos_reliable,
    std::bind(&StaticObstacleAvoidanceNode::obstaclesCallback, this, _1));

  ot_wpnts_pub_ = this->create_publisher<f110_msgs::msg::OTWpntArray>(
    "/planner/avoidance/otwpnts", qos_reliable);

  local_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(
    "/local_planning/avoidance_path", qos_reliable);

  cand_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/local_planning/candidate_paths", qos_reliable);

  auto period = std::chrono::duration<double>(1.0 / planning_frequency_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&StaticObstacleAvoidanceNode::planningTimerCallback, this));

  // 동적 파라미터 콜백 등록 (GUI 조작 시 즉각 반영)
  using std::placeholders::_1;
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&StaticObstacleAvoidanceNode::onParameterChange, this, _1));

  last_avoidance_time_ = this->now();
  last_eval_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Static Obstacle Avoidance Planner Node Initialized.");
}

void StaticObstacleAvoidanceNode::declareAndLoadParameters()
{
  planning_frequency_ = this->declare_parameter<double>("planning_frequency", 50.0);
  lookahead_distance_ = this->declare_parameter<double>("lookahead_distance", 15.0);
  corridor_width_ = this->declare_parameter<double>("corridor_width", 0.65);
  safety_margin_ = this->declare_parameter<double>("safety_margin", 0.28);
  passing_buffer_s_ = this->declare_parameter<double>("passing_buffer_s", 1.2);
  waypoint_step_ = this->declare_parameter<double>("waypoint_step", 0.1);
  max_lat_accel_ = this->declare_parameter<double>("max_lat_accel", 6.0);
  min_speed_ = this->declare_parameter<double>("min_speed", 1.5);
  static_vel_threshold_ = this->declare_parameter<double>("static_vel_threshold", 0.35);

  weight_obs_ = this->declare_parameter<double>("weight_obs", 10.0);
  weight_lat_ = this->declare_parameter<double>("weight_lat", 1.0);
  weight_smooth_ = this->declare_parameter<double>("weight_smooth", 2.5);
  weight_speed_ = this->declare_parameter<double>("weight_speed", 1.5);
}

rcl_interfaces::msg::SetParametersResult StaticObstacleAvoidanceNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "Success";

  for (const auto & param : parameters) {
    const std::string & name = param.get_name();
    if (name == "planning_frequency") {
      planning_frequency_ = param.as_double();
    } else if (name == "lookahead_distance") {
      lookahead_distance_ = param.as_double();
    } else if (name == "corridor_width") {
      corridor_width_ = param.as_double();
    } else if (name == "safety_margin") {
      safety_margin_ = param.as_double();
    } else if (name == "passing_buffer_s") {
      passing_buffer_s_ = param.as_double();
    } else if (name == "waypoint_step") {
      waypoint_step_ = param.as_double();
    } else if (name == "max_lat_accel") {
      max_lat_accel_ = param.as_double();
    } else if (name == "min_speed") {
      min_speed_ = param.as_double();
    } else if (name == "static_vel_threshold") {
      static_vel_threshold_ = param.as_double();
    } else if (name == "weight_obs") {
      weight_obs_ = param.as_double();
    } else if (name == "weight_lat") {
      weight_lat_ = param.as_double();
    } else if (name == "weight_smooth") {
      weight_smooth_ = param.as_double();
    } else if (name == "weight_speed") {
      weight_speed_ = param.as_double();
    }
  }

  // 파라미터 변경 즉시 회피 궤적 재연산 및 RViz 반영
  triggerEventDrivenPlanning();

  return result;
}

void StaticObstacleAvoidanceNode::globalWaypointsCallback(
  const f110_msgs::msg::WpntArray::SharedPtr msg)
{
  if (msg->wpnts.empty()) {
    return;
  }
  global_wpnts_ = *msg;
  track_length_ = global_wpnts_.wpnts.back().s_m;
  has_global_ = true;
}

void StaticObstacleAvoidanceNode::frenetOdomCallback(
  const nav_msgs::msg::Odometry::SharedPtr msg)
{
  frenet_odom_ = *msg;
  has_odom_ = true;
  triggerEventDrivenPlanning();
}

void StaticObstacleAvoidanceNode::obstaclesCallback(
  const f110_msgs::msg::ObstacleArray::SharedPtr msg)
{
  obstacles_ = *msg;
  has_obs_ = true;
  triggerEventDrivenPlanning();
}

void StaticObstacleAvoidanceNode::triggerEventDrivenPlanning()
{
  if (!has_global_ || !has_odom_ || !has_obs_) {
    return;
  }
  auto now = this->now();
  if ((now - last_eval_time_).seconds() >= min_eval_interval_sec_) {
    last_eval_time_ = now;
    evaluateAndPublishLocalPath();
  }
}

void StaticObstacleAvoidanceNode::planningTimerCallback()
{
  if (!has_global_ || !has_odom_) {
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for global waypoints and frenet odometry...");
    return;
  }

  last_eval_time_ = this->now();
  evaluateAndPublishLocalPath();
}

bool StaticObstacleAvoidanceNode::isObstacleBlockingCorridor(
  const f110_msgs::msg::Obstacle & obs, double car_s) const
{
  // 1) 정적 장애물 판별
  bool is_static = obs.is_static || (std::hypot(obs.vs, obs.vd) < static_vel_threshold_);
  if (!is_static) {
    return false;
  }

  // 2) 전방 유효 거리 확인 (트랙 원형 래핑 고려)
  double ds = obs.s_center - car_s;
  if (track_length_ > 0.0 && ds < -track_length_ * 0.5) {
    ds += track_length_;
  } else if (track_length_ > 0.0 && ds > track_length_ * 0.5) {
    ds -= track_length_;
  }

  if (ds < -0.5 || ds > lookahead_distance_) {
    return false;
  }

  // 3) 주행 코리도(차량 통과 경로 폭)와 장애물 횡방향 경계 충돌 여부 확인
  double half_corridor = corridor_width_ * 0.5;
  // 장애물의 좌우 경계가 [-half_corridor, half_corridor] 범위와 겹치는지 판단
  bool lateral_overlap = !(obs.d_left < -half_corridor || obs.d_right > half_corridor);
  return lateral_overlap;
}

void StaticObstacleAvoidanceNode::evaluateAndPublishLocalPath()
{
  double car_s = frenet_odom_.pose.pose.position.x;
  double car_d = frenet_odom_.pose.pose.position.y;
  double car_vs = frenet_odom_.twist.twist.linear.x;
  double car_vd = frenet_odom_.twist.twist.linear.y;

  double car_d_dot = (car_vs > 0.1) ? (car_vd / car_vs) : 0.0;
  double car_d_ddot = 0.0;

  // 전방에서 주행 경로를 차단하는 정적 장애물 찾기
  std::optional<f110_msgs::msg::Obstacle> closest_blocking_obs;
  double min_ds = lookahead_distance_ + 1.0;

  if (has_obs_) {
    for (const auto & obs : obstacles_.obstacles) {
      if (isObstacleBlockingCorridor(obs, car_s)) {
        double ds = obs.s_center - car_s;
        if (track_length_ > 0.0 && ds < 0.0) {
          ds += track_length_;
        }
        if (ds < min_ds) {
          min_ds = ds;
          closest_blocking_obs = obs;
        }
      }
    }
  }

  // =========================================================================================
  // [로컬 판단]: 경로 차단 정적 장애물이 없다면 글로벌 패스를 유지하도록 로컬패스 발행을 생략(또는 빈 경로 발행)
  // =========================================================================================
  if (!closest_blocking_obs.has_value()) {
    if (local_path_active_) {
      RCLCPP_INFO(this->get_logger(),
        "[로컬 판단] 전방 차단 정적 장애물 해제 -> 글로벌패스 유지 (로컬패스 비활성화)");
      local_path_active_ = false;

      // 빈 OTWpntArray 발행하여 wpnt_publisher가 즉시 글로벌 경로를 쓰도록 함
      f110_msgs::msg::OTWpntArray empty_msg;
      empty_msg.header.stamp = this->now();
      empty_msg.header.frame_id = "map";
      empty_msg.ot_side = "none";
      ot_wpnts_pub_->publish(empty_msg);

      nav_msgs::msg::Path empty_path;
      empty_path.header.stamp = this->now();
      empty_path.header.frame_id = "map";
      local_path_pub_->publish(empty_path);
    } else {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "[로컬 판단] 글로벌패스 유지 중 (차단 정적 장애물 없음)");
    }
    return;
  }

  // =========================================================================================
  // [로컬 판단]: 정적 장애물이 전방 경로를 차단함 -> 로컬 회피 경로(세그먼트) 생성 및 비용함수 평가
  // =========================================================================================
  local_path_active_ = true;
  last_avoidance_time_ = this->now();

  const auto & target_obs = closest_blocking_obs.value();
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "[로컬 판단] 전방 %.2fm 지점 정적 장애물(ID: %d, d: %.2f) 감지 -> 로컬 회피 경로 생성",
    min_ds, target_obs.id, target_obs.d_center);

  // 다중 후보 경로 생성
  auto candidates = generateCandidatePaths(target_obs, car_s, car_d, car_d_dot, car_d_ddot);
  evaluateCandidateCosts(candidates, target_obs);

  // 최적 후보 선택
  CandidatePath * best_cand = nullptr;
  for (auto & cand : candidates) {
    if (cand.is_valid && (best_cand == nullptr || cand.total_cost < best_cand->total_cost)) {
      best_cand = &cand;
    }
  }

  if (best_cand == nullptr) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "유효한 회피 경로를 찾지 못했습니다! 안전 마진 내 기본 좌/우회피 시도");
    if (!candidates.empty()) {
      best_cand = &candidates[0];
    } else {
      return;
    }
  }

  // 곡률 및 안전 고려 스로틀(속도) 프로파일 조정
  adjustThrottleAlongPath(*best_cand);

  // 1) OTWpntArray 세그먼트 메시지 구성 및 발행
  f110_msgs::msg::OTWpntArray ot_msg;
  ot_msg.header.stamp = this->now();
  ot_msg.header.frame_id = "map";
  ot_msg.last_switch_time = this->now();
  ot_msg.side_switch = true;
  ot_msg.ot_side = best_cand->side_str;
  ot_msg.wpnts = best_cand->waypoints;
  ot_wpnts_pub_->publish(ot_msg);

  // 2) RViz 시각화용 Path 메시지 발행
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";
  path_msg.poses.reserve(best_cand->waypoints.size());

  for (const auto & w : best_cand->waypoints) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header = path_msg.header;
    ps.pose.position.x = w.x_m;
    ps.pose.position.y = w.y_m;
    ps.pose.position.z = 0.0;

    double half_psi = w.psi_rad * 0.5;
    ps.pose.orientation.x = 0.0;
    ps.pose.orientation.y = 0.0;
    ps.pose.orientation.z = std::sin(half_psi);
    ps.pose.orientation.w = std::cos(half_psi);
    path_msg.poses.push_back(ps);
  }
  local_path_pub_->publish(path_msg);

  // 3) 후보 경로 시각화
  publishVisualization(*best_cand, candidates);
}

std::vector<CandidatePath> StaticObstacleAvoidanceNode::generateCandidatePaths(
  const f110_msgs::msg::Obstacle & target_obs,
  double car_s, double car_d, double car_d_dot, double car_d_ddot)
{
  std::vector<CandidatePath> candidates;

  double approach_len = target_obs.s_center - passing_buffer_s_ - car_s;
  if (track_length_ > 0.0 && approach_len < -0.5 * track_length_) {
    approach_len += track_length_;
  }
  approach_len = std::clamp(approach_len, 2.0, lookahead_distance_);

  // 고속 적응형 복귀 세그먼트 길이 후보
  std::vector<double> return_lengths = {10.0, 14.0, 18.0, 24.0};

  // 1) 좌측 회피 후보 오프셋
  double left_offset = target_obs.d_left + safety_margin_;
  for (double r_len : return_lengths) {
    auto path = generateSegmentPath(car_s, car_d, car_d_dot, car_d_ddot,
      left_offset, approach_len, r_len, "left", target_obs);
    candidates.push_back(std::move(path));
  }

  // 2) 우측 회피 후보 오프셋
  double right_offset = target_obs.d_right - safety_margin_;
  for (double r_len : return_lengths) {
    auto path = generateSegmentPath(car_s, car_d, car_d_dot, car_d_ddot,
      right_offset, approach_len, r_len, "right", target_obs);
    candidates.push_back(std::move(path));
  }

  return candidates;
}

CandidatePath StaticObstacleAvoidanceNode::generateSegmentPath(
  double car_s, double car_d, double car_d_dot, double car_d_ddot,
  double target_offset, double approach_len, double return_len,
  const std::string & side,
  const f110_msgs::msg::Obstacle & target_obs)
{
  (void)target_obs;
  CandidatePath cand;
  cand.target_offset = target_offset;
  cand.return_length = return_len;
  cand.side_str = side;

  double pass_len = 2.0 * passing_buffer_s_;
  // Phase 1: 차량 현재 지점 ~ 장애물 진입 완충 지점
  QuinticPolynomial poly_approach(car_d, car_d_dot, car_d_ddot, target_offset, 0.0, 0.0, approach_len);
  // Phase 2: 장애물 평행 통과 (d=target_offset, d'=0, d''=0 유지)
  // Phase 3: 장애물 통과 완료 지점 ~ 글로벌 경로 수렴 지점 (d=0, d'=0, d''=0)
  QuinticPolynomial poly_return(target_offset, 0.0, 0.0, 0.0, 0.0, 0.0, return_len);

  double total_len = approach_len + pass_len + return_len;
  int num_points = std::max(10, static_cast<int>(total_len / waypoint_step_));

  cand.waypoints.reserve(num_points + 1);

  for (int i = 0; i <= num_points; ++i) {
    double s_local = i * waypoint_step_;
    if (s_local > total_len) {
      s_local = total_len;
    }

    double s_global = car_s + s_local;
    if (track_length_ > 0.0 && s_global >= track_length_) {
      s_global -= track_length_;
    }

    double d_val = 0.0;
    double d_dot_val = 0.0;
    double d_ddot_val = 0.0;

    if (s_local <= approach_len) {
      d_val = poly_approach.calc_d(s_local);
      d_dot_val = poly_approach.calc_d_dot(s_local);
      d_ddot_val = poly_approach.calc_d_ddot(s_local);
    } else if (s_local <= approach_len + pass_len) {
      d_val = target_offset;
      d_dot_val = 0.0;
      d_ddot_val = 0.0;
    } else {
      double s_ret = s_local - (approach_len + pass_len);
      d_val = poly_return.calc_d(s_ret);
      d_dot_val = poly_return.calc_d_dot(s_ret);
      d_ddot_val = poly_return.calc_d_ddot(s_ret);
    }
    (void)d_dot_val;

    f110_msgs::msg::Wpnt w = interpolateGlobalWpnt(s_global);
    w.s_m = s_global;
    w.d_m = d_val;

    // 로컬 횡방향 보간에 따른 곡률 보정
    w.kappa_radpm += d_ddot_val;

    convertFrenetToCartesian(w);
    cand.waypoints.push_back(w);
  }

  // 수렴 보장 check (경로 끝 지점이 d=0에 도달)
  cand.is_valid = true;
  return cand;
}

f110_msgs::msg::Wpnt StaticObstacleAvoidanceNode::interpolateGlobalWpnt(double s_target) const
{
  const auto & wpnts = global_wpnts_.wpnts;
  if (wpnts.empty()) {
    return f110_msgs::msg::Wpnt();
  }

  // s_target에 가장 가까운 두 웨이포인트를 선형 보간
  int n = static_cast<int>(wpnts.size());
  for (int i = 0; i < n - 1; ++i) {
    if (s_target >= wpnts[i].s_m && s_target <= wpnts[i + 1].s_m) {
      double ratio = (s_target - wpnts[i].s_m) / std::max(1e-4, (wpnts[i + 1].s_m - wpnts[i].s_m));
      f110_msgs::msg::Wpnt out = wpnts[i];
      out.s_m = s_target;
      out.x_m = wpnts[i].x_m + ratio * (wpnts[i + 1].x_m - wpnts[i].x_m);
      out.y_m = wpnts[i].y_m + ratio * (wpnts[i + 1].y_m - wpnts[i].y_m);
      out.vx_mps = wpnts[i].vx_mps + ratio * (wpnts[i + 1].vx_mps - wpnts[i].vx_mps);
      out.psi_rad = wpnts[i].psi_rad + ratio * (wpnts[i + 1].psi_rad - wpnts[i].psi_rad);
      out.kappa_radpm = wpnts[i].kappa_radpm + ratio * (wpnts[i + 1].kappa_radpm - wpnts[i].kappa_radpm);
      out.d_left = wpnts[i].d_left;
      out.d_right = wpnts[i].d_right;
      return out;
    }
  }

  return wpnts.back();
}

void StaticObstacleAvoidanceNode::convertFrenetToCartesian(f110_msgs::msg::Wpnt & wpnt) const
{
  // Frenet d 오프셋을 글로벌 중심선의 법선 벡터 방향으로 더해 Cartesain 좌표(x, y) 갱신
  double nx = -std::sin(wpnt.psi_rad);
  double ny = std::cos(wpnt.psi_rad);

  wpnt.x_m += nx * wpnt.d_m;
  wpnt.y_m += ny * wpnt.d_m;
}

void StaticObstacleAvoidanceNode::evaluateCandidateCosts(
  std::vector<CandidatePath> & candidates,
  const f110_msgs::msg::Obstacle & target_obs)
{
  for (auto & cand : candidates) {
    if (cand.waypoints.empty()) {
      cand.is_valid = false;
      continue;
    }

    double sum_lat = 0.0;
    double sum_smooth = 0.0;
    double min_clearance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < cand.waypoints.size(); ++i) {
      const auto & w = cand.waypoints[i];

      // 1) 안전성 (장애물과의 최소 간격 확인)
      double ds = w.s_m - target_obs.s_center;
      if (track_length_ > 0.0 && ds < -track_length_ * 0.5) {
        ds += track_length_;
      } else if (track_length_ > 0.0 && ds > track_length_ * 0.5) {
        ds -= track_length_;
      }

      if (std::abs(ds) < (target_obs.s_end - target_obs.s_start) * 0.5 + 0.3) {
        double clr_left = w.d_m - target_obs.d_left;
        double clr_right = target_obs.d_right - w.d_m;
        // 장애물 범위 내부를 침범하면 무효
        if (w.d_m > target_obs.d_right && w.d_m < target_obs.d_left) {
          cand.is_valid = false;
        }
        min_clearance = std::min({min_clearance, std::abs(clr_left), std::abs(clr_right)});
      }

      // 트랙 경계 이탈 검사
      if (w.d_m > w.d_left - 0.1 || w.d_m < -w.d_right + 0.1) {
        cand.is_valid = false;
      }

      // 2) 횡방향 편차 비용
      sum_lat += w.d_m * w.d_m;

      // 3) 평활도 비용 (곡률 변화)
      sum_smooth += w.kappa_radpm * w.kappa_radpm;
    }

    cand.cost_obs = (min_clearance > 0.0 && min_clearance < 10.0)
      ? (1.0 / (min_clearance + 0.1)) : 0.0;
    cand.cost_lat = sum_lat / cand.waypoints.size();
    cand.cost_smooth = sum_smooth / cand.waypoints.size();

    cand.total_cost = weight_obs_ * cand.cost_obs +
                      weight_lat_ * cand.cost_lat +
                      weight_smooth_ * cand.cost_smooth;
  }
}

void StaticObstacleAvoidanceNode::adjustThrottleAlongPath(CandidatePath & path) const
{
  for (auto & w : path.waypoints) {
    // 횡가속도 한계(max_lat_accel_) 기반 허용 최고속도 계산
    double abs_kappa = std::abs(w.kappa_radpm);
    double v_curve = (abs_kappa > 1e-3)
      ? std::sqrt(max_lat_accel_ / abs_kappa)
      : w.vx_mps;

    w.vx_mps = std::clamp(std::min(w.vx_mps, v_curve), min_speed_, w.vx_mps);
  }
}

void StaticObstacleAvoidanceNode::publishVisualization(
  const CandidatePath & best_path,
  const std::vector<CandidatePath> & all_candidates)
{
  visualization_msgs::msg::MarkerArray markers;
  int id = 0;

  for (const auto & cand : all_candidates) {
    visualization_msgs::msg::Marker line;
    line.header.stamp = this->now();
    line.header.frame_id = "map";
    line.ns = "candidates";
    line.id = id++;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    line.scale.x = (&cand == &best_path) ? 0.08 : 0.03;

    if (&cand == &best_path) {
      line.color.r = 0.0f;
      line.color.g = 1.0f;
      line.color.b = 0.2f;
      line.color.a = 0.9f;
    } else if (cand.is_valid) {
      line.color.r = 0.0f;
      line.color.g = 0.7f;
      line.color.b = 1.0f;
      line.color.a = 0.5f;
    } else {
      line.color.r = 1.0f;
      line.color.g = 0.0f;
      line.color.b = 0.0f;
      line.color.a = 0.3f;
    }

    for (const auto & w : cand.waypoints) {
      geometry_msgs::msg::Point p;
      p.x = w.x_m;
      p.y = w.y_m;
      p.z = (&cand == &best_path) ? 0.08 : 0.02;
      line.points.push_back(p);
    }
    markers.markers.push_back(line);
  }

  cand_markers_pub_->publish(markers);
}

}  // namespace local_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<local_planning::StaticObstacleAvoidanceNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
