// Copyright 2026 2026_IFAC contributors
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <f110_msgs/msg/state_machine.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{

using namespace std::chrono_literals;

struct Event
{
  std::string id;
  std::int64_t original_source_stamp_ns{0};
  double ego_s{0.0};
  double ego_d{0.0};
  double ego_speed{0.0};
  f110_msgs::msg::WpntArray reference;
  f110_msgs::msg::ObstacleArray obstacles;
};

std::vector<std::string> splitTabs(const std::string & line)
{
  std::vector<std::string> fields;
  std::size_t begin = 0U;
  while (begin <= line.size()) {
    const auto end = line.find('\t', begin);
    fields.push_back(line.substr(begin, end == std::string::npos ? end : end - begin));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  return fields;
}

double number(const std::string & token)
{
  std::size_t consumed = 0U;
  const double value = std::stod(token, &consumed);
  if (consumed != token.size() || !std::isfinite(value)) {
    throw std::runtime_error("invalid finite number: " + token);
  }
  return value;
}

std::int64_t integer(const std::string & token)
{
  std::size_t consumed = 0U;
  const auto value = std::stoll(token, &consumed);
  if (consumed != token.size()) {
    throw std::runtime_error("invalid integer: " + token);
  }
  return value;
}

Event readEvent(const std::string & path)
{
  std::ifstream input(path);
  std::string line;
  if (!input || !std::getline(input, line) || line != "P3_ORACLE_EVENT_V1") {
    throw std::runtime_error("unsupported event file: " + path);
  }
  Event event;
  event.reference.header.frame_id = "map";
  event.obstacles.header.frame_id = "map";
  while (std::getline(input, line)) {
    if (line.empty()) {
      continue;
    }
    const auto fields = splitTabs(line);
    if (fields[0] == "EVENT" && fields.size() == 11U) {
      event.id = fields[1];
      event.ego_s = number(fields[6]);
      event.ego_d = number(fields[7]);
      event.ego_speed = number(fields[8]);
      event.original_source_stamp_ns = integer(fields[9]);
    } else if (fields[0] == "W" && fields.size() == 12U) {
      f110_msgs::msg::Wpnt waypoint;
      waypoint.id = static_cast<std::int32_t>(integer(fields[1]));
      waypoint.s_m = number(fields[2]);
      waypoint.d_m = number(fields[3]);
      waypoint.x_m = number(fields[4]);
      waypoint.y_m = number(fields[5]);
      waypoint.d_right = number(fields[6]);
      waypoint.d_left = number(fields[7]);
      waypoint.psi_rad = number(fields[8]);
      waypoint.kappa_radpm = number(fields[9]);
      waypoint.vx_mps = number(fields[10]);
      waypoint.ax_mps2 = number(fields[11]);
      event.reference.wpnts.push_back(waypoint);
    } else if (fields[0] == "O" && fields.size() == 12U) {
      f110_msgs::msg::Obstacle obstacle;
      obstacle.id = static_cast<std::int32_t>(integer(fields[1]));
      obstacle.s_center = number(fields[2]);
      obstacle.s_start = number(fields[3]);
      obstacle.s_end = number(fields[4]);
      obstacle.d_right = number(fields[5]);
      obstacle.d_left = number(fields[6]);
      obstacle.size = number(fields[7]);
      obstacle.s_var = number(fields[8]);
      obstacle.d_var = number(fields[9]);
      obstacle.is_static = integer(fields[10]) != 0;
      obstacle.is_visible = integer(fields[11]) != 0;
      obstacle.d_center = 0.5 * (obstacle.d_right + obstacle.d_left);
      event.obstacles.obstacles.push_back(obstacle);
    } else if (fields[0] == "END_EVENT") {
      break;
    }
  }
  if (event.id != "SCE018" || event.reference.wpnts.empty() ||
    event.obstacles.obstacles.size() != 1U || event.original_source_stamp_ns <= 0)
  {
    throw std::runtime_error("event is not the complete canonical SCE018 workload");
  }
  return event;
}

builtin_interfaces::msg::Time timeFromNanoseconds(std::int64_t nanoseconds)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<std::uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}

std::string jsonEscape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

std::optional<std::string> jsonToken(const std::string & json, const std::string & key)
{
  const std::string marker = "\"" + key + "\":";
  auto position = json.find(marker);
  if (position == std::string::npos) {
    return std::nullopt;
  }
  position += marker.size();
  while (position < json.size() && json[position] == ' ') {
    ++position;
  }
  if (position >= json.size()) {
    return std::nullopt;
  }
  if (json[position] == '"') {
    ++position;
    std::string value;
    bool escaped = false;
    for (; position < json.size(); ++position) {
      const char character = json[position];
      if (escaped) {
        value.push_back(character);
        escaped = false;
      } else if (character == '\\') {
        escaped = true;
      } else if (character == '"') {
        return value;
      } else {
        value.push_back(character);
      }
    }
    return std::nullopt;
  }
  const auto end = json.find_first_of(",}]", position);
  return json.substr(position, end == std::string::npos ? end : end - position);
}

template<typename IntegerT>
std::optional<IntegerT> jsonInteger(const std::string & json, const std::string & key)
{
  const auto token = jsonToken(json, key);
  if (!token.has_value()) {
    return std::nullopt;
  }
  try {
    std::size_t consumed = 0U;
    const auto parsed = std::stoull(*token, &consumed);
    if (consumed != token->size()) {
      return std::nullopt;
    }
    return static_cast<IntegerT>(parsed);
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<double> jsonDouble(const std::string & json, const std::string & key)
{
  const auto token = jsonToken(json, key);
  if (!token.has_value()) {
    return std::nullopt;
  }
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(*token, &consumed);
    if (consumed != token->size() || !std::isfinite(parsed)) {
      return std::nullopt;
    }
    return parsed;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::optional<bool> jsonBool(const std::string & json, const std::string & key)
{
  const auto token = jsonToken(json, key);
  if (token == std::optional<std::string>("true")) {
    return true;
  }
  if (token == std::optional<std::string>("false")) {
    return false;
  }
  return std::nullopt;
}

bool nearlyEqual(double first, double second)
{
  return std::abs(first - second) <= 1.0e-12;
}

}  // namespace

class Sce018ReplayDriver : public rclcpp::Node
{
public:
  Sce018ReplayDriver()
  : Node("sce018_replay_driver"),
    event_(readEvent(declare_parameter<std::string>("event_file", ""))),
    expected_digest_(declare_parameter<std::string>("expected_digest", "c7b2c19bf2af9350")),
    output_path_(declare_parameter<std::string>("output_path", "")),
    workload_id_(declare_parameter<std::string>("workload_id", "SMOKE")),
    condition_(declare_parameter<std::string>("condition", "SMOKE")),
    attempt_id_(declare_parameter<std::string>("attempt_id", "SMOKE")),
    repeat_id_(static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<std::int64_t>("repeat_id", 0)))),
    target_callbacks_(static_cast<std::size_t>(
      std::max<std::int64_t>(1, declare_parameter<std::int64_t>("target_callbacks", 4)))),
    warmup_callbacks_(static_cast<std::size_t>(
      std::max<std::int64_t>(0, declare_parameter<std::int64_t>("warmup_callbacks", 0)))),
    minimum_source_epochs_(static_cast<std::size_t>(
      std::max<std::int64_t>(2, declare_parameter<std::int64_t>("minimum_source_epochs", 2)))),
    smoke_mode_(declare_parameter<bool>("smoke_mode", true)),
    preflight_hold_sec_(declare_parameter<double>("preflight_hold_sec", 2.0)),
    timeout_sec_(declare_parameter<double>("timeout_sec", 30.0)),
    obstacle_publish_period_sec_(declare_parameter<double>("obstacle_publish_period_sec", 0.30))
  {
    if (output_path_.empty()) {
      throw std::invalid_argument("output_path must be explicit");
    }
    if (warmup_callbacks_ >= target_callbacks_) {
      throw std::invalid_argument("warmup_callbacks must be less than target_callbacks");
    }
    output_.open(output_path_, std::ios::out | std::ios::trunc);
    if (!output_) {
      throw std::runtime_error("cannot open output_path: " + output_path_);
    }

    const auto input_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();
    const auto reference_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
    reference_pub_ = create_publisher<f110_msgs::msg::WpntArray>(
      declare_parameter<std::string>("reference_topic", "/gqsc_runtime/global_waypoints"),
      reference_qos);
    obstacle_pub_ = create_publisher<f110_msgs::msg::ObstacleArray>(
      declare_parameter<std::string>("obstacle_topic", "/gqsc_runtime/confirmed_static_obs"),
      input_qos);
    odometry_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      declare_parameter<std::string>("odometry_topic", "/gqsc_runtime/frenet_odom"), input_qos);
    state_pub_ = create_publisher<f110_msgs::msg::StateMachine>(
      declare_parameter<std::string>("state_topic", "/gqsc_runtime/state"), reference_qos);

    diagnostic_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("diagnostic_topic", "/gqsc_runtime/p3_cycle"),
      rclcpp::QoS(1000).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {onDiagnostic(message->data);});
    profile_sub_ = create_subscription<std_msgs::msg::String>(
      declare_parameter<std::string>("profile_topic", "/gqsc_runtime/live_profile"),
      rclcpp::QoS(1000).reliable(),
      [this](const std_msgs::msg::String::SharedPtr message) {onProfile(message->data);});

    start_time_ = std::chrono::steady_clock::now();
    tick_timer_ = create_wall_timer(50ms, std::bind(&Sce018ReplayDriver::tick, this));
    emitHeader();
  }

  bool success() const {return success_;}

private:
  bool graphReady() const
  {
    return reference_pub_->get_subscription_count() == 1U &&
           obstacle_pub_->get_subscription_count() == 1U &&
           odometry_pub_->get_subscription_count() == 1U &&
           state_pub_->get_subscription_count() == 1U;
  }

  void emit(const std::string & row)
  {
    output_ << row << '\n';
    output_.flush();
    std::cout << row << std::endl;
  }

  void emitHeader()
  {
    std::ostringstream row;
    row << "{\"schema\":\"gqsc_s1_same_input_replay/1\",\"kind\":\"start\""
        << ",\"event_id\":\"" << jsonEscape(event_.id) << "\""
        << ",\"reference_points\":" << event_.reference.wpnts.size()
        << ",\"obstacles\":" << event_.obstacles.obstacles.size()
        << ",\"expected_digest\":\"" << jsonEscape(expected_digest_) << "\""
        << ",\"workload_id\":\"" << jsonEscape(workload_id_) << "\""
        << ",\"condition\":\"" << jsonEscape(condition_) << "\""
        << ",\"repeat_id\":" << repeat_id_
        << ",\"attempt_id\":\"" << jsonEscape(attempt_id_) << "\""
        << ",\"target_callbacks\":" << target_callbacks_
        << ",\"warmup_callbacks\":" << warmup_callbacks_
        << ",\"measurement_callbacks\":" << target_callbacks_ - warmup_callbacks_
        << ",\"minimum_source_epochs\":" << minimum_source_epochs_
        << ",\"smoke_mode\":" << (smoke_mode_ ? "true" : "false") << '}';
    emit(row.str());
  }

  void publishReferenceAndState()
  {
    const auto stamp = now();
    if (!reference_published_) {
      auto reference = event_.reference;
      reference.header.stamp = stamp;
      reference_pub_->publish(reference);
      reference_published_ = true;
      input_start_time_ = std::chrono::steady_clock::now();
    }

    nav_msgs::msg::Odometry odometry;
    odometry.header.frame_id = "map";
    odometry.header.stamp = stamp;
    odometry.pose.pose.position.x = event_.ego_s;
    odometry.pose.pose.position.y = event_.ego_d;
    odometry.pose.pose.orientation.w = 1.0;
    odometry.twist.twist.linear.x = event_.ego_speed;
    odometry_pub_->publish(odometry);

    f110_msgs::msg::StateMachine state;
    state.header.frame_id = "map";
    state.header.stamp = stamp;
    state.state = f110_msgs::msg::StateMachine::STATE_GLOBAL;
    state_pub_->publish(state);
  }

  void publishObstacle()
  {
    const std::uint64_t restart_index = obstacle_publish_index_ / 2U;
    const bool regression = obstacle_publish_index_ % 2U == 1U;
    const std::int64_t high_stamp = event_.original_source_stamp_ns +
      static_cast<std::int64_t>((restart_index + 1U) * 2000000000ULL);
    const std::int64_t source_stamp = high_stamp - (regression ? 1000000000LL : 0LL);
    auto message = event_.obstacles;
    message.header.stamp = timeFromNanoseconds(source_stamp);
    obstacle_pub_->publish(message);
    ++obstacle_publish_index_;
  }

  void tick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double total_elapsed = std::chrono::duration<double>(steady_now - start_time_).count();
    if (total_elapsed > timeout_sec_) {
      fail("TIMEOUT");
      return;
    }
    if (!graphReady()) {
      graph_ready_since_.reset();
      return;
    }
    if (!graph_ready_since_.has_value()) {
      graph_ready_since_ = steady_now;
      return;
    }
    const double graph_elapsed =
      std::chrono::duration<double>(steady_now - *graph_ready_since_).count();
    if (graph_elapsed < preflight_hold_sec_) {
      return;
    }
    publishReferenceAndState();
    if (!input_start_time_.has_value() ||
      std::chrono::duration<double>(steady_now - *input_start_time_).count() < 0.50)
    {
      return;
    }
    if (!last_obstacle_publish_time_.has_value() ||
      std::chrono::duration<double>(steady_now - *last_obstacle_publish_time_).count() >=
      obstacle_publish_period_sec_)
    {
      publishObstacle();
      last_obstacle_publish_time_ = steady_now;
    }
  }

  void onDiagnostic(const std::string & json)
  {
    const auto sequence = jsonInteger<std::uint64_t>(json, "callback_sequence");
    if (!sequence.has_value()) {
      return;
    }
    diagnostics_[*sequence] = json;
    tryJoin(*sequence);
  }

  void onProfile(const std::string & json)
  {
    const auto sequence = jsonInteger<std::uint64_t>(json, "callback_sequence");
    if (!sequence.has_value()) {
      return;
    }
    profiles_[*sequence] = json;
    tryJoin(*sequence);
  }

  void tryJoin(std::uint64_t callback_sequence)
  {
    if (joined_callbacks_.count(callback_sequence) > 0U) {
      return;
    }
    const auto diagnostic = diagnostics_.find(callback_sequence);
    const auto profile = profiles_.find(callback_sequence);
    if (diagnostic == diagnostics_.end() || profile == profiles_.end()) {
      return;
    }

    const auto fresh_count = jsonInteger<std::uint64_t>(profile->second, "fresh_evaluation_count");
    const auto pair_count = jsonInteger<std::uint64_t>(profile->second, "pair_proxies");
    const auto reconstruction_count =
      jsonInteger<std::uint64_t>(profile->second, "reconstructions");
    const auto validator_count = jsonInteger<std::uint64_t>(profile->second, "validator_calls");
    const auto o_total = jsonDouble(profile->second, "O_total");
    const auto mode = jsonToken(diagnostic->second, "mode");
    const auto epoch = jsonInteger<std::uint64_t>(diagnostic->second, "source_epoch");
    const auto source_stamp = jsonInteger<std::uint64_t>(diagnostic->second, "source_stamp_ns");
    const auto obstacle_sequence =
      jsonInteger<std::uint64_t>(diagnostic->second, "obstacle_sequence");
    const auto reference_generation =
      jsonInteger<std::uint64_t>(diagnostic->second, "global_reference_generation");
    const auto digest = jsonToken(diagnostic->second, "selected_path_digest");
    const auto identity = jsonToken(diagnostic->second, "selected_candidate_identity");
    const auto selected_source = jsonToken(diagnostic->second, "selected_source");
    const auto selected_cell = jsonToken(diagnostic->second, "selected_source_cell");
    const auto candidate_count = jsonInteger<std::uint64_t>(diagnostic->second, "candidate_count");
    const auto fresh_hard_valid = jsonBool(diagnostic->second, "fresh_hard_valid");
    const auto lifecycle_state = jsonToken(diagnostic->second, "lifecycle_state");
    const auto lifecycle_reason = jsonToken(diagnostic->second, "lifecycle_reason");
    const auto path_owner = jsonToken(diagnostic->second, "path_owner");
    const auto safe_stop = jsonBool(diagnostic->second, "safe_stop_active");
    const auto ego_s = jsonDouble(diagnostic->second, "ego_s");
    const auto ego_d = jsonDouble(diagnostic->second, "ego_d");
    const auto ego_speed = jsonDouble(diagnostic->second, "ego_speed_mps");
    const auto obstacle_id = jsonInteger<std::uint64_t>(diagnostic->second, "id");
    const auto s_start = jsonDouble(diagnostic->second, "s_start");
    const auto s_end = jsonDouble(diagnostic->second, "s_end");
    const auto d_right = jsonDouble(diagnostic->second, "d_right");
    const auto d_left = jsonDouble(diagnostic->second, "d_left");

    const auto & expected_obstacle = event_.obstacles.obstacles.front();
    const bool complete = fresh_count.has_value() && pair_count.has_value() &&
      reconstruction_count.has_value() && validator_count.has_value() && o_total.has_value() &&
      mode.has_value() && epoch.has_value() && source_stamp.has_value() &&
      obstacle_sequence.has_value() && reference_generation.has_value() && digest.has_value() &&
      identity.has_value() && selected_source.has_value() && selected_cell.has_value() &&
      candidate_count.has_value() && fresh_hard_valid.has_value() && lifecycle_state.has_value() &&
      lifecycle_reason.has_value() && path_owner.has_value() && safe_stop.has_value() &&
      ego_s.has_value() && ego_d.has_value() && ego_speed.has_value() && obstacle_id.has_value() &&
      s_start.has_value() && s_end.has_value() && d_right.has_value() && d_left.has_value();
    if (!complete) {
      fail("JOIN_FIELD_MISSING");
      return;
    }

    const bool parity = *fresh_count == 1U && *pair_count == 128U &&
      *reconstruction_count == 12U && *validator_count == 12U && *mode == "TEST_ACTIVE" &&
      *digest == expected_digest_ && !identity->empty() && *candidate_count == 12U &&
      *fresh_hard_valid && !*safe_stop && nearlyEqual(*ego_s, event_.ego_s) &&
      nearlyEqual(*ego_d, event_.ego_d) && nearlyEqual(*ego_speed, event_.ego_speed) &&
      *obstacle_id == static_cast<std::uint64_t>(expected_obstacle.id) &&
      nearlyEqual(*s_start, expected_obstacle.s_start) &&
      nearlyEqual(*s_end, expected_obstacle.s_end) &&
      nearlyEqual(*d_right, expected_obstacle.d_right) &&
      nearlyEqual(*d_left, expected_obstacle.d_left);
    if (!parity) {
      fail("PARITY_MISMATCH");
      return;
    }

    joined_callbacks_.insert(callback_sequence);
    observed_epochs_.insert(*epoch);
    ++derived_evaluation_sequence_;
    const bool warmup = derived_evaluation_sequence_ <= warmup_callbacks_;
    const std::size_t phase_index = warmup ?
      derived_evaluation_sequence_ : derived_evaluation_sequence_ - warmup_callbacks_;
    std::ostringstream row;
    row << std::setprecision(17)
        << "{\"schema\":\"gqsc_s1_same_input_replay/1\",\"kind\":\"joined\""
        << ",\"callback_sequence\":" << callback_sequence
        << ",\"derived_evaluation_sequence\":" << derived_evaluation_sequence_
        << ",\"phase\":\"" << (warmup ? "WARMUP" : "MEASUREMENT") << "\""
        << ",\"phase_index\":" << phase_index
        << ",\"source_epoch\":" << *epoch
        << ",\"source_stamp_ns\":" << *source_stamp
        << ",\"obstacle_sequence\":" << *obstacle_sequence
        << ",\"global_reference_generation\":" << *reference_generation
        << ",\"r3_invoked\":true,\"fresh_evaluation_count\":" << *fresh_count
        << ",\"pair_proxies\":" << *pair_count
        << ",\"reconstructions\":" << *reconstruction_count
        << ",\"validator_calls\":" << *validator_count
        << ",\"candidate_count\":" << *candidate_count
        << ",\"selected_candidate_identity\":\"" << jsonEscape(*identity) << "\""
        << ",\"selected_source\":\"" << jsonEscape(*selected_source) << "\""
        << ",\"selected_source_cell\":\"" << jsonEscape(*selected_cell) << "\""
        << ",\"selected_path_digest\":\"" << jsonEscape(*digest) << "\""
        << ",\"fresh_hard_valid\":true"
        << ",\"hard_usable_exposure\":\"BOOLEAN_ONLY_PUBLIC_TOPIC\""
        << ",\"lifecycle_state\":\"" << jsonEscape(*lifecycle_state) << "\""
        << ",\"lifecycle_reason\":\"" << jsonEscape(*lifecycle_reason) << "\""
        << ",\"path_owner\":\"" << jsonEscape(*path_owner) << "\""
        << ",\"safe_stop_active\":false"
        << ",\"o_total_present\":true"
        << ",\"timing_redacted\":" << (smoke_mode_ ? "true" : "false");
    if (!smoke_mode_) {
      row << ",\"o_total_us\":" << *o_total
          << ",\"raw_profile\":\"" << jsonEscape(profile->second) << "\""
          << ",\"raw_diagnostic\":\"" << jsonEscape(diagnostic->second) << "\"";
    }
    row << '}';
    emit(row.str());

    if (joined_callbacks_.size() >= target_callbacks_ &&
      observed_epochs_.size() >= minimum_source_epochs_)
    {
      success_ = true;
      std::ostringstream summary;
      summary << "{\"schema\":\"gqsc_s1_same_input_replay/1\",\"kind\":\"complete\""
              << ",\"joined_callbacks\":" << joined_callbacks_.size()
              << ",\"warmup_callbacks\":" << warmup_callbacks_
              << ",\"measurement_callbacks\":"
              << joined_callbacks_.size() - warmup_callbacks_
              << ",\"distinct_source_epochs\":" << observed_epochs_.size()
              << ",\"parity\":\"PASS\",\"timing_interpretation\":\""
              << (smoke_mode_ ? "FORBIDDEN" : "QUALIFICATION_RAW_UNINTERPRETED") << "\"}";
      emit(summary.str());
      rclcpp::shutdown();
    }
  }

  void fail(const std::string & reason)
  {
    if (finished_) {
      return;
    }
    finished_ = true;
    emit(
      "{\"schema\":\"gqsc_s1_same_input_replay/1\",\"kind\":\"failed\",\"reason\":\"" +
      jsonEscape(reason) + "\",\"timing_interpretation\":\"FORBIDDEN\"}");
    rclcpp::shutdown();
  }

  Event event_;
  std::string expected_digest_;
  std::string output_path_;
  std::string workload_id_;
  std::string condition_;
  std::string attempt_id_;
  std::size_t repeat_id_;
  std::size_t target_callbacks_;
  std::size_t warmup_callbacks_;
  std::size_t minimum_source_epochs_;
  bool smoke_mode_;
  double preflight_hold_sec_;
  double timeout_sec_;
  double obstacle_publish_period_sec_;
  std::ofstream output_;

  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr reference_pub_;
  rclcpp::Publisher<f110_msgs::msg::ObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<f110_msgs::msg::StateMachine>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr diagnostic_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr profile_sub_;
  rclcpp::TimerBase::SharedPtr tick_timer_;

  std::chrono::steady_clock::time_point start_time_;
  std::optional<std::chrono::steady_clock::time_point> graph_ready_since_;
  std::optional<std::chrono::steady_clock::time_point> input_start_time_;
  std::optional<std::chrono::steady_clock::time_point> last_obstacle_publish_time_;
  bool reference_published_{false};
  std::uint64_t obstacle_publish_index_{0U};
  std::uint64_t derived_evaluation_sequence_{0U};
  std::map<std::uint64_t, std::string> diagnostics_;
  std::map<std::uint64_t, std::string> profiles_;
  std::unordered_set<std::uint64_t> joined_callbacks_;
  std::set<std::uint64_t> observed_epochs_;
  bool success_{false};
  bool finished_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    const auto node = std::make_shared<Sce018ReplayDriver>();
    rclcpp::spin(node);
    return node->success() ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "sce018_replay_driver failed: " << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 2;
  }
}
