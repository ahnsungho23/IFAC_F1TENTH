// lap_referee: closed-loop rollout judge/recorder for f1tenth_gym_ros.
//
// The f1tenth gym bridge publishes its latched collision state. This node uses
// that state as the primary collision signal and keeps scan/stuck checks as
// fallbacks for older bridges or non-gym sources:
//   * /ego_racecar/scan  -> minimum LiDAR range (wall proximity / impact)
//   * /ego_racecar/odom  -> pose + body speed (progress, stuck detection)
//   * /drive             -> commanded speed (stuck-vs-intent disambiguation)
//   * /ego_racecar/collision -> simulator collision latch
//
// One process == one rollout. The node loads the reference raceline CSV to
// measure forward progress along the track, sets t0 at first motion, then
// terminates on lap completion, collision, off-track, stuck, or timeout. On
// termination it atomically writes:
//   * <output_dir>/<prefix>_summary.json  (episode metrics for the optimizer)
//   * <output_dir>/<prefix>_trace.csv     (per-sample realized trajectory)
// and shuts down so the orchestrator can detect rollout completion via exit.

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
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
  return out;
}

double yawFromOdom(const nav_msgs::msg::Odometry & msg)
{
  const auto & q = msg.pose.pose.orientation;
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double hypot2(double ax, double ay, double bx, double by)
{
  return std::hypot(ax - bx, ay - by);
}

struct RefPoint
{
  double x;
  double y;
  double psi;
  double s;
};
}  // namespace

class LapReferee : public rclcpp::Node
{
public:
  LapReferee()
  : Node("lap_referee")
  {
    declareParameters();
    readParameters();
    loadReferenceCsv();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {onOdom(msg);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {onScan(msg);});
    drive_sub_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      drive_topic_, 10,
      [this](const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {onDrive(msg);});
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      collision_topic_, 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {onCollision(msg);});

    const double period = 1.0 / std::max(record_rate_hz_, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(period), [this]() {step();});

    start_wall_ = now();
    RCLCPP_INFO(
      get_logger(),
      "lap_referee started: track_len=%.2fm, waypoints=%zu, output=%s/%s_summary.json",
      track_length_, reference_.size(), output_dir_.c_str(), output_prefix_.c_str());
  }

private:
  enum class Phase { WaitingForStart, Running, Done };

  void declareParameters()
  {
    declare_parameter<std::string>("waypoints_csv", "");
    declare_parameter<std::string>("odom_topic", "/ego_racecar/odom");
    // f1tenth gym bridge publishes the ego scan un-namespaced on /scan.
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("drive_topic", "/drive");
    declare_parameter<std::string>("collision_topic", "/ego_racecar/collision");
    declare_parameter<std::string>("output_dir", "/tmp/lap_referee");
    declare_parameter<std::string>("output_prefix", "rollout");

    declare_parameter<double>("record_rate_hz", 50.0);
    declare_parameter<double>("start_speed_threshold", 0.4);
    declare_parameter<double>("collision_scan_threshold", 0.13);
    declare_parameter<double>("stuck_speed_threshold", 0.2);
    declare_parameter<double>("stuck_cmd_threshold", 0.8);
    declare_parameter<double>("stuck_time_sec", 0.7);
    declare_parameter<double>("off_track_threshold", 1.5);
    declare_parameter<double>("lap_fraction", 0.97);
    declare_parameter<double>("max_episode_time_sec", 60.0);
    declare_parameter<double>("startup_grace_sec", 0.5);
    declare_parameter<double>("no_start_timeout_sec", 8.0);
    declare_parameter<bool>("stop_vehicle_on_exit", true);
    declare_parameter<bool>("shutdown_on_terminate", true);
  }

  void readParameters()
  {
    waypoints_csv_ = get_parameter("waypoints_csv").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    scan_topic_ = get_parameter("scan_topic").as_string();
    drive_topic_ = get_parameter("drive_topic").as_string();
    collision_topic_ = get_parameter("collision_topic").as_string();
    output_dir_ = get_parameter("output_dir").as_string();
    output_prefix_ = get_parameter("output_prefix").as_string();

    record_rate_hz_ = get_parameter("record_rate_hz").as_double();
    start_speed_threshold_ = get_parameter("start_speed_threshold").as_double();
    collision_scan_threshold_ = get_parameter("collision_scan_threshold").as_double();
    stuck_speed_threshold_ = get_parameter("stuck_speed_threshold").as_double();
    stuck_cmd_threshold_ = get_parameter("stuck_cmd_threshold").as_double();
    stuck_time_sec_ = get_parameter("stuck_time_sec").as_double();
    off_track_threshold_ = get_parameter("off_track_threshold").as_double();
    lap_fraction_ = get_parameter("lap_fraction").as_double();
    max_episode_time_sec_ = get_parameter("max_episode_time_sec").as_double();
    startup_grace_sec_ = get_parameter("startup_grace_sec").as_double();
    no_start_timeout_sec_ = get_parameter("no_start_timeout_sec").as_double();
    stop_vehicle_on_exit_ = get_parameter("stop_vehicle_on_exit").as_bool();
    shutdown_on_terminate_ = get_parameter("shutdown_on_terminate").as_bool();
  }

  void loadReferenceCsv()
  {
    if (waypoints_csv_.empty()) {
      throw std::runtime_error("lap_referee requires the waypoints_csv parameter.");
    }
    std::ifstream file(waypoints_csv_);
    if (!file.is_open()) {
      throw std::runtime_error("Could not open waypoints CSV: " + waypoints_csv_);
    }
    std::string header_line;
    if (!std::getline(file, header_line)) {
      throw std::runtime_error("Waypoints CSV is empty: " + waypoints_csv_);
    }
    const auto header = splitCsvLine(header_line);
    std::unordered_map<std::string, std::size_t> col;
    for (std::size_t i = 0; i < header.size(); ++i) {
      col[header[i]] = i;
    }
    auto idx = [&col](const std::string & name) -> long {
        const auto it = col.find(name);
        return it == col.end() ? -1 : static_cast<long>(it->second);
      };
    const long ix = idx("x_m");
    const long iy = idx("y_m");
    const long ipsi = idx("psi_rad");
    if (ix < 0 || iy < 0) {
      throw std::runtime_error("Waypoints CSV must contain x_m and y_m columns.");
    }

    std::string line;
    while (std::getline(file, line)) {
      if (trim(line).empty()) {
        continue;
      }
      const auto row = splitCsvLine(line);
      if (static_cast<long>(row.size()) <= std::max(ix, iy)) {
        continue;
      }
      RefPoint p;
      p.x = std::stod(row[ix]);
      p.y = std::stod(row[iy]);
      p.psi = (ipsi >= 0 && static_cast<long>(row.size()) > ipsi) ? std::stod(row[ipsi]) : 0.0;
      p.s = 0.0;
      reference_.push_back(p);
    }
    if (reference_.size() < 4) {
      throw std::runtime_error("Reference raceline needs at least 4 waypoints.");
    }
    // Recompute arc length so progress logic is independent of the CSV's own s.
    track_length_ = 0.0;
    reference_[0].s = 0.0;
    for (std::size_t i = 1; i < reference_.size(); ++i) {
      track_length_ += hypot2(
        reference_[i].x, reference_[i].y, reference_[i - 1].x, reference_[i - 1].y);
      reference_[i].s = track_length_;
    }
    // close the loop length
    track_length_ += hypot2(
      reference_.front().x, reference_.front().y, reference_.back().x, reference_.back().y);
  }

  std::size_t nearestIndex(double x, double y) const
  {
    std::size_t best = 0;
    double best_d = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < reference_.size(); ++i) {
      const double d = hypot2(x, y, reference_[i].x, reference_[i].y);
      if (d < best_d) {
        best_d = d;
        best = i;
      }
    }
    return best;
  }

  double signedLateralError(std::size_t i, double x, double y) const
  {
    const double dx = x - reference_[i].x;
    const double dy = y - reference_[i].y;
    const double nx = -std::sin(reference_[i].psi);
    const double ny = std::cos(reference_[i].psi);
    return dx * nx + dy * ny;
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    pose_x_ = msg->pose.pose.position.x;
    pose_y_ = msg->pose.pose.position.y;
    pose_yaw_ = yawFromOdom(*msg);
    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    speed_ = std::hypot(vx, vy);
    has_odom_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double m = std::numeric_limits<double>::max();
    for (const float r : msg->ranges) {
      if (std::isfinite(r) && r >= msg->range_min && r > 1e-3 && r < m) {
        m = r;
      }
    }
    min_scan_ = std::isfinite(m) ? m : msg->range_max;
    has_scan_ = true;
  }

  void onDrive(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
  {
    cmd_speed_ = msg->drive.speed;
    cmd_steer_ = msg->drive.steering_angle;
  }

  void onCollision(const std_msgs::msg::Bool::SharedPtr msg)
  {
    collision_state_ = msg->data;
    has_collision_state_ = true;
  }

  void step()
  {
    if (phase_ == Phase::Done || !has_odom_) {
      return;
    }
    const double t = (now() - start_wall_).seconds();

    const std::size_t near = nearestIndex(pose_x_, pose_y_);
    const double lat = signedLateralError(near, pose_x_, pose_y_);

    if (phase_ == Phase::WaitingForStart) {
      if (t > no_start_timeout_sec_) {
        terminate("no_start", false, t, near, lat);
        return;
      }
      if (speed_ > start_speed_threshold_) {
        phase_ = Phase::Running;
        run_start_wall_ = now();
        last_index_ = near;
        progress_m_ = 0.0;
        start_index_ = near;
      } else {
        return;  // do not record pre-motion idling
      }
    }

    const double run_t = (now() - run_start_wall_).seconds();

    // Accumulate forward arc-length progress with wrap handling.
    if (near != last_index_) {
      const double n = static_cast<double>(reference_.size());
      double fwd = std::fmod(
        static_cast<double>(near) - static_cast<double>(last_index_) + n, n);
      // Treat a large "forward" jump as a small backward step (noise), ignore it.
      if (fwd > n * 0.5) {
        fwd = 0.0;
      }
      progress_m_ += fwd * (track_length_ / n);
      last_index_ = near;
    }

    recordSample(t, near, lat);
    min_clearance_ = std::min(min_clearance_, has_scan_ ? min_scan_ : min_clearance_);
    max_speed_ = std::max(max_speed_, speed_);
    speed_sum_ += speed_;
    ++speed_count_;

    // --- termination checks (after startup grace) ---
    if (run_t > startup_grace_sec_) {
      if (has_collision_state_) {
        if (collision_state_) {
          terminate("collision", true, run_t, near, lat);
          return;
        }
      } else if (has_scan_ && min_scan_ < collision_scan_threshold_) {
        terminate("collision", true, run_t, near, lat);
        return;
      }
      if (std::abs(lat) > off_track_threshold_) {
        terminate("off_track", true, run_t, near, lat);
        return;
      }
      // Stuck: intent to move (commanded speed high) but body barely moving.
      if (cmd_speed_ > stuck_cmd_threshold_ && speed_ < stuck_speed_threshold_) {
        if (!stuck_active_) {
          stuck_active_ = true;
          stuck_start_wall_ = now();
        } else if ((now() - stuck_start_wall_).seconds() > stuck_time_sec_) {
          terminate("stuck", true, run_t, near, lat);
          return;
        }
      } else {
        stuck_active_ = false;
      }
    }

    if (progress_m_ >= track_length_ * lap_fraction_) {
      terminate("lap_complete", false, run_t, near, lat);
      return;
    }
    if (run_t > max_episode_time_sec_) {
      terminate("timeout", false, run_t, near, lat);
      return;
    }
  }

  void recordSample(double t, std::size_t near, double lat)
  {
    TraceSample s;
    s.t = t;
    s.x = pose_x_;
    s.y = pose_y_;
    s.yaw = pose_yaw_;
    s.v = speed_;
    s.cmd_v = cmd_speed_;
    s.cmd_steer = cmd_steer_;
    s.min_scan = has_scan_ ? min_scan_ : -1.0;
    s.nearest_idx = static_cast<int>(near);
    s.lat_err = lat;
    s.s = reference_[near].s;
    trace_.push_back(s);
  }

  void terminate(
    const std::string & reason, bool collided, double run_t, std::size_t near, double lat)
  {
    phase_ = Phase::Done;
    const bool lap_done = (reason == "lap_complete");
    const double lap_time = lap_done ? run_t : -1.0;
    crash_x_ = pose_x_;
    crash_y_ = pose_y_;
    crash_s_ = reference_[near].s;
    last_lat_ = lat;

    writeOutputs(reason, collided, lap_done, lap_time);

    if (stop_vehicle_on_exit_) {
      stopVehicle();
    }
    RCLCPP_INFO(
      get_logger(),
      "lap_referee terminated: reason=%s collided=%d lap_time=%.3f progress=%.2fm",
      reason.c_str(), static_cast<int>(collided), lap_time, progress_m_);

    if (shutdown_on_terminate_) {
      rclcpp::shutdown();
    }
  }

  void stopVehicle()
  {
    // Best-effort: latch a zero drive command so the bridge does not keep the
    // last requested speed after this rollout ends.
    if (!stop_pub_) {
      stop_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    }
    ackermann_msgs::msg::AckermannDriveStamped stop;
    stop.header.stamp = now();
    stop.drive.speed = 0.0;
    stop.drive.steering_angle = 0.0;
    for (int i = 0; i < 5; ++i) {
      stop_pub_->publish(stop);
    }
  }

  void writeOutputs(
    const std::string & reason, bool collided, bool lap_done, double lap_time)
  {
    std::error_code ec;
    std::filesystem::create_directories(output_dir_, ec);
    if (ec) {
      RCLCPP_ERROR(
        get_logger(), "Could not create output_dir %s: %s",
        output_dir_.c_str(), ec.message().c_str());
    }
    const std::string trace_name = output_prefix_ + "_trace.csv";
    const std::string trace_path = output_dir_ + "/" + trace_name;
    {
      std::ofstream f(trace_path);
      if (f.is_open()) {
        f << "t,x,y,yaw,v,cmd_v,cmd_steer,min_scan,nearest_idx,lat_err,s\n";
        f.setf(std::ios::fixed);
        f.precision(6);
        for (const auto & s : trace_) {
          f << s.t << ',' << s.x << ',' << s.y << ',' << s.yaw << ',' << s.v << ','
            << s.cmd_v << ',' << s.cmd_steer << ',' << s.min_scan << ','
            << s.nearest_idx << ',' << s.lat_err << ',' << s.s << '\n';
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Could not write trace CSV: %s", trace_path.c_str());
      }
    }

    const double mean_speed = speed_count_ >
      0 ? speed_sum_ / static_cast<double>(speed_count_) : 0.0;
    const double clearance = (min_clearance_ == std::numeric_limits<double>::max()) ?
      -1.0 : min_clearance_;

    std::ostringstream json;
    json.setf(std::ios::fixed);
    json.precision(6);
    json << "{\n";
    json << "  \"schema\": \"lap_referee/1\",\n";
    json << "  \"terminated\": \"" << reason << "\",\n";
    json << "  \"collided\": " << (collided ? "true" : "false") << ",\n";
    json << "  \"lap_completed\": " << (lap_done ? "true" : "false") << ",\n";
    json << "  \"lap_time_s\": " << lap_time << ",\n";
    json << "  \"progress_m\": " << progress_m_ << ",\n";
    json << "  \"track_length_m\": " << track_length_ << ",\n";
    json << "  \"start_index\": " << start_index_ << ",\n";
    json << "  \"crash_x\": " << crash_x_ << ",\n";
    json << "  \"crash_y\": " << crash_y_ << ",\n";
    json << "  \"crash_s\": " << crash_s_ << ",\n";
    json << "  \"last_lat_err\": " << last_lat_ << ",\n";
    json << "  \"min_clearance_m\": " << clearance << ",\n";
    json << "  \"mean_speed_mps\": " << mean_speed << ",\n";
    json << "  \"max_speed_mps\": " << max_speed_ << ",\n";
    json << "  \"n_samples\": " << trace_.size() << ",\n";
    json << "  \"trace_csv\": \"" << trace_name << "\"\n";
    json << "}\n";

    // Atomic publish: write tmp then rename so readers never see a partial file.
    const std::string final_path = output_dir_ + "/" + output_prefix_ + "_summary.json";
    const std::string tmp_path = final_path + ".tmp";
    {
      std::ofstream f(tmp_path);
      if (!f.is_open()) {
        RCLCPP_ERROR(get_logger(), "Could not write summary JSON: %s", tmp_path.c_str());
        return;
      }
      f << json.str();
    }
    if (std::rename(tmp_path.c_str(), final_path.c_str()) != 0) {
      RCLCPP_ERROR(get_logger(), "Could not finalize summary JSON: %s", final_path.c_str());
    }
  }

  struct TraceSample
  {
    double t, x, y, yaw, v, cmd_v, cmd_steer, min_scan, lat_err, s;
    int nearest_idx;
  };

  // parameters
  std::string waypoints_csv_, odom_topic_, scan_topic_, drive_topic_, collision_topic_;
  std::string output_dir_, output_prefix_;
  double record_rate_hz_{50.0};
  double start_speed_threshold_{0.4};
  double collision_scan_threshold_{0.13};
  double stuck_speed_threshold_{0.2};
  double stuck_cmd_threshold_{0.8};
  double stuck_time_sec_{0.7};
  double off_track_threshold_{1.5};
  double lap_fraction_{0.97};
  double max_episode_time_sec_{60.0};
  double startup_grace_sec_{0.5};
  double no_start_timeout_sec_{8.0};
  bool stop_vehicle_on_exit_{true};
  bool shutdown_on_terminate_{true};

  // reference raceline
  std::vector<RefPoint> reference_;
  double track_length_{0.0};

  // live state
  Phase phase_{Phase::WaitingForStart};
  bool has_odom_{false}, has_scan_{false}, has_collision_state_{false};
  bool collision_state_{false};
  double pose_x_{0.0}, pose_y_{0.0}, pose_yaw_{0.0}, speed_{0.0};
  double min_scan_{0.0}, cmd_speed_{0.0}, cmd_steer_{0.0};
  std::size_t last_index_{0}, start_index_{0};
  double progress_m_{0.0};
  bool stuck_active_{false};

  // aggregates
  double min_clearance_{std::numeric_limits<double>::max()};
  double max_speed_{0.0}, speed_sum_{0.0};
  long speed_count_{0};
  double crash_x_{0.0}, crash_y_{0.0}, crash_s_{0.0}, last_lat_{0.0};
  std::vector<TraceSample> trace_;

  rclcpp::Time start_wall_, run_start_wall_, stuck_start_wall_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr stop_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LapReferee>());
  } catch (const std::exception & exc) {
    std::fprintf(stderr, "lap_referee failed: %s\n", exc.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
