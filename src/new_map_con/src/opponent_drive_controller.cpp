// opponent_drive_controller.cpp
// Publishes drive commands to f1sim's opponent vehicle to follow global waypoints at scaled speed.
// Works with num_agent: 2 in f1sim config.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>

#include <cmath>
#include <vector>

namespace
{

constexpr double kPi = 3.14159265358979323846;

double normalizeAngle(double angle)
{
    while (angle > kPi) angle -= 2.0 * kPi;
    while (angle < -kPi) angle += 2.0 * kPi;
    return angle;
}

double distance2d(double x1, double y1, double x2, double y2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::hypot(dx, dy);
}

}  // namespace

class OpponentDriveController : public rclcpp::Node
{
public:
    OpponentDriveController()
    : Node("opponent_drive_controller")
    {
        declareParameters();
        loadParameters();

        waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
            waypoints_topic_, rclcpp::QoS(1).reliable().transient_local(),
            [this](const f110_msgs::msg::WpntArray::SharedPtr msg) { onWaypoints(msg); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            opp_odom_topic_, 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { onOdom(msg); });

        drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            opp_drive_topic_, 10);

        const double dt = 1.0 / control_rate_hz_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(dt),
            [this]() { controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "opponent_drive_controller started: speed_scale=%.2f, lookahead=%.2fm",
            speed_scale_, lookahead_distance_);
    }

private:
    void declareParameters()
    {
        declare_parameter<std::string>("waypoints_topic", "/global_waypoints");
        declare_parameter<std::string>("opp_odom_topic", "/opp_racecar/odom");
        declare_parameter<std::string>("opp_drive_topic", "/opp_drive");
        declare_parameter<double>("speed_scale", 0.8);
        declare_parameter<double>("lookahead_distance", 2.0);
        declare_parameter<double>("wheelbase", 0.33);
        declare_parameter<double>("control_rate_hz", 40.0);
        declare_parameter<bool>("enabled", true);
    }

    void loadParameters()
    {
        waypoints_topic_ = get_parameter("waypoints_topic").as_string();
        opp_odom_topic_ = get_parameter("opp_odom_topic").as_string();
        opp_drive_topic_ = get_parameter("opp_drive_topic").as_string();
        speed_scale_ = get_parameter("speed_scale").as_double();
        lookahead_distance_ = get_parameter("lookahead_distance").as_double();
        wheelbase_ = get_parameter("wheelbase").as_double();
        control_rate_hz_ = get_parameter("control_rate_hz").as_double();
        enabled_ = get_parameter("enabled").as_bool();
    }

    void onWaypoints(const f110_msgs::msg::WpntArray::SharedPtr msg)
    {
        if (msg->wpnts.empty()) {
            RCLCPP_WARN(get_logger(), "Received empty waypoints");
            return;
        }
        waypoints_ = msg->wpnts;
        has_waypoints_ = true;
        RCLCPP_INFO(get_logger(), "Received %zu waypoints", waypoints_.size());
    }

    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        opp_x_ = msg->pose.pose.position.x;
        opp_y_ = msg->pose.pose.position.y;

        // Extract yaw from quaternion
        const auto& q = msg->pose.pose.orientation;
        opp_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z));

        has_odom_ = true;
    }

    void controlLoop()
    {
        if (!enabled_ || !has_waypoints_ || !has_odom_) {
            return;
        }

        // Find nearest waypoint
        size_t nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            const double dist = distance2d(opp_x_, opp_y_,
                                          waypoints_[i].x_m, waypoints_[i].y_m);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        // Find lookahead point
        double accumulated_dist = 0.0;
        size_t lookahead_idx = nearest_idx;
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            const size_t curr = (nearest_idx + i) % waypoints_.size();
            const size_t next = (nearest_idx + i + 1) % waypoints_.size();

            accumulated_dist += distance2d(waypoints_[curr].x_m, waypoints_[curr].y_m,
                                          waypoints_[next].x_m, waypoints_[next].y_m);

            if (accumulated_dist >= lookahead_distance_) {
                lookahead_idx = next;
                break;
            }
        }

        // Pure pursuit steering
        const double goal_x = waypoints_[lookahead_idx].x_m;
        const double goal_y = waypoints_[lookahead_idx].y_m;

        const double dx = goal_x - opp_x_;
        const double dy = goal_y - opp_y_;
        const double alpha = normalizeAngle(std::atan2(dy, dx) - opp_yaw_);
        const double ld = std::hypot(dx, dy);

        double steering = std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld);
        steering = std::clamp(steering, -0.4, 0.4);  // Limit steering angle

        // Target speed from waypoint
        const double target_speed = waypoints_[lookahead_idx].vx_mps * speed_scale_;

        // Publish drive command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = now();
        drive_msg.drive.speed = target_speed;
        drive_msg.drive.steering_angle = steering;
        drive_pub_->publish(drive_msg);
    }

    // Parameters
    std::string waypoints_topic_;
    std::string opp_odom_topic_;
    std::string opp_drive_topic_;
    double speed_scale_;
    double lookahead_distance_;
    double wheelbase_;
    double control_rate_hz_;
    bool enabled_;

    // State
    std::vector<f110_msgs::msg::Wpnt> waypoints_;
    bool has_waypoints_{false};
    bool has_odom_{false};
    double opp_x_{0.0};
    double opp_y_{0.0};
    double opp_yaw_{0.0};

    // ROS
    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpponentDriveController>());
    rclcpp::shutdown();
    return 0;
}
