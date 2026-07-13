// opponent_simulator_node.cpp
// Standalone opponent vehicle simulator that follows global waypoints at 0.8x speed.
// Publishes its own odometry on /opponent_racecar/odom for opponent_detector to track.

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <f110_msgs/msg/wpnt_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <vector>
#include <algorithm>

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

class OpponentSimulator : public rclcpp::Node
{
public:
    OpponentSimulator()
    : Node("opponent_simulator"),
      tf_broadcaster_(this)
    {
        declareParameters();
        loadParameters();

        waypoints_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
            waypoints_topic_, rclcpp::QoS(1).reliable().transient_local(),
            [this](const f110_msgs::msg::WpntArray::SharedPtr msg) { onWaypoints(msg); });

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);

        const double dt = 1.0 / update_rate_hz_;
        timer_ = create_wall_timer(
            std::chrono::duration<double>(dt),
            [this]() { updateLoop(); });

        RCLCPP_INFO(get_logger(),
            "opponent_simulator started: speed_scale=%.2f, start_offset=%.1fm, %s",
            speed_scale_, start_offset_m_,
            publish_tf_ ? "TF enabled" : "TF disabled");
    }

private:
    void declareParameters()
    {
        declare_parameter<std::string>("waypoints_topic", "/global_waypoints");
        declare_parameter<std::string>("odom_topic", "/opponent_racecar/odom");
        declare_parameter<std::string>("frame_id", "map");
        declare_parameter<std::string>("base_frame_id", "opponent_base_link");
        declare_parameter<double>("speed_scale", 0.8);
        declare_parameter<double>("start_offset_m", 5.0);
        declare_parameter<double>("update_rate_hz", 50.0);
        declare_parameter<bool>("publish_tf", true);
        declare_parameter<bool>("enabled", true);
    }

    void loadParameters()
    {
        waypoints_topic_ = get_parameter("waypoints_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        frame_id_ = get_parameter("frame_id").as_string();
        base_frame_id_ = get_parameter("base_frame_id").as_string();
        speed_scale_ = get_parameter("speed_scale").as_double();
        start_offset_m_ = get_parameter("start_offset_m").as_double();
        update_rate_hz_ = get_parameter("update_rate_hz").as_double();
        publish_tf_ = get_parameter("publish_tf").as_bool();
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

        // Initialize opponent position at start_offset ahead on the raceline
        if (!initialized_) {
            current_s_ = start_offset_m_;
            updatePoseFromS(current_s_);
            initialized_ = true;
            RCLCPP_INFO(get_logger(), "Initialized opponent at s=%.2fm", current_s_);
        }
    }

    void updateLoop()
    {
        if (!enabled_ || !has_waypoints_ || !initialized_) {
            return;
        }

        // Get target speed at current position
        const double target_speed = getSpeedAtS(current_s_) * speed_scale_;

        // Advance along the raceline
        const double dt = 1.0 / update_rate_hz_;
        current_s_ += target_speed * dt;

        // Wrap around track (circular)
        if (!waypoints_.empty()) {
            const double track_length = waypoints_.back().s_m +
                distance2d(waypoints_.back().x_m, waypoints_.back().y_m,
                          waypoints_.front().x_m, waypoints_.front().y_m);
            if (current_s_ >= track_length) {
                current_s_ -= track_length;
            }
        }

        // Update pose and publish
        updatePoseFromS(current_s_);
        publishOdometry(target_speed);
    }

    double getSpeedAtS(double s) const
    {
        if (waypoints_.empty()) return 0.0;

        // Find waypoint at or before s
        size_t idx = 0;
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            if (waypoints_[i].s_m <= s) {
                idx = i;
            } else {
                break;
            }
        }

        // Linear interpolation to next waypoint
        const size_t next_idx = (idx + 1) % waypoints_.size();
        const double s0 = waypoints_[idx].s_m;
        const double s1 = (next_idx == 0) ?
            (waypoints_.back().s_m + distance2d(waypoints_.back().x_m, waypoints_.back().y_m,
                                                 waypoints_.front().x_m, waypoints_.front().y_m)) :
            waypoints_[next_idx].s_m;
        const double ds = s1 - s0;
        if (ds < 1e-6) {
            return waypoints_[idx].vx_mps;
        }
        const double frac = std::min(1.0, std::max(0.0, (s - s0) / ds));
        return waypoints_[idx].vx_mps + frac * (waypoints_[next_idx].vx_mps - waypoints_[idx].vx_mps);
    }

    void updatePoseFromS(double s)
    {
        if (waypoints_.empty()) return;

        // Find waypoint at or before s
        size_t idx = 0;
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            if (waypoints_[i].s_m <= s) {
                idx = i;
            } else {
                break;
            }
        }

        // Interpolate position and orientation
        const size_t next_idx = (idx + 1) % waypoints_.size();
        const double s0 = waypoints_[idx].s_m;
        const double s1 = (next_idx == 0) ?
            (waypoints_.back().s_m + distance2d(waypoints_.back().x_m, waypoints_.back().y_m,
                                                 waypoints_.front().x_m, waypoints_.front().y_m)) :
            waypoints_[next_idx].s_m;
        const double ds = s1 - s0;
        double frac = 0.0;
        if (ds > 1e-6) {
            frac = std::min(1.0, std::max(0.0, (s - s0) / ds));
        }

        const auto& wp0 = waypoints_[idx];
        const auto& wp1 = waypoints_[next_idx];

        x_ = wp0.x_m + frac * (wp1.x_m - wp0.x_m);
        y_ = wp0.y_m + frac * (wp1.y_m - wp0.y_m);

        // Interpolate yaw with wraparound
        double yaw0 = wp0.psi_rad;
        double yaw1 = wp1.psi_rad;
        double dyaw = normalizeAngle(yaw1 - yaw0);
        yaw_ = normalizeAngle(yaw0 + frac * dyaw);
    }

    void publishOdometry(double speed)
    {
        auto odom = nav_msgs::msg::Odometry();
        odom.header.stamp = now();
        odom.header.frame_id = frame_id_;
        odom.child_frame_id = base_frame_id_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = speed;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = 0.0;

        odom_pub_->publish(odom);

        if (publish_tf_) {
            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp = odom.header.stamp;
            tf.header.frame_id = frame_id_;
            tf.child_frame_id = base_frame_id_;
            tf.transform.translation.x = x_;
            tf.transform.translation.y = y_;
            tf.transform.translation.z = 0.0;
            tf.transform.rotation = odom.pose.pose.orientation;
            tf_broadcaster_.sendTransform(tf);
        }
    }

    std::string waypoints_topic_;
    std::string odom_topic_;
    std::string frame_id_;
    std::string base_frame_id_;
    double speed_scale_;
    double start_offset_m_;
    double update_rate_hz_;
    bool publish_tf_;
    bool enabled_;

    bool has_waypoints_{false};
    bool initialized_{false};
    std::vector<f110_msgs::msg::Wpnt> waypoints_;
    double current_s_{0.0};
    double x_{0.0};
    double y_{0.0};
    double yaw_{0.0};

    rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr waypoints_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpponentSimulator>());
    rclcpp::shutdown();
    return 0;
}
