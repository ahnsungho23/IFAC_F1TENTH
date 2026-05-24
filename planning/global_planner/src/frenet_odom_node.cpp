#include <limits>
#include <memory>
#include <string>

#include "f110_msgs/msg/wpnt_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

namespace global_planner
{

class FrenetOdomNode : public rclcpp::Node
{
public:
  FrenetOdomNode()
  : Node("frenet_odom_node")
  {
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/pf/pose/odom", 20, std::bind(&FrenetOdomNode::odom_cb, this, std::placeholders::_1));
    wpnt_sub_ = create_subscription<f110_msgs::msg::WpntArray>(
      "/global_waypoints", rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&FrenetOdomNode::waypoints_cb, this, std::placeholders::_1));
    out_pub_ = create_publisher<nav_msgs::msg::Odometry>("/car_state/frenet/odom", 20);
  }

private:
  void waypoints_cb(const f110_msgs::msg::WpntArray::SharedPtr msg)
  {
    wpnts_ = *msg;
    has_wpnts_ = !wpnts_.wpnts.empty();
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr odom)
  {
    if (!has_wpnts_) {
      return;
    }

    int closest_idx = 0;
    double best_d2 = std::numeric_limits<double>::max();

    const double cx = odom->pose.pose.position.x;
    const double cy = odom->pose.pose.position.y;

    for (size_t i = 0; i < wpnts_.wpnts.size(); ++i) {
      const auto & w = wpnts_.wpnts[i];
      const double dx = w.x_m - cx;
      const double dy = w.y_m - cy;
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) {
        best_d2 = d2;
        closest_idx = static_cast<int>(i);
      }
    }

    nav_msgs::msg::Odometry out = *odom;
    out.header.frame_id = "frenet";
    out.child_frame_id = std::to_string(closest_idx);
    out.pose.pose.position.x = wpnts_.wpnts[closest_idx].s_m;
    out.pose.pose.position.y = wpnts_.wpnts[closest_idx].d_m;
    out_pub_->publish(out);
  }

  bool has_wpnts_{false};
  f110_msgs::msg::WpntArray wpnts_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<f110_msgs::msg::WpntArray>::SharedPtr wpnt_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr out_pub_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planner::FrenetOdomNode>());
  rclcpp::shutdown();
  return 0;
}
