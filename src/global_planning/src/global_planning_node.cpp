#include <cstdio>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "readwrite_global_waypoints.hpp"

namespace global_planning
{

class GlobalPlannerNode : public rclcpp::Node
{
public:
  GlobalPlannerNode()
  : Node("global_planning_node")
  {
    declare_parameter("map_dir", "");
    declare_parameter("optimizer_command", "");
    declare_parameter("pythonpath_extra", "");
    declare_parameter("run_once", true);
    declare_parameter("trigger_on_start", true);

    map_dir_ = get_parameter("map_dir").as_string();
    optimizer_command_ = get_parameter("optimizer_command").as_string();
    pythonpath_extra_ = get_parameter("pythonpath_extra").as_string();
    run_once_ = get_parameter("run_once").as_bool();
    trigger_on_start_ = get_parameter("trigger_on_start").as_bool();

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&GlobalPlannerNode::map_cb, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/pf/pose/odom", 20, std::bind(&GlobalPlannerNode::odom_cb, this, std::placeholders::_1));
  }

private:
  void map_cb(const nav_msgs::msg::OccupancyGrid::SharedPtr)
  {
    got_map_ = true;
    maybe_trigger();
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr)
  {
    got_odom_ = true;
    maybe_trigger();
  }

  void maybe_trigger()
  {
    if (!trigger_on_start_ || already_ran_ || !got_map_ || !got_odom_) {
      return;
    }
    already_ran_ = true;
    execute_optimizer_bridge();
  }

  void execute_optimizer_bridge()
  {
    if (map_dir_.empty()) {
      RCLCPP_ERROR(get_logger(), "map_dir parameter is empty");
      return;
    }
    if (optimizer_command_.empty()) {
      RCLCPP_ERROR(get_logger(), "optimizer_command parameter is empty");
      return;
    }

    std::stringstream cmd;
    if (!pythonpath_extra_.empty()) {
      cmd << "export PYTHONPATH=" << pythonpath_extra_ << ":$PYTHONPATH && ";
    }
    cmd << optimizer_command_ << " --map-dir " << map_dir_;

    RCLCPP_INFO(get_logger(), "Running optimizer command: %s", cmd.str().c_str());
    const int rc = std::system(cmd.str().c_str());
    if (rc != 0) {
      RCLCPP_ERROR(get_logger(), "Optimizer bridge failed with rc=%d", rc);
      return;
    }

    GlobalWaypointBundle bundle;
    std::string err;
    if (!read_global_waypoints(map_dir_, bundle, err)) {
      RCLCPP_ERROR(get_logger(), "Optimizer finished, but json read failed: %s", err.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "global_waypoints.json created: %zu iqp wpnts / %zu centerline wpnts",
      bundle.global_traj_wpnts_iqp.wpnts.size(), bundle.centerline_waypoints.wpnts.size());

    if (!run_once_) {
      already_ran_ = false;
    }
  }

  bool got_map_{false};
  bool got_odom_{false};
  bool already_ran_{false};
  bool run_once_{true};
  bool trigger_on_start_{true};
  std::string map_dir_;
  std::string optimizer_command_;
  std::string pythonpath_extra_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

}  // namespace global_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<global_planning::GlobalPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
