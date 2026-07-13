#pragma once

#include <string>

#include "f110_msgs/msg/wpnt_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace global_planning
{

struct GlobalWaypointBundle
{
  std_msgs::msg::String map_info_str;
  std_msgs::msg::Float32 est_lap_time;
  visualization_msgs::msg::MarkerArray centerline_markers;
  f110_msgs::msg::WpntArray centerline_waypoints;
  visualization_msgs::msg::MarkerArray global_traj_markers_iqp;
  f110_msgs::msg::WpntArray global_traj_wpnts_iqp;
  visualization_msgs::msg::MarkerArray global_traj_markers_sp;
  f110_msgs::msg::WpntArray global_traj_wpnts_sp;
  visualization_msgs::msg::MarkerArray trackbounds_markers;
};

bool read_global_waypoints(
  const std::string & map_dir, GlobalWaypointBundle & out_bundle, std::string & error_msg);

bool write_global_waypoints(
  const std::string & map_dir, const GlobalWaypointBundle & bundle, std::string & error_msg);

}  // namespace global_planning
