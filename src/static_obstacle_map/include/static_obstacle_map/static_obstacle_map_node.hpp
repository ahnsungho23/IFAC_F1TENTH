#ifndef STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_NODE_HPP_
#define STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_NODE_HPP_

#include <memory>
#include <string>

#include <f110_msgs/msg/obstacle_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "static_obstacle_map/obstacle_marker_builder.hpp"
#include "static_obstacle_map/static_obstacle_map_memory.hpp"

namespace static_obstacle_map
{

class StaticObstacleMapNode : public rclcpp::Node
{
public:
  explicit StaticObstacleMapNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareParameters();
  void loadParameters();
  void confirmedStaticCallback(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void dynamicCallback(const f110_msgs::msg::ObstacleArray::SharedPtr message);
  void resetCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);
  void publishVisualization();

  std::string confirmed_static_obs_topic_;
  std::string dynamic_obs_topic_;
  std::string visualization_topic_;
  std::string frame_id_;
  std::string reset_service_;
  bool publish_visualization_{true};

  MemoryConfig memory_config_;
  ObstacleMarkerConfig marker_config_;
  StaticObstacleMapMemory memory_;

  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr confirmed_static_sub_;
  rclcpp::Subscription<f110_msgs::msg::ObstacleArray>::SharedPtr dynamic_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    visualization_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_service_server_;
};

}  // namespace static_obstacle_map

#endif  // STATIC_OBSTACLE_MAP__STATIC_OBSTACLE_MAP_NODE_HPP_
