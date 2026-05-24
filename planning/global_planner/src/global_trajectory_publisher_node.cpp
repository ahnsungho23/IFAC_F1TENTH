#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "f110_msgs/msg/wpnt_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "readwrite_global_waypoints.hpp"

namespace global_planner
{

class GlobalRepublisherNode : public rclcpp::Node
{
public:
  GlobalRepublisherNode()
  : Node("global_republisher_node")
  {
    declare_parameter("publish_markers", true); /////커(MarkerArray)를 퍼블리시할지 결정합니다. true면 /global_waypoints/markers, /trackbounds/markers 같은 시각화 토픽을 보냄.
    declare_parameter("publish_shortest_path", true); ///최단경로 트래젝토리를 퍼블리시할지 결정합니다. 
    declare_parameter("publish_centerline", false); ////////publish_centerline센터라인 웨이포인트를 퍼블리시할지 결정. true면 /centerline_waypoints(및 마커 옵션 시 /centerline_waypoints/markers) 보냄
    declare_parameter("publish_lattice", false); ///lattice 시각화 토픽을 쓸지 결정합니다. true면 /lattice_viz 퍼블리셔를 활성화합니다
    declare_parameter("map_path", ""); ///global_waypoints.json이 있는 맵 디렉터리 경로. 이 값으로 파일을 읽어와서 재퍼블리시 데이터 소스
    declare_parameter("publish_period_sec", 2.0);

    publish_markers_ = get_parameter("publish_markers").as_bool();
    publish_shortest_path_ = get_parameter("publish_shortest_path").as_bool();
    publish_centerline_ = get_parameter("publish_centerline").as_bool();
    publish_lattice_ = get_parameter("publish_lattice").as_bool();

    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();
    glb_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints", latched_qos);
    map_info_pub_ = create_publisher<std_msgs::msg::String>("/map_infos", 10);
    est_lap_time_pub_ = create_publisher<std_msgs::msg::Float32>("/estimated_lap_time", 10);

    if (publish_markers_) {
      glb_markers_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/global_waypoints/markers", 10);
      vis_track_bnds_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/trackbounds/markers", 10);
    }

    if (publish_shortest_path_) {
      glb_sp_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/global_waypoints/shortest_path", 10);
      if (publish_markers_) {
        glb_sp_markers_pub_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/global_waypoints/shortest_path/markers", 10);
      }
    }

    if (publish_centerline_) {
      centerline_wpnts_pub_ = create_publisher<f110_msgs::msg::WpntArray>("/centerline_waypoints", 10);
      if (publish_markers_) {
        centerline_markers_pub_ =
          create_publisher<visualization_msgs::msg::MarkerArray>("/centerline_waypoints/markers", 10);
      }
    }

    if (publish_lattice_) {
      lattice_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/lattice_viz", 10);
    }

    const auto map_path = get_parameter("map_path").as_string();
    if (!map_path.empty()) {
      std::string error_msg;
      if (!read_global_waypoints(map_path, bundle_, error_msg)) {
        RCLCPP_WARN(get_logger(), "%s", error_msg.c_str());
      } else {
        has_bundle_ = true;
      }
    } else {
      RCLCPP_WARN(get_logger(), "global_trajectory_publisher did not find any map_path param");
    }

    const auto period = get_parameter("publish_period_sec").as_double();
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period)),
      std::bind(&GlobalRepublisherNode::publish_all, this));
  }

private:
  void publish_all()
  {
    if (!has_bundle_) {
      return;
    }

    glb_wpnts_pub_->publish(bundle_.global_traj_wpnts_iqp);
    if (publish_markers_ && glb_markers_pub_) {
      glb_markers_pub_->publish(bundle_.global_traj_markers_iqp);
      vis_track_bnds_pub_->publish(bundle_.trackbounds_markers);
    }

    if (publish_shortest_path_ && glb_sp_wpnts_pub_) {
      glb_sp_wpnts_pub_->publish(bundle_.global_traj_wpnts_sp);
      if (publish_markers_ && glb_sp_markers_pub_) {
        glb_sp_markers_pub_->publish(bundle_.global_traj_markers_sp);
      }
    }

    if (publish_centerline_ && centerline_wpnts_pub_) {
      centerline_wpnts_pub_->publish(bundle_.centerline_waypoints);
      if (publish_markers_ && centerline_markers_pub_) {
        centerline_markers_pub_->publish(bundle_.centerline_markers);
      }
    }

    map_info_pub_->publish(bundle_.map_info_str);
    est_lap_time_pub_->publish(bundle_.est_lap_time);
  }

  bool publish_markers_{true};
  bool publish_shortest_path_{true};
  bool publish_centerline_{false};
  bool publish_lattice_{false};

  bool has_bundle_{false};
  GlobalWaypointBundle bundle_;

  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr glb_markers_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr vis_track_bnds_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr glb_sp_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr glb_sp_markers_pub_;
  rclcpp::Publisher<f110_msgs::msg::WpntArray>::SharedPtr centerline_wpnts_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr centerline_markers_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr map_info_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr est_lap_time_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr lattice_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace global_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<global_planner::GlobalRepublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
