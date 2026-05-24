cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_rw src/readwrite_global_waypoints.cpp)
ament_target_dependencies(${PROJECT_NAME}_rw
  std_msgs
  geometry_msgs
  visualization_msgs
  f110_msgs
)
target_link_libraries(${PROJECT_NAME}_rw nlohmann_json::nlohmann_json)

add_executable(global_planner_node src/global_planner_node.cpp)
ament_target_dependencies(global_planner_node
  rclcpp
  nav_msgs
)
target_link_libraries(global_planner_node ${PROJECT_NAME}_rw)

add_executable(global_trajectory_publisher_node src/global_trajectory_publisher_node.cpp)
ament_target_dependencies(global_trajectory_publisher_node
  rclcpp
  std_msgs
  visualization_msgs
  f110_msgs
)
target_link_libraries(global_trajectory_publisher_node ${PROJECT_NAME}_rw)

add_executable(frenet_odom_node src/frenet_odom_node.cpp)
ament_target_dependencies(frenet_odom_node
  rclcpp
  nav_msgs
  f110_msgs
)

install(TARGETS
  global_planner_node
  global_trajectory_publisher_node
  frenet_odom_node
  ${PROJECT_NAME}_rw
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config data
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
