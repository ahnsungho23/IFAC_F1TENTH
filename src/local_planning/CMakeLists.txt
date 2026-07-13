cmake_minimum_required(VERSION 3.8)
project(local_planning)

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
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(local_planner_node src/local_planner_node.cpp)
ament_target_dependencies(local_planner_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  f110_msgs
)
target_link_libraries(local_planner_node Eigen3::Eigen)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
