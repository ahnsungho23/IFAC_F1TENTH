cmake_minimum_required(VERSION 3.8)
project(obstacle_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
# CLCS Frenet converter library. src/global_planning installs include/ and exports the
# `export_global_planning` target set with HAS_LIBRARY_TARGET, so ament_target_dependencies()
# below pulls in both the headers (global_planning/clcs_frenet_converter.hpp) and the library.
find_package(global_planning REQUIRED)

# ---- obstacle_detector_node: layered LiDAR obstacle detection (/static_obs, /opp_obs) ----
set(detector_deps
  rclcpp
  sensor_msgs
  nav_msgs
  std_msgs
  geometry_msgs
  visualization_msgs
  builtin_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  f110_msgs
  global_planning
)
add_executable(obstacle_detector_node
  src/obstacle_detector_node.cpp
  src/frenet_projector.cpp
  src/obstacle_tracker.cpp
)
target_include_directories(obstacle_detector_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(obstacle_detector_node Eigen3::Eigen)
ament_target_dependencies(obstacle_detector_node ${detector_deps})

install(TARGETS obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  rviz
  docs
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_obstacle_tracker
    test/test_obstacle_tracker.cpp
    src/obstacle_tracker.cpp
    src/frenet_projector.cpp
  )
  target_include_directories(test_obstacle_tracker PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  target_link_libraries(test_obstacle_tracker Eigen3::Eigen)
  ament_target_dependencies(test_obstacle_tracker global_planning)

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
