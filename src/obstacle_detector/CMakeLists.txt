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
# CLCS Frenet 변환기 라이브러리. src/global_planning이 include/를 설치하고
# HAS_LIBRARY_TARGET이 설정된 `export_global_planning` target을 내보낸다. 따라서 아래
# ament_target_dependencies()가 헤더와 라이브러리를 함께 연결한다.
find_package(global_planning REQUIRED)

# 계층형 LiDAR 장애물 검출 실행 파일(/static_obs, /opp_obs)
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
  src/aabb_frenet_projector.cpp
  src/frenet_marker_builder.cpp
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
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  # 추적 상태 전이, AABB 투영, 최종 marker 형상은 각각 독립 gtest로 검증한다.
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

  ament_add_gtest(test_aabb_frenet_projector
    test/test_aabb_frenet_projector.cpp
    src/aabb_frenet_projector.cpp
  )
  target_include_directories(test_aabb_frenet_projector PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_aabb_frenet_projector global_planning)

  ament_add_gtest(test_frenet_marker_builder
    test/test_frenet_marker_builder.cpp
    src/frenet_marker_builder.cpp
    src/frenet_projector.cpp
  )
  target_include_directories(test_frenet_marker_builder PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_frenet_marker_builder
    f110_msgs
    geometry_msgs
    visualization_msgs
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
