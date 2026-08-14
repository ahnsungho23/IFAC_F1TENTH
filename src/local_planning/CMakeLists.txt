cmake_minimum_required(VERSION 3.8)
project(local_planning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

include_directories(
  include
)

# ROS-free planner core, exported so other packages (map_creator) can reuse the
# exact left/right avoidance decision logic instead of replicating it.
add_library(raceline_planner STATIC
  src/raceline_spline_planner.cpp
  src/p3_shadow.cpp
)
set_target_properties(raceline_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(raceline_planner PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(raceline_planner
  std_msgs
  f110_msgs
)

add_library(local_planner_core STATIC
  src/local_planner_node.cpp
  src/obstacle_guard.cpp
  src/p3_maneuver_lifecycle.cpp
  src/safe_stop_lifecycle.cpp
)
ament_target_dependencies(local_planner_core
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  f110_msgs
)
target_link_libraries(local_planner_core raceline_planner)

add_executable(local_planner_node src/local_planner_main.cpp)
ament_target_dependencies(local_planner_node rclcpp)
target_link_libraries(local_planner_node local_planner_core)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS raceline_planner
  EXPORT export_raceline_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
ament_export_targets(export_raceline_planner HAS_LIBRARY_TARGET)
ament_export_dependencies(std_msgs f110_msgs)
ament_export_include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_raceline_spline test/test_raceline_spline.cpp)
  if(TARGET test_raceline_spline)
    ament_target_dependencies(test_raceline_spline
      rclcpp
      nav_msgs
      f110_msgs
    )
    target_link_libraries(test_raceline_spline local_planner_core)
  endif()

  ament_add_gtest(test_obstacle_guard test/test_obstacle_guard.cpp)
  if(TARGET test_obstacle_guard)
    ament_target_dependencies(test_obstacle_guard
      f110_msgs
    )
    target_link_libraries(test_obstacle_guard local_planner_core)
  endif()

  ament_add_gtest(test_safe_stop_lifecycle test/test_safe_stop_lifecycle.cpp)
  if(TARGET test_safe_stop_lifecycle)
    target_link_libraries(test_safe_stop_lifecycle local_planner_core)
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_p3_maneuver_lifecycle.cpp")
    ament_add_gtest(
      test_p3_maneuver_lifecycle
      test/test_p3_maneuver_lifecycle.cpp)
    if(TARGET test_p3_maneuver_lifecycle)
      ament_target_dependencies(test_p3_maneuver_lifecycle f110_msgs)
      target_link_libraries(test_p3_maneuver_lifecycle local_planner_core)
    endif()
  endif()

  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/test_p3_production_parity.cpp")
    ament_add_gtest(test_p3_production_parity test/test_p3_production_parity.cpp)
    if(TARGET test_p3_production_parity)
      ament_target_dependencies(test_p3_production_parity f110_msgs)
      target_link_libraries(test_p3_production_parity local_planner_core)
    endif()
  endif()

  add_executable(
    p3_production_parity_harness
    test/p3_production_parity_harness.cpp)
  ament_target_dependencies(p3_production_parity_harness f110_msgs)
  target_link_libraries(p3_production_parity_harness local_planner_core)

endif()

ament_package()
