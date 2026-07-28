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
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(global_planning REQUIRED)

include_directories(
  include
)

add_library(local_planner_core STATIC
  src/local_planner_node.cpp
  src/raceline_spline_planner.cpp
  src/safe_corridor.cpp
)
target_link_libraries(local_planner_core PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${f110_msgs_TARGETS}
  # CLCS Frenet converter lib exported by global_planning (ament_export_targets)
  global_planning::global_planning_frenet_conversion
)

add_executable(local_planner_node src/local_planner_main.cpp)
target_link_libraries(local_planner_node PUBLIC local_planner_core rclcpp::rclcpp)

install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_frenet_lattice test/test_frenet_lattice.cpp)
  if(TARGET test_frenet_lattice)
    target_link_libraries(test_frenet_lattice
      local_planner_core
      rclcpp::rclcpp
      ${nav_msgs_TARGETS}
      ${f110_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_raceline_spline test/test_raceline_spline.cpp)
  if(TARGET test_raceline_spline)
    target_link_libraries(test_raceline_spline
      local_planner_core
      rclcpp::rclcpp
      ${nav_msgs_TARGETS}
      ${f110_msgs_TARGETS}
    )
  endif()

  ament_add_gtest(test_safe_corridor test/test_safe_corridor.cpp)
  if(TARGET test_safe_corridor)
    target_link_libraries(test_safe_corridor local_planner_core ${nav_msgs_TARGETS})
  endif()
endif()

ament_package()
