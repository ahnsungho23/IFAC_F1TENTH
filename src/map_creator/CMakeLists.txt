cmake_minimum_required(VERSION 3.8)
project(map_creator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(local_planning REQUIRED)   # exports local_planning::raceline_planner
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)
find_package(yaml-cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(include)

# ROS-free pipeline modules (unit-testable without rclcpp).
add_library(map_creator_core STATIC
  src/obstacle_ledger.cpp
  src/side_planner_adapter.cpp
  src/map_painter.cpp
  src/regeneration_manager.cpp
)
ament_target_dependencies(map_creator_core
  std_msgs
  f110_msgs
)
target_link_libraries(map_creator_core
  local_planning::raceline_planner
  ${OpenCV_LIBS}
  yaml-cpp
)

add_executable(map_creator_node src/map_creator_node.cpp)
ament_target_dependencies(map_creator_node
  rclcpp
  std_msgs
  std_srvs
  f110_msgs
)
target_link_libraries(map_creator_node
  map_creator_core
  nlohmann_json::nlohmann_json
)

install(TARGETS map_creator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_obstacle_ledger test/test_obstacle_ledger.cpp)
  if(TARGET test_obstacle_ledger)
    ament_target_dependencies(test_obstacle_ledger f110_msgs)
    target_link_libraries(test_obstacle_ledger map_creator_core)
  endif()

  ament_add_gtest(test_map_painter test/test_map_painter.cpp)
  if(TARGET test_map_painter)
    ament_target_dependencies(test_map_painter f110_msgs)
    target_link_libraries(test_map_painter map_creator_core)
  endif()
endif()

ament_package()
