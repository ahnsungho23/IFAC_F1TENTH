cmake_minimum_required(VERSION 3.8)
project(gqsc_runtime_replay)

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
find_package(f110_msgs REQUIRED)

add_executable(sce018_replay_driver src/sce018_replay_driver.cpp)
ament_target_dependencies(sce018_replay_driver rclcpp std_msgs nav_msgs f110_msgs)

install(TARGETS sce018_replay_driver DESTINATION lib/${PROJECT_NAME})
ament_package()

