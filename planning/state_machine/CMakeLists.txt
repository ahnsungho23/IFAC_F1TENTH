cmake_minimum_required(VERSION 3.8)
project(state_machine)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

add_executable(state_machine_node src/state_machine_node.cpp)
ament_target_dependencies(state_machine_node
  rclcpp
  nav_msgs
  sensor_msgs
  f110_msgs
)

install(TARGETS
  state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
