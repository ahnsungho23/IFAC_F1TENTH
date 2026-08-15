cmake_minimum_required(VERSION 3.8)
project(opponent_simulator)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

add_executable(opponent_drive_controller src/opponent_drive_controller.cpp)
target_link_libraries(opponent_drive_controller
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${f110_msgs_TARGETS}
)

install(TARGETS opponent_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
