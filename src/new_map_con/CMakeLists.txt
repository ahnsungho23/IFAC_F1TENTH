cmake_minimum_required(VERSION 3.8)
project(new_map_con)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(map_controller src/map_controller_node.cpp)
target_link_libraries(map_controller
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${f110_msgs_TARGETS}
)

add_executable(opponent_simulator src/opponent_simulator_node.cpp)
target_link_libraries(opponent_simulator
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${f110_msgs_TARGETS}
  tf2::tf2
  tf2_ros::tf2_ros
)

add_executable(opponent_drive_controller src/opponent_drive_controller.cpp)
target_link_libraries(opponent_drive_controller
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
  ${ackermann_msgs_TARGETS}
  ${f110_msgs_TARGETS}
)

install(TARGETS
  map_controller
  opponent_simulator
  opponent_drive_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
  FILES_MATCHING
    PATTERN "*.csv"
    PATTERN "*.yaml"
    PATTERN "*.yml"
    PATTERN "*.pgm"
    PATTERN "*.png"
)

install(DIRECTORY docs/
  DESTINATION share/${PROJECT_NAME}/docs
  FILES_MATCHING PATTERN "*.md"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  # Legacy Python controller files are kept for reference but are no longer installed.
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  # Avoid network-dependent schema fetches in offline build environments.
  set(ament_cmake_xmllint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
