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

add_executable(map_controller src/map_controller_node.cpp)
ament_target_dependencies(map_controller
  ament_index_cpp
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  ackermann_msgs
  f110_msgs
)

install(TARGETS
  map_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml"
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
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
