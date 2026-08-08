cmake_minimum_required(VERSION 3.8)
project(static_obstacle_map)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(f110_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(static_obstacle_map_core
  src/obstacle_marker_builder.cpp
  src/static_obstacle_map_memory.cpp
)
target_include_directories(static_obstacle_map_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(static_obstacle_map_core
  f110_msgs
  std_msgs
  visualization_msgs
)

add_executable(static_obstacle_map_node
  src/static_obstacle_map_node.cpp
)
target_include_directories(static_obstacle_map_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(static_obstacle_map_node static_obstacle_map_core)
ament_target_dependencies(static_obstacle_map_node
  f110_msgs
  rclcpp
  std_srvs
  visualization_msgs
)

install(
  TARGETS static_obstacle_map_core static_obstacle_map_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_add_gtest(test_static_obstacle_map_memory
    test/test_static_obstacle_map_memory.cpp
  )
  target_link_libraries(test_static_obstacle_map_memory static_obstacle_map_core)
  ament_target_dependencies(test_static_obstacle_map_memory
    f110_msgs
    std_msgs
    visualization_msgs
  )

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
