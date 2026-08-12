cmake_minimum_required(VERSION 3.8)
project(cma_gt_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(gt_localization_conversion src/odometry_conversion.cpp)
target_include_directories(gt_localization_conversion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(gt_localization_conversion nav_msgs)

add_executable(gt_localization_bridge_node src/gt_localization_bridge_node.cpp)
target_link_libraries(gt_localization_bridge_node gt_localization_conversion)
ament_target_dependencies(gt_localization_bridge_node nav_msgs rclcpp)

install(TARGETS gt_localization_conversion gt_localization_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch docs DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_odometry_conversion test/test_odometry_conversion.cpp)
  target_link_libraries(test_odometry_conversion gt_localization_conversion)
  ament_target_dependencies(test_odometry_conversion nav_msgs)
endif()

ament_export_include_directories(include)
ament_export_dependencies(nav_msgs)
ament_package()

