cmake_minimum_required(VERSION 3.8)
project(global_planning)

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
find_package(nlohmann_json REQUIRED)
# Public interface dependencies of the exported frenet_conversion / crccosy libs.
# Needed in this scope so ament_export_dependencies() can re-export them downstream.
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(include)

add_subdirectory(vendor/commonroad_clcs)

add_library(${PROJECT_NAME}_rw src/readwrite_global_waypoints.cpp)
target_link_libraries(${PROJECT_NAME}_rw PUBLIC
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${f110_msgs_TARGETS}
  nlohmann_json::nlohmann_json
)

add_executable(global_planning_node src/global_planning_node.cpp)
target_link_libraries(global_planning_node PUBLIC
  ${PROJECT_NAME}_rw
  rclcpp::rclcpp
  ${nav_msgs_TARGETS}
)

add_executable(global_trajectory_publisher_node src/global_trajectory_publisher_node.cpp)
target_link_libraries(global_trajectory_publisher_node PUBLIC
  ${PROJECT_NAME}_rw
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${f110_msgs_TARGETS}
)

add_library(${PROJECT_NAME}_frenet_conversion src/clcs_frenet_converter.cpp)
target_include_directories(${PROJECT_NAME}_frenet_conversion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}_frenet_conversion
  PUBLIC
    CommonRoadCLCS::crccosy
)

add_executable(frenet_odom_node src/frenet_odom_node.cpp)
target_link_libraries(frenet_odom_node PUBLIC
  ${PROJECT_NAME}_frenet_conversion
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${f110_msgs_TARGETS}
)

install(TARGETS
  global_planning_node
  global_trajectory_publisher_node
  frenet_odom_node
  ${PROJECT_NAME}_rw
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
)

# --- Export the Frenet conversion library + headers for downstream packages ---
# opponent_detector #includes "global_planning/clcs_frenet_converter.hpp" (which
# transitively includes the vendored "geometry/..." headers) and links this lib.
install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}_frenet_conversion
  EXPORT export_global_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_global_planning HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 Boost spdlog OpenMP)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_clcs_frenet_converter test/test_clcs_frenet_converter.cpp)
  if(TARGET test_clcs_frenet_converter)
    target_link_libraries(test_clcs_frenet_converter ${PROJECT_NAME}_frenet_conversion)
  endif()
endif()

ament_package()
