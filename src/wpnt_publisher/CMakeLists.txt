cmake_minimum_required(VERSION 3.8)
project(wpnt_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

add_executable(wpnt_publisher src/wpnt_publisher.cpp)
target_link_libraries(wpnt_publisher rclcpp::rclcpp ${nav_msgs_TARGETS} ${f110_msgs_TARGETS})

install(TARGETS wpnt_publisher
  DESTINATION lib/${PROJECT_NAME})

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



ament_package()
