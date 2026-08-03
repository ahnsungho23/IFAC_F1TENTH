cmake_minimum_required(VERSION 3.8)
project(local_planning)

# ROS 2 Jazzy 런타임 구현은 C++17을 사용한다.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 노드, 메시지, 경로 및 RViz marker 의존성
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(f110_msgs REQUIRED)

include_directories(
  include
)

# 계획 알고리즘과 ROS 노드 구현을 하나의 정적 라이브러리로 묶어 실행 파일과 테스트가 공유한다.
add_library(local_planner_core STATIC
  src/local_planner_node.cpp
  src/obstacle_guard.cpp
  src/raceline_spline_planner.cpp
)
ament_target_dependencies(local_planner_core
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  f110_msgs
)

# main은 executor 구성만 담당하고 실제 기능은 local_planner_core에 둔다.
add_executable(local_planner_node src/local_planner_main.cpp)
ament_target_dependencies(local_planner_node rclcpp)
target_link_libraries(local_planner_node local_planner_core)

# ros2 run/launch가 찾을 수 있도록 실행 파일을 package 전용 lib 경로에 설치한다.
install(TARGETS
  local_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# 다른 target이 공개 헤더를 include할 수 있게 설치한다.
install(DIRECTORY include/
  DESTINATION include
)

# launch, YAML, 운영 문서를 package share에 함께 설치한다.
install(DIRECTORY launch config docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

# BUILD_TESTING일 때 spline과 uncertainty Guard의 단위 테스트를 등록한다.
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_raceline_spline test/test_raceline_spline.cpp)
  if(TARGET test_raceline_spline)
    ament_target_dependencies(test_raceline_spline
      rclcpp
      nav_msgs
      f110_msgs
    )
    target_link_libraries(test_raceline_spline local_planner_core)
  endif()

  ament_add_gtest(test_obstacle_guard test/test_obstacle_guard.cpp)
  if(TARGET test_obstacle_guard)
    ament_target_dependencies(test_obstacle_guard
      f110_msgs
    )
    target_link_libraries(test_obstacle_guard local_planner_core)
  endif()
endif()

ament_package()
