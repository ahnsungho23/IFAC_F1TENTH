# Standalone offline trajectory generator (no ROS dependencies).
# Build:  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build
# Binaries are placed in bin/ (the GUI and map_creator expect them there).
# Portable across x86_64 and arm64 (Jetson): no arch-specific flags.
cmake_minimum_required(VERSION 3.16)
project(offline_trajectory_generator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)

add_library(trajectory_core STATIC src/trajectory_core.cpp)
target_include_directories(trajectory_core PUBLIC
  src
  third_party/LBFGSpp/include
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(trajectory_core PUBLIC
  ${OpenCV_LIBS} yaml-cpp Eigen3::Eigen nlohmann_json::nlohmann_json)
if(TARGET opencv_ximgproc)
  target_compile_definitions(trajectory_core PUBLIC HAVE_OPENCV_XIMGPROC)
endif()

add_executable(generate_global_trajectory src/generate_main.cpp)
target_link_libraries(generate_global_trajectory PRIVATE trajectory_core)

add_executable(regenerate_obstacle_map src/regenerate_main.cpp)
target_link_libraries(regenerate_obstacle_map PRIVATE trajectory_core)

set_target_properties(generate_global_trajectory regenerate_obstacle_map PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
