cmake_minimum_required(VERSION 3.8)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(spdlog REQUIRED)
find_package(OpenMP REQUIRED)

set(CR_CLCS_SRC_FILES
  cpp/src/segment.cc
  cpp/src/curvilinear_coordinate_system.cc
  cpp/src/projection_domain.cc
  cpp/src/path_segments.cc
  cpp/src/util.cc
  cpp/src/util_sweep_line.cc
  cpp/src/util_projection_domain.cc
)

add_library(crccosy STATIC ${CR_CLCS_SRC_FILES})
add_library(CommonRoadCLCS::crccosy ALIAS crccosy)

target_compile_features(crccosy PUBLIC cxx_std_17)
target_compile_definitions(crccosy PUBLIC ENABLE_SERIALIZER=0)
target_include_directories(crccosy
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/cpp/include>
    $<INSTALL_INTERFACE:include>
)

set(_commonroad_boost_targets)
if(TARGET Boost::headers)
  list(APPEND _commonroad_boost_targets Boost::headers)
elseif(TARGET Boost::boost)
  list(APPEND _commonroad_boost_targets Boost::boost)
endif()

if(TARGET Boost::geometry)
  list(APPEND _commonroad_boost_targets Boost::geometry)
endif()

target_link_libraries(crccosy
  PRIVATE
    OpenMP::OpenMP_CXX
  PUBLIC
    Eigen3::Eigen
    spdlog::spdlog
    ${_commonroad_boost_targets}
)

target_include_directories(crccosy SYSTEM PUBLIC ${Boost_INCLUDE_DIRS})

# --- Install / export so downstream packages (e.g. opponent_detector) can link ---
# frenet_conversion PUBLIC-links crccosy, so crccosy must live in the same export
# set that global_planning exports, and its headers must be installed.
install(TARGETS crccosy
  EXPORT export_global_planning
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY cpp/include/
  DESTINATION include
)
