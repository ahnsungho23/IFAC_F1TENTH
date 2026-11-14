# Install script for directory: /home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/gqsc_runtime_replay

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay" TYPE EXECUTABLE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/sce018_replay_driver")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver"
         OLD_RPATH "/opt/ros/humble/lib:/home/sungho/Documents/GitHub/2026_IFAC/install/f110_msgs/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/gqsc_runtime_replay/sce018_replay_driver")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/gqsc_runtime_replay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/gqsc_runtime_replay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay/environment" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay/environment" TYPE FILE FILES "/opt/ros/humble/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay/environment" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_index/share/ament_index/resource_index/packages/gqsc_runtime_replay")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay/cmake" TYPE FILE FILES
    "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_core/gqsc_runtime_replayConfig.cmake"
    "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/ament_cmake_core/gqsc_runtime_replayConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gqsc_runtime_replay" TYPE FILE FILES "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/gqsc_runtime_replay/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/sungho/Documents/GitHub/2026_IFAC/planning_study/gqsc_s1_live_runtime_qualification_v1/tools/_build/gqsc_runtime_replay/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
