# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_gqsc_runtime_replay_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED gqsc_runtime_replay_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(gqsc_runtime_replay_FOUND FALSE)
  elseif(NOT gqsc_runtime_replay_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(gqsc_runtime_replay_FOUND FALSE)
  endif()
  return()
endif()
set(_gqsc_runtime_replay_CONFIG_INCLUDED TRUE)

# output package information
if(NOT gqsc_runtime_replay_FIND_QUIETLY)
  message(STATUS "Found gqsc_runtime_replay: 0.1.0 (${gqsc_runtime_replay_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'gqsc_runtime_replay' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${gqsc_runtime_replay_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(gqsc_runtime_replay_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${gqsc_runtime_replay_DIR}/${_extra}")
endforeach()
