# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_laplace_pathfinder_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED laplace_pathfinder_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(laplace_pathfinder_FOUND FALSE)
  elseif(NOT laplace_pathfinder_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(laplace_pathfinder_FOUND FALSE)
  endif()
  return()
endif()
set(_laplace_pathfinder_CONFIG_INCLUDED TRUE)

# output package information
if(NOT laplace_pathfinder_FIND_QUIETLY)
  message(STATUS "Found laplace_pathfinder: 1.0.0 (${laplace_pathfinder_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'laplace_pathfinder' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${laplace_pathfinder_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(laplace_pathfinder_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${laplace_pathfinder_DIR}/${_extra}")
endforeach()
