# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_ulisse_ctrl_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED ulisse_ctrl_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(ulisse_ctrl_FOUND FALSE)
  elseif(NOT ulisse_ctrl_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(ulisse_ctrl_FOUND FALSE)
  endif()
  return()
endif()
set(_ulisse_ctrl_CONFIG_INCLUDED TRUE)

# output package information
if(NOT ulisse_ctrl_FIND_QUIETLY)
  message(STATUS "Found ulisse_ctrl: 0.0.1 (${ulisse_ctrl_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'ulisse_ctrl' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message(WARNING "${_msg}")
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(ulisse_ctrl_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake")
foreach(_extra ${_extras})
  include("${ulisse_ctrl_DIR}/${_extra}")
endforeach()
