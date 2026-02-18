cmake_minimum_required(VERSION 3.5)
project(nav_filter)

#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 20)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Enable ccache for faster rebuilds
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Option to enable/disable unity builds (faster compilation)
option(ENABLE_UNITY_BUILD "Enable unity/jumbo builds for faster compilation" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(surface_vehicle_model REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_driver REQUIRED)
find_package(rml REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    #"msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${ulisse_driver_INCLUDE_DIRS}
  ${ulisse_msgs_INCLUDE_DIRS}
  ${ulisse_ctrl_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)

#message(STATUS "****** ulisse_driver_LIBRARIES ******: ${ulisse_driver_LIBRARIES}")


## Libraries ##
set(nav_filter_lib_sources
    # Main Classes
    src/${PROJECT_NAME}/navigation_filter.cpp
    src/${PROJECT_NAME}/luenberger_observer/pos_vel_observer.cpp
    # Helpers
    src/${PROJECT_NAME}/nav_data_structs.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/gps.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/compass.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/magnetometer.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/accelerometer.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/gyro.cpp
    src/${PROJECT_NAME}/kalman_filter/ulisse_vehicle_model.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/z_meter.cpp
    src/${PROJECT_NAME}/kalman_filter/measurements/rpm.cpp
)

## Libraries
add_library(nav_filter_lib ${nav_filter_lib_sources})

# Enable unity builds for faster compilation (if ENABLE_UNITY_BUILD is ON)
if(ENABLE_UNITY_BUILD)
  set_target_properties(nav_filter_lib PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 16
  )
endif()

#ament_target_dependencies(nav_filter_lib rclcpp std_msgs ulisse_msgs surface_vehicle_model)
target_link_libraries(nav_filter_lib
  ${sensor_msgs_LIBRARIES}
  ${surface_vehicle_model_TARGETS}
  ${std_msgs_TARGETS}
  ${ulisse_msgs_TARGETS}
  ament_index_cpp::ament_index_cpp
  rclcpp::rclcpp
  GeographicLib
  rml
  fsm
  ctrl_toolbox
)

## Executables
add_executable(nav_filter_node src/nav_filter_node.cpp)
#ament_target_dependencies(nav_filter_node rclcpp std_msgs ulisse_msgs rml ctrl_toolbox ament_index_cpp)
target_link_libraries(nav_filter_node rml nav_filter_lib ctrl_toolbox config++)

#add_executable(nav_filter_udp_sender src/nav_filter_udp_sender.cpp)
#ament_target_dependencies(nav_filter_udp_sender rclcpp std_msgs ulisse_msgs rml ctrl_toolbox ament_index_cpp)
#target_link_libraries(nav_filter_udp_sender rml nav_filter_lib ctrl_toolbox config++)

ament_export_include_directories(include)

## Install
install(TARGETS
  nav_filter_node
#  nav_filter_udp_sender
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
