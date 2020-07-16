cmake_minimum_required(VERSION 3.5)
project(ulisse_ctrl)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(surface_vehicle_model REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_driver REQUIRED)
find_package(rml REQUIRED)
find_package(fsm REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(tpik REQUIRED)
find_package(ikcl REQUIRED)
find_package(Boost REQUIRED)
find_package(ament_index_cpp REQUIRED)

link_directories(${Boost_LIBRARIES})

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} sisl)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_driver_INCLUDE_DIRS}
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)

set(use_SYSTEM_BOOST TRUE)

## Libraries ##
set(ulisse_ctrl_lib_sources
    # Main Classes
    src/ulisse_ctrl/vehiclecontroller.cpp
    # Helpers
    src/ulisse_ctrl/configuration.cpp
    src/ulisse_ctrl/nurbs.cpp
    # States
    src/ulisse_ctrl/states/genericstate.cpp
    src/ulisse_ctrl/states/state_halt.cpp
    src/ulisse_ctrl/states/state_latlong.cpp
    src/ulisse_ctrl/states/state_hold.cpp
    src/ulisse_ctrl/states/state_speedheading.cpp
    src/ulisse_ctrl/states/state_navigate.cpp
    # Commands
    src/ulisse_ctrl/commands/genericCommand.cpp
    src/ulisse_ctrl/commands/command_halt.cpp
    src/ulisse_ctrl/commands/command_hold.cpp
    src/ulisse_ctrl/commands/command_latlong.cpp
    src/ulisse_ctrl/commands/command_speedheading.cpp
    src/ulisse_ctrl/commands/command_navigate.cpp
    # Events
    src/ulisse_ctrl/events/generic_event.cpp
    src/ulisse_ctrl/events/event_rc_enabled.cpp
    src/ulisse_ctrl/events/event_near_goal_position.cpp

    # TASKS
    src/ulisse_ctrl/tasks/SafetyBoundaries.cpp


        )

## Libraries
add_library(ulisse_ctrl_lib ${ulisse_ctrl_lib_sources})
ament_target_dependencies(ulisse_ctrl_lib rclcpp std_msgs ulisse_msgs surface_vehicle_model ament_index_cpp)
target_link_libraries(ulisse_ctrl_lib ${surface_vehicle_model_LIBRARIES} ${ulisse_msgs_LIBRARIES} Geographic rml fsm ctrl_toolbox)

## Executables
add_executable(control_test test/control_test.cpp)
ament_target_dependencies(control_test rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(control_test rml ulisse_ctrl_lib ${ulisse_driver_LIBRARIES})

add_executable(kinematic_control_node src/kinematic_control_node.cpp)
ament_target_dependencies(kinematic_control_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(kinematic_control_node rml ulisse_ctrl_lib ${JSONCPP_LIBRARIES})

add_executable(dynamic_control_node src/dynamic_control_node.cpp)
ament_target_dependencies(dynamic_control_node rclcpp std_msgs ulisse_msgs rml ament_index_cpp ctrl_toolbox)
target_link_libraries(dynamic_control_node rml ulisse_ctrl_lib )

#add_executable(navigation_filter_node src/navigation_filter_node.cpp)
#ament_target_dependencies(navigation_filter_node rclcpp std_msgs ulisse_msgs rml)
#target_link_libraries(navigation_filter_node rml ulisse_ctrl_lib)

add_executable(controller_console_node src/controller_console_node.cpp)
ament_target_dependencies(controller_console_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(controller_console_node rml ulisse_ctrl_lib)

add_executable(monitor_node src/monitor_node.cpp)
ament_target_dependencies(monitor_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(monitor_node rml ulisse_ctrl_lib)

target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ikcl tpik rml config++ )
ament_export_include_directories(include)

## Install
install(TARGETS
  kinematic_control_node
  dynamic_control_node
#  navigation_filter_node
  controller_console_node
  monitor_node
  control_test
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
