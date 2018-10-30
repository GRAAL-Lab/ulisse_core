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
find_package(ulisse_msgs REQUIRED)
find_package(rml REQUIRED)
find_package(fsm REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(surface_vehicle_model REQUIRED)


# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)


## Libraries ##
set(ulisse_ctrl_lib_sources
    # Main Class
    src/ulisse_ctrl/vehiclecontroller.cpp
    # Helpers
    src/ulisse_ctrl/helper_functions.cpp
    # States
    src/ulisse_ctrl/states/genericstate.cpp
    src/ulisse_ctrl/states/statehalt.cpp
    src/ulisse_ctrl/states/statemove.cpp
    # Commands
    src/ulisse_ctrl/commands/genericcommand.cpp
    src/ulisse_ctrl/commands/commandhalt.cpp
    src/ulisse_ctrl/commands/commandmove.cpp    
)

## Libraries
add_library(ulisse_ctrl_lib ${ulisse_ctrl_lib_sources})
ament_target_dependencies(ulisse_ctrl_lib rclcpp std_msgs ulisse_msgs)
target_link_libraries(ulisse_ctrl_lib ${surface_vehicle_model_LIBRARIES} Geographic rml fsm ctrl_toolbox)


## Executables
add_executable(publisher src/publisher_test.cpp)
ament_target_dependencies(publisher rclcpp std_msgs ulisse_msgs)

add_executable(subscriber src/subscriber_test.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(subscriber rml)

add_executable(controller_node src/controller_node.cpp)
ament_target_dependencies(controller_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(controller_node rml ulisse_ctrl_lib)

## Install
install(TARGETS
  publisher
  subscriber
  controller_node
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
