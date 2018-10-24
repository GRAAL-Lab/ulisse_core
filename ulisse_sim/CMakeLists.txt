cmake_minimum_required(VERSION 3.5)
project(ulisse_sim)

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
find_package(Eigen3 REQUIRED)
find_package(surface_vehicle_model REQUIRED)
find_package(ulisse_msgs REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
#    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)

add_library(VehicleSimulatorClass src/ulisse_sim/vehiclesimulator.cpp)
ament_target_dependencies(VehicleSimulatorClass rclcpp ulisse_msgs)
target_link_libraries(VehicleSimulatorClass ${surface_vehicle_model_LIBRARIES} Geographic rml)

add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(simulator_node rclcpp std_msgs ulisse_msgs surface_vehicle_model)
target_link_libraries(simulator_node VehicleSimulatorClass)

install(TARGETS
  simulator_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
