cmake_minimum_required(VERSION 3.5)
project(ulisse_sim)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(ament_index_cpp REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "logs/*.log"
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
    "meshes/*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)

## Libraries
add_library(VehicleSimulatorClass src/ulisse_sim/vehicle_simulator.cpp)
ament_target_dependencies(VehicleSimulatorClass rclcpp ulisse_msgs geometry_msgs tf2 tf2_ros ament_index_cpp visualization_msgs)
target_link_libraries(VehicleSimulatorClass ${surface_vehicle_model_LIBRARIES} Geographic rml)

## Executables
add_executable(simulator_node src/simulator_node.cpp)
ament_target_dependencies(simulator_node rclcpp std_msgs ulisse_msgs surface_vehicle_model ament_index_cpp)
target_link_libraries(simulator_node VehicleSimulatorClass config++)

add_executable(sim_from_log_node src/sim_from_log_node.cpp)
ament_target_dependencies(sim_from_log_node rclcpp std_msgs ulisse_msgs ctrl_toolbox ament_index_cpp)
target_link_libraries(sim_from_log_node Geographic config++ )

target_include_directories(sim_from_log_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


install(TARGETS
  simulator_node
  sim_from_log_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
#  logs
  meshes
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
