cmake_minimum_required(VERSION 3.5)
project(bags_to_csv)

# Default to C++17
#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 17)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${ulisse_msgs_INCLUDE_DIRS}
)

## Libraries
add_library(offline_bag_converter src/bags_to_csv/offline_bag_converter.cpp)
#ament_target_dependencies(offline_bag_converter rclcpp rosbag2_cpp rosbag2_storage ulisse_msgs)
target_link_libraries(offline_bag_converter
  ${sensor_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${bags_to_csv_LIBRARIES}
  ${ulisse_msgs_LIBRARIES}
  rclcpp::rclcpp
  rosbag2_cpp::rosbag2_cpp
)

## Executables
add_executable(to_csv_node src/to_csv_node.cpp)
#ament_target_dependencies(to_csv_node rclcpp std_msgs ulisse_msgs ctrl_toolbox ament_index_cpp)
target_link_libraries(to_csv_node ctrl_toolbox ${std_msgs_TARGETS} ${ulisse_msgs_TARGETS} rclcpp::rclcpp ament_index_cpp::ament_index_cpp)

add_executable(offline_bag2csv_node src/offline_bag2csv_node.cpp)
#ament_target_dependencies(offline_bag2csv_node rclcpp std_msgs ulisse_msgs rosbag2_cpp ament_index_cpp)
target_link_libraries(offline_bag2csv_node offline_bag_converter ${std_msgs_TARGETS} ${ulisse_msgs_TARGETS} ament_index_cpp::ament_index_cpp rclcpp::rclcpp)

ament_export_include_directories(include)

## Install
install(TARGETS
  to_csv_node
  offline_bag2csv_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
