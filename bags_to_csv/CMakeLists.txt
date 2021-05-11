cmake_minimum_required(VERSION 3.5)
project(bags_to_csv)

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
find_package(ament_index_cpp REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ctrl_toolbox REQUIRED)

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
)

## Libraries
add_library(OfflineBagConverter src/bags_to_csv/offline_bag_converter.cpp)
ament_target_dependencies(OfflineBagConverter rclcpp)
target_link_libraries(OfflineBagConverter ${bags_to_csv_LIBRARIES})

## Executables
add_executable(to_csv_node src/to_csv_node.cpp)
ament_target_dependencies(to_csv_node rclcpp std_msgs ulisse_msgs ctrl_toolbox ament_index_cpp)
target_link_libraries(to_csv_node ctrl_toolbox  ${ulisse_msgs_LIBRARIES})

ament_export_include_directories(include)

## Install
install(TARGETS
  to_csv_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
