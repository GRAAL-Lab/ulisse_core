cmake_minimum_required(VERSION 3.5)
project(ulisse_catl)

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

find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ctrl_toolbox REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS} ${MOSQUITTOPP_INCLUDE_DIR})

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "logs/*.log"
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
  ${surface_vehicle_model_INCLUDE_DIRS}
)

## Libraries

## Executables
add_executable(ulisse_catl src/publishToMqtt.cpp)
ament_target_dependencies(ulisse_catl rclcpp std_msgs ament_index_cpp ulisse_msgs)
target_link_libraries(ulisse_catl config++ mqttt stng json_utils ${OpenCV_LIBS} stdc++fs)

target_include_directories(ulisse_catl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS
  ulisse_catl
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
#  logs
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
