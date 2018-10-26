cmake_minimum_required(VERSION 3.5)
project(ulisse_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fastrtps)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

rosidl_generate_interfaces(ulisse_msgs
  # Sensors
  "msg/IMUData.msg"
  "msg/AmbientSensors.msg"
  "msg/Compass.msg"
  "msg/Magnetometer.msg"
  "msg/MotorReference.msg"
  "msg/TimeInfo.msg"
  "msg/GPS.msg"
  # Commands
  "msg/CommandMove.msg"
  DEPENDENCIES builtin_interfaces
)

ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
