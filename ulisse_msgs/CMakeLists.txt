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
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
#find_package(fastrtps)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "srv/*.srv"
    "*.xml"
    "launch/*.py"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

rosidl_generate_interfaces(ulisse_msgs
  # Sensors
  "msg/Time.msg"
  "msg/LatLong.msg"
  "msg/PIDData.msg"
  "msg/PositionContext.msg"
  "msg/ControlContext.msg"
  "msg/IMUData.msg"
  "msg/AmbientSensors.msg"
  "msg/Compass.msg"
  "msg/Magnetometer.msg"
  "msg/MotorReference.msg"
  "msg/MicroLoopCount.msg"
  "msg/GPSData.msg"
  "msg/GPSStatus.msg"
  # Commands
  "msg/CommandLatLong.msg"
  "msg/CommandSpeedHeading.msg"
  # EES
  "msg/EESAck.msg"
  #"msg/BatteryData.msg"
  "msg/EESBattery.msg"
  "msg/Sw485InOut.msg"
  "msg/EESSw485Status.msg"
  "msg/EESVersion.msg"
  "msg/MotorData.msg"
  "msg/EESMotors.msg"
  "msg/EESStatus.msg"
  "msg/EESBeep.msg"
  "msg/EESConfig.msg"
  "msg/EESEnableRef.msg"
  "msg/EESPowerButtons.msg"
  "msg/EESPumps.msg"
  # Services
  "srv/ControlCommand.srv"
  "srv/EESCommand.srv"
  "srv/RosbagCmd.srv"

)

ament_export_include_directories(include)

ament_export_dependencies(rosidl_default_runtime)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
