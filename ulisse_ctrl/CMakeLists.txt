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

#message(WARNING "\nrml libraries: ${rml_LIBRARIES}")


# Add files in directory tree to "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
)
add_custom_target(.Files. SOURCES ${SOURCE_FILES})

include_directories(
  include
  ${ulisse_msgs_INCLUDE_DIRS}
)

add_executable(publisher src/publisher_test.cpp)
ament_target_dependencies(publisher rclcpp std_msgs ulisse_msgs)

add_executable(subscriber src/subscriber_test.cpp)
ament_target_dependencies(subscriber rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(subscriber rml)


install(TARGETS
  publisher
  subscriber
  DESTINATION lib/${PROJECT_NAME})
  
#install(DIRECTORY
#  launch
#  DESTINATION share/${PROJECT_NAME}/
#)

ament_package()
