cmake_minimum_required(VERSION 3.5)
project(ulisse_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ulisse_msgs REQUIRED)

# Add files in directory tree to "Projects" view in QtCreator
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
)

set(ulisse_driver_lib_sources
    src/ulisse_driver/CSerialHelper.cc
    src/ulisse_driver/EESHelper.cc
    src/ulisse_driver/EESHelperDataStructs.cc
)

## Libraries
add_library(ulisse_driver_lib SHARED ${ulisse_driver_lib_sources})
ament_target_dependencies(ulisse_driver_lib rclcpp std_msgs ulisse_msgs)
target_link_libraries(ulisse_driver_lib Geographic rml ctrl_toolbox)

add_library(driver_threads SHARED
            src/ulisse_driver/thread_sender.cpp
            src/ulisse_driver/thread_receiver.cpp)
target_compile_definitions(driver_threads
  PRIVATE "MINIMAL_COMPOSITION_DLL")
ament_target_dependencies(driver_threads rclcpp std_msgs class_loader)
target_link_libraries(driver_threads ulisse_driver_lib)

## Executables
add_executable(driver_node src/driver_node.cpp)
ament_target_dependencies(driver_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(driver_node rml ulisse_driver_lib driver_threads)

install(TARGETS
  driver_threads
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
