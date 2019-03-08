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
    src/ulisse_driver/LLCHelper.cc
    src/ulisse_driver/LLCHelperDataStructs.cc
)


## Libraries
add_library(ulisse_driver_lib SHARED ${ulisse_driver_lib_sources})
ament_target_dependencies(ulisse_driver_lib rclcpp std_msgs ulisse_msgs)
target_link_libraries(ulisse_driver_lib Geographic rml ctrl_toolbox)

add_library(llc_driver_threads SHARED
            src/ulisse_driver/thread_sender.cpp
            src/ulisse_driver/thread_receiver.cpp)
target_compile_definitions(llc_driver_threads
  PRIVATE "THREADS_COMPOSITION_DLL")
ament_target_dependencies(llc_driver_threads rclcpp std_msgs class_loader)
target_link_libraries(llc_driver_threads ulisse_driver_lib)

## Executables
add_executable(llc_driver_node src/llc_driver_node.cpp)
ament_target_dependencies(llc_driver_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(llc_driver_node rml ulisse_driver_lib llc_driver_threads)

add_executable(gps_setup_node src/gps_setup_node.cpp)
ament_target_dependencies(gps_setup_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(gps_setup_node ulisse_driver_lib gps)

add_executable(gps_publisher_node src/gps_publisher_node.cpp)
ament_target_dependencies(gps_publisher_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(gps_publisher_node ulisse_driver_lib gps)

add_executable(driver_console_node src/driver_console_node.cpp)
ament_target_dependencies(driver_console_node rclcpp std_msgs ulisse_msgs rml)
target_link_libraries(driver_console_node rml ulisse_driver_lib llc_driver_threads)

install(TARGETS
  ulisse_driver_lib
  ARCHIVE DESTINATION lib#/${PROJECT_NAME}
  LIBRARY DESTINATION lib#/${PROJECT_NAME}
  RUNTIME DESTINATION bin#/${PROJECT_NAME}
)

install(TARGETS
  llc_driver_node
  llc_driver_threads
  gps_setup_node
  gps_publisher_node
  driver_console_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  conf
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(ulisse_driver_lib)
ament_package()
