cmake_minimum_required(VERSION 3.8)
project(ulisse_avoidance)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_ctrl REQUIRED)

find_package(rclcpp REQUIRED)
find_package(ctrl_toolbox REQUIRED)
find_package(sisl_toolbox REQUIRED)

find_package(ament_index_cpp)

find_package(detav_msgs REQUIRED)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE SOURCE_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.ProjectFiles. SOURCES ${SOURCE_FILES})

include_directories(
        include
        ${rclcpp_INCLUDE_DIRS}
        ${ulisse_msgs_INCLUDE_DIRS}
        ${detav_msgs_INCLUDE_DIRS}
)

add_executable(oal_interface src/planner_node.cpp src/ulisse_avoidance/local_planner.cpp src/ulisse_avoidance/plan.cpp)
ament_target_dependencies(
        oal_interface
        ament_index_cpp
        rclcpp
        ulisse_msgs
        detav_msgs
        ulisse_ctrl
        ctrl_toolbox
        
        
)
target_link_libraries(oal_interface Geographic oal config++ sisl_toolbox)
target_include_directories(oal_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(oal_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# Install target
install(TARGETS oal_interface DESTINATION lib/${PROJECT_NAME})




# Export Include Directories
ament_export_include_directories(include)

# Install Include Directory
install(
  DIRECTORY include/
  DESTINATION include
)


install(DIRECTORY conf
  DESTINATION share/${PROJECT_NAME}/
)

# Testing Configuration
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_dependencies(ament_cmake)
ament_package()

