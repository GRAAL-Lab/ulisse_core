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
find_package(oal REQUIRED)

include_directories(
        include
        ${ulisse_msgs_INCLUDE_DIRS}
)

add_executable(oal_interface src/oal_interface.cpp)
target_include_directories(oal_interface PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(oal_interface PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        oal_interface
        "rclcpp"
        "ulisse_msgs"
        "ulisse_ctrl"
        "ctrl_toolbox"
        "oal"
)
target_link_libraries(oal_interface Geographic oal config++)

add_executable(weird_test src/test.cpp)
target_include_directories(weird_test PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_compile_features(weird_test PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
        weird_test
        "rclcpp"
        "ulisse_msgs"
        "ulisse_ctrl"
        "ctrl_toolbox"
        "oal"
)
target_link_libraries(weird_test Geographic oal config++)

install(TARGETS oal_interface weird_test
        DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
        conf
        launch
        DESTINATION share/${PROJECT_NAME}
        )


ament_export_dependencies(ament_cmake)
ament_package()
