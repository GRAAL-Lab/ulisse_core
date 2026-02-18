cmake_minimum_required(VERSION 3.5)
project(ulisse_map)

#if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 20)
#endif()

#if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#endif()

if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC -Wno-deprecated-copy")
endif()

# Enable ccache for faster rebuilds
find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
  set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
endif()

# Option to enable/disable unity builds (faster compilation)
option(ENABLE_UNITY_BUILD "Enable unity/jumbo builds for faster compilation" OFF)

# near the top of CMakeLists.txt
option(QT_QML_GENERATE_QMLLS_INI "Generate qmlls.ini for QML Language Server" ON)
# ensure the option is cached so GUI/CLI sees it:
set(QT_QML_GENERATE_QMLLS_INI ${QT_QML_GENERATE_QMLLS_INI} CACHE BOOL "Generate qmlls ini")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5QuickControls2 REQUIRED)
find_package(Qt5Location REQUIRED)
find_package(Qt5Positioning REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_driver REQUIRED)
find_package(ulisse_ctrl REQUIRED)
find_package(nav_filter REQUIRED)
find_package(sisl_toolbox REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} sisl)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include_directories(
    include
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${ament_index_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${ulisse_msgs_INCLUDE_DIRS}
    ${ulisse_ctrl_INCLUDE_DIRS}
    ${ulisse_driver_INCLUDE_DIRS}
    ${nav_filter_INCLUDE_DIRS}
    ${sisl_toolbox_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
)

# Add files in the tree of "Projects" view in QtCreator
file(GLOB_RECURSE ALL_FILES
    "msg/*.msg"
    "*.xml"
    "launch/*.py"
    "conf/*.yaml"
    "conf/*.conf"
    "include/*.hpp"
)
add_custom_target(.Files. SOURCES ${ALL_FILES})

file(GLOB SOURCE_FILES
        "src/*.cpp"
        "include/ulisse_map/*.hpp"
)

add_library(ulissemap_lib
    src/feedbackupdater.cpp
    src/taskdataupdater.cpp
    src/commandwrapper.cpp
    src/addonsbridge.cpp
)

# Enable unity builds for faster compilation (if ENABLE_UNITY_BUILD is ON)
if(ENABLE_UNITY_BUILD)
  set_target_properties(ulissemap_lib PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 8
  )
endif()

target_link_libraries(ulissemap_lib
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
)

# The header files must be included in the add_executable directive
# otherwise QT automoc won't parse the files and you'll get
# "undefined reference to vtable"
add_executable(${PROJECT_NAME}_node ${SOURCE_FILES} "src/qml.qrc" )

# Enable unity builds for faster compilation (Qt projects benefit especially, if ENABLE_UNITY_BUILD is ON)
if(ENABLE_UNITY_BUILD)
  set_target_properties(${PROJECT_NAME}_node PROPERTIES
    UNITY_BUILD ON
    UNITY_BUILD_BATCH_SIZE 8
  )
endif()

#ament_target_dependencies(${PROJECT_NAME}_node
#        rclcpp
#        ament_index_cpp
#        std_msgs
#        ulisse_msgs
#        rmw_implementation
#        nav_filter
#)
target_link_libraries(${PROJECT_NAME}_node
        Qt5::Core
        Qt5::Widgets
        Qt5::Quick
        Qt5::Location
        Qt5::Positioning
        Qt5::QuickControls2
        ulissemap_lib
        rclcpp::rclcpp
        ament_index_cpp::ament_index_cpp
        ${nav_filter_TARGETS}
        ${std_msgs_TARGETS}
        ${geometry_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${ulisse_msgs_TARGETS}
        ${JSONCPP_TARGETS}
        GeographicLib
        config++
        sisl_toolbox
        rml
)

#ament_export_include_directories(include)

## Install
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install GUI Icon
install(FILES
  src/images/ulisse_icon.png
  DESTINATION $ENV{HOME}/.local/share/icons
)

# Install GUI Desktop File
install(FILES
  src/scripts/ulisse_map.desktop
  DESTINATION $ENV{HOME}/.local/share/applications
)


ament_package()


