cmake_minimum_required(VERSION 3.5)
project(ulisse_map)

set (CMAKE_CXX_STANDARD 17)
if(NOT WIN32)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -Wall -Wextra -fPIC -Wno-deprecated-copy")
endif()

SET(CMAKE_BUILD_TYPE "Release")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5QuickControls2 REQUIRED)
find_package(Qt5Location REQUIRED)
find_package(Qt5Positioning REQUIRED)
find_package(ulisse_msgs REQUIRED)
find_package(ulisse_driver REQUIRED)
find_package(ulisse_ctrl REQUIRED)
find_package(nav_filter REQUIRED)
find_package(sisl_toolbox REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP jsoncpp)
link_libraries(${JSONCPP_LIBRARIES} sisl)


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include_directories(
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${ament_index_INCLUDE_DIRS}
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Quick_INCLUDE_DIRS}
    ${ulisse_msgs_INCLUDE_DIRS}
    ${ulisse_ctrl_INCLUDE_DIRS}
    ${ulisse_driver_INCLUDE_DIRS}
    ${nav_filter_INCLUDE_DIRS}
    ${sisl_toolbox_INCLUDE_DIRS}
)

file(GLOB SOURCE_FILES
        "src/*.cpp"
        )

add_library(ulissemap_lib
    src/feedbackupdater.cpp
    src/taskdataupdater.cpp
    src/commandwrapper.cpp
    src/addonsbridge.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES} "src/qml.qrc")
ament_target_dependencies(${PROJECT_NAME}_node
        rclcpp
        ament_index_cpp
        std_msgs
        ulisse_msgs
        rmw_implementation
        nav_filter
)
target_link_libraries(${PROJECT_NAME}_node
        Qt5::Core
        Qt5::Quick
        Qt5::Location
        Qt5::Positioning
        Qt5::QuickControls2
        ulissemap_lib
        ${ulisse_msgs_LIBRARIES}
        ${JSONCPP_LIBRARIES}
        Geographic
        config++
        sisl_toolbox
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


