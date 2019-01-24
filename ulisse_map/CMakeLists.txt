cmake_minimum_required(VERSION 3.5)
project(ulisse_map)

set (CMAKE_CXX_STANDARD 14)
if(NOT WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -fPIC")
endif()

IF (NOT DEFINED BUILD_VERSION)
	SET(BUILD_VERSION "not set")
ENDIF()
ADD_DEFINITIONS(-DBUILD_VERSION="${BUILD_VERSION}")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Quick REQUIRED)
find_package(Qt5Location REQUIRED)
find_package(Qt5Positioning REQUIRED)
find_package(ulisse_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


include_directories(
	${rclcpp_INCLUDE_DIRS}
	${std_msgs_INCLUDE_DIRS}
	${Qt5Core_INCLUDE_DIRS}
	${Qt5Quick_INCLUDE_DIRS}
    ${ulisse_msgs_INCLUDE_DIRS}
	)

file(GLOB SOURCE_FILES
	"src/*.cpp"
	)
	
add_library(ulissemap_lib
    src/feedbackupdater.cpp
)

add_executable(${PROJECT_NAME}_node ${SOURCE_FILES} "src/qml.qrc")

ament_target_dependencies(${PROJECT_NAME}_node
	rclcpp
	std_msgs
    ulisse_msgs
	rmw_implementation
	)
target_link_libraries(${PROJECT_NAME}_node
	Qt5::Core
	Qt5::Quick
    Qt5::Location
    Qt5::Positioning
	ulissemap_lib
	)

## Install
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
