cmake_minimum_required(VERSION 2.8.3)
project(velodyne_style_repub)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  std_msgs
  sensor_msgs
  livox_ros_driver
)

catkin_package(
  INCLUDE_DIRS
  DEPENDS

  CATKIN_DEPENDS 
  std_msgs
  sensor_msgs
  livox_ros_driver
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

# link directories
link_directories(
	include
)

###########
## Build ##
###########

add_executable(${PROJECT_NAME} src/livox_repub.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
