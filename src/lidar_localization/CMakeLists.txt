cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE )
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msg
  std_msgs
  pcl_ros
  tf
  geometry_msgs
  eigen_conversions
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_localization
#  CATKIN_DEPENDS roscpp rospy sensor_msg std_msgs
#  DEPENDS system_lib
)


