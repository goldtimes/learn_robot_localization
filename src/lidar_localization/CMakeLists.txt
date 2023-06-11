cmake_minimum_required(VERSION 3.0.2)
project(lidar_localization)

SET(CMAKE_BUILD_TYPE "Release")
# SET(CMAKE_CXX_FLAGS_RELEASE )
add_compile_options(-std=c++11)
add_definitions(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
  tf
  geometry_msgs
  eigen_conversions
)
# find_package(eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# 定义变量 来保存lib
set(ALL_TARGET_LIBRARIES "")

# include project's cmake dir all cmake file
include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/geographic.cmake)
include(cmake/global_defination.cmake)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES lidar_localization
#  CATKIN_DEPENDS roscpp rospy sensor_msg std_msgs
#  DEPENDS system_lib
)
# 遍历所有的cc文件
file(GLOB_RECURSE ALL_SRCS "*.cc")
# 遍历src *_node.cc 
file(GLOB_RECURSE NODE_SRCS "src/*_node.cc")
# 遍历 third_party
file(GLOB_RECURSE THIRD_PARTY_SRCS "third_party/*.cpp")
# 删除*_node.cc 
list(REMOVE_ITEM ALL_SRCS ${NODE_SRCS})
# 删除 third_party .cc文件
list(REMOVE_ITEM ALL_SRCS ${THIRD_PARTY_SRCS})

add_executable(test_frame_node src/test_frame_node.cc ${ALL_SRCS})
target_link_libraries(test_frame_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

add_executable(front_end_node src/front_end_node.cc ${ALL_SRCS})
target_link_libraries(front_end_node ${catkin_LIBRARIES} ${ALL_TARGET_LIBRARIES})

install(TARGETS front_end_node 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS test_frame_node 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch rviz include cmake 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
