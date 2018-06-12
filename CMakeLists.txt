cmake_minimum_required(VERSION 2.8.3)
project(laserscan_stacker)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_filters
  laser_geometry
  my_new_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  pcl_ros
)

find_package(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(laserscan_stacker src/laserscan_stacker.cpp)
target_link_libraries(laserscan_stacker ${catkin_LIBRARIES})
