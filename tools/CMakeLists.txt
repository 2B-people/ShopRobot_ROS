cmake_minimum_required(VERSION 2.8.3)
project(tools)

find_package(catkin REQUIRED COMPONENTS
  data
  std_msgs
  dynamic_reconfigure
  roscpp
  rospy
  geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS 
)