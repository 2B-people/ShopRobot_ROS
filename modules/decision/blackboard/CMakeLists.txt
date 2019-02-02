cmake_minimum_required(VERSION 2.8.3)
project(blackboard)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common
  data
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrts
  CATKIN_DEPENDS roscpp 
)

include_directories(include)
include_directories(  
  ${catkin_INCLUDE_DIRS}
)
