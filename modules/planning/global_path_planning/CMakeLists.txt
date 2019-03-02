cmake_minimum_required(VERSION 2.8.3)
project(global_path_planning)

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
)

catkin_package()

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

