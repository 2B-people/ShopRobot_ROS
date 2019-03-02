cmake_minimum_required(VERSION 2.8.3)
project(distinguish_learn)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  data
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

