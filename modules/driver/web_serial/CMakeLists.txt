cmake_minimum_required(VERSION 2.8.3)
project(web_serial)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  roscpp
  rospy
  std_msgs
)

