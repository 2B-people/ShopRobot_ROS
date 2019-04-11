cmake_minimum_required(VERSION 2.8.3)
project(global_path_planing)

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
  actionlib
)

catkin_package()

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(global_path_planing src/global_path_planning_node.cpp)
add_dependencies(global_path_planing rrts  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_path_planing rrts  ${catkin_LIBRARIES} )