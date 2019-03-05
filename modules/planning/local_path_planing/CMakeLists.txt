cmake_minimum_required(VERSION 2.8.3)
project(local_path_planing)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  common
  data
  roscpp
  std_msgs
  actionlib
)

catkin_package()

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(local_path_planing src/local_plan_node.cpp)
add_dependencies(local_path_planing rrts  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_path_planing rrts  ${catkin_LIBRARIES} )


