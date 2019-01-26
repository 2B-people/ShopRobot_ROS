cmake_minimum_required(VERSION 2.8.3)
project(decision)

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
)

catkin_package()

target_sources(behavior_tree
  INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_node.h
  ${CMAKE_CURRENT_SOURCE_DIR}/behavior_tree.h
  ${CMAKE_CURRENT_SOURCE_DIR}/black_board.h)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(decision_test src/decision_test.cpp)
target_link_libraries(decision_test  ${catkin_LIBRARIES} )

