cmake_minimum_required(VERSION 2.8.3)
project(web_serial)

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

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(web_server_test src/web_server_test.cpp)
target_link_libraries(web_server_test  ${catkin_LIBRARIES} )


