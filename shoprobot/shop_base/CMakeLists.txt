cmake_minimum_required(VERSION 2.8.3)
project(shop_base)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  shop_msgs
  std_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES shop_base
#  CATKIN_DEPENDS roscpp rospy rosserial_python sensor_msgsshop_msgs std_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(shop_base src/shop_base_node.cpp)
target_link_libraries(shop_base ${catkin_LIBRARIES})