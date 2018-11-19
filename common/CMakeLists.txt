cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES motordriver
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# rrts library
add_library(rrts
  src/rrts.cpp
)
target_link_libraries(rrts
  ${YAML_LIBRARIES}
)