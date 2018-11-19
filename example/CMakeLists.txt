cmake_minimum_required(VERSION 2.8.3)
project(example)

find_package(catkin REQUIRED COMPONENTS
  common
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

include_directories(include ${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES example_node
# CATKIN_DEPENDS     roscpp rospy 
#  DEPENDS system_lib
)
###########
## Build ##
###########
add_executable(example_node src/example_node.cpp )
target_link_libraries(example_node ${catkin_LIBRARIES} )


#############
## Install ##
#############


#############
## Testing ##
#############

