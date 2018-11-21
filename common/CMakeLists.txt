cmake_minimum_required(VERSION 2.8.3)
project(common)

find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES common
#  CATKIN_DEPENDS roscpp 
)

include_directories(include)
include_directories(  
  ${catkin_INCLUDE_DIRS}
)

# rrts library
add_library(common src/rrts.cpp
)
target_link_libraries(common
  ${catkin_LIBRARIES}
)




