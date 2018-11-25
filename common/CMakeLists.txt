cmake_minimum_required(VERSION 2.8.3)
project(common)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rrts
  CATKIN_DEPENDS roscpp 
)

include_directories(include)
include_directories(  
  ${catkin_INCLUDE_DIRS}
)

# rrts library
add_library(rrts src/rrts.cpp
)
target_link_libraries(rrts
  ${catkin_LIBRARIES}
)




