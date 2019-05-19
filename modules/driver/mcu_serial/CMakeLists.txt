cmake_minimum_required(VERSION 2.8.3)
project(mcu_serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  common
  data
  roscpp
  serial
  std_msgs
  actionlib

)
  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp 
  )

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(mcu_serial_test src/mcu_serial_test.cpp)
target_link_libraries(mcu_serial_test  ${catkin_LIBRARIES} )

add_executable(box_serial_node src/box_serial_node.cpp)
add_dependencies(box_serial_node rrts ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(box_serial_node  rrts ${catkin_LIBRARIES} )

# add_executable(mcu_serial_node src/mcu_serial_node.cpp )
# add_dependencies(mcu_serial_node rrts ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(mcu_serial_node rrts ${catkin_LIBRARIES} )
