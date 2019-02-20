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
  actionlib

)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

add_executable(web_server_test src/web_server_test.cpp)
target_link_libraries(web_server_test  ${catkin_LIBRARIES} )

add_library(web_server src/web_server_class.cpp)
target_link_libraries(web_server  ${catkin_LIBRARIES})

add_executable(robot1_web_server src/robot1_web_server.cpp)
add_dependencies(robot1_web_server rrts web_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot1_web_server rrts web_server  ${catkin_LIBRARIES} )

add_executable(robot2_web_server src/robot2_web_server.cpp)
add_dependencies(robot2_web_server rrts web_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot2_web_server rrts web_server  ${catkin_LIBRARIES} )

add_executable(robot3_web_server src/robot3_web_server.cpp)
add_dependencies(robot3_web_server rrts web_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot3_web_server rrts web_server  ${catkin_LIBRARIES} )

add_executable(robot4_web_server src/robot4_web_server.cpp)
add_dependencies(robot4_web_server rrts web_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot4_web_server rrts web_server  ${catkin_LIBRARIES} )



