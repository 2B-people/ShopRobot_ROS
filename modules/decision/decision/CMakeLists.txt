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
  blackboard
  actionlib
)

catkin_package()

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})

# add_executable(decision_test src/decision_test.cpp)
# add_dependencies(decision_test  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(decision_test PRIVATE  ${catkin_LIBRARIES})

add_executable(decision src/decision.cpp)
add_dependencies(decision  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision PRIVATE  ${catkin_LIBRARIES})

add_executable(debug_decision src/debug_decision.cpp)
add_dependencies(debug_decision  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(debug_decision PRIVATE  ${catkin_LIBRARIES})

add_executable(decision_test src/decision_test.cpp)
add_dependencies(decision_test  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_test PRIVATE  ${catkin_LIBRARIES})

