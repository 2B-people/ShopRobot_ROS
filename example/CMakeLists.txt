cmake_minimum_required(VERSION 2.8.3)
project(example)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  common
  data
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
  actionlib
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp 
)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})
###########
## Build ##
###########
add_executable(example_node src/example_node.cpp )
add_dependencies(example_node rrts ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node rrts ${catkin_LIBRARIES} )

add_executable(example_action_server src/example_action_server.cpp)
add_dependencies(example_action_server  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_server  ${catkin_LIBRARIES} )

add_executable(example_action_clint src/example_action_clint.cpp)
add_dependencies(example_action_clint  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_action_clint  ${catkin_LIBRARIES} )

# #############
# ## Install ##
# #############
# # executables and libraries
# install(TARGETS example_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# # header files
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

#############
## Testing ##
#############

