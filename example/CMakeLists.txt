cmake_minimum_required(VERSION 2.8.3)
project(example)


set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  common
  pluginlib
  roscpp
  std_msgs
  geometry_msgs
)

## System dependencies are found with CMake's conventions
#find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES example_node
# CATKIN_DEPENDS     roscpp 
#  DEPENDS system_lib
)

link_directories(common)

include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS})
###########
## Build ##
###########
add_executable(example_node src/example_node.cpp )
add_dependencies(example_node common ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(example_node common ${catkin_LIBRARIES} )


#############
## Install ##
#############
# executables and libraries
install(TARGETS example_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

