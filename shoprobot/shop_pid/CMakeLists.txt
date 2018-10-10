cmake_minimum_required(VERSION 2.8.3)
project(shop_pid)

find_package(catkin REQUIRED COMPONENTS dynamic_reconfigure message_generation roscpp std_msgs)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/shopPID.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

set (BINNAME1 shop_pid_cfg)

set (SRCS1 ${SRCS1} src/shop_pid_cfg.cpp)

# Build the executable that will be used to run this node.
add_executable (${BINNAME1} ${SRCS1})
target_link_libraries(${BINNAME1} ${catkin_LIBRARIES})
add_dependencies(${BINNAME1} ${PROJECT_NAME}_gencfg)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS ${BINNAME1}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)