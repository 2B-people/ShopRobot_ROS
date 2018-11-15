cmake_minimum_required(VERSION 2.8.3)
project(pid_config)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  message_generation
  roscpp
  data
  std_msgs
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)


catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure message_runtime roscpp std_msgs
)
###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(riki_base_node src/riki_base_node.cpp)
target_link_libraries(riki_base_node ${catkin_LIBRARIES})
