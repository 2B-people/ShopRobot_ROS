cmake_minimum_required(VERSION 2.8.3)
project(data)
find_package(catkin REQUIRED COMPONENTS std_msgs nav_msgs sensor_msgs std_srvs actionlib_msgs message_generation)

add_message_files(DIRECTORY msg
                      FILES
                      Pid.msg
                      Point.msg
                      SerialTest.msg
                      Vector3.msg
                      Remote.msg
                      CarScan.msg
                      CoordinateMsg.msg
                 )

add_service_files(FILES
                      Goods.srv
                      SetBool.srv                      
                      Roadblock.srv
                      Coordinate.srv
                 )

generate_messages(DEPENDENCIES std_msgs actionlib_msgs std_srvs nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs sensor_msgs actionlib_msgs std_srvs)