cmake_minimum_required(VERSION 2.8.3)
project(data)
find_package(catkin REQUIRED COMPONENTS 
                     std_msgs
                     nav_msgs 
                     sensor_msgs 
                     std_srvs
                     actionlib 
                     actionlib_msgs 
                     message_generation
                     )

add_message_files(DIRECTORY msg
                      FILES
                      Pid.msg
                      Point.msg
                      SerialTest.msg
                      Vector3.msg
                      Remote.msg
                      CarScan.msg
                      Coord.msg
                      Action.msg
                      Cargo.msg
                      Barrier.msg
                      RoadblockMsg.msg
                      Photo.msg
                 )

add_service_files(FILES
                      Goods.srv
                      ActionName.srv
                      SetBool.srv                      
                      Roadblock.srv
                      Coordinate.srv
                      ShelfBarrier.srv
                      PhotoSrv.srv
                 )

add_action_files(FILES
                      Test.action
                      ShopAction.action
                      Move.action
                      Opening.action
                      Camera.action
                      Detection.action
                      LocalPlan.action
                      GlobalPlan.action
)

generate_messages(DEPENDENCIES std_msgs actionlib_msgs std_srvs nav_msgs sensor_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs nav_msgs sensor_msgs actionlib_msgs std_srvs)