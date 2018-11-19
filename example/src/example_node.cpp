#include "example/example_node.h"
#include "common/main_interface.h"

namespace example{
MainTest::MainTest(std::string name):RRTS::RRTS(name)
{
    ros::NodeHandle nh;
    sub_=nh.subscribe<geometry_msgs::PoseStamped>("test_topic",10, boost::bind(&MainTest::CB,this,_1));
    ROS_INFO("node go");
}

void MainTest::CB(const geometry_msgs::PoseStamped::ConstPtr & pose){
    ROS_INFO(" Start!");
}

}

MAIN(example::MainTest,"test_node");

