#include "example/example_node.h"
#include "common/main_interface.h"

namespace example{
    
MainTest::MainTest(std::string name) : common::RRTS(name)
{
    ros::NodeHandle nh;
    sub_ = nh.subscribe<geometry_msgs::PoseStamped>("test_topic", 10, boost::bind(&MainTest::CB, this, _1));
    ROS_INFO("node go");
}

MainTest::~MainTest(){
    ROS_ERROR("aaaa");
}

void MainTest::CB(const geometry_msgs::PoseStamped::ConstPtr &pose)
{
    ROS_INFO(" Start!");
}

} // namespace example

MAIN(example::MainTest,"test_node")
