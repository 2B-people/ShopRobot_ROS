#include "shop_pid/shop_pid.h"

shop_pid::shop_pid()
{

}

void Shop_pid::configCallback(Shop_pid::shop_pid_config &config, double level)
{
    //for PID GUI
    p = config.p;
    i = config.i;
    d = config.d;
}

void Shop_pid::publishMessage(ros::Publisher *pub_message)
{
    shop_msgs::PID msg;
    msg.p = p;
    msg.i = i;
    msg.d = D;

    pub_message->publish(msg);
}

void Shop_pid::messageCallback(const shop_msgs::PID::ConstPtr &msg)
{
    p = msg->p;
    i = msg->i;
    d = msg->d;

    //echo P,I,D
    ROS_INFO("P: %f", p);
    ROS_INFO("I: %f", i);
    ROS_INFO("D: %f", d);
    
}
