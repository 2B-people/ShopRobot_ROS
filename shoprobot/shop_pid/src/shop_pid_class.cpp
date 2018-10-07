#include "shop_pid/shop_pid.h"

Shop_pid::shop_pid()
{

}

void Shop_pid::configCallback(Shop_pid::ShopPIDConfig &config, double level)
{
    //for PID GUI
    motor1_p = config.p1;
    motor1_i = config.i1;
    motor1_d = config.d1;
    motor2_p = config.p2;
    motor2_i = config.i2;
    motor2_d = config.d2;    
    motor3_p = config.p3;
    motor3_i = config.i3;
    motor3_d = config.d3;    
    motor4_p = config.p4;
    motor4_i = config.i4;
    motor4_d = config.d4;
    pid_max = config.max;
    pid_max = config.min;
}

void Shop_pid::publishMessage(ros::Publisher *pub_message)
{
    shop_msgs::PID msg;
    msg.p1 = motor1_p;
    msg.i1 = motor1_i;
    msg.d1 = motor1_D;
    msg.p2 = motor2_p;
    msg.i2 = motor2_i;
    msg.d2 = motor2_D;
    msg.p3 = motor3_p;
    msg.i3 = motor3_i;
    msg.d3 = motor3_D;
    msg.p4 = motor4_p;
    msg.i4 = motor4_i;
    msg.d4 = motor4_D;
    msg.min = pid_min;
    msg.max = pid_max;
    pub_message->publish(msg);
}

void Shop_pid::messageCallback(const shop_msgs::PID::ConstPtr &msg)
{
    motor1_p = msg->p1;
    motor1_i = msg->i1;
    motor1_d = msg->d1;
    motor2_p = msg->p2;
    motor2_i = msg->i2;
    motor2_d = msg->d2;    
    motor3_p = msg->p3;
    motor3_i = msg->i3;
    motor3_d = msg->d3;    
    motor4_p = msg->p4;
    motor4_i = msg->i4;
    motor4_d = msg->d4;

    pid_max = msg->max;
    pid_min = msg->min;

    //echo P,I,D
    ROS_INFO("P1: %f", motor1_p);
    ROS_INFO("I1: %f", motor1_i);
    ROS_INFO("D1: %f", motor1_d);

    ROS_INFO("P1: %f", motor2_p);
    ROS_INFO("I1: %f", motor2_i);
    ROS_INFO("D1: %f", motor2_d);

    ROS_INFO("P1: %f", motor3_p);
    ROS_INFO("I1: %f", motor3_i);
    ROS_INFO("D1: %f", motor3_d);

    ROS_INFO("P1: %f", motor3_p);
    ROS_INFO("I1: %f", motor3_i);
    ROS_INFO("D1: %f", motor3_d);

}
