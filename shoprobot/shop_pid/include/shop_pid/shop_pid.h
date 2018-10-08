#ifndef SHOP_PID_H__
#define SHOP_PID_H__

#include "ros/ros.h"
#include "ros/time.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <Shop_pid/shop_pid_config.h>

//pid_msg
#include "shop_msgs/PID.h"

class shop_pid
{
  public:
    shop_pid();

    void configCallback(Shop_pid::shop_pid_config &config, double level);
    void publishMessage(ros::Publisher *pub_message);
    void messageCallback(const shop_msgs::PID::ConstPtr &msg);

    double p,i,d;
    double pid_max,pid_min;
};



#endif //SHOP_PID_H__