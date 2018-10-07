#ifndef SHOP_PID_H__
#define SHOP_PID_H__

#include "ros/ros.h"
#include "ros/time.h"

// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <Shop_pid/ShopPIDConfig.h>

//pid_msg
#include "shop_msgs/PID.h"


class Shop_pid
{
    public:
    Shop_pid();

  void configCallback(Shop_pid::ShopPIDConfig &config, double level);
  void publishMessage(ros::Publisher *pub_message);
  void messageCallback(const shop_msgs::PID::ConstPtr &msg);

  double motor1_p,motor1_i,motor1_D;
  double motor2_p,motor2_i,motor2_D;
  double motor3_p,motor3_i,motor3_D;
  double motor4_p,motor4_i,motor4_D; 
  double pid_max,pid_min;
};



#endif //SHOP_PID_H__