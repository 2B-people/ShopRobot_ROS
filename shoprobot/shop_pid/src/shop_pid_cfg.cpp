#include <ros/ros.h>
#include <ros/time.h>

#include "shop_msgs/PID.h"

#include <dynamic_reconfigure/server.h>
#include <shop_pid/shopPIDConfig.h>

double p, i, d;

void Callback(shop_pid::shopPIDConfig &config, int level)
{
    p = config.p;
    i = config.i;
    d = config.d;

    //echo P,I,D
    ROS_INFO("NEW set!");
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "shop_pid");

    ROS_INFO("Please use rqt_reconfigure to change pid");

    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    int rate;
    pn.param("p", p, 3.0);
    pn.param("i", i, 0.3);
    pn.param("d", d, 0.3);
    pn.param("rate", rate, 10);

    dynamic_reconfigure::Server<shop_pid::shopPIDConfig> server;
    dynamic_reconfigure::Server<shop_pid::shopPIDConfig>::CallbackType cb;
    cb = boost::bind(&Callback, _1, _2);
    server.setCallback(cb);

    shop_msgs::PID msg;

    ros::Publisher pub = nh.advertise<shop_msgs::PID>("pid", 10);

    ros::Rate r(rate);

    while (nh.ok())
    {
        msg.p = p;
        msg.i = i;
        msg.d = d;

        pub.publish(msg);

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
