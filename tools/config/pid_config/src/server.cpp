#include <ros/ros.h>

#include <dynameic_reconfigure/server.h>
#include <shop_pid/shopPIDdConfig.h>

#include "pid_config/pid_lib.h"

tools::config::MoveMotorPid MovePid;
tools::config::LiftMotorPid LiftPid;

void callback(pid_config::PIDConfig &config, double level)
{
    
    switch (config.equipment)
    {
        case 0:
            MovePid.setPid(config.p,config.i,config.d,config.test);
            break;
        case 1:
            LiftPid.setPid(config.p,config.i,config.d,config.test);
            break;    
        default:
            break;
    }
}

int main(int argc, char const *argv[])
{
    
    ros::init(argc, agrv, "shop_pid");
    
    dynamic_reconfigure::Server<pid_config::PIDConfig> server;
    dynamic_reconfigure::Server<pid_config::PIDConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::Rate r(rate);

    while (nh.ok())
    {
        MovePid.runTest();
        LiftPid.runTest();

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
