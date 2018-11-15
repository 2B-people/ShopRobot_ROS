#include "pid_config/pid_core.h"

namespace tools
{
namespace config
{
PidCfg::PidCfg(std::string name)
{
    ros::NodeHandle nh;
    pub_ = nh.advertise<data::Pid>(name, 10);

    p_ = 0;
    i_ = 0;
    d_ = 0;
    test_ = false;
}

PidCfg::setPid(double p,double i,double d,bool test)
{
    p_ = p;
    i_ = i;
    d_ = d;
    test_ = test;
}


PidCfg::publishPid()
{
    data::Pid msg;
    msg.p = P_;
    msg.i = i_;
    msg.d = d_;
    pub_.publish(msg);

}

PidCfg::runTest()
{   
    if (test_) {
        ROS_INFO("P: %f", P_);
        ROS_INFO("i: %f", i_);
        ROS_INFO("d: %f", d_);
    }
    
}
} // namespace config
} // namespace tools