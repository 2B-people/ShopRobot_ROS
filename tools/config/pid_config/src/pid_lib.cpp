#include "pid_config/pid_lib.h"


MoveMotorPid::MoveMotorPid():PidCfg::PidCfg("move_pid")
{

}

MoveMotorPid::runTest()
{
    if (test_) {
        ROS_INFO("P: %f", P_);
        ROS_INFO("i: %f", i_);
        ROS_INFO("d: %f", d_);
    }
}


LiftMotorPid::LiftMotorPid():PidCfg::PidCfg("lift_pid")
{

}

LiftMotorPid::runTest()
{
    if (test_) {
        ROS_INFO("P: %f", P_);
        ROS_INFO("i: %f", i_);
        ROS_INFO("d: %f", d_);
    }
}