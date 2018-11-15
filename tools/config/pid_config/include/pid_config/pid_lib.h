#if !defined(_PID_LIB_H_)
#define _PID_LIB_H_

#include <pid_config/pid_core.h>

class MoveMotorPid : public tools::config::PidCfg
{
  public:
    MoveMotorPid();
    virtual void runtest();

  private:
    ros::Publisher pub_;
};

class LiftMotorPid : public tools::config::PidCfg
{
  public:
    LiftMotorPid();
    virtual void runtest();

  private:
    ros::Publisher pub_;
};

#endif // _PID_LIB_H_
