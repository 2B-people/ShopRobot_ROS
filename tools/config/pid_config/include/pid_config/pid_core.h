#if !defined(__PID_CORE_H_)
#define __PID_CORE_H_

#include <ros/ros.h>
#include <ros/time.h>

#include <string>

includes <data::Pid.h>

#include <pid_config/PIDconfig.h>

namespace tools
{
namespace config
{

class PidCfg
{
  public:
    PidCfg(std::string name);

    void setPid(double p,double i,double d,bool test);
    void publishPid();

    //@note 建议重写此函数
    virtual void runTest();

  private:
   ros::Publisher pub_;

   double p_;
   double i_;
   double d_;
   bool test_;
};

} // namespace config
} // namespace tools

#endif // __PID_CORE_H_
