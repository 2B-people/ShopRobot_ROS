#include <ros/ros.h>
#include <ros/time.h>

#include <string>

#include <global_path_planning/global_path_planning.h>

namespace shop
{
namespace pathplan
{
GlobalPlan::GlobalPlan(std::string name):shop::common::RRTS(name)
{
    ros::NodeHandle nh("~");
    
    nh.getParam()
}
GlobalPlan::~GlobalPlan()
{

}
} // namespace pathplan
} // namespace shop
