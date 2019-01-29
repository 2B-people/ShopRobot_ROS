#ifndef GLOBAL_PATH_PLANNING_H
#define GLOBAL_PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <common/rrts.h>

namespace shop
{
namespace pathplan
{
class GlobalPlan : public shop::common::RRTS
{
  public:
    GlobalPlan(std::string name);
    ~GlobalPlan();

  private:
    ros::Subscriber robot1_pose_now_sub_;
    ros::Subscriber robot2_pose_now_sub_;
    ros::Subscriber robot3_pose_now_sub_;
    ros::Subscriber robot4_pose_now_sub_;
    ros::Publisher robot1_pose_target_pub_;
    ros::Publisher robot2_pose_target_pub_;
    ros::Publisher robot3_pose_target_pub_;
    ros::Publisher robot4_pose_target_pub_;
    ros::NodeHandle nh_;
};

} // namespace pathplan

} // namespace shop

#endif