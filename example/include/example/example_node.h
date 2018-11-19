#ifndef EXAMPLE_NODE_H_
#define EXAMPLE_NODE_H_

#include "geometry_msgs/PoseStamped.h"

#include <common/timer.h>
#include <common/rrts.h>
#include <ros/ros.h>

namespace example
{
class MainTest : public common::RRTS
{
  public:
    MainTest(std::string name);
    ~MainTest();

  private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    void CB(const geometry_msgs::PoseStamped::ConstPtr &pose);
};
} // namespace

#endif //EXAMPLE_NODE_H_