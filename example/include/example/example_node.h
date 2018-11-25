#ifndef EXAMPLE_NODE_H_
#define EXAMPLE_NODE_H_

#include "geometry_msgs/PoseStamped.h"

#include <common/rrts.h>
#include <common/timer.h>
#include <ros/ros.h>
namespace shop {
namespace common {
class MainTest : public shop::common::RRTS {
public:
  MainTest(std::string name);
  ~MainTest();

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  void CB(const geometry_msgs::PoseStamped::ConstPtr &pose);
};
} // namespace
}

#endif // EXAMPLE_NODE_H_