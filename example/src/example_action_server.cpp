#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <common/rrts.h>
#include <common/timer.h>

#include <data/TestAction.h>

typedef actionlib::SimpleActionServer<data::TestAction> Server;
class TestAction
{
public:
  TestAction(std::string name) : name_(name), as_(nh_, name, boost::bind(&TestAction::ExecuteCB, this, _1), false)
  {
    ROS_INFO("Action is exit");
    as_.registerPreemptCallback(boost::bind(&TestAction::CancelCB, this));
    as_.start();
  }
  virtual ~TestAction() = default;

  void ExecuteCB(const data::TestGoal::ConstPtr &goal)
  {
    ros::Rate r(1);
    ROS_INFO("Actioin Server is doing goal is %d", goal->goal);
    for (int i = 0; i < (int)goal->goal; i++)
    {
      feedback_.progress = i;
      as_.publishFeedback(feedback_);
      r.sleep();
    }
    if (as_.isActive())
    {

      ROS_INFO("Action is done");
      result_.finsh = true;
      as_.setSucceeded(result_);
      // while (1)
      // {
      //   ROS_WARN("!!!!!");
      // }
    }
  }

  void CancelCB()
  {
    ros::Rate r(1);
    ROS_INFO("Cancel is run!");
    if(as_.isActive())
    {
      as_.setPreempted();
    }
  }

private:
  ros::NodeHandle nh_;
  Server as_;
  data::TestFeedback feedback_;
  data::TestResult result_;
  std::string name_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "testAction_server");
  TestAction action("testAction");
  ros::spin();
  return 0;
}