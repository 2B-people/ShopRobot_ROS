#ifndef GLOBAL_PATH_PLANNING_H
#define GLOBAL_PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/simple_action_server.h>

#include <string>

#include <common/rrts.h>
#include <common/main_interface.h>

#include <data/Coord.h>
#include <data/RoadblockMsg.h>

#include <data/SetBool.h>
#include <data/Goods.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <data/ShelfBarrier.h>
#include <data/ActionName.h>

#include <data/GlobalPlanAction.h>

namespace shop
{
namespace pathplan
{

typedef actionlib::SimpleActionServer<data::GlobalPlanAction> PLANACTIONSERVER;

class GlobalBase : public shop::common::RRTS
{
public:
  GlobalBase(std::string name) : common::RRTS(name, 1),
                                 plan_as_(nh_, "shop/global_plan", boost::bind(&GlobalBase::PlanExecuteCB, this, _1), false)
  {
    robot1_coord_now_.x = 10;
    robot1_coord_now_.y = 10;
    robot1_coord_now_.pose = 0;
    robot2_coord_now_.x = 10;
    robot2_coord_now_.y = 10;
    robot2_coord_now_.pose = 0;
    robot3_coord_now_.x = 10;
    robot3_coord_now_.y = 10;
    robot3_coord_now_.pose = 0;
    robot4_coord_now_.x = 10;
    robot4_coord_now_.y = 10;
    robot4_coord_now_.pose = 0;
    robot1_target_coord_.x = 10;
    robot1_target_coord_.y = 10;
    robot1_target_coord_.pose = 0;
    robot2_target_coord_.x = 10;
    robot2_target_coord_.y = 10;
    robot2_target_coord_.pose = 0;
    robot3_target_coord_.x = 10;
    robot3_target_coord_.y = 10;
    robot3_target_coord_.pose = 0;
    robot4_target_coord_.x = 10;
    robot4_target_coord_.y = 10;
    robot4_target_coord_.pose = 0;
    roadblock_.location_place = 0;

    roadblock_sub_ = nh_.subscribe<data::RoadblockMsg>("shop/roadblock", 10, boost::bind(&GlobalBase::RoadblockCB, this, _1));

    robot1_coordinate_sub_ = nh_.subscribe<data::Coord>("shop/robot1/now_coord", 10, boost::bind(&GlobalBase::Robo1CoordNowCB, this, _1));
    robot2_coordinate_sub_ = nh_.subscribe<data::Coord>("shop/robot2/now_coord", 10, boost::bind(&GlobalBase::Robo2CoordNowCB, this, _1));
    robot3_coordinate_sub_ = nh_.subscribe<data::Coord>("shop/robot3/now_coord", 10, boost::bind(&GlobalBase::Robo3CoordNowCB, this, _1));
    robot4_coordinate_sub_ = nh_.subscribe<data::Coord>("shop/robot4/now_coord", 10, boost::bind(&GlobalBase::Robo4CoordNowCB, this, _1));

    target_robot1_coord_sub_ = nh_.subscribe<data::Coord>("shop/robot1/target_coord", 10, boost::bind(&GlobalBase::Robo1TargetCoordCB, this, _1));
    target_robot2_coord_sub_ = nh_.subscribe<data::Coord>("shop/robot2/target_coord", 10, boost::bind(&GlobalBase::Robo2TargetCoordCB, this, _1));
    target_robot3_coord_sub_ = nh_.subscribe<data::Coord>("shop/robot3/target_coord", 10, boost::bind(&GlobalBase::Robo3TargetCoordCB, this, _1));
    target_robot4_coord_sub_ = nh_.subscribe<data::Coord>("shop/robot4/target_coord", 10, boost::bind(&GlobalBase::Robo4TargetCoordCB, this, _1));

    robot1_cmd_coord_pub_ = nh_.advertise<data::Coord>("shop/robot1/cmd_coord", 1);
    robot2_cmd_coord_pub_ = nh_.advertise<data::Coord>("shop/robot2/cmd_coord", 1);
    robot3_cmd_coord_pub_ = nh_.advertise<data::Coord>("shop/robot3/cmd_coord", 1);
    robot4_cmd_coord_pub_ = nh_.advertise<data::Coord>("shop/robot4/cmd_coord", 1);

    plan_as_.start();

    ROS_WARN("global is init");
  }

  virtual ~GlobalBase() = default;

  virtual data::Coord GetFinalCoord(uint8_t robot_num_) = 0;
  virtual void RobotGlobalPlanning(void) = 0;

  void PlanExecuteCB(const data::GlobalPlanGoal::ConstPtr &goal)
  {
    ROS_INFO("Global plan is Run");
    //反馈
    data::GlobalPlanFeedback feedback;
    //结果
    data::GlobalPlanResult result;

    if (goal->do_flag == true)
    {
      ROS_WARN("in here");
      RobotGlobalPlanning();
      auto robot1_coord = GetFinalCoord(1);
      auto robot2_coord = GetFinalCoord(2);
      auto robot3_coord = GetFinalCoord(3);
      auto robot4_coord = GetFinalCoord(4);
      robot1_cmd_coord_pub_.publish(robot1_coord);
      robot2_cmd_coord_pub_.publish(robot2_coord);
      robot3_cmd_coord_pub_.publish(robot3_coord);
      robot4_cmd_coord_pub_.publish(robot4_coord);
    }
    else
    {
      ROS_INFO("Global plan is stop");
      plan_as_.setPreempted();
      return;
    }

    ROS_INFO("%s FININSH", __FUNCTION__);
    result.success_flag = true;
    plan_as_.setSucceeded(result);
    return;
  }

  // auto now = GetNowCoord(1);
  // x = now.x;
  data::Coord GetNowCoord(uint8_t robot_num)
  {
    switch (robot_num)
    {
    case 1:
      return robot1_coord_now_;
      break;
    case 2:
      return robot2_coord_now_;
      break;
    case 3:
      return robot3_coord_now_;
      break;
    case 4:
      return robot4_coord_now_;
      break;
    default:
      ROS_ERROR("no robot num in %s in %s", __FUNCTION__, name_.c_str());
      break;
    }
  }

  int8_t GetRoadblock()
  {
    return roadblock_.location_place;
  }

  data::Coord GetTargetCoord(uint8_t robot_num)
  {
    switch (robot_num)
    {
    case 1:
      return robot1_target_coord_;
      break;
    case 2:
      return robot2_target_coord_;
      break;
    case 3:
      return robot3_target_coord_;
      break;
    case 4:
      return robot4_target_coord_;      
      break;
    default:
      ROS_ERROR("no robot num in %s in %s", __FUNCTION__, name_.c_str());
      break;
    }
  }

protected:
  data::Coord robot1_coord_now_;
  data::Coord robot2_coord_now_;
  data::Coord robot3_coord_now_;
  data::Coord robot4_coord_now_;

  data::Coord robot1_target_coord_;
  data::Coord robot2_target_coord_;
  data::Coord robot3_target_coord_;
  data::Coord robot4_target_coord_;

  data::RoadblockMsg roadblock_;

  ros::NodeHandle nh_;
  ros::Subscriber robot1_coordinate_sub_;
  ros::Subscriber robot2_coordinate_sub_;
  ros::Subscriber robot3_coordinate_sub_;
  ros::Subscriber robot4_coordinate_sub_;

  ros::Subscriber roadblock_sub_;

  ros::Subscriber target_robot1_coord_sub_;
  ros::Subscriber target_robot2_coord_sub_;
  ros::Subscriber target_robot3_coord_sub_;
  ros::Subscriber target_robot4_coord_sub_;

  ros::Publisher robot1_cmd_coord_pub_;
  ros::Publisher robot2_cmd_coord_pub_;
  ros::Publisher robot3_cmd_coord_pub_;
  ros::Publisher robot4_cmd_coord_pub_;

  PLANACTIONSERVER plan_as_;

  void Robo1CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot1_coord_now_.x = msg->x;
    robot1_coord_now_.y = msg->y;
    robot1_coord_now_.pose = msg->pose;
  }

  void Robo2CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot2_coord_now_.x = msg->x;
    robot2_coord_now_.y = msg->y;
    robot2_coord_now_.pose = msg->pose;
  }

  void Robo3CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot3_coord_now_.x = msg->x;
    robot3_coord_now_.y = msg->y;
    robot3_coord_now_.pose = msg->pose;
  }

  void Robo4CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot4_coord_now_.x = msg->x;
    robot4_coord_now_.y = msg->y;
    robot4_coord_now_.pose = msg->pose;
  }

  void Robo1TargetCoordCB(const data::Coord::ConstPtr &msg)
  {
    robot1_target_coord_.x = msg->x;
    robot1_target_coord_.y = msg->y;
    robot1_target_coord_.pose = msg->pose;
  }
  void Robo2TargetCoordCB(const data::Coord::ConstPtr &msg)
  {
    robot2_target_coord_.x = msg->x;
    robot2_target_coord_.y = msg->y;
    robot2_target_coord_.pose = msg->pose;
  }
  void Robo3TargetCoordCB(const data::Coord::ConstPtr &msg)
  {
    robot3_target_coord_.x = msg->x;
    robot3_target_coord_.y = msg->y;
    robot3_target_coord_.pose = msg->pose;
  }
  void Robo4TargetCoordCB(const data::Coord::ConstPtr &msg)
  {
    robot4_target_coord_.x = msg->x;
    robot4_target_coord_.y = msg->y;
    robot4_target_coord_.pose = msg->pose;
  }

  void RoadblockCB(const data::RoadblockMsg::ConstPtr &msg)
  {
    roadblock_.location_place = msg->location_place;
  }

private:
};

} // namespace pathplan

} // namespace shop

#endif