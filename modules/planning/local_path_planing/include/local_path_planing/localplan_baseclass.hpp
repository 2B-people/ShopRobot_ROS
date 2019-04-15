/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-28 21:02:38
 * @LastEditTime: 2019-04-14 20:34:59
 */
#ifndef LOCALPLAN_BASECLASS_H
#define LOCALPLAN_BASECLASS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <actionlib/server/simple_action_server.h>

#include <string>

#include <common/rrts.h>
#include <common/main_interface.h>

#include <data/Coord.h>
#include <data/Action.h>
#include <data/Cargo.h>
#include <data/Barrier.h>
#include <data/RoadblockMsg.h>

#include <data/SetBool.h>
#include <data/Goods.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <data/ShelfBarrier.h>
#include <data/ActionName.h>

#include <data/LocalPlanAction.h>

namespace shop
{
namespace pathplan
{

typedef actionlib::SimpleActionServer<data::LocalPlanAction> PLANACTIONSERVER;

class LocalBase : public shop::common::RRTS
{
public:
  LocalBase(std::string name)
      : shop::common::RRTS(name, 10), all_done_(false),
        plan_as_(nh_, "shop/local_plan", boost::bind(&LocalBase::PlanExecuteCB, this, _1), false)
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

    roadblock_.location_place = 0;

    robot1_action_.name = "NONE";
    robot1_action_.is_action = false;
    robot1_action_.action_state = 0;

    robot2_action_.name = "NONE";
    robot2_action_.is_action = false;
    robot2_action_.action_state = 0;

    robot3_action_.name = "NONE";
    robot3_action_.is_action = false;
    robot3_action_.action_state = 0;

    robot4_action_.name = "NONE";
    robot4_action_.is_action = false;
    robot4_action_.action_state = 0;

    for (int i = 0; i < 12; i++)
    {
      a_barrier_.shelf_barrier_all[i] = false;
      b_barrier_.shelf_barrier_all[i] = false;
      c_barrier_.shelf_barrier_all[i] = false;
      d_barrier_.shelf_barrier_all[i] = false;
      goods_.goods_loction[i] = 0;
    }

    nh_.param("debug", is_debug_, false);

    target_robot1_action_sub_ = nh_.subscribe<data::Action>("shop/robot1/target_action", 10, boost::bind(&LocalBase::Robo1ActionCB, this, _1));
    target_robot2_action_sub_ = nh_.subscribe<data::Action>("shop/robot2/target_action", 10, boost::bind(&LocalBase::Robo2ActionCB, this, _1));
    target_robot3_action_sub_ = nh_.subscribe<data::Action>("shop/robot3/target_action", 10, boost::bind(&LocalBase::Robo3ActionCB, this, _1));
    target_robot4_action_sub_ = nh_.subscribe<data::Action>("shop/robot4/target_action", 10, boost::bind(&LocalBase::Robo4ActionCB, this, _1));

    robot1_coordinate_now_ = nh_.subscribe<data::Coord>("shop/robot1/now_coord", 10, boost::bind(&LocalBase::Robo1CoordNowCB, this, _1));
    robot2_coordinate_now_ = nh_.subscribe<data::Coord>("shop/robot2/now_coord", 10, boost::bind(&LocalBase::Robo2CoordNowCB, this, _1));
    robot3_coordinate_now_ = nh_.subscribe<data::Coord>("shop/robot3/now_coord", 10, boost::bind(&LocalBase::Robo3CoordNowCB, this, _1));
    robot4_coordinate_now_ = nh_.subscribe<data::Coord>("shop/robot4/now_coord", 10, boost::bind(&LocalBase::Robo4CoordNowCB, this, _1));

    a_barrier_sub_ = nh_.subscribe<data::Barrier>("shop/a_barrier", 10, boost::bind(&LocalBase::ABarrierCB, this, _1));
    b_barrier_sub_ = nh_.subscribe<data::Barrier>("shop/b_barrier", 10, boost::bind(&LocalBase::BBarrierCB, this, _1));
    c_barrier_sub_ = nh_.subscribe<data::Barrier>("shop/c_barrier", 10, boost::bind(&LocalBase::CBarrierCB, this, _1));
    d_barrier_sub_ = nh_.subscribe<data::Barrier>("shop/d_barrier", 10, boost::bind(&LocalBase::DBarrierCB, this, _1));

    goods_sub_ = nh_.subscribe<data::Cargo>("shop/goods", 10, boost::bind(&LocalBase::GoodsCB, this, _1));
    roadblock_sub_ = nh_.subscribe<data::RoadblockMsg>("shop/roadblock", 10, boost::bind(&LocalBase::RoadblockCB, this, _1));

    goods_write_clt_ = nh_.serviceClient<data::Goods>("shop/goods_write_srv");

    a_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/A_shelf_barrier_wirte");
    b_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_wirte");
    c_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_wirte");
    d_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_wirte");

    target_coordinate_lock_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot/coordinate_srv");
    robot1_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot1/target_coordinate_write");
    robot2_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot2/target_coordinate_write");
    robot3_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot3/target_coordinate_write");
    robot4_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot4/target_coordinate_write");

    robot1_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot1/target_actionname_write");
    robot2_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot2/target_actionname_write");
    robot3_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot3/target_actionname_write");
    robot4_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot4/target_actionname_write");

    plan_as_.start();
  }

  virtual ~LocalBase() = default;

  virtual void PlanPlace(uint8_t robot_num) = 0;
  virtual void PlanCarry(uint8_t robot_num) = 0;

  void PlanExecuteCB(const data::LocalPlanGoal::ConstPtr &goal)
  {
    //反馈
    data::LocalPlanFeedback feedback;
    //结果
    data::LocalPlanResult result;

    ros::Rate r(2); //10HZ
    if (is_debug_)
    {
    }

    if (all_done_ == false)
    {
      if (JudgePlanIsDone())
      {
        result.success_flag = false;
        plan_as_.setPreempted(result);
        return;
      }
      if (robot1_action_.is_action == false)
      {
        if (robot1_action_.action_state == 0)
        {
          ROS_INFO("robot1 PlanCarry is begin");
          PlanCarry(1);
        }
        if (robot1_action_.action_state == 1)
        {
          ROS_INFO("robot1 PlanCarry is begin");
          PlanCarry(1);
        }
        if (robot1_action_.action_state == 2)
        {
          ROS_INFO("robot1 PlanPlace is begin");
          PlanPlace(1);
        }
        r.sleep();
        if (JudgePlanIsDone())
        {
          result.success_flag = false;
          plan_as_.setPreempted(result);
          return;
        }
      }

      if (robot2_action_.is_action == false)
      {
        if (robot2_action_.action_state == 0)
        {
          ROS_INFO("robot2 PlanCarry is begin");
          PlanCarry(2);
        }
        if (robot2_action_.action_state == 1)
        {
          ROS_INFO("robot2 PlanCarry is begin");
          PlanCarry(2);
        }
        if (robot2_action_.action_state == 2)
        {
          ROS_INFO("robot2 PlanPlace is begin");
          PlanPlace(2);
        }
        r.sleep();
        if (JudgePlanIsDone())
        {
          result.success_flag = false;
          plan_as_.setPreempted(result);
          return;
        }
      }

      if (robot3_action_.is_action == false)
      {
        if (robot3_action_.action_state == 0)
        {
          ROS_INFO("robot3 PlanCarry is begin");
          PlanCarry(3);
        }
        if (robot3_action_.action_state == 1)
        {
          ROS_INFO("robot3 PlanCarry is begin");
          PlanCarry(3);
        }
        if (robot3_action_.action_state == 2)
        {
          ROS_INFO("robot3 PlanPlace is begin");
          PlanPlace(3);
        }
        r.sleep();
        if (JudgePlanIsDone())
        {
          result.success_flag = false;
          plan_as_.setPreempted(result);
          return;
        }
      }

      if (robot4_action_.is_action == false)
      {
        if (robot4_action_.action_state == 0)
        {
          ROS_INFO("robot4 PlanCarry is begin");
          PlanCarry(4);
        }
        if (robot4_action_.action_state == 1)
        {
          ROS_INFO("robot4 PlanCarry is begin");
          PlanCarry(4);
        }
        if (robot4_action_.action_state == 2)
        {
          ROS_INFO("robot4 PlanPlace is begin");
          PlanPlace(4);
        }
      }
      ROS_INFO("%s FININSH", __FUNCTION__);
      result.success_flag = true;
      plan_as_.setSucceeded(result);
    }
    else
    {
      ROS_INFO("%s is Done", __FUNCTION__);
      result.success_flag = false;
      plan_as_.setPreempted(result);
    }
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

  bool JudgePlanIsDone(void)
  {
    int index = 0;
    data::LocalPlanResult result;

    for (int i = 0; i < 12; i++)
    {
      auto temp = GetGoods(i);
      // ROS_WARN("is %d", temp);
      if (temp == 0)
      {
        index++;
        if (index == 12)
        {
          all_done_ = true;
          ROS_INFO("%s is Done", __FUNCTION__);
          return true;
        }
      }
    }
    return false;
  }
  // //货物类别
  // enum class GoodsName : int8_t
  // {
  //   NONE = 0, //空,放好或者未识别
  //   RED,
  //   BLUE,
  //   GREEN,
  //   SYY,
  //   YLD,
  //   ADG,
  //   XH,
  //   HN,
  //   LH,
  //   WQ,
  //   MF,
  //   TLS,
  // };
  //read 货物类别 按照上诉规则
  int8_t GetGoods(int8_t location)
  {
    return goods_.goods_loction[location];
  }

  //直接设置目标货物为空
  void SetGoodsNONE(int8_t location)
  {
    data::Goods srv;
    srv.request.location = location - 1;
    srv.request.name = 0;
    goods_write_clt_.call(srv);
  }

  int8_t GetRoadblock()
  {
    return roadblock_.location_place;
  }

  void SetRobotTargetCoord(int8_t robot_num, int16_t x, int16_t y, int8_t pose)
  {
    data::Coordinate srv;
    srv.request.number = robot_num;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.pose = 10;
    ROS_WARN("robot%d setx:%d,sety:%d",robot_num,x,y);
    target_coordinate_lock_clt_.call(srv);
    switch (robot_num)
    {
    case 1:
      robot1_target_coordinate_write_clt_.call(srv);
      break;
    case 2:
      robot2_target_coordinate_write_clt_.call(srv);
      break;
    case 3:
      robot3_target_coordinate_write_clt_.call(srv);
      break;
    case 4:
      robot4_target_coordinate_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      return;
      break;
    }
  }

  //shelf num 为a:1,b:2,c:3,d:4
  void GetShelfBarrier(bool *shelf_barrier, int8_t shelf_num)
  {
    data::ShelfBarrier srv;
    switch (shelf_num)
    {
    case 1:
      for (size_t i = 0; i < 12; i++)
      {
        shelf_barrier[i] = a_barrier_.shelf_barrier_all[i];
      }

      break;
    case 2:
      for (size_t i = 0; i < 12; i++)
      {
        shelf_barrier[i] = b_barrier_.shelf_barrier_all[i];
      }
      break;
    case 3:
      for (size_t i = 0; i < 12; i++)
      {
        shelf_barrier[i] = c_barrier_.shelf_barrier_all[i];
      }
      break;
    case 4:
      for (size_t i = 0; i < 12; i++)
      {
        shelf_barrier[i] = d_barrier_.shelf_barrier_all[i];
      }
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
    // for (size_t i = 0; i < 12; i++)
    // {
    //   ROS_ERROR("%d is %d",i,srv.response.shelf_barrier_all[i]);
    // }
  }
  int8_t GetNowToShelf(int8_t robot_num)
  {
    data::Action action;
    switch (robot_num)
    {
    case 1:
      action.name = robot1_action_.name;
      break;
    case 2:
      action.name = robot1_action_.name;
      break;
    case 3:
      action.name = robot1_action_.name;
      break;
    case 4:
      action.name = robot1_action_.name;
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
    std::string action_name = action.name;
    if (action_name == "NONE")
    {
      ROS_WARN("cant use GetNowToShelf when no target_action_name");
      return 5;
    }

    //eg action_name is "C-1"
    std::string temp = action_name.erase(0, 2);
    // bug in these
    int num = stoi(temp);
    if (num == 1 || num == 2 || num == 3)
    {
      return 1; //A
    }
    else if (num == 4 || num == 5 || num == 6)
    {
      return 2; //B
    }
    else if (num == 7 || num == 8 || num == 9)
    {
      return 3; //C
    }
    else if (num == 10 || num == 11 || num == 12)
    {
      return 4; //D
    }
  }

  // @breif 使得目标位置为true
  // @param location:0~11,0 is 1,1 is 2
  // @param shelf_num =a:1,b:2,c:3,d:4
  // @breif 设定目标位置为有东西
  void SetShelfToFalse(int8_t shelf_num, int8_t location)
  {
    data::ShelfBarrier srv;
    srv.request.location = location - 1;
    srv.request.shelf_barrier = true;
    switch (shelf_num)
    {
    case 1:
      a_shelf_barrier_write_clt_.call(srv);
      break;
    case 2:
      b_shelf_barrier_write_clt_.call(srv);
      break;
    case 3:
      c_shelf_barrier_write_clt_.call(srv);
      break;
    case 4:
      d_shelf_barrier_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
  }

  void SetRobotTargetAction(int8_t robot_num, std::string action_name,int8_t action_state)
  {
    data::ActionName srv;
    srv.request.action_name = action_name;
    srv.request.action_state = action_state;
    ROS_WARN("%d is set %s", robot_num, action_name.c_str());
    switch (robot_num)
    {
    case 1:
      robot1_target_actionname_write_clt_.call(srv);
      break;
    case 2:
      robot2_target_actionname_write_clt_.call(srv);
      break;
    case 3:
      robot3_target_actionname_write_clt_.call(srv);
      break;
    case 4:
      robot4_target_actionname_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
  }

  bool is_debug_;
  bool all_done_;

protected:
  data::Coord robot1_coord_now_;
  data::Coord robot2_coord_now_;
  data::Coord robot3_coord_now_;
  data::Coord robot4_coord_now_;

  data::Action robot1_action_;
  data::Action robot2_action_;
  data::Action robot3_action_;
  data::Action robot4_action_;

  data::Barrier a_barrier_;
  data::Barrier b_barrier_;
  data::Barrier c_barrier_;
  data::Barrier d_barrier_;

  data::RoadblockMsg roadblock_;

  data::Cargo goods_;

  ros::NodeHandle nh_;
  ros::Subscriber robot1_coordinate_now_;
  ros::Subscriber robot2_coordinate_now_;
  ros::Subscriber robot3_coordinate_now_;
  ros::Subscriber robot4_coordinate_now_;

  ros::Subscriber goods_sub_;
  ros::Subscriber a_barrier_sub_;
  ros::Subscriber b_barrier_sub_;
  ros::Subscriber c_barrier_sub_;
  ros::Subscriber d_barrier_sub_;

  ros::Subscriber roadblock_sub_;

  ros::Subscriber target_robot1_action_sub_;
  ros::Subscriber target_robot2_action_sub_;
  ros::Subscriber target_robot3_action_sub_;
  ros::Subscriber target_robot4_action_sub_;

  ros::ServiceClient goods_write_clt_;

  ros::ServiceClient target_coordinate_lock_clt_;
  ros::ServiceClient robot1_target_coordinate_write_clt_;
  ros::ServiceClient robot2_target_coordinate_write_clt_;
  ros::ServiceClient robot3_target_coordinate_write_clt_;
  ros::ServiceClient robot4_target_coordinate_write_clt_;

  ros::ServiceClient robot1_target_actionname_write_clt_;
  ros::ServiceClient robot2_target_actionname_write_clt_;
  ros::ServiceClient robot3_target_actionname_write_clt_;
  ros::ServiceClient robot4_target_actionname_write_clt_;

  ros::ServiceClient a_shelf_barrier_write_clt_;
  ros::ServiceClient b_shelf_barrier_write_clt_;
  ros::ServiceClient c_shelf_barrier_write_clt_;
  ros::ServiceClient d_shelf_barrier_write_clt_;

  PLANACTIONSERVER plan_as_;

  void Robo1CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot1_coord_now_.x = msg->x;
    robot1_coord_now_.y = msg->y;
    robot1_coord_now_.pose = msg->pose;
    // ROS_INFO("new coord is%d %d %d",msg->x,msg->y,msg->pose);
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
    // ROS_INFO("new coord is%d %d %d",msg->x,msg->y,msg->pose);
  }

  void GoodsCB(const data::Cargo::ConstPtr &msg)
  {
    for (int i = 0; i < 12; i++)
    {
      goods_.goods_loction[i] = msg->goods_loction[i];
    }
  }

  void ABarrierCB(const data::Barrier::ConstPtr &msg)
  {
    for (int i = 0; i < 12; i++)
    {
      a_barrier_.shelf_barrier_all[i] = msg->shelf_barrier_all[i];
    }
  }

  void BBarrierCB(const data::Barrier::ConstPtr &msg)
  {
    for (int i = 0; i < 12; i++)
    {
      b_barrier_.shelf_barrier_all[i] = msg->shelf_barrier_all[i];
    }
  }

  void CBarrierCB(const data::Barrier::ConstPtr &msg)
  {
    for (int i = 0; i < 12; i++)
    {
      c_barrier_.shelf_barrier_all[i] = msg->shelf_barrier_all[i];
    }
  }

  void DBarrierCB(const data::Barrier::ConstPtr &msg)
  {
    for (int i = 0; i < 12; i++)
    {
      d_barrier_.shelf_barrier_all[i] = msg->shelf_barrier_all[i];
    }
  }

  void RoadblockCB(const data::RoadblockMsg::ConstPtr &msg)
  {
    roadblock_.location_place = msg->location_place;
  }

  void Robo1ActionCB(const data::Action::ConstPtr &msg)
  {
    robot1_action_.name = msg->name;
    robot1_action_.is_action = msg->is_action;
    robot1_action_.action_state = msg->action_state;
  }

  void Robo2ActionCB(const data::Action::ConstPtr &msg)
  {
    robot2_action_.name = msg->name;
    robot2_action_.is_action = msg->is_action;
    robot2_action_.action_state = msg->action_state;
  }

  void Robo3ActionCB(const data::Action::ConstPtr &msg)
  {
    robot3_action_.name = msg->name;
    robot3_action_.is_action = msg->is_action;
    robot3_action_.action_state = msg->action_state;
  }

  void Robo4ActionCB(const data::Action::ConstPtr &msg)
  {
    robot4_action_.name = msg->name;
    robot4_action_.is_action = msg->is_action;
    robot4_action_.action_state = msg->action_state;
  }
};

} // namespace pathplan
} // namespace shop

#endif