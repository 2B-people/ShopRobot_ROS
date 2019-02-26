#ifndef GOAL_ACTION_H
#define GOAL_ACTION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include <data/MoveAction.h>
#include <data/OpeningAction.h>
#include <data/ShopActionAction.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>

namespace shop
{
namespace decision
{
typedef actionlib::SimpleActionClient<data::MoveAction> MOVEACTIONCLINT;
typedef actionlib::SimpleActionClient<data::OpeningAction> OPENINGCLINT;
typedef actionlib::SimpleActionClient<data::ShopActionAction> SHOPACTION;

class GoalAction
{
public:
  typedef std::shared_ptr<GoalAction> Ptr;


  GoalAction(const Blackboard::Ptr &blackboard_ptr) : blackboard_ptr_(blackboard_ptr),
                                                    robot1_move_action_clint_("robot1_web/move_action", true),
                                                    robot1_open_action_clint_("robot1_web/opening_action", true),
                                                    robot1_shop_action_clint_("robot1_web/shop_action", true),
                                                    robot2_move_action_clint_("robot2_web/move_action", true),
                                                    robot2_open_action_clint_("robot2_web/opening_action", true),
                                                    robot2_shop_action_clint_("robot2_web/shop_action", true),
                                                    robot3_move_action_clint_("robot3_web/move_action", true),
                                                    robot3_open_action_clint_("robot3_web/opening_action", true),
                                                    robot3_shop_action_clint_("robot3_web/shop_action", true),
                                                    robot4_move_action_clint_("robot4_web/move_action", true),
                                                    robot4_shop_action_clint_("robot4_web/shop_action", true)
  {
    ROS_INFO("Waiting for action server to start");
    robot1_move_action_clint_.waitForServer();
    robot1_open_action_clint_.waitForServer();
    robot1_shop_action_clint_.waitForServer();
    robot2_move_action_clint_.waitForServer();
    robot2_open_action_clint_.waitForServer();
    robot2_shop_action_clint_.waitForServer();
    robot3_move_action_clint_.waitForServer();
    robot3_open_action_clint_.waitForServer();
    robot3_shop_action_clint_.waitForServer();
    robot4_move_action_clint_.waitForServer();
    robot4_shop_action_clint_.waitForServer();
    ROS_INFO(" ALL Action server is started");
  }
  ~GoalAction() = default;

  void SendMoveGoal(int8_t robot_num, int16_t x, int16_t y, int8_t pose)
  {
    data::MoveGoal goal;
    goal.x = x;
    goal.y = y;
    goal.pose = pose;
    switch (robot_num)
    {
    case 1:
      robot1_move_action_clint_.sendGoal(goal,
                                         MOVEACTIONCLINT::SimpleDoneCallback(),
                                         MOVEACTIONCLINT::SimpleActiveCallback(),
                                         MOVEACTIONCLINT::SimpleFeedbackCallback());
      break;
    case 2:
      robot2_move_action_clint_.sendGoal(goal, 
                                         MOVEACTIONCLINT::SimpleDoneCallback(),
                                         MOVEACTIONCLINT::SimpleActiveCallback(),
                                         MOVEACTIONCLINT::SimpleFeedbackCallback());
      break;
    case 3:
      robot3_move_action_clint_.sendGoal(goal,                                          
                                         MOVEACTIONCLINT::SimpleDoneCallback(),
                                         MOVEACTIONCLINT::SimpleActiveCallback(), 
                                         MOVEACTIONCLINT::SimpleFeedbackCallback()
      );
      break;
    case 4:
      robot4_move_action_clint_.sendGoal(goal,                                          
                                         MOVEACTIONCLINT::SimpleDoneCallback(),
                                         MOVEACTIONCLINT::SimpleActiveCallback(), 
                                         MOVEACTIONCLINT::SimpleFeedbackCallback()
      );
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  void SendShopGoal(int8_t robot_num, std::string action_name)
  {
    data::ShopActionGoal goal;
    goal.action_name = action_name;
    switch (robot_num)
    {
    case 1:
      robot1_shop_action_clint_.sendGoal(goal,                                          
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(), 
                                         SHOPACTION::SimpleFeedbackCallback()
      );
      break;
    case 2:
      robot2_shop_action_clint_.sendGoal(goal,                                          
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(), 
                                         SHOPACTION::SimpleFeedbackCallback()
      );
      break;
    case 3:
      robot3_shop_action_clint_.sendGoal(goal,                                          
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(), 
                                         SHOPACTION::SimpleFeedbackCallback()
      );
      break;
    case 4:
      robot4_shop_action_clint_.sendGoal(goal,                                         
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(), 
                                         SHOPACTION::SimpleFeedbackCallback()
      );
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  void SendOpenGoal(int8_t robot_num, std::string car_begin)
  {
    data::OpeningGoal goal;
    goal.car_begin = car_begin;
    switch (robot_num)
    {
    case 1:
      robot1_open_action_clint_.sendGoal(goal,                                          
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(), 
                                         OPENINGCLINT::SimpleFeedbackCallback()
      );
    case 2:
      robot2_open_action_clint_.sendGoal(goal,                                           
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(), 
                                         OPENINGCLINT::SimpleFeedbackCallback()
      );
      break;
    case 3:
      robot2_open_action_clint_.sendGoal(goal,                                           
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(), 
                                         OPENINGCLINT::SimpleFeedbackCallback()
      );
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  void CancelMoveGoal(int8_t robot_num)
  {
    switch (robot_num)
    {
    case 1:
      robot1_move_action_clint_.cancelGoal();
      ROS_INFO("robot1 is cancel!");
      break;
    case 2:
      robot2_move_action_clint_.cancelGoal();
      ROS_INFO("robot2 is cancel!");
      break;
    case 3:
      robot3_move_action_clint_.cancelGoal();
      ROS_INFO("robot3 is cancel!");
      break;
    case 4:
      robot3_move_action_clint_.cancelGoal();
      ROS_INFO("robot3 is cancel!");
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  void CancelShopGoal(int8_t robot_num)
  {
    switch (robot_num)
    {
    case 1:
      robot1_shop_action_clint_.cancelGoal();
      ROS_INFO("robot1 is cancel!");
      break;
    case 2:
      robot1_shop_action_clint_.cancelGoal();
      ROS_INFO("robot2 is cancel!");
      break;
    case 3:
      robot1_shop_action_clint_.cancelGoal();
      ROS_INFO("robot3 is cancel!");
      break;
    case 4:
      robot1_shop_action_clint_.cancelGoal();
      ROS_INFO("robot3 is cancel!");
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  BehaviorState GetMoveBehaviorState(int8_t robot_num)
  {
    auto state = robot1_move_action_clint_.getState();

    switch (robot_num)
    {
    case 1:
      state = robot1_move_action_clint_.getState();
      break;
    case 2:
      state = robot2_move_action_clint_.getState();
      break;
    case 3:
      state = robot3_move_action_clint_.getState();
      break;
    case 4:
      state = robot4_move_action_clint_.getState();
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
    if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return BehaviorState::SUCCESS;
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return BehaviorState::FAILURE;
    }
    else
    {
      return BehaviorState::FAILURE;
    }
  }

  BehaviorState GetShopBehaviorState(int8_t robot_num)
  {
    auto state = robot1_shop_action_clint_.getState();

    switch (robot_num)
    {
    case 1:
      state = robot1_shop_action_clint_.getState();
      break;
    case 2:
      state = robot2_shop_action_clint_.getState();
      break;
    case 3:
      state = robot3_shop_action_clint_.getState();
      break;
    case 4:
      state = robot4_shop_action_clint_.getState();
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
    if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return BehaviorState::SUCCESS;
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return BehaviorState::FAILURE;
    }
    else
    {
      return BehaviorState::FAILURE;
    }
  }

  BehaviorState GetOpenBehaviorState(int8_t robot_num)
  {
    auto state = robot1_open_action_clint_.getState();

    switch (robot_num)
    {
    case 1:
      state = robot1_open_action_clint_.getState();
      break;
    case 2:
      state = robot2_open_action_clint_.getState();
      break;
    case 3:
      state = robot3_open_action_clint_.getState();
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
    if (state == actionlib::SimpleClientGoalState::ACTIVE)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::PENDING)
    {
      return BehaviorState::RUNNING;
    }
    else if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      return BehaviorState::SUCCESS;
    }
    else if (state == actionlib::SimpleClientGoalState::ABORTED)
    {
      return BehaviorState::FAILURE;
    }
    else
    {
      return BehaviorState::FAILURE;
    }
  }

private:
  Blackboard::Ptr blackboard_ptr_;

  ros::NodeHandle nh_;
  MOVEACTIONCLINT robot1_move_action_clint_;
  OPENINGCLINT robot1_open_action_clint_;
  SHOPACTION robot1_shop_action_clint_;

  MOVEACTIONCLINT robot2_move_action_clint_;
  OPENINGCLINT robot2_open_action_clint_;
  SHOPACTION robot2_shop_action_clint_;

  MOVEACTIONCLINT robot3_move_action_clint_;
  OPENINGCLINT robot3_open_action_clint_;
  SHOPACTION robot3_shop_action_clint_;

  MOVEACTIONCLINT robot4_move_action_clint_;
  SHOPACTION robot4_shop_action_clint_;
};

} // namespace decision
} // namespace shop

#endif
