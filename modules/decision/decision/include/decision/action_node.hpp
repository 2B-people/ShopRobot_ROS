#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <decision/goal_action.hpp>

namespace shop
{
namespace decision
{

class MoveAction : public ActionNode
{
public:
  MoveAction(int8_t robot_num, std::string name, const Blackboard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr)
  {
  }
  ~MoveAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  virtual BehaviorState Update()
  {
    BehaviorState state = goalaction_ptr_->GetMoveBehaviorState(robot_num_);
    // if (state != BehaviorState::RUNNING)
    // {
    //   if(blackboard_ptr_->GetBoolValue("robot%d/local_plan/flag",robot_num_))
    //   {
    //     uint16_t target_x = blackboard_ptr_->GetCoordinateX("robot%d",robot_num_);
    //     uint16_t target_y = blackboard_ptr_->GetCoordinateY("robot%d",robot_num_);
    //     int8_t target_pose = blackboard_ptr_->GetCoordinatepose("robot%d",robot_num_);
    //     goalaction_ptr_->SendMoveGoal(robot_num_,target_x,target_y,target_pose);
    //   }else
    //   {
    //     ROS_INFO("%d is wait for local plan");
    //   }
    // }
    return goalaction_ptr_->GetMoveBehaviorState(robot_num_);
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      goalaction_ptr_->CancelMoveGoal(robot_num_);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
  }

  int8_t robot_num_;
  GoalAction::Ptr goalaction_ptr_;
};

class ShopAction : public ActionNode
{
public:
  ShopAction(uint8_t robot_num, std::string name, const Blackboard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr)
  {
  }
  ~ShopAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  virtual BehaviorState Update()
  {
    BehaviorState state = goalaction_ptr_->GetShopBehaviorState(robot_num_);
    // if (state != BehaviorState::RUNNING)
    // {
    //   blackboard_ptr_->;

    //   //todo 读入写的坐标
    // }
    return goalaction_ptr_->GetShopBehaviorState(robot_num_);
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      goalaction_ptr_->CancelShopGoal(robot_num_);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
  }
  uint8_t robot_num_;
  GoalAction::Ptr goalaction_ptr_;
};

class OpenAction : public ActionNode
{
public:
  OpenAction(uint8_t robot_num, std::string name, const Blackboard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr)
  {
  }
  ~OpenAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  virtual BehaviorState Update()
  {
    BehaviorState state = goalaction_ptr_->GetOpenBehaviorState(robot_num_);
    // if (state != BehaviorState::RUNNING)
    // {
    //   blackboard_ptr_->;

    //   //todo 读入写的坐标
    // }
    return goalaction_ptr_->GetOpenBehaviorState(robot_num_);
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      goalaction_ptr_->CancelShopGoal(robot_num_);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
  }

  uint8_t robot_num_;
  GoalAction::Ptr goalaction_ptr_;
};

} // namespace decision
} // namespace shop

#endif