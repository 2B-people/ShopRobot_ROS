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

//移动的执行叶节点
class MoveAction : public ActionNode
{
public:
  MoveAction(int8_t robot_num, std::string name, const Blackboard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr)
  {
    auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
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
    //key键的str
    std::string bool_key = "robot" + std::to_string(robot_num_) + "/local_plan/flag";
    std::string coordinate_key = "robot" + std::to_string(robot_num_) + "/run_coordinate";
    if (state != BehaviorState::RUNNING)
    {
      // if (private_blackboard_ptr_->GetBoolValue(bool_key))
      // {
      //   auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key);
      //   auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
      //   uint16_t target_x = dir_ptr->GetCoordinateX();
      //   uint16_t target_y = dir_ptr->GetCoordinateY();
      //   uint16_t target_pose = dir_ptr->GetCoordinatePOSE();
      //   goalaction_ptr_->SendMoveGoal(robot_num_, target_x, target_y, target_pose);
      // }
      // else
      // {
      //   ROS_INFO("%d is wait for local plan", robot_num_);
      // }

      auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key);
      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
      uint16_t target_x = dir_ptr->GetCoordinateX();
      uint16_t target_y = dir_ptr->GetCoordinateY();
      uint16_t target_pose = dir_ptr->GetCoordinatePOSE();
      goalaction_ptr_->SendMoveGoal(robot_num_, target_x, target_y, target_pose);
    }
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
  PrivateBoard::Ptr private_blackboard_ptr_;
};

//动作节点
class ShopAction : public ActionNode
{
public:
  ShopAction(uint8_t robot_num, std::string name, const Blackboard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr)
  {
    auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
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
    std::string name_key = "robot" + std::to_string(robot_num_) + "/action_name";
    if (state != BehaviorState::RUNNING)
    {
      auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(name_key);
      auto dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(temp_dir_ptr);
      std::string goal_name = dir_ptr->GetActionName();
      goalaction_ptr_->SendShopGoal(robot_num_, goal_name);
    }
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
  PrivateBoard::Ptr private_blackboard_ptr_;
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
    auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
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
    if (state != BehaviorState::RUNNING)
    {
      goalaction_ptr_->SendOpenGoal(robot_num_, "go");
    }
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
  PrivateBoard::Ptr private_blackboard_ptr_;
};

} // namespace decision
} // namespace shop

#endif