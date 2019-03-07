/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-07 21:11:54
 * @LastEditTime: 2019-03-07 21:14:00
 */
#ifndef ACTION_NODE_H
#define ACTION_NODE_H

#include <ros/ros.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <decision/goal_action.hpp>

#include <data/ActionName.h>

//fixed
namespace shop
{
namespace decision
{

//移动的执行叶节点
class MoveAction : public ActionNode
{
public:
  MoveAction(int8_t robot_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr),
        goal_flag_(false)
  {
    coordinate_key_ = "robot" + std::to_string(robot_num_) + "/run_coordinate";
    // auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
  }
  ~MoveAction() = default;

private:
  virtual void OnInitialize()
  {
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key_);
    auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
    uint16_t target_x = dir_ptr->GetCoordinateX();
    uint16_t target_y = dir_ptr->GetCoordinateY();
    uint16_t target_pose = dir_ptr->GetCoordinatePOSE();
    std::string bool_key = "robot" + std::to_string(robot_num_) + "/local_plan/flag";
    if (target_x != 10 && target_y != 10 && target_pose != 10)
    {
      goalaction_ptr_->SendMoveGoal(robot_num_, target_x, target_y, target_pose);
      goal_flag_ = true;
    }
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }

  //fixed!!!
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetMoveBehaviorState(robot_num_);
  }

  //结束是把对应目标坐标清零
  virtual void OnTerminate(BehaviorState state)
  {
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key_);
    auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
    dir_ptr->OpenLock();
    dir_ptr->Set(10, 10, 10);

    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      if (goal_flag_)
      {
        goalaction_ptr_->CancelMoveGoal(robot_num_);
        goal_flag_ = false;
      }
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
  bool goal_flag_;
  std::string coordinate_key_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

//动作节点
class ShopAction : public ActionNode
{
public:
  ShopAction(uint8_t robot_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr),
        goal_flag_(false)
  {
    std::string name_key = "shop/robot" + std::to_string(robot_num_) + "/target_actionname_write";
    client_ = nh_.serviceClient<data::ActionName>(name_key);
  }
  ~ShopAction() = default;

private:
  virtual void OnInitialize()
  {
    data::ActionName srv;
    client_.call(srv);
    std::string goal_name = srv.response.action_name;
    if (goal_name != "NONE")
    {
      goalaction_ptr_->SendShopGoal(robot_num_, goal_name);
      goal_flag_ = true;
    }
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetShopBehaviorState(robot_num_);
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      if (goal_flag_)
      {
        goalaction_ptr_->CancelShopGoal(robot_num_);
        goal_flag_ = false;
      }
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
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  bool goal_flag_;
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
        goalaction_ptr_(goalaction_ptr),
        goal_flag_(false)
  {
    auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
  }
  ~OpenAction() = default;

private:
  virtual void OnInitialize()
  {
    goalaction_ptr_->SendOpenGoal(robot_num_, "go");
    goal_flag_ = true;
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetOpenBehaviorState(robot_num_);
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      if (goal_flag_)
      {
        goalaction_ptr_->CancelShopGoal(robot_num_);
        goal_flag_ = false;
      }
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
  bool goal_flag_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

class CameraAction : public ActionNode
{
public:
  CameraAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
               const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr),
        num_count_(1)
  {
  }
  ~CameraAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goalaction_ptr_->SendCameraGoal(num_count_);
  }
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetCameraState();
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
    {
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      private_blackboard_ptr_->SetBoolValue(true, "photo_done_flag");
      auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("robot4/run_coordinate");
      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
      dir_ptr->OpenLock();
      switch (num_count_)
      {
      case 1:
        dir_ptr->Set(4, 2, 1);
        break;
      case 2:
        dir_ptr->Set(7, 4, 4);
        break;
      case 3:
        dir_ptr->Set(5, 7, 3);
        break;
      case 4:
        dir_ptr->Set(2, 4, 2);
        break;
      default:
        ROS_ERROR("photo  number is %d !!err!", num_count_);
        break;
      }
      num_count_++;
    }
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
  uint8_t num_count_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

class DetectionAction : public ActionNode
{
public:
  DetectionAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr),
        num_count_(1)
  {
  }
  ~DetectionAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goalaction_ptr_->SendDectionGoal(num_count_);
  }
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetDetectionState();
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      num_count_++;
      ROS_INFO("%d disticion is done", num_count_);
      private_blackboard_ptr_->SetBoolValue(false, "photo_done_flag");
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
  uint8_t num_count_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

} // namespace decision
} // namespace shop

#endif