/*
 * @Description: In User Settings Edit
 * @Author: your name
 * @LastEditors: Please set LastEditors
 * @Date: 2019-03-07 21:11:54
 * @LastEditTime: 2019-03-15 22:32:15
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
  //在此调用actionlib
  virtual void OnInitialize()
  {
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key_);
    auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
    uint16_t target_x = dir_ptr->GetCoordinateX();
    uint16_t target_y = dir_ptr->GetCoordinateY();
    uint16_t target_pose = dir_ptr->GetCoordinatePOSE();
    std::string bool_key = "robot" + std::to_string(robot_num_) + "/local_plan/flag";
    ROS_INFO("%d %d %d", target_x, target_y, target_pose);
    if (target_x != 10 && target_y != 10 && target_pose != 10)
    {
      goalaction_ptr_->SendMoveGoal(robot_num_, target_x, target_y, target_pose);
      goal_flag_ = true;
    }
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
  }
  //得到状态
  virtual BehaviorState Update()
  {
    if (goal_flag_)
    {
      return goalaction_ptr_->GetMoveBehaviorState(robot_num_);
    }
    else
    {
      return BehaviorState::IDLE;
    }
  }

  virtual void OnTerminate(BehaviorState state)
  {
    // auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key_);
    // auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
    // dir_ptr->OpenLock();
    // dir_ptr->Set(10, 10, 10);
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
      //成功时候把对应坐标清理
      private_blackboard_ptr_->SetCoordValue(robot_num_, 10, 10, 10);
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
  //初始化开始发送目标
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
  //更新状态
  virtual BehaviorState Update()
  {
    if (goal_flag_)
    {
      return goalaction_ptr_->GetShopBehaviorState(robot_num_);
    }
    else
    {
      return BehaviorState::IDLE;
    }
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
      private_blackboard_ptr_->SetActionName(robot_num_, "NONE");
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

//开局动作
class OpenAction : public ActionNode
{
public:
  OpenAction(uint8_t robot_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        private_blackboard_ptr_(blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        goal_flag_(false)
  {
    // auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
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
    if (goal_flag_)
    {
      return goalaction_ptr_->GetOpenBehaviorState(robot_num_);
    }
    else
    {
      return BehaviorState::IDLE;
    }
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

//拍照动作
class CameraAction : public ActionNode
{
public:
  CameraAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
               const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~CameraAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("photo_number");
    auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(temp_dir_ptr);
    dir_ptr->OpenLock();
    dir_ptr->Set(dir_ptr->GetPhotoNumber() + 1);
    goalaction_ptr_->SendCameraGoal(dir_ptr->GetPhotoNumber());
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
      auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("photo_number");
      auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(temp_dir_ptr);
      switch (dir_ptr->GetPhotoNumber())
      {
      case 1:
        //写下一拍照的坐标
        private_blackboard_ptr_->SetCoordValue(4, 7, 4, 3);
        break;
      case 2:
        private_blackboard_ptr_->SetCoordValue(4, 5, 7, 2);
        break;
      case 3:
        private_blackboard_ptr_->SetCoordValue(4, 2, 5, 1);
        break;
      case 4:
        private_blackboard_ptr_->SetCoordValue(4, 2, 5, 0);
        break;
      default:
        ROS_ERROR("photo  number is %d !!err!", dir_ptr->GetPhotoNumber());
        break;
      }
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
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

//识别类
class DetectionAction : public ActionNode
{
public:
  DetectionAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~DetectionAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("photo_number");
    auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(temp_dir_ptr);
    goalaction_ptr_->SendDectionGoal(dir_ptr->GetPhotoNumber());
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
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

} // namespace decision
} // namespace shop

#endif