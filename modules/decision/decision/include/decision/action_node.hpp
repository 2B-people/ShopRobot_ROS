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
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(coordinate_key_);
    auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
    // 得坐標
    uint16_t target_x = dir_ptr->GetCoordinateX();
    uint16_t target_y = dir_ptr->GetCoordinateY();
    uint16_t target_pose = dir_ptr->GetCoordinatePOSE();

    ROS_INFO("%d %d %d", target_x, target_y, target_pose);
    // 發布目標
    if (target_x != 10 && target_y != 10 && target_pose != 10)
    {
      goalaction_ptr_->SendMoveGoal(robot_num_, target_x, target_y, target_pose);
      goal_flag_ = true;
    }
  }
  //得到状态
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_)
    {
      return goalaction_ptr_->GetMoveBehaviorState(robot_num_);
    }
    else
    {
      // 掛起
      return BehaviorState::IDLE;
    }
  }

  virtual void OnTerminate(BehaviorState state)
  {
    switch (state)
    {
    case BehaviorState::IDLE:
      ROS_INFO("%s %s IDLE", name_.c_str(), __FUNCTION__);
      // 調用reset，時候使用stop
      if (goal_flag_)
      {
        goalaction_ptr_->CancelMoveGoal(robot_num_);
        // 把对应坐标清理
        private_blackboard_ptr_->SetCoordValue(robot_num_, 10, 10, 10);
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
    name_key_ = "robot" + std::to_string(robot_num_) + "/action_name";
  }

  ~ShopAction() = default;

private:
  //初始化开始发送目标
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    // 得目標
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr(name_key_);
    auto dir_ptr = std::dynamic_pointer_cast<ActionNameDir>(temp_dir_ptr);
    std::string goal_name = dir_ptr->GetActionName();
    // 發送目標
    if (goal_name != "NONE")
    {
      goalaction_ptr_->SendShopGoal(robot_num_, goal_name);
      goal_flag_ = true;
    }
  }
  //更新状态
  virtual BehaviorState Update()
  {
    // 判斷目標狀態
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
        private_blackboard_ptr_->SetActionName(robot_num_, "NONE");
        goal_flag_ = false;
      }
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      // 成功時請空
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

  // data
  uint8_t robot_num_;
  std::string name_key_;
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
    flag_name_ = "robot" + std::to_string(robot_num) + "_opening_flag";
  }

  ~OpenAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    // 發送目標
    goalaction_ptr_->SendOpenGoal(robot_num_, "go");
    goal_flag_ = true;
  }

  virtual BehaviorState Update()
  {
    // 判斷目標狀態
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
      // 調用resetstop目標
      if (goal_flag_)
      {
        goalaction_ptr_->CancelShopGoal(robot_num_);
        goal_flag_ = false;
      }
      break;
    case BehaviorState::SUCCESS:
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      private_blackboard_ptr_->SetBoolValue(false,flag_name_);
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
  }

  std::string flag_name_;
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
    //发送目标照片
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("photo_number");
    auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(temp_dir_ptr);
    dir_ptr->OpenLock();
    // 這是爲了照片加1
    dir_ptr->Set(dir_ptr->GetPhotoNumber() + 1);
    // 發送目標
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
      // 可以識別flag
      private_blackboard_ptr_->SetBoolValue(true, "photo_done_flag");
      // 更新坐標
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
        ROS_ERROR("photo number is %d !!err!", dir_ptr->GetPhotoNumber());
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
    // 得到識別照片數
    auto temp_dir_ptr = private_blackboard_ptr_->GetDirPtr("photo_number");
    auto dir_ptr = std::dynamic_pointer_cast<PhotoNemberDir>(temp_dir_ptr);
    // 識別開始
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
      // 等待下一次識別
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

//局部规划action类
class LocalPlanAction : public ActionNode
{
public:
  LocalPlanAction(int8_t robot_num, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        robot_num_(robot_num),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
    flag_name_key_ = "robot" + std::to_string(robot_num_) + "/local_plan/flag";
    fuc_name_key_ = "robot" + std::to_string(robot_num_) + "/local_plan/fuc";
  }
  ~LocalPlanAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    //正在規劃or等待規劃
    private_blackboard_ptr_->SetBoolValue(false, flag_name_key_);
    //funcname:1为规划取物,2为放物
    // 如果有其他車在用局部規劃，此處不可發坐標
    if (private_blackboard_ptr_->GetBoolValue("local_plan_run") == false)
    {
      if (private_blackboard_ptr_->GetBoolValue(fuc_name_key_) == false)
      {
        goalaction_ptr_->SendLocalPlanGoal(robot_num_, 1);
      }
      else
      {
        goalaction_ptr_->SendLocalPlanGoal(robot_num_, 2);
      }
      private_blackboard_ptr_->SetBoolValue(true, "local_plan_run");
    }
    else
    {
      ROS_INFO("%s is writ for local plan", name_.c_str());
    }
  }
  virtual BehaviorState Update()
  {
    // 此處的失敗了
    if (private_blackboard_ptr_->GetBoolValue("local_plan_run") == false)
    {
      return BehaviorState::FAILURE;
    }
    else
    {
      return goalaction_ptr_->GetLocalPlanState();
    }
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
      // 可以執行動作了
      private_blackboard_ptr_->SetBoolValue(true, flag_name_key_);
      // 其他局部規劃節點可以使用
      private_blackboard_ptr_->SetBoolValue(false, "local_plan_run");
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
  std::string flag_name_key_;
  std::string fuc_name_key_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

//全局规划action类
class GlobalPlanAction : public ActionNode
{
public:
  GlobalPlanAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~GlobalPlanAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goalaction_ptr_->SendGlobalPlanGoal(true);
  }
  virtual BehaviorState Update()
  {
    return goalaction_ptr_->GetGlobalPlanState();
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
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
  }

  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

} // namespace decision
} // namespace shop

#endif