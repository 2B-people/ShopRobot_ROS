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
  MoveAction(int8_t robot_num, int8_t move_function, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        move_function_(move_function),
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
    goal_flag_ = false;
  }
  //得到状态
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendMoveGoal(robot_num_, 10, 10, move_function_);
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetMoveBehaviorState(robot_num_);
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
      if (move_function_ == 2)
      {
        // goalaction_ptr_->SetTargetActionName(4, "T");
        ros::Rate r(5); //10HZ
        r.sleep();
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

  int8_t robot_num_;
  bool goal_flag_;
  int8_t move_function_;

  std::string coordinate_key_;
  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
};

//动作节点
class ShopAction : public ActionNode
{
public:
  ShopAction(uint8_t robot_num, int8_t function_flag, std::string name, const PrivateBoard::Ptr &blackboard_ptr,
             const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        function_flag_(function_flag),
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
    goal_flag_ = false;
  }
  //更新状态
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendShopGoal(robot_num_, "NONE");
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetShopBehaviorState(robot_num_);
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
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
    // goalaction_ptr_->SetTargetActionName(robot_num_, "NONE");
  }

  // data
  uint8_t robot_num_;
  std::string name_key_;
  bool goal_flag_;
  int8_t function_flag_;

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
    flag_name_ = "robot" + std::to_string(robot_num) + "_opening_flag";
  }

  ~OpenAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goal_flag_ = false;
  }

  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendOpenGoal(robot_num_, "go");
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetOpenBehaviorState(robot_num_);
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
      // 單車測試不用這個
      private_blackboard_ptr_->SetBoolValue(false, flag_name_);
      if (robot_num_ == 4)
      {
        private_blackboard_ptr_->SetBoolValue(true, "robot4_photo_flag");
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
        number_(0),
        goal_flag_(false),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~CameraAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goal_flag_ = false;
  }
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendCameraGoal(1);
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetCameraState();
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
    {
      ROS_INFO("%s %s SUCCESS", name_.c_str(), __FUNCTION__);
      data::Coord coord;
      number_++;
      switch (number_)
      {
      case 1:
        coord.x = 7;
        coord.y = 4;
        goalaction_ptr_->SetTargetCoord(4, coord);
        goalaction_ptr_->SetTargetActionName(4, "D");
        break;
      case 2:
        coord.x = 5;
        coord.y = 7;
        goalaction_ptr_->SetTargetCoord(4, coord);
        goalaction_ptr_->SetTargetActionName(4, "D");
        break;
      case 3:
        coord.x = 2;
        coord.y = 5;
        goalaction_ptr_->SetTargetCoord(4, coord);
        goalaction_ptr_->SetTargetActionName(4, "D");
        break;
      case 4:
        goalaction_ptr_->SetTargetActionName(4, "D");
        blackboard_ptr_->SetBoolValue(false, "opening_flag");
        // private_blackboard_ptr_->SetBoolValue(false, "robot4_photo_flag");
        break;
      default:
        ROS_ERROR("photo number is!!err!");
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
    ros::Rate r(5); //10HZ
    r.sleep();
    return;
  }
  bool goal_flag_;
  uint8_t robot_num_;
  int8_t number_;
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
        goal_flag_(false),
        goalaction_ptr_(goalaction_ptr),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~DetectionAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goal_flag_ = false;
  }
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendDectionGoal(1);
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetDetectionState();
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

//局部规划action类
class LocalPlanAction : public ActionNode
{
public:
  LocalPlanAction(std::string name, const PrivateBoard::Ptr &blackboard_ptr,
                  const GoalAction::Ptr &goalaction_ptr)
      : ActionNode(name, blackboard_ptr),
        goal_flag_(false),
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
    goal_flag_ = false;
  }
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendLocalPlanGoal(1, 1);
      goal_flag_ = true;
      return BehaviorState::RUNNING;
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
        goal_flag_(false),
        private_blackboard_ptr_(blackboard_ptr)
  {
  }
  ~GlobalPlanAction() = default;

private:
  virtual void OnInitialize()
  {
    ROS_INFO("%s is %s", name_.c_str(), __FUNCTION__);
    goal_flag_ = false;
  }
  virtual BehaviorState Update()
  {
    //判斷任務發布狀態
    if (goal_flag_ == false)
    {
      goalaction_ptr_->SendGlobalPlanGoal(true);
      goal_flag_ = true;
      return BehaviorState::RUNNING;
    }
    else
    {
      return goalaction_ptr_->GetGlobalPlanState();
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
      break;
    case BehaviorState::FAILURE:
      ROS_INFO("%s %s FAILURE", name_.c_str(), __FUNCTION__);
      break;
    default:
      ROS_ERROR("%s is err", name_.c_str());
      return;
    }
    ros::Rate r(5); //10HZ
    r.sleep();
    return;
  }

  GoalAction::Ptr goalaction_ptr_;
  PrivateBoard::Ptr private_blackboard_ptr_;
  bool goal_flag_;
};

} // namespace decision
} // namespace shop

#endif