#ifndef GOAL_ACTION_H
#define GOAL_ACTION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <string>

#include <data/MoveAction.h>
#include <data/OpeningAction.h>
#include <data/ShopActionAction.h>
#include <data/CameraAction.h>
#include <data/DetectionAction.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

namespace shop
{
namespace decision
{
typedef actionlib::SimpleActionClient<data::MoveAction> MOVEACTIONCLINT;
typedef actionlib::SimpleActionClient<data::OpeningAction> OPENINGCLINT;
typedef actionlib::SimpleActionClient<data::ShopActionAction> SHOPACTION;
typedef actionlib::SimpleActionClient<data::CameraAction> CAMERAACTION;
typedef actionlib::SimpleActionClient<data::DetectionAction> DETECTION;

//行为树专用黑板,
class PrivateBoard : public Blackboard
{
public:
  typedef std::shared_ptr<PrivateBoard> Ptr;
  PrivateBoard()
      : Blackboard()
  {
    //game结束的flag
    auto end_game = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("end_flag", end_game);

    //各类flag
    auto robot1_opeing_flag_ptr = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("robot1_opeing_flag", robot1_opeing_flag_ptr);

    auto robot2_opeing_flag_ptr = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("robot2_opeing_flag", robot2_opeing_flag_ptr);


    auto robot3_opeing_flag_ptr = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("robot3_opeing_flag", robot3_opeing_flag_ptr);

    auto robot4_opeing_flag_ptr = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("robot4_opeing_flag", robot4_opeing_flag_ptr);
    
    //拍照成功flag
    auto photo_done_flag_ptr = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("photo_done_flag", photo_done_flag_ptr);

    //局部规划的flag
    auto robot1_local_plan_flag = std::make_shared<BoolDir>(false);
    auto robot2_local_plan_flag = std::make_shared<BoolDir>(false);
    auto robot3_local_plan_flag = std::make_shared<BoolDir>(false);
    auto robot4_local_plan_flag = std::make_shared<BoolDir>(false);
    AddDataIntoWorld("robot1/local_plan/flag", robot1_local_plan_flag);
    AddDataIntoWorld("robot2/local_plan/flag", robot2_local_plan_flag);
    AddDataIntoWorld("robot3/local_plan/flag", robot3_local_plan_flag);
    AddDataIntoWorld("robot4/local_plan/flag", robot4_local_plan_flag);

    //可以在任何地方写的目标坐标
    auto robot1_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot2_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot3_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
    auto robot4_run_coordinate = std::make_shared<CoordinateDir>(10, 10, 10);
    AddDataIntoWorld("robot1/run_coordinate", robot1_run_coordinate);
    AddDataIntoWorld("robot2/run_coordinate", robot2_run_coordinate);
    AddDataIntoWorld("robot3/run_coordinate", robot3_run_coordinate);
    AddDataIntoWorld("robot4/run_coordinate", robot4_run_coordinate);

    //@breif 写在全局黑板中了
    //目标动作
    // auto robot1_action_name = std::make_shared<ActionNameDir>("NONE");
    // auto robot2_action_name = std::make_shared<ActionNameDir>("NONE");
    // auto robot3_action_name = std::make_shared<ActionNameDir>("NONE");
    // auto robot4_action_name = std::make_shared<ActionNameDir>("NONE");
    // AddDataIntoWorld("robot1/action_name", robot1_action_name);
    // AddDataIntoWorld("robot2/action_name", robot2_action_name);
    // AddDataIntoWorld("robot3/action_name", robot3_action_name);
    // AddDataIntoWorld("robot4/action_name", robot4_action_name);
  }
  ~PrivateBoard() = default;

  bool GetBoolValue(std::string key)
  {
    auto dir_ptr = GetDirPtr(key);
    auto bool_dir_ptr = std::dynamic_pointer_cast<BoolDir>(dir_ptr);
    return bool_dir_ptr->GetValue();
  }

  void SetBoolValue(bool set_bool, std::string key)
  {
    auto dir_ptr = GetDirPtr(key);
    auto bool_dir_ptr = std::dynamic_pointer_cast<BoolDir>(dir_ptr);
    bool_dir_ptr->OpenLock();
    bool_dir_ptr->Set(set_bool);
  }

private:
};

//动作任务发布的对象
class GoalAction
{
public:
  typedef std::shared_ptr<GoalAction> Ptr;

  GoalAction(const Blackboard::Ptr &blackboard_ptr) : camera_action_clint_("camera_action_server", true),
                                                      detection_clint_("detection_action_server", true),
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
    auto private_blackboard_ptr_ = std::dynamic_pointer_cast<PrivateBoard>(blackboard_ptr);
    camera_action_clint_.waitForServer();
    // detection_clint_.waitForServer();
    robot1_move_action_clint_.waitForServer();
    robot1_open_action_clint_.waitForServer();
    robot1_shop_action_clint_.waitForServer();
    // robot2_move_action_clint_.waitForServer();
    // robot2_open_action_clint_.waitForServer();
    // robot2_shop_action_clint_.waitForServer();
    // robot3_move_action_clint_.waitForServer();
    // robot3_open_action_clint_.waitForServer();
    // robot3_shop_action_clint_.waitForServer();
    // robot4_move_action_clint_.waitForServer();
    // robot4_shop_action_clint_.waitForServer();
    ROS_INFO(" ALL Action server is started");
  }
  ~GoalAction() = default;

  void SendCameraGoal(int8_t image_num)
  {
    data::CameraGoal goal;
    goal.number = image_num;
    camera_action_clint_.sendGoal(goal,
                                  CAMERAACTION::SimpleDoneCallback(),
                                  CAMERAACTION::SimpleActiveCallback(),
                                  CAMERAACTION::SimpleFeedbackCallback());
  }

  void SendDectionGoal(int8_t image_num)
  {
    data::DetectionGoal goal;
    goal.image_num = image_num;
    detection_clint_.sendGoal(goal,
                              DETECTION::SimpleDoneCallback(),
                              DETECTION::SimpleActiveCallback(),
                              DETECTION::SimpleFeedbackCallback());
  }

  // @brief 发布车移动目标
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
                                         MOVEACTIONCLINT::SimpleFeedbackCallback());
      break;
    case 4:
      robot4_move_action_clint_.sendGoal(goal,
                                         MOVEACTIONCLINT::SimpleDoneCallback(),
                                         MOVEACTIONCLINT::SimpleActiveCallback(),
                                         MOVEACTIONCLINT::SimpleFeedbackCallback());
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
    ROS_INFO("is here %s",__FUNCTION__);

  }

  // @brief 发布车动作目标
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
                                         SHOPACTION::SimpleFeedbackCallback());
      break;
    case 2:
      robot2_shop_action_clint_.sendGoal(goal,
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(),
                                         SHOPACTION::SimpleFeedbackCallback());
      break;
    case 3:
      robot3_shop_action_clint_.sendGoal(goal,
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(),
                                         SHOPACTION::SimpleFeedbackCallback());
      break;
    case 4:
      robot4_shop_action_clint_.sendGoal(goal,
                                         SHOPACTION::SimpleDoneCallback(),
                                         SHOPACTION::SimpleActiveCallback(),
                                         SHOPACTION::SimpleFeedbackCallback());
      break;
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  // @brief 发布发车命令
  // @note 无robot4,robot4为上位机控制
  //TODO 需要写一个回调函数来实现不同车的发出
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
                                         OPENINGCLINT::SimpleFeedbackCallback());
    case 2:
      robot2_open_action_clint_.sendGoal(goal,
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(),
                                         OPENINGCLINT::SimpleFeedbackCallback());
      break;
    case 3:
      robot2_open_action_clint_.sendGoal(goal,
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(),
                                         OPENINGCLINT::SimpleFeedbackCallback());
    default:
      ROS_ERROR("%s no num %d in robot_num", __FUNCTION__, (int)robot_num);
      break;
    }
  }

  // @brief 紧急停车
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

  // @brief 紧急停止动作
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

  // @brief 得到move action的状态
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

  // @brief 得到shop action的状态
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

  // @brief 得到open action的状态
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

  BehaviorState GetCameraState()
  {
    auto state = camera_action_clint_.getState();
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

  BehaviorState GetDetectionState()
  {
    auto state = detection_clint_.getState();
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
  PrivateBoard::Ptr private_blackboard_ptr_;

  ros::NodeHandle nh_;

  CAMERAACTION camera_action_clint_;
  DETECTION detection_clint_;

  //robot1
  MOVEACTIONCLINT robot1_move_action_clint_;
  OPENINGCLINT robot1_open_action_clint_;
  SHOPACTION robot1_shop_action_clint_;
  //robot2
  MOVEACTIONCLINT robot2_move_action_clint_;
  OPENINGCLINT robot2_open_action_clint_;
  SHOPACTION robot2_shop_action_clint_;
  //robot3
  MOVEACTIONCLINT robot3_move_action_clint_;
  OPENINGCLINT robot3_open_action_clint_;
  SHOPACTION robot3_shop_action_clint_;
  //robot4
  MOVEACTIONCLINT robot4_move_action_clint_;
  SHOPACTION robot4_shop_action_clint_;
};

} // namespace decision
} // namespace shop

#endif
