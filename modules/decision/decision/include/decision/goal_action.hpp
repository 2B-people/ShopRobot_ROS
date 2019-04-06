#ifndef GOAL_ACTION_H
#define GOAL_ACTION_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
//action
#include <data/MoveAction.h>
#include <data/OpeningAction.h>
#include <data/ShopActionAction.h>
#include <data/CameraAction.h>
#include <data/DetectionAction.h>
#include <data/LocalPlanAction.h>
#include <data/GlobalPlanAction.h>

//server
#include <data/Coordinate.h>
#include <data/ActionName.h>
#include <data/Coord.h>

#include <decision/behavior_node.hpp>
#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

#include <decision/private_board.hpp>

namespace shop
{
namespace decision
{
typedef actionlib::SimpleActionClient<data::MoveAction> MOVEACTIONCLINT;
typedef actionlib::SimpleActionClient<data::OpeningAction> OPENINGCLINT;
typedef actionlib::SimpleActionClient<data::ShopActionAction> SHOPACTION;
typedef actionlib::SimpleActionClient<data::CameraAction> CAMERAACTION;
typedef actionlib::SimpleActionClient<data::DetectionAction> DETECTION;
typedef actionlib::SimpleActionClient<data::LocalPlanAction> LOCALPLANACTION;
typedef actionlib::SimpleActionClient<data::GlobalPlanAction> GLOBALPLANACTION;

//动作任务发布的对象
class GoalAction
{
public:
  typedef std::shared_ptr<GoalAction> Ptr;

  GoalAction(const PrivateBoard::Ptr &blackboard_ptr) : private_blackboard_ptr_(blackboard_ptr),
                                                        camera_action_clint_("camera_action_server", true),
                                                        detection_clint_("detection_action_server", true),
                                                        localplan_clint_("local_plan", true),
                                                        globalplan_clint_("global_plan", true),
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
                                                        robot4_shop_action_clint_("robot4_web/shop_action", true),
                                                        robot4_open_action_clint_("robot4_web/opening_action", true)
  {
    ROS_INFO("Waiting for action server to start");

    // camera_action_clint_.waitForServer();
    // detection_clint_.waitForServer();

    localplan_clint_.waitForServer();
    globalplan_clint_.waitForServer();

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
    robot4_open_action_clint_.waitForServer();
    robot4_shop_action_clint_.waitForServer();
    ROS_INFO(" ALL Action server is started!");

    robot1_target_actionname_read_clt_ = nh_.serviceClient<data::ActionName>("shop/robot1/target_actionname_read");
    robot2_target_actionname_read_clt_ = nh_.serviceClient<data::ActionName>("shop/robot2/target_actionname_read");
    robot3_target_actionname_read_clt_ = nh_.serviceClient<data::ActionName>("shop/robot3/target_actionname_read");
    robot4_target_actionname_read_clt_ = nh_.serviceClient<data::ActionName>("shop/robot4/target_actionname_read");

    robot1_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot1/target_coordinate_read");
    robot2_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot2/target_coordinate_read");
    robot3_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot3/target_coordinate_read");
    robot4_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot4/target_coordinate_read");
    ROS_INFO("ALL Server is started!");
  }

  ~GoalAction() = default;

  data::Coord GetTargetCoord(uint8_t robot_num)
  {
    data::Coordinate srv;
    data::Coord coord;
    switch (robot_num)
    {
    case 1:
      robot1_target_coordinate_read_clt_.call(srv);
      break;
    case 2:
      robot2_target_coordinate_read_clt_.call(srv);
      break;
    case 3:
      robot3_target_coordinate_read_clt_.call(srv);
      break;
    case 4:
      robot4_target_coordinate_read_clt_.call(srv);
      break;
    default:
      ROS_ERROR("no robot num in %s", __FUNCTION__);
      coord.x = 10;
      coord.y = 10;
      coord.pose = 10;
      return coord;
      break;
    }
    coord.x = srv.response.x;
    coord.y = srv.response.y;
    coord.pose = srv.response.pose;
    return coord;
  }

  std::string GetTargetActionName(uint8_t robot_num)
  {
    data::ActionName srv;
    switch (robot_num)
    {
    case 1:
      robot1_target_actionname_read_clt_.call(srv);
      break;
    case 2:
      robot2_target_actionname_read_clt_.call(srv);
      break;
    case 3:
      robot3_target_actionname_read_clt_.call(srv);
      break;
    case 4:
      robot4_target_actionname_read_clt_.call(srv);
      break;
    default:
      ROS_ERROR("no robot in %s ", __FUNCTION__);
      return "NONE";
      break;
    }
    return srv.response.action_name;
  }

  //@brief 发送拍照目标
  void SendCameraGoal(int8_t image_num)
  {
    data::CameraGoal goal;
    goal.number = image_num;
    camera_action_clint_.sendGoal(goal,
                                  CAMERAACTION::SimpleDoneCallback(),
                                  CAMERAACTION::SimpleActiveCallback(),
                                  CAMERAACTION::SimpleFeedbackCallback());
  }

  //@brief 发送识别目标
  void SendDectionGoal(int8_t image_num)
  {
    data::DetectionGoal goal;
    goal.image_num = image_num;
    detection_clint_.sendGoal(goal,
                              DETECTION::SimpleDoneCallback(),
                              DETECTION::SimpleActiveCallback(),
                              DETECTION::SimpleFeedbackCallback());
  }

  //@brief 发送局部规划目标
  void SendLocalPlanGoal(int8_t robot_num, int8_t plan_function)
  {
    data::LocalPlanGoal goal;
    goal.robot_num = robot_num;
    goal.plan_function = plan_function;
    localplan_clint_.sendGoal(goal,
                              LOCALPLANACTION::SimpleDoneCallback(),
                              LOCALPLANACTION::SimpleActiveCallback(),
                              LOCALPLANACTION::SimpleFeedbackCallback());
  }

  //@brief 发送全局规划目标
  void SendGlobalPlanGoal(bool do_flag)
  {
    data::GlobalPlanGoal goal;
    goal.do_flag = do_flag;
    globalplan_clint_.sendGoal(goal,
                                // GLOBALPLANACTION::SimpleDoneCallback(),
                               boost::bind(&GoalAction::GlobalPlanDoneCB, this, _1,_2),
                               GLOBALPLANACTION::SimpleActiveCallback(),
                               GLOBALPLANACTION::SimpleFeedbackCallback());
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
    ROS_INFO("is here %s", __FUNCTION__);
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
  //TODO 需要写一个回调函数来实现不同车的发出 //note 应该不用
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
                                         boost::bind(&GoalAction::OpeningFB1, this, _1));
      break;
    case 2:
      robot2_open_action_clint_.sendGoal(goal,
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(),
                                         boost::bind(&GoalAction::OpeningFB2, this, _1));
      break;
    case 3:
      robot3_open_action_clint_.sendGoal(goal,
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(),
                                         boost::bind(&GoalAction::OpeningFB3, this, _1));
      break;
    case 4:
      robot4_open_action_clint_.sendGoal(goal,
                                         OPENINGCLINT::SimpleDoneCallback(),
                                         OPENINGCLINT::SimpleActiveCallback(),
                                         OPENINGCLINT::SimpleFeedbackCallback());
      break;
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
      robot4_move_action_clint_.cancelGoal();
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
      robot2_shop_action_clint_.cancelGoal();
      ROS_INFO("robot2 is cancel!");
      break;
    case 3:
      robot3_shop_action_clint_.cancelGoal();
      ROS_INFO("robot3 is cancel!");
      break;
    case 4:
      robot4_shop_action_clint_.cancelGoal();
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
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE);
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
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE);

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
    actionlib::SimpleClientGoalState state(actionlib::SimpleClientGoalState::ACTIVE);

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
    case 4:
      state = robot4_open_action_clint_.getState();
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

  //得到相机的状态
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

  //得到识别的状态
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

  //得到局部規劃的狀態
  BehaviorState GetLocalPlanState()
  {
    auto state = localplan_clint_.getState();
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

  //得到全局規劃的狀態
  BehaviorState GetGlobalPlanState()
  {
    auto state = globalplan_clint_.getState();
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
  // 拍照action
  CAMERAACTION camera_action_clint_;
  // 识别action
  DETECTION detection_clint_;
  //局部规划
  LOCALPLANACTION localplan_clint_;
  //全局規劃
  GLOBALPLANACTION globalplan_clint_;
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
  OPENINGCLINT robot4_open_action_clint_;

  //server client
  ros::ServiceClient robot1_target_actionname_read_clt_;
  ros::ServiceClient robot2_target_actionname_read_clt_;
  ros::ServiceClient robot3_target_actionname_read_clt_;
  ros::ServiceClient robot4_target_actionname_read_clt_;
  ros::ServiceClient robot1_target_coordinate_read_clt_;
  ros::ServiceClient robot2_target_coordinate_read_clt_;
  ros::ServiceClient robot3_target_coordinate_read_clt_;
  ros::ServiceClient robot4_target_coordinate_read_clt_;

  void GlobalPlanDoneCB(const actionlib::SimpleClientGoalState &state,const data::GlobalPlanResult::ConstPtr &result)
  {
    data::Coord coord;
    coord.x = result->robot1_coord[0];
    coord.y = result->robot1_coord[1];
    coord.pose = result->robot1_coord[2];
    private_blackboard_ptr_->SetCoordValue(1, coord.x, coord.y, coord.pose);

    coord.x = result->robot2_coord[0];
    coord.y = result->robot2_coord[1];
    coord.pose = result->robot2_coord[2];
    private_blackboard_ptr_->SetCoordValue(2, coord.x, coord.y, coord.pose);

    coord.x = result->robot3_coord[0];
    coord.y = result->robot3_coord[1];
    coord.pose = result->robot3_coord[2];
    private_blackboard_ptr_->SetCoordValue(3, coord.x, coord.y, coord.pose);

    coord.x = result->robot4_coord[0];
    coord.y = result->robot4_coord[1];
    coord.pose = result->robot4_coord[2];
    private_blackboard_ptr_->SetCoordValue(4, coord.x, coord.y, coord.pose);
  }

  void OpeningFB1(const data::OpeningFeedback::ConstPtr &feedback)
  {
    if (feedback->begin_flag == true)
    {
      private_blackboard_ptr_->SetBoolValue(true, "robot2_opening_flag");
    }
  }

  void OpeningFB2(const data::OpeningFeedback::ConstPtr &feedback)
  {
    if (feedback->begin_flag == true)
    {
      private_blackboard_ptr_->SetBoolValue(true, "robot3_opening_flag");
    }
  }

  void OpeningFB3(const data::OpeningFeedback::ConstPtr &feedback)
  {
    if (feedback->begin_flag == true)
    {
      private_blackboard_ptr_->SetBoolValue(true, "robot4_opening_flag");
    }
  }


};

} // namespace decision
} // namespace shop

#endif
