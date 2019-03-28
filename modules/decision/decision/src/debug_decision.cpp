#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>
#include <data/Coord.h>

//此文件用于debug,

using namespace shop::decision;
data::Coord robot1_coord_now_;

void Robo1CoordNowCB(const data::Coord::ConstPtr &msg)
{
    robot1_coord_now_.x = msg->x;
    robot1_coord_now_.y = msg->y;
    robot1_coord_now_.pose = msg->pose;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision");
    ros::NodeHandle nh;
    bool is_debug_;
    nh.param("debug", is_debug_, false);

    auto blackboard_ptr_ = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr_);

    //decision

    //执行action节点
    auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr_, goal_action_ptr);
    auto robot2_opening_ptr = std::make_shared<shop::decision::OpenAction>(2, "robot2 opening", blackboard_ptr_, goal_action_ptr);
    auto robot3_opening_ptr = std::make_shared<shop::decision::OpenAction>(3, "robot3 opening", blackboard_ptr_, goal_action_ptr);
    auto robot4_opening_ptr = std::make_shared<shop::decision::OpenAction>(4, "robot4 opening", blackboard_ptr_, goal_action_ptr);

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, "robot1 move", blackboard_ptr_, goal_action_ptr);
    auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, "robot2 move", blackboard_ptr_, goal_action_ptr);
    auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, "robot3 move", blackboard_ptr_, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, "robot4 move", blackboard_ptr_, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, "robot1 shop", blackboard_ptr_, goal_action_ptr);
    auto robot2_action_ptr = std::make_shared<shop::decision::ShopAction>(2, "robot2 shop", blackboard_ptr_, goal_action_ptr);
    auto robot3_action_ptr = std::make_shared<shop::decision::ShopAction>(3, "robot3 shop", blackboard_ptr_, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, "robot4 shop", blackboard_ptr_, goal_action_ptr);

    auto robot1_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(1, "robot1 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot2_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(2, "robot2 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot3_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(3, "robot3 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot4_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(4, "robot4 local plan", blackboard_ptr_, goal_action_ptr);

    auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo ", blackboard_ptr_, goal_action_ptr);
    auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr_, goal_action_ptr);

    // **********************************Carry action*************************************
    auto robot4_action_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4 action done", blackboard_ptr_,
                                                                                  robot4_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetBoolValue(!blackboard_ptr_->GetBoolValue("robot4/local_plan/fuc"), "robot4/local_plan/fuc");
                                                                                      blackboard_ptr_->SetBoolValue(false, "robot4/local_plan/flag");
                                                                                      return true;
                                                                                  });

    auto robot4_action_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action set", blackboard_ptr_,
                                                                                    robot4_action_done_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(4, goal_action_ptr->GetTargetActionName(4));
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::SELF);

    auto robot4_move_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 move set", blackboard_ptr_,
                                                                                  robot4_move_ptr,
                                                                                  [&]() {
                                                                                      auto coord = goal_action_ptr->GetTargetCoord(4);
                                                                                      blackboard_ptr_->SetCoordValue(4, coord.x, coord.y, coord.pose);
                                                                                      return true;
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot4_plan_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 plan jud", blackboard_ptr_,
                                                                                  robot4_local_plan_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot4/local_plan/flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 debug carry", blackboard_ptr_);
    robot4_carry_seq_ptr->AddChildren(robot4_move_set_ptr);
    robot4_carry_seq_ptr->AddChildren(robot4_action_set_ptr);

    auto robot4_action_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action jud ", blackboard_ptr_,
                                                                                  robot4_carry_seq_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot4/local_plan/flag") == false)
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_carry_jud_ptr = std::make_shared<shop::decision::SelectorNode>("robot4 carry sel", blackboard_ptr_);
    robot4_carry_jud_ptr->AddChildren(robot4_plan_jud_ptr);
    robot4_carry_jud_ptr->AddChildren(robot4_action_jud_ptr);

    auto robot4_carry_pre_ptr = std::make_shared<shop::decision::PreconditionNode>("robot carry jud", blackboard_ptr_,
                                                                                   robot4_carry_jud_ptr,
                                                                                   [&]() {
                                                                                       if (blackboard_ptr_->GetBoolValue("robot4_opening_flag"))
                                                                                       {
                                                                                           return false;
                                                                                       }
                                                                                       else
                                                                                       {
                                                                                           return true;
                                                                                       }
                                                                                   },
                                                                                   shop::decision::AbortType::LOW_PRIORITY);

    //*******************************robot4 opening what to do********************************
    auto robot4_action_T_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action T", blackboard_ptr_,
                                                                                  robot4_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetActionName(4, "T");
                                                                                      return true;
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot4_action_D_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action D", blackboard_ptr_,
                                                                                  robot4_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetActionName(4, "D");
                                                                                      return true;
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto photo_done_set_ptr = std::make_shared<shop::decision::SuccessDoNode>("photo done set flag", blackboard_ptr_,
                                                                              photo_ptr,
                                                                              [&]() {
                                                                                  blackboard_ptr_->SetBoolValue(false, "photo_done_flag");
                                                                                  return true;
                                                                              });

    auto distinguish_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("distinguish done", blackboard_ptr_,
                                                                                distinguish_ptr,
                                                                                [&]() {
                                                                                    blackboard_ptr_->SetBoolValue(false, "photo_done_flag");
                                                                                    return true;
                                                                                });

    auto distinguish_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("distinguish jud", blackboard_ptr_,
                                                                                  distinguish_done_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("photo_done_flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot4_behavior_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 opening behavior", blackboard_ptr_);
    robot4_behavior_seq_ptr->AddChildren(robot4_move_ptr);
    robot4_behavior_seq_ptr->AddChildren(robot4_action_T_ptr);
    robot4_behavior_seq_ptr->AddChildren(photo_done_set_ptr);
    robot4_behavior_seq_ptr->AddChildren(distinguish_jud_ptr);
    robot4_behavior_seq_ptr->AddChildren(robot4_action_D_ptr);

    auto robot4_cycle_ptr = std::make_shared<shop::decision::CycleNode>(4, "robot4 cycle",
                                                                        blackboard_ptr_, robot4_behavior_seq_ptr);

    auto robot4_cycle_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4 cycle done", blackboard_ptr_,
                                                                                 robot4_cycle_ptr,
                                                                                 [&]() {
                                                                                     blackboard_ptr_->SetBoolValue(false, "robot4_opening_flag");
                                                                                     return true;
                                                                                 });

    auto robot4_opening_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4 opening done", blackboard_ptr_,
                                                                                   robot4_opening_ptr,
                                                                                   [&]() {
                                                                                       blackboard_ptr_->SetCoordValue(4, 4, 2, 0);
                                                                                       return true;
                                                                                   });

    auto robot4_opening_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 open seq", blackboard_ptr_);
    robot4_opening_seq_ptr->AddChildren(robot4_opening_done_ptr);
    robot4_opening_seq_ptr->AddChildren(robot4_cycle_done_ptr);

    // 沒必要,拍照好就識別也來得及
    // auto robot_opening_par_ptr = std::make_shared<shop::decision::ParallelNode>("opening par", blackboard_ptr_, 0);
    // robot_opening_par_ptr->AddChildren(robot4_opening_seq_ptr);
    // robot_opening_par_ptr->AddChildren(distinguish_jud_ptr);

    auto robot_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("opening jud", blackboard_ptr_,
                                                                                    robot4_opening_seq_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr_->GetBoolValue("robot4_opening_flag"))
                                                                                        {
                                                                                            return true;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    // *******************************************opening test*****************************************************

    auto robot4_opening_done_test_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4_opening_done_test_ptr", blackboard_ptr_,
                                                                                        robot4_opening_ptr,
                                                                                        [&]() {
                                                                                            blackboard_ptr_->SetCoordValue(4, 2, 2, 0);
                                                                                            return true;
                                                                                        });

    auto robot4_opening_seq_test_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 open seq", blackboard_ptr_);
    robot4_opening_seq_test_ptr->AddChildren(robot4_opening_done_test_ptr);
    robot4_opening_seq_test_ptr->AddChildren(robot4_move_ptr);

    auto robot4_test_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4_test_done_ptr", blackboard_ptr_,
                                                                                robot4_opening_seq_test_ptr,
                                                                                [&]() {
                                                                                    blackboard_ptr_->SetBoolValue(false, "robot4_opening_flag");
                                                                                    return true;
                                                                                });

    auto robot_opening_jud_test_ptr = std::make_shared<shop::decision::PreconditionNode>("opening jud", blackboard_ptr_,
                                                                                         robot4_test_done_ptr,
                                                                                         [&]() {
                                                                                             if (blackboard_ptr_->GetBoolValue("robot4_opening_flag"))
                                                                                             {
                                                                                                 return true;
                                                                                             }
                                                                                             else
                                                                                             {
                                                                                                 return false;
                                                                                             }
                                                                                         },
                                                                                         shop::decision::AbortType::LOW_PRIORITY);
// *********************************************************************************************************


    auto debug_game_sel_ptr = std::make_shared<shop::decision::SelectorNode>("test", blackboard_ptr_);
    debug_game_sel_ptr->AddChildren(robot_opening_jud_test_ptr);
    debug_game_sel_ptr->AddChildren(robot4_carry_pre_ptr);

    blackboard_ptr_->SetBoolValue(true, "robot4_opening_flag");

    shop::decision::BehaviorTree se(debug_game_sel_ptr, 20);
    se.Execute();
}