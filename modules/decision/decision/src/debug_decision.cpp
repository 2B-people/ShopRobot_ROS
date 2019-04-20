#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

#include <blackboard/black_board.hpp>

#include <data/Coord.h>

//此文件用于debug,

using namespace shop::decision;

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

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, 1, "robot1 move", blackboard_ptr_, goal_action_ptr);
    auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, 1, "robot2 move", blackboard_ptr_, goal_action_ptr);
    auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, 1, "robot3 move", blackboard_ptr_, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, 1, "robot4 move", blackboard_ptr_, goal_action_ptr);

    auto robot4_special_move_ptr = std::make_shared<shop::decision::MoveAction>(4, 2, "robot4 special move", blackboard_ptr_, goal_action_ptr);
    auto robot4_special_action_ptr = std::make_shared<shop::decision::ShopAction>(4, 1, "robot4 special shop", blackboard_ptr_, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, 0, "robot1 shop", blackboard_ptr_, goal_action_ptr);
    auto robot2_action_ptr = std::make_shared<shop::decision::ShopAction>(2, 0, "robot2 shop", blackboard_ptr_, goal_action_ptr);
    auto robot3_action_ptr = std::make_shared<shop::decision::ShopAction>(3, 0, "robot3 shop", blackboard_ptr_, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, 0, "robot4 shop", blackboard_ptr_, goal_action_ptr);

    auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo", blackboard_ptr_, goal_action_ptr);
    auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr_, goal_action_ptr);
    auto global_plan_ptr = std::make_shared<shop::decision::GlobalPlanAction>("global plan", blackboard_ptr_, goal_action_ptr);
    auto local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>("local plan", blackboard_ptr_, goal_action_ptr);

    auto robot1_open_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 open jud", blackboard_ptr_,
                                                                                  robot1_opening_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot1_opening_flag") == true)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot2_open_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 open jud", blackboard_ptr_,
                                                                                  robot2_opening_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot2_opening_flag") == true)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot3_open_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 open jud", blackboard_ptr_,
                                                                                  robot3_opening_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot3_opening_flag") == true)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_open_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 open jud", blackboard_ptr_,
                                                                                  robot4_opening_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot4_opening_flag") == true)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_photo_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 photo seq", blackboard_ptr_);
    robot4_photo_seq_ptr->AddChildren(robot4_special_move_ptr);
    robot4_photo_seq_ptr->AddChildren(robot4_special_action_ptr);
    robot4_photo_seq_ptr->AddChildren(photo_ptr);
    robot4_photo_seq_ptr->AddChildren(robot4_special_action_ptr);

    auto robot4_test_success_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4 test ptr", blackboard_ptr_,
                                                                                        robot4_photo_seq_ptr,
                                                                                        [&]() {
                                                                                            if (blackboard_ptr_->GetBoolValue("robot4_photo_flag") == false)
                                                                                                blackboard_ptr_->SetBoolValue(false, "opening_flag");
                                                                                            return true;
                                                                                        });

    auto robot4_photo_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 photo jud", blackboard_ptr_,
                                                                                   robot4_photo_seq_ptr,
                                                                                //    robot4_test_success_done_ptr,
                                                                                   [&]() {
                                                                                       if (blackboard_ptr_->GetBoolValue("robot4_photo_flag") == true)
                                                                                       {
                                                                                           return true;
                                                                                       }
                                                                                       else
                                                                                       {
                                                                                           return false;
                                                                                       }
                                                                                   },
                                                                                   shop::decision::AbortType::LOW_PRIORITY);

    auto open_while_ptr = std::make_shared<shop::decision::WhileNode>("open while", blackboard_ptr_);
    open_while_ptr->AddChildren(distinguish_ptr);
    open_while_ptr->AddChildren(robot1_open_jud_ptr);
    open_while_ptr->AddChildren(robot4_open_jud_ptr);
    open_while_ptr->AddChildren(robot4_photo_jud_ptr);
    // open_while_ptr->AddChildren(robot2_open_jud_ptr);
    // open_while_ptr->AddChildren(robot3_open_jud_ptr);

    auto open_while_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("open while jud", blackboard_ptr_,
                                                                                 open_while_ptr,
                                                                                 [&]() {
                                                                                     if (blackboard_ptr_->GetBoolValue("opening_flag") == true)
                                                                                     {
                                                                                         return true;
                                                                                     }
                                                                                     else
                                                                                     {
                                                                                         return false;
                                                                                     }
                                                                                 },
                                                                                 shop::decision::AbortType::BOTH);

    auto robot1_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot1 carry seq", blackboard_ptr_);
    robot1_carry_seq_ptr->AddChildren(robot1_move_ptr);
    robot1_carry_seq_ptr->AddChildren(robot1_action_ptr);
    auto robot4_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 carry seq", blackboard_ptr_);
    robot4_carry_seq_ptr->AddChildren(robot4_move_ptr);
    robot4_carry_seq_ptr->AddChildren(robot4_action_ptr);
#if 0
    auto robot2_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot2 carry seq", blackboard_ptr_);
    robot2_carry_seq_ptr->AddChildren(robot2_move_ptr);
    robot2_carry_seq_ptr->AddChildren(robot2_action_ptr);

    auto robot3_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot3 carry seq", blackboard_ptr_);
    robot3_carry_seq_ptr->AddChildren(robot3_move_ptr);
    robot3_carry_seq_ptr->AddChildren(robot3_action_ptr);
#endif

    auto plan_seq_ptr = std::make_shared<shop::decision::SequenceNode>("plan seq", blackboard_ptr_);
    plan_seq_ptr->AddChildren(local_plan_ptr);
    plan_seq_ptr->AddChildren(global_plan_ptr);

    auto carry_while_ptr = std::make_shared<shop::decision::WhileNode>("carry while", blackboard_ptr_);
    carry_while_ptr->AddChildren(plan_seq_ptr);
    carry_while_ptr->AddChildren(robot1_carry_seq_ptr);
    carry_while_ptr->AddChildren(robot4_carry_seq_ptr);
    // carry_while_ptr->AddChildren(robot2_carry_seq_ptr);
    // carry_while_ptr->AddChildren(robot3_carry_seq_ptr);

    auto carrt_while_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("carry while jud", blackboard_ptr_,
                                                                                  carry_while_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("opening_flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::BOTH);

    auto final_jud_ptr = std::make_shared<shop::decision::SelectorNode>("final jud", blackboard_ptr_);
    final_jud_ptr->AddChildren(open_while_jud_ptr);
    final_jud_ptr->AddChildren(carrt_while_jud_ptr);

    data::Coord coord;
    coord.x = 4;
    coord.y = 2;
    goal_action_ptr->SetTargetCoord(4, coord);
    goal_action_ptr->SetTargetActionName(4, "T");
    blackboard_ptr_->SetBoolValue(true, "robot1_opening_flag");
    blackboard_ptr_->SetBoolValue(true, "opening_flag");

    ros::Rate r(2); //10HZ
    r.sleep();

    auto sh = BehaviorTree(final_jud_ptr, 10);

    sh.Execute();
}