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
    auto global_plan_ptr = std::make_shared<shop::decision::GlobalPlanAction>("global plan", blackboard_ptr_, goal_action_ptr);

    //
    //
    //
    //
    // ********************************************Robot4 Carry action*************************************************************************/
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
                                                                                      auto temp_dir_ptr = blackboard_ptr_->GetDirPtr("robot4/run_coordinate");
                                                                                      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
                                                                                      if (coord.x == dir_ptr->GetCoordinateX() && coord.y == dir_ptr->GetCoordinateY())
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
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

    auto robot4_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 carry", blackboard_ptr_);
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
    /******************************************************************************************************************************************/

    //
    //
    //
    //
    // ********************************************Robot3 Carry action*************************************************************************/
    auto robot3_action_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot3 action done", blackboard_ptr_,
                                                                                  robot3_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetBoolValue(!blackboard_ptr_->GetBoolValue("robot3/local_plan/fuc"), "robot3/local_plan/fuc");
                                                                                      blackboard_ptr_->SetBoolValue(false, "robot3/local_plan/flag");
                                                                                      return true;
                                                                                  });

    auto robot3_action_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 action set", blackboard_ptr_,
                                                                                    robot3_action_done_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(3, goal_action_ptr->GetTargetActionName(3));
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::SELF);

    auto robot3_move_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 move set", blackboard_ptr_,
                                                                                  robot3_move_ptr,
                                                                                  [&]() {
                                                                                      auto coord = goal_action_ptr->GetTargetCoord(3);
                                                                                      auto temp_dir_ptr = blackboard_ptr_->GetDirPtr("robot3/run_coordinate");
                                                                                      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
                                                                                      if (coord.x == dir_ptr->GetCoordinateX() && coord.y == dir_ptr->GetCoordinateY())
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      };
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot3_plan_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 plan jud", blackboard_ptr_,
                                                                                  robot3_local_plan_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot3/local_plan/flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot3_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot3 carry", blackboard_ptr_);
    robot3_carry_seq_ptr->AddChildren(robot3_move_set_ptr);
    robot3_carry_seq_ptr->AddChildren(robot3_action_set_ptr);

    auto robot3_action_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 action jud ", blackboard_ptr_,
                                                                                    robot3_carry_seq_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr_->GetBoolValue("robot3/local_plan/flag") == false)
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return true;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot3_carry_jud_ptr = std::make_shared<shop::decision::SelectorNode>("robot3 carry sel", blackboard_ptr_);
    robot3_carry_jud_ptr->AddChildren(robot3_plan_jud_ptr);
    robot3_carry_jud_ptr->AddChildren(robot3_action_jud_ptr);

    auto robot3_carry_pre_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 carry jud", blackboard_ptr_,
                                                                                   robot3_carry_jud_ptr,
                                                                                   [&]() {
                                                                                       if (blackboard_ptr_->GetBoolValue("robot3_opening_flag"))
                                                                                       {
                                                                                           return false;
                                                                                       }
                                                                                       else
                                                                                       {
                                                                                           return true;
                                                                                       }
                                                                                   },
                                                                                   shop::decision::AbortType::LOW_PRIORITY);
    /*********************************************************************************************/
    //
    //
    //
    //

    // ********************************************Robot2 Carry action*************************************************************************/
    auto robot2_action_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot2 action done", blackboard_ptr_,
                                                                                  robot2_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetBoolValue(!blackboard_ptr_->GetBoolValue("robot2/local_plan/fuc"), "robot2/local_plan/fuc");
                                                                                      blackboard_ptr_->SetBoolValue(false, "robot2/local_plan/flag");
                                                                                      return true;
                                                                                  });

    auto robot2_action_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 action set", blackboard_ptr_,
                                                                                    robot2_action_done_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(2, goal_action_ptr->GetTargetActionName(2));
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::SELF);

    auto robot2_move_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 move set", blackboard_ptr_,
                                                                                  robot2_move_ptr,
                                                                                  [&]() {
                                                                                      auto coord = goal_action_ptr->GetTargetCoord(2);
                                                                                      auto temp_dir_ptr = blackboard_ptr_->GetDirPtr("robot2/run_coordinate");
                                                                                      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
                                                                                      if (coord.x == dir_ptr->GetCoordinateX() && coord.y == dir_ptr->GetCoordinateY())
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot2_plan_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 plan jud", blackboard_ptr_,
                                                                                  robot2_local_plan_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot2/local_plan/flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot2_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot2 carry", blackboard_ptr_);
    robot2_carry_seq_ptr->AddChildren(robot2_move_set_ptr);
    robot2_carry_seq_ptr->AddChildren(robot2_action_set_ptr);

    auto robot2_action_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 action jud ", blackboard_ptr_,
                                                                                    robot2_carry_seq_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr_->GetBoolValue("robot2/local_plan/flag") == false)
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return true;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot2_carry_jud_ptr = std::make_shared<shop::decision::SelectorNode>("robot2 carry sel", blackboard_ptr_);
    robot2_carry_jud_ptr->AddChildren(robot2_plan_jud_ptr);
    robot2_carry_jud_ptr->AddChildren(robot2_action_jud_ptr);

    /*********************************************************************************************/

    //
    //
    //
    //
    //

    // ********************************************Robot1 Carry action*************************************************************************/
    auto robot1_action_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot1 action done", blackboard_ptr_,
                                                                                  robot1_action_ptr,
                                                                                  [&]() {
                                                                                      blackboard_ptr_->SetBoolValue(!blackboard_ptr_->GetBoolValue("robot1/local_plan/fuc"), "robot1/local_plan/fuc");
                                                                                      blackboard_ptr_->SetBoolValue(false, "robot1/local_plan/flag");
                                                                                      return true;
                                                                                  });

    auto robot1_action_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 action set", blackboard_ptr_,
                                                                                    robot1_action_done_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(1, goal_action_ptr->GetTargetActionName(1));
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::SELF);

    auto robot1_move_set_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 move set", blackboard_ptr_,
                                                                                  robot1_move_ptr,
                                                                                  [&]() {
                                                                                      auto coord = goal_action_ptr->GetTargetCoord(1);
                                                                                      auto temp_dir_ptr = blackboard_ptr_->GetDirPtr("robot1/run_coordinate");
                                                                                      auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
                                                                                      if (coord.x == dir_ptr->GetCoordinateX() && coord.y == dir_ptr->GetCoordinateY())
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::SELF);

    auto robot1_plan_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 plan jud", blackboard_ptr_,
                                                                                  robot1_local_plan_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr_->GetBoolValue("robot1/local_plan/flag") == false)
                                                                                      {
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot1_carry_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot1 carry", blackboard_ptr_);
    robot1_carry_seq_ptr->AddChildren(robot1_move_set_ptr);
    robot1_carry_seq_ptr->AddChildren(robot1_action_set_ptr);

    auto robot1_action_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 action jud ", blackboard_ptr_,
                                                                                    robot1_carry_seq_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr_->GetBoolValue("robot1/local_plan/flag") == false)
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return true;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot1_carry_jud_ptr = std::make_shared<shop::decision::SelectorNode>("robot1 carry sel", blackboard_ptr_);
    robot1_carry_jud_ptr->AddChildren(robot1_plan_jud_ptr);
    robot1_carry_jud_ptr->AddChildren(robot1_action_jud_ptr);

    /*********************************************************************************************/

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
    auto robot1_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 opening jud", blackboard_ptr_,
                                                                                     robot1_opening_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr_->GetBoolValue("robot1_opening_flag"))
                                                                                         {
                                                                                             return true;
                                                                                         }
                                                                                         else
                                                                                         {
                                                                                             return false;
                                                                                         }
                                                                                     },
                                                                                     shop::decision::AbortType::LOW_PRIORITY);

    auto robot2_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 opening jud", blackboard_ptr_,
                                                                                     robot2_opening_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr_->GetBoolValue("robot2_opening_flag"))
                                                                                         {
                                                                                             return true;
                                                                                         }
                                                                                         else
                                                                                         {
                                                                                             return false;
                                                                                         }
                                                                                     },
                                                                                     shop::decision::AbortType::LOW_PRIORITY);
    auto robot3_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 opening jud", blackboard_ptr_,
                                                                                     robot3_opening_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr_->GetBoolValue("robot3_opening_flag"))
                                                                                         {
                                                                                             return true;
                                                                                         }
                                                                                         else
                                                                                         {
                                                                                             return false;
                                                                                         }
                                                                                     },
                                                                                     shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_opening_test_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("robot4 opening done", blackboard_ptr_,
                                                                                        robot4_opening_ptr,
                                                                                        [&]() {
                                                                                            blackboard_ptr_->SetCoordValue(4, 2, 2, 0);
                                                                                            return true;
                                                                                        });

    auto robot4_opening_seq_test_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 open seq", blackboard_ptr_);
    robot4_opening_seq_test_ptr->AddChildren(robot4_opening_test_done_ptr);
    robot4_opening_seq_test_ptr->AddChildren(robot4_move_ptr);

    auto robot4_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 s jud", blackboard_ptr_,
                                                                                     robot4_opening_seq_test_ptr,
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

    auto open_behavior_ptr = std::make_shared<shop::decision::ParallelNode>("open behavior", blackboard_ptr_, 4);

    open_behavior_ptr->AddChildren(robot1_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot2_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot3_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot4_opening_jud_ptr);

    auto open_behavior_success_ptr = std::make_shared<shop::decision::SuccessDoNode>("open_behavior_success", blackboard_ptr_,
                                                                                     open_behavior_ptr,
                                                                                     [&]() {
                                                                                         blackboard_ptr_->SetBoolValue(true, "end_flag");
                                                                                         return true;
                                                                                     });

    // *********************************************************************************************************
    auto carry_behavior_ptr = std::make_shared<shop::decision::ParallelNode>("carry behavior", blackboard_ptr_,5 );
    carry_behavior_ptr->AddChildren(global_plan_ptr);
    carry_behavior_ptr->AddChildren(robot1_carry_jud_ptr);
    carry_behavior_ptr->AddChildren(robot2_carry_jud_ptr);
    carry_behavior_ptr->AddChildren(robot3_carry_jud_ptr);
    carry_behavior_ptr->AddChildren(robot4_carry_jud_ptr);

    auto carry_pre_ptr = std::make_shared<shop::decision::PreconditionNode>("carry jud", blackboard_ptr_,
                                                                                     carry_behavior_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr_->GetBoolValue("end_flag"))
                                                                                         {
                                                                                             return true;
                                                                                         }
                                                                                         else
                                                                                         {
                                                                                             return false;
                                                                                         }
                                                                                     },
                                                                                     shop::decision::AbortType::LOW_PRIORITY);

    auto debug_game_sel_ptr = std::make_shared<shop::decision::SelectorNode>("test", blackboard_ptr_);
    debug_game_sel_ptr->AddChildren(open_behavior_success_ptr);
    debug_game_sel_ptr->AddChildren(carry_pre_ptr);

    blackboard_ptr_->SetBoolValue(true, "robot1_opening_flag");

    shop::decision::BehaviorTree se(debug_game_sel_ptr, 20);
    se.Execute();
}