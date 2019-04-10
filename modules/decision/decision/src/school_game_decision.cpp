#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

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

    auto robot1_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1_opening_jud", blackboard_ptr_,
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

    auto robot2_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2_opening_jud", blackboard_ptr_,
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
    auto robot3_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3_opening_jud", blackboard_ptr_,
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
    auto robot4_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4_opening_jud", blackboard_ptr_,
                                                                                     robot4_opening_ptr,
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

    auto opening_done_ptr = std::make_shared<shop::decision::SuccessDoNode>("opening done", blackboard_ptr_,
                                                                            robot4_cycle_done_ptr,
                                                                            [&]() {
                                                                                blackboard_ptr_->SetBoolValue(true, "end_flag");
                                                                                blackboard_ptr_->SetBoolValue(false, "robot4_opening_flag");
                                                                                blackboard_ptr_->SetBoolValue(false, "robot1_opening_flag");
                                                                                blackboard_ptr_->SetBoolValue(false, "robot2_opening_flag");

                                                                                return true;
                                                                            });

    auto robot4_opening_seq_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 open seq", blackboard_ptr_);
    robot4_opening_seq_ptr->AddChildren(robot4_opening_done_ptr);
    robot4_opening_seq_ptr->AddChildren(opening_done_ptr);

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

    auto open_behavior_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("opening_jud", blackboard_ptr_,
                                                                                    opening_done_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr_->GetBoolValue("end_flag") == false)
                                                                                        {
                                                                                            return true;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    //  *****************************************************************
    //  ******************************************************************
    auto move_1 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 5, 2, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::LOW_PRIORITY);
    auto action_1 = std::make_shared<shop::decision::PreconditionNode>("action1", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "C-1");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto move_2 = std::make_shared<shop::decision::PreconditionNode>("move2", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 5, 0, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::LOW_PRIORITY);
    auto action_2 = std::make_shared<shop::decision::PreconditionNode>("action2", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "P-1");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto move_3 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 4, 2, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::LOW_PRIORITY);
    auto action_3 = std::make_shared<shop::decision::PreconditionNode>("action3", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "C-2");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto move_4 = std::make_shared<shop::decision::PreconditionNode>("move4", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 4, 0, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::SELF);
    auto action_4 = std::make_shared<shop::decision::PreconditionNode>("action4", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "P-1");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto move_5 = std::make_shared<shop::decision::PreconditionNode>("move5", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 3, 2, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::LOW_PRIORITY);
    auto action_5 = std::make_shared<shop::decision::PreconditionNode>("action5", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "C-2");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto move_6 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                     robot2_move_ptr,
                                                                     [&]() {
                                                                         blackboard_ptr_->SetCoordValue(2, 3, 0, 0);
                                                                         return true;
                                                                     },
                                                                     shop::decision::AbortType::SELF);
    auto action_6 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                       robot2_action_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetActionName(2, "P-1");
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto carrt2 = std::make_shared<shop::decision::SequenceNode>("seq 1", blackboard_ptr_);
    carrt2->AddChildren(move_1);
    carrt2->AddChildren(action_1);
    carrt2->AddChildren(move_2);
    carrt2->AddChildren(action_2);
    carrt2->AddChildren(move_3);
    carrt2->AddChildren(action_3);
    carrt2->AddChildren(move_4);
    carrt2->AddChildren(action_4);
    carrt2->AddChildren(move_5);
    carrt2->AddChildren(action_5);
    carrt2->AddChildren(move_6);
    carrt2->AddChildren(action_6);

    auto carry_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("carry_jud_ptr", blackboard_ptr_,
                                                                            carrt2,
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

    //  ******************************************************************
    //  ******************************************************************

    //  *****************************************************************
    //  ******************************************************************
    auto move1_1 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 7, 5, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action1_1 = std::make_shared<shop::decision::PreconditionNode>("action1", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "C-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_2 = std::make_shared<shop::decision::PreconditionNode>("move2", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 9, 5, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action1_2 = std::make_shared<shop::decision::PreconditionNode>("action2", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_3 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 7, 4, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action1_3 = std::make_shared<shop::decision::PreconditionNode>("action3", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "C-2");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_4 = std::make_shared<shop::decision::PreconditionNode>("move4", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 9, 4, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action1_4 = std::make_shared<shop::decision::PreconditionNode>("action4", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_5 = std::make_shared<shop::decision::PreconditionNode>("move5", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 7, 3, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action1_5 = std::make_shared<shop::decision::PreconditionNode>("action5", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "C-2");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_6 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 9, 3, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action1_6 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_7 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 6, 7, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action1_7 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "C-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_8 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 6, 9, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action1_8 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_9 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot1_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(1, 5, 7, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action1_9 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot1_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(1, "C-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move1_10 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                       robot1_move_ptr,
                                                                       [&]() {
                                                                           blackboard_ptr_->SetCoordValue(1, 5, 9, 0);
                                                                           return true;
                                                                       },
                                                                       shop::decision::AbortType::SELF);
    auto action1_10 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                         robot1_action_ptr,
                                                                         [&]() {
                                                                             blackboard_ptr_->SetActionName(1, "P-1");
                                                                             return true;
                                                                         },
                                                                         shop::decision::AbortType::LOW_PRIORITY);

    auto carrt3 = std::make_shared<shop::decision::SequenceNode>("seq 1", blackboard_ptr_);
    carrt3->AddChildren(move1_1);
    carrt3->AddChildren(action1_1);
    carrt3->AddChildren(move1_2);
    carrt3->AddChildren(action1_2);
    carrt3->AddChildren(move1_3);
    carrt3->AddChildren(action1_3);
    carrt3->AddChildren(move1_4);
    carrt3->AddChildren(action1_4);
    carrt3->AddChildren(move1_5);
    carrt3->AddChildren(action1_5);
    carrt3->AddChildren(move1_6);
    carrt3->AddChildren(action1_6);
    carrt3->AddChildren(move1_7);
    carrt3->AddChildren(action1_7);
    carrt3->AddChildren(move1_8);
    carrt3->AddChildren(action1_8);
    carrt3->AddChildren(move1_9);
    carrt3->AddChildren(action1_9);
    carrt3->AddChildren(move1_10);
    carrt3->AddChildren(action1_10);

    auto carry1_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("carry_jud_ptr", blackboard_ptr_,
                                                                             carrt3,
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

    //  ******************************************************************
    //  ******************************************************************

    //  *****************************************************************
    //  ******************************************************************
    auto move2_1 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 2, 5, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action2_1 = std::make_shared<shop::decision::PreconditionNode>("action1", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "C-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_2 = std::make_shared<shop::decision::PreconditionNode>("move2", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 4, 9, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action2_2 = std::make_shared<shop::decision::PreconditionNode>("action2", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_3 = std::make_shared<shop::decision::PreconditionNode>("move1", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 4, 7, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action2_3 = std::make_shared<shop::decision::PreconditionNode>("action3", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "C-2");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_4 = std::make_shared<shop::decision::PreconditionNode>("move4", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 0, 7, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action2_4 = std::make_shared<shop::decision::PreconditionNode>("action4", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_5 = std::make_shared<shop::decision::PreconditionNode>("move5", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 2, 6, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::LOW_PRIORITY);
    auto action2_5 = std::make_shared<shop::decision::PreconditionNode>("action5", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "C-2");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_6 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 0, 6, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action2_6 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_7 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 2, 4, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);
    auto action2_7 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "C-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);
    auto move2_8 = std::make_shared<shop::decision::PreconditionNode>("move6", blackboard_ptr_,
                                                                      robot4_move_ptr,
                                                                      [&]() {
                                                                          blackboard_ptr_->SetCoordValue(4, 0, 4, 0);
                                                                          return true;
                                                                      },
                                                                      shop::decision::AbortType::SELF);

    auto action2_8 = std::make_shared<shop::decision::PreconditionNode>("action6", blackboard_ptr_,
                                                                        robot4_action_ptr,
                                                                        [&]() {
                                                                            blackboard_ptr_->SetActionName(4, "P-1");
                                                                            return true;
                                                                        },
                                                                        shop::decision::AbortType::LOW_PRIORITY);

    auto carrt4 = std::make_shared<shop::decision::SequenceNode>("seq 1", blackboard_ptr_);
    carrt4->AddChildren(move2_1);
    carrt4->AddChildren(action2_1);
    carrt4->AddChildren(move2_2);
    carrt4->AddChildren(action2_2);
    carrt4->AddChildren(move2_3);
    carrt4->AddChildren(action2_3);
    carrt4->AddChildren(move2_4);
    carrt4->AddChildren(action2_4);
    carrt4->AddChildren(move2_5);
    carrt4->AddChildren(action2_5);
    carrt4->AddChildren(move2_6);
    carrt4->AddChildren(action2_6);
    carrt4->AddChildren(move2_7);
    carrt4->AddChildren(action2_7);
    carrt4->AddChildren(move2_8);
    carrt4->AddChildren(action2_8);

    auto carry2_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("carry_jud_ptr", blackboard_ptr_,
                                                                             carrt4,
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

    //  ******************************************************************
    //  ******************************************************************

    // begin
    blackboard_ptr_->SetBoolValue(true, "robot1_opening_flag");

    auto open_behavior_ptr = std::make_shared<shop::decision::ParallelNode>("test", blackboard_ptr_, 5);
    open_behavior_ptr->AddChildren(robot1_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot2_opening_jud_ptr);
    // open_behavior_ptr->AddChildren(robot3_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot_opening_jud_ptr);
    open_behavior_ptr->AddChildren(carry_jud_ptr);
    open_behavior_ptr->AddChildren(carry1_jud_ptr);
    // open_behavior_ptr->AddChildren(carry2_jud_ptr);

    auto seq = std::make_shared<shop::decision::SelectorNode>("seq test", blackboard_ptr_);
    carrt2->AddChildren(open_behavior_jud_ptr);
    carrt2->AddChildren(carrt2);

    shop::decision::BehaviorTree se(open_behavior_ptr, 10);
    se.Execute();

    // while (ros::ok)
    // {
    //     robot1_local_plan_ptr->Run();
    //     if (robot1_local_plan_ptr->GetBehaviorState() == BehaviorState::SUCCESS)
    //     {
    //         blackboard_ptr->SetBoolValue(!blackboard_ptr->GetBoolValue("robot1/local_plan/fuc"), "robot1/local_plan/fuc");
    //     }
    // }
}