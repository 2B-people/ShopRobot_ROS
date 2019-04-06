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
    auto blackboard_ptr = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);
    int index = 0;

    //decision

    //执行action节点
    auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr, goal_action_ptr);
    auto robot2_opening_ptr = std::make_shared<shop::decision::OpenAction>(2, "robot2 opening", blackboard_ptr, goal_action_ptr);
    auto robot3_opening_ptr = std::make_shared<shop::decision::OpenAction>(3, "robot3 opening", blackboard_ptr, goal_action_ptr);
    auto robot4_opening_ptr = std::make_shared<shop::decision::OpenAction>(4, "robot4 opening", blackboard_ptr, goal_action_ptr);

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, "robot1 move", blackboard_ptr, goal_action_ptr);
    auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, "robot2 move", blackboard_ptr, goal_action_ptr);
    auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, "robot3 move", blackboard_ptr, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, "robot4 move", blackboard_ptr, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, "robot1 shop action", blackboard_ptr, goal_action_ptr);
    auto robot2_action_ptr = std::make_shared<shop::decision::ShopAction>(2, "robot2 shop action", blackboard_ptr, goal_action_ptr);
    auto robot3_action_ptr = std::make_shared<shop::decision::ShopAction>(3, "robot3 shop action", blackboard_ptr, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, "robot4 shop action", blackboard_ptr, goal_action_ptr);

    auto robot1_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(1, "robot1 plan", blackboard_ptr, goal_action_ptr);

    auto robot1_action_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 action_jud", blackboard_ptr,
                                                                                    robot1_action_ptr,
                                                                                    [&]() {
                                                                                        if (blackboard_ptr->GetBoolValue("robot1/local_plan/flag"))
                                                                                        {
                                                                                            blackboard_ptr->SetActionName(1, goal_action_ptr->GetTargetActionName(1));
                                                                                            blackboard_ptr->SetBoolValue(!blackboard_ptr->GetBoolValue("robot1/local_plan/fuc"), "robot1/local_plan/fuc");
                                                                                            blackboard_ptr->SetBoolValue(false, "robot1/local_plan/flag");

                                                                                            return true;
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                            return false;
                                                                                        }
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot1_move_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 move_jud", blackboard_ptr,
                                                                                  robot1_move_ptr,
                                                                                  [&]() {
                                                                                      if (blackboard_ptr->GetBoolValue("robot1/local_plan/flag"))
                                                                                      {
                                                                                          auto coord = goal_action_ptr->GetTargetCoord(1);
                                                                                          blackboard_ptr->SetCoordValue(1, coord.x, coord.y, coord.pose);
                                                                                          return true;
                                                                                      }
                                                                                      else
                                                                                      {
                                                                                          return false;
                                                                                      }
                                                                                  },
                                                                                  shop::decision::AbortType::LOW_PRIORITY);

    auto robot1_opening_jud_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 robot1_opening_jud", blackboard_ptr,
                                                                                     robot1_opening_ptr,
                                                                                     [&]() {
                                                                                         if (blackboard_ptr->GetBoolValue("robot1_opening_flag"))
                                                                                         {
                                                                                             return true;
                                                                                         }
                                                                                         else
                                                                                         {
                                                                                             return false;
                                                                                         }
                                                                                     },
                                                                                     shop::decision::AbortType::LOW_PRIORITY);

    auto behavior_ptr = std::make_shared<shop::decision::SequenceNode>("debug local plan", blackboard_ptr);
    behavior_ptr->AddChildren(robot1_local_plan_ptr);
    behavior_ptr->AddChildren(robot1_move_jud_ptr);
    behavior_ptr->AddChildren(robot1_action_jud_ptr);

    auto open_behavior_ptr = std::make_shared<shop::decision::SelectorNode>("test", blackboard_ptr);
    open_behavior_ptr->AddChildren(robot1_opening_jud_ptr);
    open_behavior_ptr->AddChildren(behavior_ptr);

    blackboard_ptr->SetBoolValue(true, "robot1_opening_flag");

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