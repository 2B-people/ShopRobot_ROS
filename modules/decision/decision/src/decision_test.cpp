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
    auto open_behavior_ptr = std::make_shared<shop::decision::ParallelNode>("test", blackboard_ptr_, 2);
    open_behavior_ptr->AddChildren(robot1_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot2_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot3_opening_jud_ptr);
    open_behavior_ptr->AddChildren(robot4_opening_jud_ptr);

    blackboard_ptr_->SetBoolValue(true, "robot1_opening_flag");

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