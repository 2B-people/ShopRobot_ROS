#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <blackboard/black_board.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

using namespace shop::decision;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "decision");
    auto blackboard_ptr = std::make_shared<PrivateBoard>(); 
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);

    //decision

    //执行action节点
    auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr, goal_action_ptr);
    auto robot2_opening_ptr = std::make_shared<shop::decision::OpenAction>(2, "robot2 opening", blackboard_ptr, goal_action_ptr);
    auto robot3_opening_ptr = std::make_shared<shop::decision::OpenAction>(3, "robot3 opening", blackboard_ptr, goal_action_ptr);

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, "robot1 move", blackboard_ptr, goal_action_ptr);
    auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, "robot2 move", blackboard_ptr, goal_action_ptr);
    auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, "robot3 move", blackboard_ptr, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, "robot4 move", blackboard_ptr, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, "robot1 shop", blackboard_ptr, goal_action_ptr);
    auto robot2_action_ptr = std::make_shared<shop::decision::ShopAction>(2, "robot2 shop", blackboard_ptr, goal_action_ptr);
    auto robot3_action_ptr = std::make_shared<shop::decision::ShopAction>(3, "robot3 shop", blackboard_ptr, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, "robot4 shop", blackboard_ptr, goal_action_ptr);

    // TODO auto photo_ptr = std::make_shared<shop::decision::>("photo ",blackboard_ptr,goal_action_ptr);
    // TODO auto distinguish_ptr = std::make_shared<shop::decision::>("distinguish",blackboard_ptr,goal_action_ptr);

    // TODO auto global_plan = std::make_shared<shop::decision::>("global plan",blackboard_ptr,goal_action_ptr);
    // TODO auto local_plan = std::make_shared<shop::decision::>("local plan",blackboard_ptr,goal_action_ptr);

    //选择节点
    //车4为识别车,由上位机直接控制,此时两种行为,移动和拍照
    auto robot4_opening_behavior_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 opening", blackboard_ptr);
    robot4_opening_behavior_ptr->AddChildren(robot4_move_ptr);
    //TODO robot4_opening_behavior_ptr->AddChildren(photo_ptr);

    auto robot1_opening_judge_ptr = std::make_shared<shop::decision::PreconditionNode>("robot1 opeing judge", blackboard_ptr,
                                                                                       robot1_opening_ptr,
                                                                                       [&]() {
                                                                                           return blackboard_ptr->GetBoolValue("robot1_opeing_flag");
                                                                                       },
                                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto robot2_opening_judge_ptr = std::make_shared<shop::decision::PreconditionNode>("robot2 opeing judge", blackboard_ptr,
                                                                                       robot2_opening_ptr,
                                                                                       [&]() {
                                                                                           return blackboard_ptr->GetBoolValue("robot2_opeing_flag");
                                                                                       },
                                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto robot3_opening_judge_ptr = std::make_shared<shop::decision::PreconditionNode>("robot3 opeing judge", blackboard_ptr,
                                                                                       robot3_opening_ptr,
                                                                                       [&]() {
                                                                                           return blackboard_ptr->GetBoolValue("robot3_opeing_flag");
                                                                                       },
                                                                                       shop::decision::AbortType::LOW_PRIORITY);
    auto robot4_opening_judge_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 opeing judge", blackboard_ptr,
                                                                                       robot4_opening_behavior_ptr,
                                                                                       [&]() {
                                                                                           return blackboard_ptr->GetBoolValue("robot4_opeing_flag");
                                                                                       },
                                                                                       shop::decision::AbortType::LOW_PRIORITY);

    auto opening_ptr = std::make_shared<shop::decision::ParallelNode>("opening", blackboard_ptr, /*此决定了成功几个子节点此节点成功*/ 5);
    opening_ptr->AddChildren(robot1_opening_judge_ptr);
    opening_ptr->AddChildren(robot2_opening_judge_ptr);
    opening_ptr->AddChildren(robot3_opening_judge_ptr);
    opening_ptr->AddChildren(robot4_opening_behavior_ptr);
    //TODO opening_ptr->AddChildren(distinguish_ptr);

    //TODO 抓取
    auto carry_ptr = std::make_shared<shop::decision::ParallelNode>("carrying", blackboard_ptr, 4);

    auto robot_ptr = std::make_shared<shop::decision::SequenceNode>("game", blackboard_ptr);
    robot_ptr->AddChildren(opening_ptr);
    robot_ptr->AddChildren(carry_ptr);

    auto end_judge_ptr = std::make_shared<shop::decision::PreconditionNode>("end_judge", blackboard_ptr,
                                                                            robot_ptr,
                                                                            [&]() {
                                                                                return blackboard_ptr->GetBoolValue("end_flag");
                                                                                // return true;
                                                                            },
                                                                            shop::decision::AbortType::LOW_PRIORITY);
    shop::decision::BehaviorTree se(end_judge_ptr, 100);
    se.Execute();
}