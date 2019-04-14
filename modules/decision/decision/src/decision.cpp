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

    // //执行action节点
    // auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr_, goal_action_ptr);
    // auto robot2_opening_ptr = std::make_shared<shop::decision::OpenAction>(2, "robot2 opening", blackboard_ptr_, goal_action_ptr);
    // auto robot3_opening_ptr = std::make_shared<shop::decision::OpenAction>(3, "robot3 opening", blackboard_ptr_, goal_action_ptr);
    // auto robot4_opening_ptr = std::make_shared<shop::decision::OpenAction>(4, "robot4 opening", blackboard_ptr_, goal_action_ptr);

    // auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, "robot1 move", blackboard_ptr_, goal_action_ptr);
    // auto robot2_move_ptr = std::make_shared<shop::decision::MoveAction>(2, "robot2 move", blackboard_ptr_, goal_action_ptr);
    // auto robot3_move_ptr = std::make_shared<shop::decision::MoveAction>(3, "robot3 move", blackboard_ptr_, goal_action_ptr);
    // auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, "robot4 move", blackboard_ptr_, goal_action_ptr);

    // auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, "robot1 shop", blackboard_ptr_, goal_action_ptr);
    // auto robot2_action_ptr = std::make_shared<shop::decision::ShopAction>(2, "robot2 shop", blackboard_ptr_, goal_action_ptr);
    // auto robot3_action_ptr = std::make_shared<shop::decision::ShopAction>(3, "robot3 shop", blackboard_ptr_, goal_action_ptr);
    // auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, "robot4 shop", blackboard_ptr_, goal_action_ptr);

    // auto robot1_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(1, "robot1 local plan", blackboard_ptr_, goal_action_ptr);
    // auto robot2_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(2, "robot2 local plan", blackboard_ptr_, goal_action_ptr);
    // auto robot3_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(3, "robot3 local plan", blackboard_ptr_, goal_action_ptr);
    // auto robot4_local_plan_ptr = std::make_shared<shop::decision::LocalPlanAction>(4, "robot4 local plan", blackboard_ptr_, goal_action_ptr);

    // auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo ", blackboard_ptr_, goal_action_ptr);
    // auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr_, goal_action_ptr);
    // auto global_plan_ptr = std::make_shared<shop::decision::GlobalPlanAction>("global plan", blackboard_ptr_, goal_action_ptr);

}