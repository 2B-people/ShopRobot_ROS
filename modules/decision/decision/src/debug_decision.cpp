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
    auto blackboard_ptr = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr);
    int index = 0;

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

    // auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo ", blackboard_ptr, goal_action_ptr);
    // auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr, goal_action_ptr);

    while (ros::ok)
    {
        ros::spinOnce();
        auto temp_dir_ptr = blackboard_ptr->GetDirPtr("robot1/run_coordinate");
        auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);
        
        dir_ptr->OpenLock();
        dir_ptr->Set(1,1,1);
        robot1_move_ptr->Run();
        auto state = robot1_move_ptr->GetBehaviorState();
        if (state == BehaviorState::SUCCESS) {
            while(1){}   
        }
        
        index++;
        ROS_INFO("tree is run %d",index);
    }
}