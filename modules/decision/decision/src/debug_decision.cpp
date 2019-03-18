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
    robot1_coord_now_.x = 0;
    robot1_coord_now_.y = 0;
    robot1_coord_now_.pose = 0;
    ros::Subscriber robot1_coordinate_now_ = nh.subscribe("robot1_web/coord_now", 10, Robo1CoordNowCB);

    auto blackboard_ptr_ = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr_);
    int index = 0;

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

    auto robot1_local_plan = std::make_shared<shop::decision::LocalPlanAction>(1, "robot1 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot2_local_plan = std::make_shared<shop::decision::LocalPlanAction>(2, "robot2 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot3_local_plan = std::make_shared<shop::decision::LocalPlanAction>(3, "robot3 local plan", blackboard_ptr_, goal_action_ptr);
    auto robot4_local_plan = std::make_shared<shop::decision::LocalPlanAction>(4, "robot4 local plan", blackboard_ptr_, goal_action_ptr);

    auto photo_ptr = std::make_shared<shop::decision::CameraAction>("photo ", blackboard_ptr_, goal_action_ptr);
    auto distinguish_ptr = std::make_shared<shop::decision::DetectionAction>("distinguish", blackboard_ptr_, goal_action_ptr);

    auto robot4_action_sw1_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action_sw1_ptr", blackboard_ptr_,
                                                                                    robot4_action_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(4, "T");
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot4_action_sw2_ptr = std::make_shared<shop::decision::PreconditionNode>("robot4 action_sw2_ptr", blackboard_ptr_,
                                                                                    robot4_action_ptr,
                                                                                    [&]() {
                                                                                        blackboard_ptr_->SetActionName(4, "D");
                                                                                        return true;
                                                                                    },
                                                                                    shop::decision::AbortType::LOW_PRIORITY);

    auto robot_4_opening_behavior_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 opening behavior", blackboard_ptr_);
    robot_4_opening_behavior_ptr->AddChildren(robot4_move_ptr);
    robot_4_opening_behavior_ptr->AddChildren(robot4_action_sw1_ptr);
    robot_4_opening_behavior_ptr->AddChildren(photo_ptr);
    robot_4_opening_behavior_ptr->AddChildren(robot4_action_sw2_ptr);

    auto robot4_cycle_ptr = std::make_shared<shop::decision::CycleNode>(4, "robot cycle",
                                                                        blackboard_ptr_, robot_4_opening_behavior_ptr);

    auto robot4_opening_behavior_ptr = std::make_shared<shop::decision::SequenceNode>("robot4 open", blackboard_ptr_);
    robot4_opening_behavior_ptr->AddChildren(robot4_opening_ptr);
    robot4_opening_behavior_ptr->AddChildren(robot4_cycle_ptr);

    blackboard_ptr_->SetCoordValue(4, 4, 2, 0);
    blackboard_ptr_->SetActionName(3, "T");

    while (ros::ok)
    {
        ros::spinOnce();
        robot4_opening_behavior_ptr->Run();
        // if (robot1_coord_now_.x == 4 && robot1_coord_now_.y == 2)
        // {
        //     goal_action_ptr->CancelMoveGoal(1);
        //     ROS_INFO("in this");
        //     while (1)
        //     {
        //     }
        // }
        auto state = robot4_opening_behavior_ptr->GetBehaviorState();
        if (state == BehaviorState::SUCCESS)
        {
            // auto temp_dir_ptr = blackboard_ptr_->GetDirPtr("robot1/run_coordinate");
            // auto dir_ptr = std::dynamic_pointer_cast<CoordinateDir>(temp_dir_ptr);

            // dir_ptr->OpenLock();
            // dir_ptr->Set(7, 4, 4);
            // robot4_opening_behavior_ptr->Reset();
            ROS_INFO("success");
            while (1)
            {
            }
        }

        index++;
        // ROS_INFO("tree is run %d",index);
    }
}