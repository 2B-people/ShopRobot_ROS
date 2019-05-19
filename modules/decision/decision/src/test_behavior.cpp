#include <ros/ros.h>

#include <thread>

#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

#include <blackboard/black_board.hpp>

#include <data/Coord.h>
#include <data/RoadblockMsg.h>

//此文件用于debug,

using namespace shop::decision;
void Command();
char command = '0';

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    bool is_debug_;
    nh.param("debug", is_debug_, false);
    auto robot4_cmd_coord_pub_ = nh.advertise<data::Coord>("shop/robot4/cmd_coord", 1);

    auto blackboard_ptr_ = std::make_shared<PrivateBoard>();
    auto goal_action_ptr = std::make_shared<GoalAction>(blackboard_ptr_);

    auto robot1_opening_ptr = std::make_shared<shop::decision::OpenAction>(1, "robot1 opening", blackboard_ptr_, goal_action_ptr);
    auto robot4_opening_ptr = std::make_shared<shop::decision::OpenAction>(4, "robot4 opening", blackboard_ptr_, goal_action_ptr);

    auto robot1_move_ptr = std::make_shared<shop::decision::MoveAction>(1, 1, "robot1 move", blackboard_ptr_, goal_action_ptr);
    auto robot4_move_ptr = std::make_shared<shop::decision::MoveAction>(4, 1, "robot4 move", blackboard_ptr_, goal_action_ptr);

    auto robot4_special_move_ptr = std::make_shared<shop::decision::MoveAction>(4, 2, "robot4 special move", blackboard_ptr_, goal_action_ptr);
    auto robot4_special_action_ptr = std::make_shared<shop::decision::ShopAction>(4, 1, "robot4 special shop", blackboard_ptr_, goal_action_ptr);

    auto robot1_action_ptr = std::make_shared<shop::decision::ShopAction>(1, 0, "robot1 shop", blackboard_ptr_, goal_action_ptr);
    auto robot4_action_ptr = std::make_shared<shop::decision::ShopAction>(4, 0, "robot4 shop", blackboard_ptr_, goal_action_ptr);

    auto test_seq_ptr = std::make_shared<shop::decision::SequenceNode>("test seq", blackboard_ptr_);
    test_seq_ptr->AddChildren(robot4_move_ptr);
    test_seq_ptr->AddChildren(robot4_action_ptr);

    data::Coord coord;
    coord.x = 1;
    coord.y = 1;
    goal_action_ptr->SetTargetActionName(4, "NONE");
    auto command_thread = std::thread(Command);
    ros::Rate rate(10);
    while (ros::ok)
    {
        robot4_cmd_coord_pub_.publish(coord);
        switch (command)
        {
        case '1':
            goal_action_ptr->SetTargetActionName(4, "P-2");
            command = '0';
            break;
        case '2':
            goal_action_ptr->SetTargetActionName(4, "C-1");
            command = '0';
            break;
        case 'w':
            coord.x = coord.x + 1;
            goal_action_ptr->SetTargetCoord(4, coord);
            command = '0';
            break;
        case 'a':
            coord.y = coord.y + 1;
            goal_action_ptr->SetTargetCoord(4, coord);
            command = '0';
            break;
        case 's':
            coord.x = coord.x - 1;
            goal_action_ptr->SetTargetCoord(4, coord);
            command = '0';
            break;
        case 'd':
            coord.y = coord.y - 1;
            goal_action_ptr->SetTargetCoord(4, coord);
            command = '0';
            break;
        case 27:
            if (command_thread.joinable())
            {
                command_thread.join();
            }
            return 0;
        default:
            break;
        }
        test_seq_ptr->Run();
        rate.sleep();
    }
}

void Command()
{

    while (command != 27)
    {
        std::cout << "**************************************" << std::endl;
        std::cout << "*********please send a command********" << std::endl;
        std::cout << "> ";
        std::cout << "1:p-2" << std::endl;
        std::cout << "2:c-1" << std::endl;
        std::cout << "fanxianjian kou " << std::endl;
        std::cout << "esc: exit program" << std::endl;
        std::cout << "**************************************" << std::endl;

        std::cin >> command;
        if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != 27)
        {
            std::cout << "please input again!" << std::endl;
            std::cout << "> ";
            std::cin >> command;
        }
    }
}