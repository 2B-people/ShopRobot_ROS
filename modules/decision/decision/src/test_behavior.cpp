#include <ros/ros.h>

#include <thread>

#include <decision/behavior_node.hpp>
#include <decision/behavior_tree.hpp>
#include <decision/action_node.hpp>
#include <decision/goal_action.hpp>

#include <blackboard/black_board.hpp>

#include <data/Coord.h>
#include <data/RoadblockMsg.h>

using namespace shop::decision;

#define DOONCE(func, flag) \
    {                      \
        if (flag == false) \
        {                  \
            func;          \
            flag = true;   \
        }                  \
    }

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

    // auto test_seq_ptr = std::make_shared<shop::decision::SequenceNode>("test seq", blackboard_ptr_);
    // test_seq_ptr->AddChildren(robot4_move_ptr);
    // test_seq_ptr->AddChildren(robot4_action_ptr);

    data::Coord coord;
    coord.x = 1;
    coord.y = 1;
    goal_action_ptr->SetTargetActionName(4, "NONE");
    auto command_thread = std::thread(Command);
    ros::Rate rate(10);
    bool is_run = false;
    while (ros::ok)
    {
        robot4_cmd_coord_pub_.publish(coord);
        switch (command)
        {
        case '1':
            goal_action_ptr->SetTargetActionName(4, "P-1");
            command = '0';
            break;
        case '2':
            goal_action_ptr->SetTargetActionName(4, "P-2");
            command = '0';
            break;
        case '3':
            goal_action_ptr->SetTargetActionName(4, "C-01");
            command = '0';
            break;
        case '4':
            goal_action_ptr->SetTargetActionName(4, "C-02");
            command = '0';
            break;
        case '5':
            goal_action_ptr->SetTargetActionName(4, "C-03");
            command = '0';
            break;
        case '6':
            goal_action_ptr->SetTargetActionName(4, "C-04");
            command = '0';
            break;
        case '7':
            goal_action_ptr->SetTargetActionName(4, "C-05");
            command = '0';
            break;
        case '8':
            goal_action_ptr->SetTargetActionName(4, "C-06");
            command = '0';
            break;
        case '9':
            goal_action_ptr->SetTargetActionName(4, "C-07");
            command = '0';
            break;
        case '[':
            goal_action_ptr->SetTargetActionName(4, "C-08");
            command = '0';
            break;
        case '-':
            goal_action_ptr->SetTargetActionName(4, "C-09");
            command = '0';
            break;
        case '=':
            goal_action_ptr->SetTargetActionName(4, "C-10");
            command = '0';
            break;
        case '`':
            goal_action_ptr->SetTargetActionName(4, "C-11");
            command = '0';
            break;
        case ']':
            goal_action_ptr->SetTargetActionName(4, "C-11");
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
        case 'i':
            goal_action_ptr->SetTargetActionName(4, "T");
            command = '0';
            break;
        case 'u':
            goal_action_ptr->SetTargetActionName(4, "D");
            command = '0';
            break;
        case 'j':
            robot4_move_ptr->Run();
            break;
        case 'k':
            robot4_action_ptr->Run();
            break;
        case 'l':
            robot4_opening_ptr->Run();
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

        // move test is done
        if (robot4_move_ptr->GetBehaviorState() == BehaviorState::SUCCESS)
        {
            robot4_move_ptr->Reset();
            command = '0';
        }

        // action test is done
        if (robot4_action_ptr->GetBehaviorState() == BehaviorState::SUCCESS)
        {
            robot4_action_ptr->Reset();
            goal_action_ptr->SetTargetActionName(4, "NONE");
            command = '0';
        }
        if (robot4_opening_ptr->GetBehaviorState() == BehaviorState::SUCCESS)
        {
            robot4_opening_ptr->Reset();
            command = '0';
        }

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
        std::cout << "1:p-1" << std::endl;
        std::cout << "2:p-2" << std::endl;
        std::cout << "3:c-1" << std::endl;
        std::cout << "4:c-2" << std::endl;
        std::cout << "5:c-3" << std::endl;
        std::cout << "6:c-4" << std::endl;
        std::cout << "7:c-5" << std::endl;
        std::cout << "8:c-6" << std::endl;
        std::cout << "9:c-7" << std::endl;
        std::cout << "[:c-8" << std::endl;
        std::cout << "-:c-9" << std::endl;
        std::cout << "=:c-10" << std::endl;
        std::cout << "`:c-11" << std::endl;
        std::cout << "]:c-12" << std::endl;
        std::cout << "i:T" << std::endl;
        std::cout << "u:D" << std::endl;
        std::cout << "W:coord.x+1 ,S:coord.x-1" << std::endl;
        std::cout << "A:coord.y+1 ,D:coord.y-1" << std::endl;
        std::cout << "J:To robot move" << std::endl;
        std::cout << "K:To robot action" << std::endl;
        std::cout << "esc: exit program" << std::endl;
        std::cout << "**************************************" << std::endl;

        std::cin >> command;
    }
}