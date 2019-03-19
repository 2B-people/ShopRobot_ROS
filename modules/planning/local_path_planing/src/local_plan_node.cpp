#include <ros/ros.h>

#include <queue>
#include <iostream>
#include <string>

#include <common/main_interface.h>
#include <local_path_planing/localplan_baseclass.hpp>

#define num_x 10
#define num_y 10

namespace shop
{
namespace pathplan
{
using namespace std;
typedef pair<int, int> Coord;

class LocalPlan : public LocalBase
{

  public:
    LocalPlan(std::string name)
        : LocalBase::LocalBase(name), INF(999)
    {
        for(int i=0; i<4; i++)
        {
            switch (i)
            {
                case 0:
                    move_x_[i] = 1;
                    move_y_[i] = 0;
                    break;
                case 1:
                    move_x_[i] = 0;
                    move_y_[i] = 1;
                    break;
                case 2:
                    move_x_[i] = -1;
                    move_y_[i] = 0;
                    break;
                case 3:
                    move_x_[i] = 0;
                    move_y_[i] = -1;
                    break;

                default:
                    break;
            }
        }
        ROS_INFO("small plan init OK");
    }

    void PlanPlace(uint8_t robot_num)
    {
        bool shelves[12];
        vector<Coord> determined_location;
        vector<int> determined_action;
        vector<int> determined_shelf_location;
        int final_x, final_y;
        int final_action, final_shelf_location;

        auto goal_shelf = GetNowToShelf(robot_num);
        GetShelfBarrier(shelves, goal_shelf);

        for (int i = 1; i < 12; i = i + 2)
        {
            if (shelves[i - 1] == false || shelves[i] == false)
            {
                if (goal_shelf == 1)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(1);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(2);
                            }
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(3);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(4);
                            }
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(5);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(6);
                            }
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(7);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(8);
                            }
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(9);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(10);
                            }
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(11);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(12);
                            }
                            break;
                        default:
                            break;
                    }
                }

                else if (goal_shelf == 2)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(1);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(2);
                            }
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(3);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(4);
                            }
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(5);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(6);
                            }
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(7);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(8);
                            }
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(9);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(10);
                            }
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(11);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(12);
                            }
                            break;
                        default:
                            break;
                    }
                }

                else if (goal_shelf == 3)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(1);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(2);
                            }
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(3);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(4);
                            }
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(5);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(6);
                            }
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(7);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(8);
                            }
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(9);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(10);
                            }
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(11);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(12);
                            }
                            break;
                        default:
                            break;
                    }
                }

                else if (goal_shelf == 4)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(1);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(2);
                            }
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(3);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(4);
                            }
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(5);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(6);
                            }
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(7);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(8);
                            }
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(9);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(10);
                            }
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if (shelves[i - 1] == false)
                            {
                                determined_action.push_back(1);
                                determined_shelf_location.push_back(11);
                            }
                            else
                            {
                                determined_action.push_back(2);
                                determined_shelf_location.push_back(12);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }

        auto now_coord = GetNowCoord(robot_num);

        for (int i = 0; i < determined_location.size(); i++)
        {
            queue<Coord> que;
            que.push(Coord((int)now_coord.x, (int)now_coord.y));

            Coord location = determined_location[i];
            int end_x = location.first;
            int end_y = location.second;
            int final_distance = 999;

            int map[num_x][num_y] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
            PositionOfObstacles(map);

            int distance[num_x][num_y];
            for (int i; i < num_x; i++)
                for (int j; j < num_y; j++)
                    distance[i][j] = INF;

            distance[(int)now_coord.x][(int)now_coord.y] = 0;

            while (que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int j = 0;
                for (j = 0; j < 4; j++)
                {
                    int now_x = get_coord.first + move_x_[j];
                    int now_y = get_coord.second + move_y_[j];

                    if (0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        distance[now_x][now_y] = distance[get_coord.first][get_coord.second] + 1;
                        if (now_x == end_x && now_y == end_y)
                        {
                            if (distance[end_x][end_y] < final_distance)
                            {
                                final_distance = distance[end_x][end_y];
                                final_x = end_x;
                                final_y = end_y;
                                final_action = determined_action[i];
                                final_shelf_location = determined_shelf_location[i];
                            }
                            break;
                        }
                    }
                }
                break;
            }
        }
        TransmissonMessageToRobot(1, robot_num, int16_t(final_x), int16_t(final_y), goal_shelf=goal_shelf, 
                                  final_action=final_action);
        SetShelfToFalse(goal_shelf, int8_t(final_shelf_location));
    }


    void PlanCarry(uint8_t robot_num)
    {
        vector<Coord> determined_coord;
        vector<int> determined_location;
        vector<int> determined_category;
        int final_x, final_y;
        int final_category, final_location;
        int temp_category;

        for (int i = 0; i < 12; i++)
        {
            temp_category = GetGoods(int8_t(i+1));
            if (temp_category != 0)
            {
                switch (i + 1)
                {
                case 1:
                    determined_coord.push_back(Coord(3, 2));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 2:
                    determined_coord.push_back(Coord(4, 2));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 3:
                    determined_coord.push_back(Coord(5, 2));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 4:
                    determined_coord.push_back(Coord(7, 3));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 5:
                    determined_coord.push_back(Coord(7, 4));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 6:
                    determined_coord.push_back(Coord(7, 5));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 7:
                    determined_coord.push_back(Coord(6, 7));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 8:
                    determined_coord.push_back(Coord(5, 7));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 9:
                    determined_coord.push_back(Coord(4, 7));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 10:
                    determined_coord.push_back(Coord(2, 6));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 11:
                    determined_coord.push_back(Coord(2, 5));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;
                case 12:
                    determined_coord.push_back(Coord(2, 4));
                    determined_location.push_back(i + 1);
                    determined_category.push_back(temp_category);
                    break;

                default:
                    break;
                }
            }
        }

        auto now_coord = GetNowCoord(robot_num);

        for (int i = 0; i < determined_coord.size(); i++)
        {
            queue<Coord> que;

            que.push(Coord(int(now_coord.x), int(now_coord.y)));

            Coord location = determined_coord[i];
            int end_x = location.first;
            int end_y = location.second;
            int final_distance = 999;

            int map[num_x][num_y] = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};

            PositionOfObstacles(map);

            int distance[num_x][num_y];
            for (int i = 0; i < num_x; i++)
                for (int j = 0; j < num_y; j++)
                    distance[i][j] = INF;

            distance[(int)now_coord.x][(int)now_coord.y] = 0;

            while (que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int j = 0;
                for (j = 0; j < 4; j++)
                {
                    int now_x = get_coord.first + move_x_[j];
                    int now_y = get_coord.second + move_y_[j];

                    if (0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        distance[now_x][now_y] = distance[get_coord.first][get_coord.second] + 1;
                        if (now_x == end_x && now_y == end_y)
                        {
                            if (distance[end_x][end_y] < final_distance)
                            {
                                final_distance = distance[end_x][end_y];
                                final_x = end_x;
                                final_y = end_y;
                                final_location = determined_location[i];
                                final_category = determined_category[i];
                            }
                            break;
                        }
                    }
                }
                if (j != 4)
                    break;
            }
        }
        TransmissonMessageToRobot(2, robot_num, int16_t(final_x), int16_t(final_y), 
                                  final_location=final_location, final_category=final_category);
        SetGoodsNONE(int8_t(final_location));
    }

    void TransmissonMessageToRobot(int flag, int8_t robot_num, int16_t final_x, int16_t final_y, int8_t goal_shelf=0, 
                                   int final_location=0, int final_action=0, int final_category=0)
    {
        if(flag == 1)
        {
            if (goal_shelf == 1)
            {
                SetRobotTargetCoord(robot_num, final_x, final_y, 2);
                switch (final_action)
                {
                case 1:
                    SetRobotTargetAction(robot_num, "P-1");
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "P-2");
                    break;
                default:
                    break;
                }
            }
            else if (goal_shelf == 2)
            {
                SetRobotTargetCoord(robot_num, final_x, final_y, 1);
                switch (final_action)
                {
                case 1:
                    SetRobotTargetAction(robot_num, "P-1");
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "P-2");
                    break;
                default:
                    break;
                }
            }
            else if (goal_shelf == 3)
            {
                SetRobotTargetCoord(robot_num, final_x, final_y, 4);
                switch (final_action)
                {
                case 1:
                    SetRobotTargetAction(robot_num, "P-1");
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "P-2");
                    break;
                default:
                    break;
                }
            }
            else if (goal_shelf == 4)
            {
                SetRobotTargetCoord(robot_num, final_x, final_y, 3);
                switch (final_action)
                {
                case 1:
                    SetRobotTargetAction(robot_num, "P-1");
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "P-2");
                    break;
                default:
                    break;
                }
            }
        }
        else if(flag == 2)
        {
            switch (final_location)
            {
                case 1:
                case 2:
                case 3:
                    SetRobotTargetCoord(robot_num, final_x, final_y, 4);
                    switch (final_category)
                    {
                        case 1:
                            SetRobotTargetAction(robot_num, "C-1");
                            break;
                        case 2:
                            SetRobotTargetAction(robot_num, "C-2");
                            break;
                        case 3:
                            SetRobotTargetAction(robot_num, "C-3");
                            break;
                        case 4:
                            SetRobotTargetAction(robot_num, "C-4");
                            break;
                        case 5:
                            SetRobotTargetAction(robot_num, "C-5");
                            break;
                        case 6:
                            SetRobotTargetAction(robot_num, "C-6");
                            break;
                        case 7:
                            SetRobotTargetAction(robot_num, "C-7");
                            break;
                        case 8:
                            SetRobotTargetAction(robot_num, "C-8");
                            break;
                        case 9:
                            SetRobotTargetAction(robot_num, "C-9");
                            break;
                        case 10:
                            SetRobotTargetAction(robot_num, "C-10");
                            break;
                        case 11:
                            SetRobotTargetAction(robot_num, "C-11");
                            break;
                        case 12:
                            SetRobotTargetAction(robot_num, "C-12");
                            break;
                        default:
                            break;
                    }
                    break;

                case 4:
                case 5:
                case 6:
                    SetRobotTargetCoord(robot_num, final_x, final_y, 3);
                    switch (final_category)
                    {
                        case 1:
                            SetRobotTargetAction(robot_num, "C-1");
                            break;
                        case 2:
                            SetRobotTargetAction(robot_num, "C-2");
                            break;
                        case 3:
                            SetRobotTargetAction(robot_num, "C-3");
                            break;
                        case 4:
                            SetRobotTargetAction(robot_num, "C-4");
                            break;
                        case 5:
                            SetRobotTargetAction(robot_num, "C-5");
                            break;
                        case 6:
                            SetRobotTargetAction(robot_num, "C-6");
                            break;
                        case 7:
                            SetRobotTargetAction(robot_num, "C-7");
                            break;
                        case 8:
                            SetRobotTargetAction(robot_num, "C-8");
                            break;
                        case 9:
                            SetRobotTargetAction(robot_num, "C-9");
                            break;
                        case 10:
                            SetRobotTargetAction(robot_num, "C-10");
                            break;
                        case 11:
                            SetRobotTargetAction(robot_num, "C-11");
                            break;
                        case 12:
                            SetRobotTargetAction(robot_num, "C-12");
                            break;
                        default:
                            break;
                    }
                    break;

                case 7:
                case 8:
                case 9:
                    SetRobotTargetCoord(robot_num, final_x, final_y, 2);
                    switch (final_category)
                    {
                        case 1:
                            SetRobotTargetAction(robot_num, "C-1");
                            break;
                        case 2:
                            SetRobotTargetAction(robot_num, "C-2");
                            break;
                        case 3:
                            SetRobotTargetAction(robot_num, "C-3");
                            break;
                        case 4:
                            SetRobotTargetAction(robot_num, "C-4");
                            break;
                        case 5:
                            SetRobotTargetAction(robot_num, "C-5");
                            break;
                        case 6:
                            SetRobotTargetAction(robot_num, "C-6");
                            break;
                        case 7:
                            SetRobotTargetAction(robot_num, "C-7");
                            break;
                        case 8:
                            SetRobotTargetAction(robot_num, "C-8");
                            break;
                        case 9:
                            SetRobotTargetAction(robot_num, "C-9");
                            break;
                        case 10:
                            SetRobotTargetAction(robot_num, "C-10");
                            break;
                        case 11:
                            SetRobotTargetAction(robot_num, "C-11");
                            break;
                        case 12:
                            SetRobotTargetAction(robot_num, "C-12");
                            break;
                        default:
                            break;
                    }
                    break;

                case 10:
                case 11:
                case 12:
                    SetRobotTargetCoord(robot_num, final_x, final_y, 1);
                    switch (final_category)
                    {
                        case 1:
                            SetRobotTargetAction(robot_num, "C-1");
                            break;
                        case 2:
                            SetRobotTargetAction(robot_num, "C-2");
                            break;
                        case 3:
                            SetRobotTargetAction(robot_num, "C-3");
                            break;
                        case 4:
                            SetRobotTargetAction(robot_num, "C-4");
                            break;
                        case 5:
                            SetRobotTargetAction(robot_num, "C-5");
                            break;
                        case 6:
                            SetRobotTargetAction(robot_num, "C-6");
                            break;
                        case 7:
                            SetRobotTargetAction(robot_num, "C-7");
                            break;
                        case 8:
                            SetRobotTargetAction(robot_num, "C-8");
                            break;
                        case 9:
                            SetRobotTargetAction(robot_num, "C-9");
                            break;
                        case 10:
                            SetRobotTargetAction(robot_num, "C-10");
                            break;
                        case 11:
                            SetRobotTargetAction(robot_num, "C-11");
                            break;
                        case 12:
                            SetRobotTargetAction(robot_num, "C-12");
                            break;
                        default:
                            break;
                    }
                    break;

                default:
                    break;
            }
        }
    }

    void PositionOfObstacles(int map[num_x][num_y])
    {
        auto location = GetRoadblock();
        switch (location)
        {
            case 1:
                map[8][1] = 1;
                break;
            case 2:
                map[8][4] = 1;
                break;
            case 3:
                map[8][8] = 11;
                break;
            case 4:
                map[5][7] = 1;
                break;
            case 5:
                map[1][8] = 1;
                break;
            case 6:
                map[1][5] = 1;
                break;
            default:
                break;
        }
    }

  private:
    const int INF;
    int move_x_[4], move_y_[4];
};


} // namespace pathplan
} // namespace shop

MAIN(shop::pathplan::LocalPlan, "local_plan_node")