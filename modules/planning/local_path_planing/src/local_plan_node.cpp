#include <local_path_planing/localplan_baseclass.hpp>
#include <queue>
#include <iostream>

#define num_x 10
#define num_y 10
typedef pair<int, int> Coord;

namespace shop
{
namespace pathplan
{

class LocalPlan : public LocalBase
{
public:
    const int INF;
    int move_x[4], move_y[4];


    LocalPlan() : LocalBase(std::string name)
    {
        INF = 999;
        move_x = {1, 0, -1, 0};
        move_y = {0, 1, 0, -1};
    }


    void PlanPlace(uint8_t robot_num)
    {
        bool[12] shelves;
        vector<Coord> determined_location;
        vector<int> determined_action;
        int final_x, final_y;
        int final_distance;

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

        int distance[num_x][num_y];
        for(int i; i<num_x; i++)
            for(int j; j<num_y; j++)
                distance[i][j] = INF;

        auto goal_shelf = GetNowToShelf(robot_num);
        GetShelfBarrier(shelves, goal_shelf);

        for(int i=0; i<12; i=i+2)
        {
            if(shelves[i] == false || shelves[i+1] == false)
            {
                if(goal_shelf == 1)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if(shelves[i] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
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
                            determined_location.push_back(Coord(9, 5));
                            break;
                        case 3:
                            determined_location.push_back(Coord(9, 4));
                            break;
                        case 5:
                            determined_location.push_back(Coord(9, 3));
                            break;
                        case 7:
                            determined_location.push_back(Coord(9, 2));
                            break;
                        case 9:
                            determined_location.push_back(Coord(9, 1));
                            break;
                        case 11:
                            determined_location.push_back(Coord(9, 0));
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
                            determined_location.push_back(Coord(4, 9));
                            break;
                        case 3:
                            determined_location.push_back(Coord(5, 9));
                            break;
                        case 5:
                            determined_location.push_back(Coord(6, 9));
                            break;
                        case 7:
                            determined_location.push_back(Coord(7, 9));
                            break;
                        case 9:
                            determined_location.push_back(Coord(8, 9));
                            break;
                        case 11:
                            determined_location.push_back(Coord(9, 9));
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
                            determined_location.push_back(Coord(0, 4));
                            break;
                        case 3:
                            determined_location.push_back(Coord(0, 5));
                            break;
                        case 5:
                            determined_location.push_back(Coord(0, 6));
                            break;
                        case 7:
                            determined_location.push_back(Coord(0, 7));
                            break;
                        case 9:
                            determined_location.push_back(Coord(0, 8));
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 9));
                            break;
                        default:
                            break;
                    }
                }
                
            }
        }


        auto now_coord = GetNowCoord(robot_num);
        distance[now_coord.x][now_coord.y] = 0;

        for(int i=0; i<determined_location.size(); i++)
        {
            queue<Coord> que;
            que.push(Coord(now_coord.x, now_coord.y));

            Coord location = determined_location[i];
            end_x = location.first;
            end_y = location.second;
            final_distance = 999;
            int temp_map[num_x][num_y] = map;
            int temp_distance[num_x][num_y] = distance;

            while(que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int i = 0;
                for(i=0; i<4; i++)
                {
                    int now_x = get_coord.first() + move_x[i];
                    int now_y = get_coord.second() + move_y[i];

                    if(0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && temp_map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        temp_distance[now_x][now_y] = temp_distance[get_coord.first][get_coord.second] + 1;
                        if(now_x == end_x && now_y == end_y)
                        {
                            if(temp_distance[end_x][end_y] < final_distance)
                            {
                                final_distance = temp_distance[end_x][end_y];
                                final_x = end_x;
                                final_y = end_y;
                            }
                            break;
                        }
                    }
                }
                if(i != 4)
                    break;
            }
        }
        if(goal_shelf == 1)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 2);
            SetRobotTargetAction(robot_num, "")
        }
        else if(goal_shelf == 2)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 1);
            SetRobotTargetAction(robot_num, "")
        }
        else if(goal_shelf == 3)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 4);
            SetRobotTargetAction(robot_num, "")
        }
        else if(goal_shelf == 4)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 3);
            SetRobotTargetAction(robot_num, "")
        }
    }

    void PlanCarry(uint8_t robot_num)
    {
        vector<Coord> determined_coord;
        vector<int> determined_location;
        vector<int> determined_category;
        int final_x, final_y;
        int final_distance, final_category, final_location;
        int category;

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

        int distance[num_x][num_y];
        for(int i=0; i<num_x; i++)
            for(int j=0; j<num_y; j++)
                distance[i][j] = INF;


        for(int i=0; i<12; i++)
        {
            category = GetGoods(i+1);
            if(category != 0)
            {
                switch (i+1)
                {
                    case 1:
                        determined_coord.push_back(Coord(3, 2));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 2:
                        determined_coord.push_back(Coord(4, 2));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 3:
                        determined_coord.push_back(Coord(5, 2));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 4:
                        determined_coord.push_back(Coord(7, 3));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 5:
                        determined_coord.push_back(Coord(7, 4));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 6:
                        determined_coord.push_back(Coord(7, 5));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 7:
                        determined_coord.push_back(Coord(6, 7));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 8:
                        determined_coord.push_back(Coord(5, 7));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 9:
                        determined_coord.push_back(Coord(4, 7));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 10:
                        determined_coord.push_back(Coord(2, 6));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 11:
                        determined_coord.push_back(Coord(2, 5));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                    case 12:
                        determined_coord.push_back(Coord(2, 4));
                        determined_location.push_back(i+1);
                        determined_category.push_back(category);
                        break;
                
                    default:
                        break;
                }
                
            }
        }


        auto now_coord = GetNowCoord(robot_num);
        distance[now_coord.x][now_coord.y] = 0;


        for(int i=0; i<determined_location.size(); i++)
        {
            queue<Coord> que;
            que.push(Coord(now_coord.x, now_coord.y));

            Coord location = determined_location[i];
            end_x = location.first;
            end_y = location.second;
            final_distance = 999;
            int temp_map[num_x][num_y] = map;
            int temp_distance[num_x][num_y] = distance;

            while(que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int j = 0;
                for(j=0; j<4; j++)
                {
                    int now_x = get_coord.first() + move_x[j];
                    int now_y = get_coord.second() + move_y[j];

                    if(0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && temp_map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        temp_distance[now_x][now_y] = temp_distance[get_coord.first][get_coord.second] + 1;
                        if(now_x == end_x && now_y == end_y)
                        {
                            if(temp_distance[end_x][end_y] < final_distance)
                            {
                                final_distance = temp_distance[end_x][end_y];
                                final_x = end_x;
                                final_y = end_y;
                                final_location = determined_location[i];
                                final_category = determined_category[i];
                            }
                            break;
                        }
                    }
                }
                if(i != 4)
                    break;
            }
        }
        SetGoodsNONE(final_location);
        SetRobotTargetAction(robot_num, )
    }
    
}

} // namespace pathplan
} // namespace shop