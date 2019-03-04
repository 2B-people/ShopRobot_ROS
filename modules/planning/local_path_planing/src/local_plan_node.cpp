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
        int final_distance, final_action;


        auto goal_shelf = GetNowToShelf(robot_num);
        GetShelfBarrier(shelves, goal_shelf);

        for(int i=1; i<12; i=i+2)
        {
            if(shelves[i-1] == false || shelves[i] == false)
            {
                if(goal_shelf == 1)
                {
                    switch (i)
                    {
                        case 1:
                            determined_location.push_back(Coord(5, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 3:
                            determined_location.push_back(Coord(4, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 5:
                            determined_location.push_back(Coord(3, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 7:
                            determined_location.push_back(Coord(2, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 9:
                            determined_location.push_back(Coord(1, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 0));
                            if(shelves[i-1] == false)
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
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 3:
                            determined_location.push_back(Coord(9, 4));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 5:
                            determined_location.push_back(Coord(9, 3));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 7:
                            determined_location.push_back(Coord(9, 2));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 9:
                            determined_location.push_back(Coord(9, 1));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 11:
                            determined_location.push_back(Coord(9, 0));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
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
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 3:
                            determined_location.push_back(Coord(5, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 5:
                            determined_location.push_back(Coord(6, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 7:
                            determined_location.push_back(Coord(7, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 9:
                            determined_location.push_back(Coord(8, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 11:
                            determined_location.push_back(Coord(9, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
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
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 3:
                            determined_location.push_back(Coord(0, 5));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 5:
                            determined_location.push_back(Coord(0, 6));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 7:
                            determined_location.push_back(Coord(0, 7));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 9:
                            determined_location.push_back(Coord(0, 8));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
                            break;
                        case 11:
                            determined_location.push_back(Coord(0, 9));
                            if(shelves[i-1] == false)
                                determined_action.push_back(1);
                            else
                                determined_action.push_back(2);
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

            while(que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int j = 0;
                for(j=0; j<4; j++)
                {
                    int now_x = get_coord.first() + move_x[j];
                    int now_y = get_coord.second() + move_y[j];

                    if(0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        distance[now_x][now_y] = distance[get_coord.first][get_coord.second] + 1;
                        if(now_x == end_x && now_y == end_y)
                        {
                            if(distance[end_x][end_y] < final_distance)
                            {
                                final_distance = distance[end_x][end_y];
                                final_x = end_x;
                                final_y = end_y;
                                final_action = determined_action[i];
                            }
                            break;
                        }
                    }
                }
                if(j != 4)
                    break;
            }
        }

        if(goal_shelf == 1)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 2);
            switch (final_action)
            {
                case 1:
                    SetRobotTargetAction(robot_num, "UP")
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "DOWM")
                    break;
                default:
                    break;
            }
        }
        else if(goal_shelf == 2)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 1);
            switch (final_action)
            {
                case 1:
                    SetRobotTargetAction(robot_num, "UP")
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "DOWM")
                    break;
                default:
                    break;
            }
        }
        else if(goal_shelf == 3)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 4);
            switch (final_action)
            {
                case 1:
                    SetRobotTargetAction(robot_num, "UP")
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "DOWM")
                    break;
                default:
                    break;
            }
        }
        else if(goal_shelf == 4)
        {
            SetRobotTargetCoord(robot_num, final_x, final_y, 3);
            switch (final_action)
            {
                case 1:
                    SetRobotTargetAction(robot_num, "UP")
                    break;
                case 2:
                    SetRobotTargetAction(robot_num, "DOWM")
                    break;
                default:
                    break;
            }
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
            //
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

            while(que.size())
            {
                Coord get_coord = que.front();
                que.pop();

                int j = 0;
                for(j=0; j<4; j++)
                {
                    int now_x = get_coord.first() + move_x[j];
                    int now_y = get_coord.second() + move_y[j];

                    if(0 <= now_x && now_x < num_x && 0 <= now_y && now_y < num_y && map[now_x][now_y] != 1 && distance[now_x][now_y] == INF)
                    {
                        que.push(Coord(now_x, now_y));
                        distance[now_x][now_y] = distance[get_coord.first][get_coord.second] + 1;
                        if(now_x == end_x && now_y == end_y)
                        {
                            if(distance[end_x][end_y] < final_distance)
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
                if(j != 4)
                    break;
            }
        }
        
        switch (final_location)
        {
            case 1:
            case 2:
            case 3:
                SetRobotTargetCoord(robot_num, final_x, final_y, 4);
                switch (final_category)
                {
                    case 1:
                        SetRobotTargetAction(robot_num, "RED");
                        break;
                    case 2:
                        SetRobotTargetAction(robot_num, "BLUE");
                        break;
                    case 3:
                        SetRobotTargetAction(robot_num, "GREEN");
                        break;
                    case 4:
                        SetRobotTargetAction(robot_num, "SYY");
                        break;
                    case 5:
                        SetRobotTargetAction(robot_num, "YLD");
                        break;
                    case 6:
                        SetRobotTargetAction(robot_num, "ADG");
                        break;
                    case 7:
                        SetRobotTargetAction(robot_num, "XH");
                        break;
                    case 8:
                        SetRobotTargetAction(robot_num, "HN");
                        break;
                    case 9:
                        SetRobotTargetAction(robot_num, "LH");
                        break;
                    case 10:
                        SetRobotTargetAction(robot_num, "WQ");
                        break;
                    case 11:
                        SetRobotTargetAction(robot_num, "MF");
                        break;
                    case 12:
                        SetRobotTargetAction(robot_num, "TLS");
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
                        SetRobotTargetAction(robot_num, "RED");
                        break;
                    case 2:
                        SetRobotTargetAction(robot_num, "BLUE");
                        break;
                    case 3:
                        SetRobotTargetAction(robot_num, "GREEN");
                        break;
                    case 4:
                        SetRobotTargetAction(robot_num, "SYY");
                        break;
                    case 5:
                        SetRobotTargetAction(robot_num, "YLD");
                        break;
                    case 6:
                        SetRobotTargetAction(robot_num, "ADG");
                        break;
                    case 7:
                        SetRobotTargetAction(robot_num, "XH");
                        break;
                    case 8:
                        SetRobotTargetAction(robot_num, "HN");
                        break;
                    case 9:
                        SetRobotTargetAction(robot_num, "LH");
                        break;
                    case 10:
                        SetRobotTargetAction(robot_num, "WQ");
                        break;
                    case 11:
                        SetRobotTargetAction(robot_num, "MF");
                        break;
                    case 12:
                        SetRobotTargetAction(robot_num, "TLS");
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
                        SetRobotTargetAction(robot_num, "RED");
                        break;
                    case 2:
                        SetRobotTargetAction(robot_num, "BLUE");
                        break;
                    case 3:
                        SetRobotTargetAction(robot_num, "GREEN");
                        break;
                    case 4:
                        SetRobotTargetAction(robot_num, "SYY");
                        break;
                    case 5:
                        SetRobotTargetAction(robot_num, "YLD");
                        break;
                    case 6:
                        SetRobotTargetAction(robot_num, "ADG");
                        break;
                    case 7:
                        SetRobotTargetAction(robot_num, "XH");
                        break;
                    case 8:
                        SetRobotTargetAction(robot_num, "HN");
                        break;
                    case 9:
                        SetRobotTargetAction(robot_num, "LH");
                        break;
                    case 10:
                        SetRobotTargetAction(robot_num, "WQ");
                        break;
                    case 11:
                        SetRobotTargetAction(robot_num, "MF");
                        break;
                    case 12:
                        SetRobotTargetAction(robot_num, "TLS");
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
                        SetRobotTargetAction(robot_num, "RED");
                        break;
                    case 2:
                        SetRobotTargetAction(robot_num, "BLUE");
                        break;
                    case 3:
                        SetRobotTargetAction(robot_num, "GREEN");
                        break;
                    case 4:
                        SetRobotTargetAction(robot_num, "SYY");
                        break;
                    case 5:
                        SetRobotTargetAction(robot_num, "YLD");
                        break;
                    case 6:
                        SetRobotTargetAction(robot_num, "ADG");
                        break;
                    case 7:
                        SetRobotTargetAction(robot_num, "XH");
                        break;
                    case 8:
                        SetRobotTargetAction(robot_num, "HN");
                        break;
                    case 9:
                        SetRobotTargetAction(robot_num, "LH");
                        break;
                    case 10:
                        SetRobotTargetAction(robot_num, "WQ");
                        break;
                    case 11:
                        SetRobotTargetAction(robot_num, "MF");
                        break;
                    case 12:
                        SetRobotTargetAction(robot_num, "TLS");
                        break;
                    default:
                        break;
                }
                break;
        
            default:
                break;
        }
        SetGoodsNONE(final_location);
    }
    
}

} // namespace pathplan
} // namespace shop