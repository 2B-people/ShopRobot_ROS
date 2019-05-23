#include <queue>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

#include <global_path_planning/global_plan_baseclass.hpp>

#define num_x 10
#define num_y 10

namespace shop
{
namespace pathplan
{

using namespace std;
typedef pair<int, int> Coord;

class GlobalPlan : public GlobalBase
{
public:
    GlobalPlan(std::string name) : GlobalBase(name), INF(999)
    {
        arrive_flag_1 = arrive_flag_2 = true;
        out_wall = false;
        final_coord_1 = final_coord_2 = Coord(10, 10);
    }

    void PositionOfObstacles(int map[num_x][num_y])
    {
        auto location = GetRoadblock();
        ROS_WARN("PositionOfObstacles:%d", int(location));
        switch (location)
        {
        case 1:
            map[8][1] = 1;
            break;
        case 2:
            map[8][4] = 1;
            break;
        case 3:
            map[8][8] = 1;
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

    queue<Coord> RecordShortestPath(Coord end, Coord begin, Coord map_path[num_x][num_y])
    {
        queue<Coord> que;
        if (end.first == begin.first && end.second == begin.second)
        {

            return que;
        }
        int x = end.first, y = end.second;
        Coord local = map_path[x][y];
        que = RecordShortestPath(local, begin, map_path);
        que.push(local);
        return que;
    }

    queue<Coord> PathPlanning(Coord begin, Coord end)
    {
        queue<Coord> que, path;
        Coord map_path[num_x][num_y];

        int move_x[4] = {0, 0, 1, -1}, move_y[4] = {1, -1, 0, 0};

        int map[num_x][num_y] = {{0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
        PositionOfObstacles(map);
        // auto now_1 = GetNowCoord(1);
        // auto now_2 = GetNowCoord(2);

        // auto end_1 = GetTargetCoord(1);
        // auto end_2 = GetTargetCoord(2);
        // if(end_2.x != now_1.x || end_2.y != now_1.y)
        // {
        //     map[now_1.x][now_1.y] = 1;

        // }
        // if(end_1.x != now_2.x || end_1.y != now_2.y)
        // {
        //     map[now_2.x][now_2.y] = 1;
        // }
        // map[now_1.x][now_1.y] = 1;
        // map[now_2.x][now_2.y] = 1;

        que.push(begin);
        map[begin.first][begin.second] = 1;
        map_path[begin.first][begin.second] = Coord(-1, -1);

        while (que.size())
        {
            Coord local = que.front();
            que.pop();
            int i = 0;
            for (i = 0; i < 4; i++)
            {
                int temp_x = local.first + move_x[i];
                int temp_y = local.second + move_y[i];
                if (0 <= temp_x && temp_x < num_x && 0 <= temp_y && temp_y < num_y && map[temp_x][temp_y] != 1)
                {
                    que.push(Coord(temp_x, temp_y));
                    map_path[temp_x][temp_y] = local;
                    map[temp_x][temp_y] = 1;
                    if (temp_x == end.first && temp_y == end.second)
                        break;
                }
            }
            if (i != 4)
                break;
        }

        path = RecordShortestPath(end, begin, map_path);
        return path;
    }

    queue<Coord> PlanOutWall(Coord begin, int map[num_x][num_y])
    {
        queue<Coord> path;

        int move_x[4] = {0, 0, 1, -1}, move_y[4] = {1, -1, 0, 0};
        for (int i = 0; i < 4; i++)
        {
            int temp_x = int(begin.first) + move_x[i];
            int temp_y = int(begin.second) + move_y[i];
            if (0 <= temp_x && temp_x < num_x && 0 <= temp_y && temp_y < num_y && map[temp_x][temp_y] != 1)
            {
                path.push(Coord(temp_x, temp_y));
            }
        }
        return path;
    }

    bool JudgeCoordInWall(Coord local, queue<Coord> path_1)
    {
        auto end = GetTargetCoord(2);
        path_1.push(Coord(end.x, end.y));
        while (path_1.size())
        {
            Coord arrive = path_1.front();
            path_1.pop();
            if (int(arrive.first) == int(local.first) && int(arrive.second) == int(local.second))
            {
                return false;
            }
        }
        return true;
    }

    void SetUpGrabObstacles(int flag, Coord now1, Coord now2, int map1[num_x][num_y], int map2[num_x][num_y], int *create_flag)
    {

        if (flag == 1)
        {
            if ((int(now1.first) == 4 && int(now1.second) == 2) || (int(now1.first) == 3 && int(now1.second) == 2) || (int(now1.first) == 5 && int(now1.second) == 2))
            {
                map2[4][2] += 1;
                map2[3][2] += 1;
                map2[5][2] += 1;
                // create_flag = 1;
            }
            if ((int(now1.first) == 7 && int(now1.second) == 4) || (int(now1.first) == 7 && int(now1.second) == 3) || (int(now1.first) == 7 && int(now1.second) == 5))
            {
                map2[7][4] += 1;
                map2[7][3] += 1;
                map2[7][5] += 1;
                // create_flag = 2;
            }
            if ((int(now1.first) == 5 && int(now1.second) == 7) || (int(now1.first) == 6 && int(now1.second) == 7) || (int(now1.first) == 4 && int(now1.second) == 7))
            {
                map2[5][7] += 1;
                map2[4][7] += 1;
                map2[6][7] += 1;
                // create_flag = 3;
            }
            if ((int(now1.first) == 2 && int(now1.second) == 5) || (int(now1.first) == 2 && int(now1.second) == 6) || (int(now1.first) == 2 && int(now1.second) == 4))
            {
                map2[2][5] += 1;
                map2[2][4] += 1;
                map2[2][6] += 1;
                // create_flag = 4;
            }
        }
        else if (flag == 2)
        {
            if ((int(now2.first) == 4 && int(now2.second) == 2) || (int(now2.first) == 3 && int(now2.second) == 2) || (int(now2.first) == 5 && int(now2.second) == 2))
            {
                map1[3][2] += 1;
                map1[5][2] += 1;
                map1[4][2] += 1;
                *create_flag = 1;
            }
            if ((int(now2.first) == 7 && int(now2.second) == 4) || (int(now2.first) == 7 && int(now2.second) == 3) || (int(now2.first) == 7 && int(now2.second) == 5))
            {
                map1[7][3] += 1;
                map1[7][5] += 1;
                map1[7][4] += 1;
                *create_flag = 2;
            }
            if ((int(now2.first) == 5 && int(now2.second) == 7) || (int(now2.first) == 6 && int(now2.second) == 7) || (int(now2.first) == 4 && int(now2.second) == 7))
            {
                map1[4][7] += 1;
                map1[6][7] += 1;
                map1[5][7] += 1;
                *create_flag = 3;
            }
            if ((int(now2.first) == 2 && int(now2.second) == 5) || (int(now2.first) == 2 && int(now2.second) == 6) || (int(now2.first) == 2 && int(now2.second) == 4))
            {
                map1[2][4] += 1;
                map1[2][6] += 1;
                map1[2][5] += 1;
                *create_flag = 4;
            }
        }
    }

    Coord CreatePath(int create_flag, int map2[num_x][num_y])
    {
        auto now_1 = GetNowCoord(1);
        Coord temp = Coord(0, 0);
        ROS_WARN("SUCCESS:%d", create_flag);
        if (create_flag == 1)
        {
            if (map2[3][2] >= 2 || map2[4][2] >= 2 || map2[5][2] >= 2)
            {
                if ((now_1.x == 3 && now_1.y == 1) || (now_1.x == 2 && now_1.y == 2) || (now_1.x == 4 && now_1.y == 1))
                {
                    temp.first = 6;
                    temp.second = 2;
                }
                if ((now_1.x == 5 && now_1.y == 1) || (now_1.x == 6 && now_1.y == 2))
                {
                    temp.first = 2;
                    temp.second = 2;
                }
            }
        }

        if (create_flag == 2)
        {
            if (map2[7][3] >= 2 || map2[7][4] >= 2 || map2[7][5] >= 2)
            {
                if ((now_1.x == 8 && now_1.y == 3) || (now_1.x == 8 && now_1.y == 4) || (now_1.x == 7 && now_1.y == 2))
                {
                    temp.first = 7;
                    temp.second = 6;
                }
                if ((now_1.x == 8 && now_1.y == 5) || (now_1.x == 7 && now_1.y == 6))
                {
                    temp.first = 7;
                    temp.second = 2;
                }
            }
        }

        if (create_flag == 3)
        {
            if (map2[4][7] >= 2 || map2[6][7] >= 2 || map2[5][7] >= 2)
            {
                if ((now_1.x == 4 && now_1.y == 8) || (now_1.x == 3 && now_1.y == 7) || (now_1.x == 5 && now_1.y == 7))
                {
                    temp.first = 7;
                    temp.second = 7;
                }
                if ((now_1.x == 6 && now_1.y == 8) || (now_1.x == 7 && now_1.y == 7))
                {
                    temp.first = 3;
                    temp.second = 7;
                }
            }
        }

        if (create_flag == 4)
        {
            if (map2[2][4] >= 2 || map2[2][5] >= 2 || map2[2][6] >= 2)
            {
                if ((now_1.x == 2 && now_1.y == 3) || (now_1.x == 1 && now_1.y == 5) || (now_1.x == 1 && now_1.y == 4))
                {
                    temp.first = 2;
                    temp.second = 7;
                }
                if ((now_1.x == 1 && now_1.y == 6) || (now_1.x == 2 && now_1.y == 7))
                {
                    temp.first = 2;
                    temp.second = 3;
                }
            }
        }
        return temp;
    }

    void RobotGlobalPlanning(void)
    {
        int create_path_flag = 0;

        arrive_flag_1 = arrive_flag_2 = true;
        int map[num_x][num_y] = {{0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                 {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
        PositionOfObstacles(map);
        int map_1[num_x][num_y], map_2[num_x][num_y];

        memcpy(map_1, map, sizeof(map));
        memcpy(map_2, map, sizeof(map));
        
        auto now_1 = GetNowCoord(1);
        auto end_1 = GetTargetCoord(1);

        auto now_2 = GetNowCoord(4);
        auto end_2 = GetTargetCoord(4);

        ROS_WARN("1 nowx:%d nowy:%d", now_1.x, now_1.y);
        ROS_WARN("1 endx:%d endy:%d", end_1.x, end_1.y);

        ROS_WARN("2 nowx:%d nowy:%d", now_2.x, now_2.y);
        ROS_WARN("2 endx:%d endy:%d", end_2.x, end_2.y);

        if (out_wall)
        {
            if (int(last_coord_2.first) == now_2.x && int(last_coord_2.second) == now_2.y)
            {
                out_wall = false;
            }
            else
            {
                arrive_flag_1 = arrive_flag_2 = false;
            }
        }

        queue<Coord> path_1, path_2;

        if (end_1.x != 10 && end_1.y != 10)
        {
            path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
            path_1.push(Coord(end_1.x, end_1.y));
        }
        if (end_2.x != 10 && end_2.y != 10)
        {
            path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
            path_2.push(Coord(end_2.x, end_2.y));
        }

        queue<Coord> temp_path_1 = path_1, temp_path_2 = path_2;
        while (temp_path_1.size())
        {
            Coord local_1 = temp_path_1.front();
            temp_path_1.pop();
            ROS_WARN("Coord 1:%d, %d", (int)local_1.first, (int)local_1.second);
            map_1[int(local_1.first)][int(local_1.second)] += 1;
        }
        memcpy(map_2, map_1, sizeof(map_1));
        SetUpGrabObstacles(2, Coord(int(now_1.x), int(now_1.y)), Coord(int(now_2.x), int(now_2.y)), map_1, map_2, &create_path_flag);

        while (temp_path_2.size())
        {
            Coord local_2 = temp_path_2.front();
            temp_path_2.pop();
            ROS_WARN("Coord 2:%d, %d", local_2.first, local_2.second);
            map_2[int(local_2.first)][int(local_2.second)] += 1;
        }
        SetUpGrabObstacles(1, Coord(int(now_1.x), int(now_1.y)), Coord(int(now_2.x), int(now_2.y)), map_1, map_2, &create_path_flag);

        ROS_WARN("create_path_flag:%d", create_path_flag);
        if (create_path_flag)
        {
            ROS_WARN("success");
            Coord temp = CreatePath(create_path_flag, map_2);
            if (temp.first != 0 && temp.second != 0)
            {
                final_coord_2 = temp;
                arrive_flag_1 = arrive_flag_2 = false;
            }
        }

        if (map_2[int(now_2.x)][int(now_2.y)] >= 2)
        {
            map_2[int(now_2.x)][int(now_2.y)] = 1;
        }

        temp_path_1 = path_1, temp_path_2 = path_2;
        //打印1 2 车路径所经过的路线
        while (temp_path_1.size())
        {
            Coord local_1 = temp_path_1.front();
            temp_path_1.pop();
            ROS_WARN("End 1:%d", map_1[int(local_1.first)][int(local_1.second)]);
        }

        while (temp_path_2.size())
        {
            Coord local_2 = temp_path_2.front();
            temp_path_2.pop();
            ROS_WARN("End 2:%d", map_2[int(local_2.first)][int(local_2.second)]);
        }
        temp_path_1 = path_1, temp_path_2 = path_2;

        //判断2机器人是否在1机器人所经过的路径上
        if (out_wall == false)
        {
            temp_path_1.push(Coord(end_1.x, end_1.y));
            while (temp_path_1.size())
            {
                Coord arrive = temp_path_1.front();
                temp_path_1.pop();
                if (int(arrive.first) == now_2.x && int(arrive.second) == now_2.y)
                {
                    arrive_flag_1 = arrive_flag_2 = false;

                    final_coord_1 = Coord(now_1.x, now_1.y);
                    queue<Coord> out_coord = PlanOutWall(Coord(now_2.x, now_2.y), map_1);
                    while (out_coord.size())
                    {
                        Coord temp_out_coord = out_coord.front();
                        out_coord.pop();
                        bool flag = JudgeCoordInWall(temp_out_coord, path_1);
                        if (flag)
                        {
                            final_coord_2 = temp_out_coord;
                            last_coord_2 = final_coord_2;
                            break;
                        }
                    }
                    out_wall = true;
                    break;
                }
            }
            temp_path_1 = path_1, temp_path_2 = path_2;
        }
        ROS_WARN("YYY:%d", map_1[7][4]);
        temp_path_1 = path_1, temp_path_2 = path_2;
        //计算1机器人所停位置
        if (arrive_flag_1)
        {
            Coord last_coord = Coord(now_1.x, now_1.y);
            while (1)
            {
                if (temp_path_1.size() == 0)
                {
                    final_coord_1.first = end_1.x;
                    final_coord_1.second = end_1.y;
                    break;
                }

                Coord stop_coord = temp_path_1.front();

                // ROS_WARN("LAST:%d, %d", last_coord.first, last_coord.second);
                // ROS_WARN("stop_coord: %d, %d", stop_coord.first, stop_coord.second);
                if (map_1[int(stop_coord.first)][int(stop_coord.second)] != 1)
                {
                    final_coord_1 = last_coord;
                    // ROS_WARN("STOP:%d, %d", last_coord.first, last_coord.second);
                    break;
                }
                last_coord = stop_coord;
                temp_path_1.pop();
            }
        }

        //计算2机器人所停位置
        if (arrive_flag_2)
        {
            Coord last_coord = Coord(now_2.x, now_2.y);
            while (1)
            {
                if (temp_path_2.size() == 0)
                {
                    final_coord_2.first = end_2.x;
                    final_coord_2.second = end_2.y;
                    break;
                }

                Coord stop_coord = temp_path_2.front();

                if (map_2[int(stop_coord.first)][int(stop_coord.second)] != 1)
                {
                    final_coord_2 = last_coord;
                    break;
                }
                last_coord = stop_coord;
                temp_path_2.pop();
            }
        }

        if (arrive_flag_1)
            ROS_WARN("1 IS GUIHUA");
        else
        {
            ROS_WARN("1 IS NO");
        }

        if (arrive_flag_2)
            ROS_WARN("2 IS GUIHUA");
        else
        {
            ROS_WARN("2 IS NO");
        }

        ROS_WARN("final-1:%d, %d", final_coord_1.first, final_coord_1.second);
        ROS_WARN("final-2:%d, %d", final_coord_2.first, final_coord_2.second);
        ROS_INFO("OK!!!");
    }

    data::Coord GetFinalCoord(uint8_t robot_num_)
    {
        data::Coord coord;
        switch (robot_num_)
        {
        case 1:
            coord.x = final_coord_1.first;
            coord.y = final_coord_1.second;
            coord.pose = 5;
            return coord;
            break;
        case 2:
            coord.x = final_coord_2.first;
            coord.y = final_coord_2.second;
            coord.pose = 5;
            return coord;
            break;
        default:
            break;
        }
        coord.x = 10;
        coord.y = 10;
        coord.pose = 10;
        return coord;
    }

private:
    const int INF;

    bool arrive_flag_1, arrive_flag_2;
    bool out_wall;
    bool create_path_flag;

    Coord final_coord_1, final_coord_2;
    Coord last_coord_2;
};

} // namespace pathplan
} // namespace shop

MAIN(shop::pathplan::GlobalPlan, "global_plan_node")
