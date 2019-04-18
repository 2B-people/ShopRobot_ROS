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
        // plan_flag_1 = plan_flag_2 = 0
        // plan_flag_3 = plan_flag_4 = 0;
        arrive_flag_1 = arrive_flag_2 = true;
        out_wall = false;
        // arrive_flag_3 = arrive_flag_4 = 1;
        final_coord_1 = final_coord_2 = Coord(10, 10);
        // final_coord_3 = final_coord_4 = Coord(10, 10);


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

        memcpy(map_1, map, sizeof(map));
        memcpy(map_2, map, sizeof(map));
        // memcpy(map_3, map, sizeof(map));
        // memcpy(map_4, map, sizeof(map));
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

    queue<Coord> RecordShortestPath(Coord end, Coord begin, Coord map_path[num_x][num_y])
    {
        queue<Coord> que;
        if (end.first == begin.first && end.second == begin.second)
        {
            // ROS_WARN("FINALLLLLLLLLLLL");
            return que;
        }
        // ROS_WARN("END:%d, %d", end.first, end.second);
        // ROS_WARN("BEGIN:%d, %d", begin.first, begin.second);
        int x = end.first, y = end.second;
        Coord local = map_path[x][y];
        // ROS_WARN("LOCAL1:%d, %d", (int)local.first, (int)local.second);
        que = RecordShortestPath(local, begin, map_path);
        que.push(local);
        return que;
    }

    queue<Coord> PathPlanning(Coord begin, Coord end)
    {
        queue<Coord> que, path;
        Coord map_path[num_x][num_y];

        int move_x[4] = {0, 0, 1, -1}, move_y[4] = {1, -1, 0, 0};

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
        auto now_1 = GetNowCoord(1);
        auto now_2 = GetNowCoord(2);
        // auto now_3 = GetNowCoord(3);
        // auto now_4 = GetNowCoord(4);

        map[now_1.x][now_1.y] = 1;
        map[now_2.x][now_2.y] = 1;
        // map[now_3.x][now_3.y] = 1;
        // map[now_4.x][now_4.y] = 1;

        que.push(begin);
        map[begin.first][begin.second] = 1;
        map_path[begin.first][begin.second] = Coord(-1, -1);

        while (que.size())
        {
            Coord local = que.front();
            que.pop();
            // ROS_WARN("START:%d, %d", local.first, local.second);
            int i = 0;
            for (i = 0; i < 4; i++)
            {
                int temp_x = local.first + move_x[i];
                int temp_y = local.second + move_y[i];
                // ROS_WARN("$$$:%d, %d", temp_x, temp_y);
                if (0 <= temp_x && temp_x < num_x && 0 <= temp_y && temp_y < num_y && map[temp_x][temp_y] != 1)
                {
                    // ROS_WARN("&&&:%d, %d", temp_x, temp_y);
                    que.push(Coord(temp_x, temp_y)); 
                    map_path[temp_x][temp_y] = local;
                    // ROS_INFO("MAP_PATH:%d, %d", map_path[temp_x][temp_y].first, map_path[temp_x][temp_y].second);
                    map[temp_x][temp_y] = 1;
                    if (temp_x == end.first && temp_y == end.second)
                        break;
                }
            }
            if (i != 4)
                break;
        }

        // ROS_WARN("kaishijilvlujing");
        // ROS_WARN("END:%d, %d", end.first, end.second);
        // ROS_WARN("BEGIN:%d, %d", begin.first, begin.second);
        // int x = end.first, y = end.second;
        // Coord local = map_path[x][y];
        // ROS_WARN("LOCAL:%d, %d", local.first, local.second);
        // ROS_WARN("BBBBB:%d, %d", map_path[begin.first][begin.second].first, map_path[begin.first][begin.second].second);
        path = RecordShortestPath(end, begin, map_path);
        // ROS_WARN("jieshu");
        return path;
    }

    queue<Coord> PlanOutWall(Coord begin, int map[num_x][num_y])
    {
        queue<Coord> path;

        int move_x[4] = {0, 0, 1, -1}, move_y[4] = {1, -1, 0, 0};
        for(int i = 0; i<4; i++)
        {
            int temp_x = int(begin.first) + move_x[i];
            int temp_y = int(begin.second) + move_y[i];
            if(0 <= temp_x && temp_x < num_x && 0 <= temp_y && temp_y < num_y && map[temp_x][temp_y] != 1)
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
        while(path_1.size())
        {
            Coord arrive = path_1.front();
            path_1.pop();
            if(int(arrive.first) == int(local.first) && int(arrive.second) == int(local.second))
            {
                return false;
            }
        }
        return true;
    }

    void RobotGlobalPlanning(void)
    {      
        arrive_flag_1 = arrive_flag_2 = true;
        // arrive_flag_3 = arrive_flag_4 = 1;

        auto now_1 = GetNowCoord(1);
        auto end_1 = GetTargetCoord(1);

        auto now_2 = GetNowCoord(4);
        auto end_2 = GetTargetCoord(4);

        // auto now_3 = GetNowCoord(3);
        // auto end_3 = GetTargetCoord(3);

        // auto now_4 = GetNowCoord(4);
        // auto end_4 = GetTargetCoord(4);
        
        ROS_WARN("1 nowx:%d nowy:%d",now_1.x,now_1.y);
        ROS_WARN("2 nowx:%d nowy:%d",now_2.x,now_2.y);
        // ROS_WARN("3 nowx:%d nowy:%d",now_3.x,now_3.y);
        // ROS_WARN("4 nowx:%d nowy:%d",now_4.x,now_4.y);

        ROS_WARN("1 endx:%d endy:%d",end_1.x,end_1.y);
        ROS_WARN("2 endx:%d endy:%d",end_2.x,end_2.y);
        // ROS_WARN("3 endx:%d endy:%d",end_3.x,end_3.y);
        // ROS_WARN("4 endx:%d endy:%d",end_4.x,end_4.y);
        if(out_wall)
        {
            if(int(last_coord_2.first) == now_2.x && int(last_coord_2.second) == now_2.y)
            {
                out_wall = false;
            }
            else
            {
                arrive_flag_1 = arrive_flag_2 = false;
            }
            
        }

        queue<Coord> path_1, path_2;
        // queue<Coord> path_3, path_4;

        if(end_1.x != 10 && end_1.y != 10)
            path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
        // ROS_WARN("path_1 of size:%d", (int)path_1.size());
        if(end_2.x != 10 && end_2.y != 10)
            path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
        // ROS_WARN("path_2 of size:%d", (int)path_2.size());
        // if(end_3.x != 10 && end_3.y != 10)
        //     path_3 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
        // ROS_WARN("path_3 of size:%d", path_3.size());
        // if(end_4.x != 10 && end_4.y != 10)
        //     path_4 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));
        // ROS_WARN("path_4 of size:%d", path_4.size());
        queue<Coord> temp_path_1 = path_1, temp_path_2 = path_2; 
        // queue<Coord> temp_path_3 = path_3, temp_path_4 = path_4;

        while (temp_path_1.size())
        {
            Coord local_1 = temp_path_1.front();
            temp_path_1.pop();
            // ROS_INFO("path_1:%d, %d", local_1.first, local_1.second);
            map_1[int(local_1.first)][int(local_1.second)] = 1;
        }
        map_1[int(end_1.x)][int(end_1.y)] = 1;
        memcpy(map_2, map_1, sizeof(map_1));

        while (temp_path_2.size())
        {
            Coord local_2 = temp_path_2.front();
            temp_path_2.pop();
            // ROS_INFO("path_2:%d, %d", local_2.first, local_2.second);
            map_2[int(local_2.first)][int(local_2.second)] += 1;
        }
        map_2[int(end_2.x)][int(end_2.y)] = 1;
        temp_path_1 = path_1, temp_path_2 = path_2;

        //判断2机器人是否在1机器人所经过的路径上
        if(out_wall == false)
        {
            temp_path_1.push(Coord(end_1.x, end_1.y));
            while(temp_path_1.size())
            {
                Coord arrive = temp_path_1.front();
                temp_path_1.pop();
                if(int(arrive.first) == now_2.x && int(arrive.second) == now_2.y)
                {
                    arrive_flag_1 = arrive_flag_2 = false;

                    final_coord_1 = Coord(10, 10);
                    queue<Coord> out_coord = PlanOutWall(Coord(now_2.x, now_2.y), map_1);
                    while(out_coord.size())
                    {
                        Coord temp_out_coord = out_coord.front();
                        out_coord.pop();
                        bool flag = JudgeCoordInWall(temp_out_coord, path_1);
                        if(flag)
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
            temp_path_1 = path_1;
        }

        //计算1机器人所停位置
        if (arrive_flag_1)
        {
            final_coord_1 = Coord(end_1.x, end_1.y);
        }

        //计算2机器人所停位置
        if (arrive_flag_2)
        {
            Coord last_coord = Coord(10, 10);
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

        // if (arrive_flag_3)//计算3机器人所停位置
        // {
        //     Coord last_coord = Coord(10, 10);
        //     while (1)
        //     {
        //         if (temp_path_3.size() == 0)
        //         {
        //             final_coord_3.first = end_3.x;
        //             final_coord_3.second = end_3.y;
        //             break;
        //         }
                
        //         Coord stop_coord = temp_path_3.front();

        //         if (map_3[int(stop_coord.first)][int(stop_coord.second)] != 1)
        //         {
        //             final_coord_3 = last_coord;
        //             break;
        //         }
        //         last_coord = stop_coord;
        //         temp_path_3.pop();
        //     }
        // }


        // if (arrive_flag_4)//计算4机器人所停位置
        // {
        //     Coord last_coord = Coord(10, 10);
        //     while (1)
        //     {

        //         if (temp_path_4.size() == 0)
        //         {
        //             final_coord_4.first = end_4.x;
        //             final_coord_4.second = end_4.y;
        //             break;
        //         }

        //         Coord stop_coord = temp_path_4.front();
                

        //         if (map_4[int(stop_coord.first)][int(stop_coord.second)] != 1)
        //         {
        //             final_coord_4 = last_coord;
        //             break;
        //         }
        //         last_coord = stop_coord;
        //         temp_path_4.pop();
        //     }
        // }

        ROS_WARN("final_1:%d, %d", final_coord_1.first, final_coord_1.second);
        ROS_WARN("final_2:%d, %d", final_coord_2.first, final_coord_2.second);
        // ROS_WARN("final_3:%d, %d", final_coord_3.first, final_coord_3.second);
        // ROS_WARN("final_4:%d, %d", final_coord_4.first, final_coord_4.second);
        ROS_WARN("FINALY_GLOBAL!!!");
        // for (int i = 0; i < 4; i++) //得到优先级为1的机器人所走过的路径并更新地图
        // {
        //     if (robot_level[i] == 1)
        //     {
        //         int vehicle_num = i + 1;
        //         if (vehicle_num == 1)
        //         {
        //             auto now_1 = GetNowCoord(1);
        //             int no_arrive = 1;

        //             while (no_arrive)
        //             {
        //                 if (path_1.size() == 0)
        //                 {
        //                     break;
        //                 }

        //                 Coord arrive_coord = path_1.front();

        //                 if (arrive_coord.first != now_1.x || arrive_coord.second != now_1.y)
        //                 {
        //                     path_1.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                 }
        //                 else
        //                 {
        //                     path_1.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                     no_arrive = 0;
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 2)
        //         {
        //             auto now_2 = GetNowCoord(2);
        //             int no_arrive = 1;

        //             while (no_arrive)
        //             {
        //                 if (path_2.size() == 0)
        //                 {
        //                     break;
        //                 }

        //                 Coord arrive_coord = path_2.front();

        //                 if (arrive_coord.first != now_2.x || arrive_coord.second != now_2.y)
        //                 {
        //                     path_2.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                 }
        //                 else
        //                 {
        //                     path_2.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                     no_arrive = 0;
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 3)
        //         {
        //             auto now_3 = GetNowCoord(3);
        //             int no_arrive = 1;

        //             while (no_arrive)
        //             {
        //                 if (path_3.size() == 0)
        //                 {
        //                     break;
        //                 }

        //                 Coord arrive_coord = path_3.front();

        //                 if (arrive_coord.first != now_3.x || arrive_coord.second != now_3.y)
        //                 {
        //                     path_3.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                 }
        //                 else
        //                 {
        //                     path_3.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                     no_arrive = 0;
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 4)
        //         {
        //             auto now_4 = GetNowCoord(4);
        //             int no_arrive = 1;

        //             while (no_arrive)
        //             {
        //                 if (path_4.size() == 0)
        //                 {
        //                     break;
        //                 }

        //                 Coord arrive_coord = path_4.front();

        //                 if (arrive_coord.first != now_4.x || arrive_coord.second != now_4.y)
        //                 {
        //                     path_4.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                 }
        //                 else
        //                 {
        //                     path_4.pop();
        //                     map_1[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_2[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_3[arrive_coord.first][arrive_coord.second] -= 1;
        //                     map_4[arrive_coord.first][arrive_coord.second] -= 1;
        //                     no_arrive = 0;
        //                 }
        //             }
        //         }
        //     }
            
        // }


        // for (int i = 0; i < 4; i++) //为优先级为2的机器人定义剩下路径
        // {
        //     if (robot_level[i] == 2)
        //     {
        //         int vehicle_num = i + 1;
        //         if (vehicle_num == 1)
        //         {
        //             auto now_1 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_1.front();

        //                 if (prepare_arrive.first == now_1.x && prepare_arrive.second == now_1.y)
        //                 {
        //                     if (final_coord_1.first == now_1.x && final_coord_1.second == now_1.y)
        //                         arrive_flag_1 = 1;
        //                     path_1.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_1.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 2)
        //         {
        //             auto now_2 = GetNowCoord(2);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_2.front();

        //                 if (prepare_arrive.first == now_2.x && prepare_arrive.second == now_2.y)
        //                 {
        //                     if (final_coord_2.first == now_2.x && final_coord_2.second == now_2.y)
        //                         arrive_flag_2 = 1;
        //                     path_2.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_2.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 3)
        //         {
        //             auto now_3 = GetNowCoord(3);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_3.front();

        //                 if (prepare_arrive.first == now_3.x && prepare_arrive.second == now_3.y)
        //                 {
        //                     if (final_coord_3.first == now_3.x && final_coord_3.second == now_3.y)
        //                         arrive_flag_3 = 1;
        //                     path_3.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_3.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 4)
        //         {
        //             auto now_4 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_4.front();

        //                 if (prepare_arrive.first == now_4.x && prepare_arrive.second == now_4.y)
        //                 {
        //                     if (final_coord_4.first == now_4.x && final_coord_4.second == now_4.y)
        //                         arrive_flag_4 = 1;
        //                     path_4.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_4.pop();
        //                 }
        //             }
        //         }
        //     }
        // }

        // for (int i = 0; i < 4; i++) //为优先级为3的机器人定义剩下路径
        // {
        //     if (robot_level[i] == 3)
        //     {
        //         int vehicle_num = i + 1;
        //         if (vehicle_num == 1)
        //         {
        //             auto now_1 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_1.front();

        //                 if (prepare_arrive.first == now_1.x && prepare_arrive.second == now_1.y)
        //                 {
        //                     if (final_coord_1.first == now_1.x && final_coord_1.second == now_1.y)
        //                         arrive_flag_1 = 1;
        //                     path_1.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_1.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 2)
        //         {
        //             auto now_2 = GetNowCoord(2);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_2.front();

        //                 if (prepare_arrive.first == now_2.x && prepare_arrive.second == now_2.y)
        //                 {
        //                     if (final_coord_2.first == now_2.x && final_coord_2.second == now_2.y)
        //                         arrive_flag_2 = 1;
        //                     path_2.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_2.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 3)
        //         {
        //             auto now_3 = GetNowCoord(3);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_3.front();

        //                 if (prepare_arrive.first == now_3.x && prepare_arrive.second == now_3.y)
        //                 {
        //                     if (final_coord_3.first == now_3.x && final_coord_3.second == now_3.y)
        //                         arrive_flag_3 = 1;
        //                     path_3.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_3.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 4)
        //         {
        //             auto now_4 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_4.front();

        //                 if (prepare_arrive.first == now_4.x && prepare_arrive.second == now_4.y)
        //                 {
        //                     if (final_coord_4.first == now_4.x && final_coord_4.second == now_4.y)
        //                         arrive_flag_4 = 1;
        //                     path_4.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_4.pop();
        //                 }
        //             }
        //         }
        //     }
        // }

        // for (int i = 0; i < 4; i++) //为优先级为4的机器人定义剩下路径
        // {
        //     if (robot_level[i] == 4)
        //     {
        //         int vehicle_num = i + 1;
        //         if (vehicle_num == 1)
        //         {
        //             auto now_1 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_1.front();

        //                 if (prepare_arrive.first == now_1.x && prepare_arrive.second == now_1.y)
        //                 {
        //                     if (final_coord_1.first == now_1.x && final_coord_1.second == now_1.y)
        //                         arrive_flag_1 = 1;
        //                     path_1.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_1.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 2)
        //         {
        //             auto now_2 = GetNowCoord(2);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_2.front();

        //                 if (prepare_arrive.first == now_2.x && prepare_arrive.second == now_2.y)
        //                 {
        //                     if (final_coord_2.first == now_2.x && final_coord_2.second == now_2.y)
        //                         arrive_flag_2 = 1;
        //                     path_2.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_2.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 3)
        //         {
        //             auto now_3 = GetNowCoord(3);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_3.front();

        //                 if (prepare_arrive.first == now_3.x && prepare_arrive.second == now_3.y)
        //                 {
        //                     if (final_coord_3.first == now_3.x && final_coord_3.second == now_3.y)
        //                         arrive_flag_3 = 1;
        //                     path_3.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_3.pop();
        //                 }
        //             }
        //         }

        //         if (vehicle_num == 4)
        //         {
        //             auto now_4 = GetNowCoord(1);

        //             while (1)
        //             {
        //                 Coord prepare_arrive = path_4.front();

        //                 if (prepare_arrive.first == now_4.x && prepare_arrive.second == now_4.y)
        //                 {
        //                     if (final_coord_4.first == now_4.x && final_coord_4.second == now_4.y)
        //                         arrive_flag_4 = 1;
        //                     path_4.pop();
        //                     break;
        //                 }
        //                 else
        //                 {
        //                     path_4.pop();
        //                 }
        //             }
        //         }
        //     }
        // }

        // now_1 = GetNowCoord(1);
        // end_1 = GetTargetCoord(1);

        // if (now_1.x == end_1.x && now_1.y == end_1.y && plan_flag_1 == 0)
        //     plan_flag_1 = 1;

        // now_2 = GetNowCoord(2);
        // end_2 = GetTargetCoord(2);

        // if (now_2.x == end_2.x && now_2.y == end_2.y && plan_flag_2 == 0)
        //     plan_flag_2 = 1;

        // now_3 = GetNowCoord(3);
        // end_3 = GetTargetCoord(3);

        // if (now_3.x == end_3.x && now_3.y == end_3.y && plan_flag_3 == 0)
        //     plan_flag_3 = 1;

        // now_4 = GetNowCoord(4);
        // end_4 = GetTargetCoord(4);

        // if (now_4.x == end_4.x && now_4.y == end_4.y && plan_flag_4 == 0)
        //     plan_flag_4 = 1;

        // if (plan_flag_1 == 1) //给编号为1的机器人重新规划路径
        // {
        //     if (robot_level[0] == 1)
        //     {
        //         memcpy(map_1, map_2, sizeof(map_2));
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 2)
        //                 robot_level[i] = 1;
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_1 = GetNowCoord(1);
        //         auto end_1 = GetTargetCoord(1);

        //         path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
        //         temp_path_1 = path_1;

        //         while (temp_path_1.size())
        //         {
        //             Coord local_1 = temp_path_1.front();
        //             temp_path_1.pop();

        //             map_4[local_1.first][local_1.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 2)
        //     {
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_1 = GetNowCoord(1);
        //         auto end_1 = GetTargetCoord(1);

        //         path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
        //         temp_path_1 = path_1;

        //         while (temp_path_1.size())
        //         {
        //             Coord local_1 = temp_path_1.front();
        //             temp_path_1.pop();

        //             map_4[local_1.first][local_1.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 3)
        //     {

        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_1 = GetNowCoord(1);
        //         auto end_1 = GetTargetCoord(1);

        //         path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
        //         temp_path_1 = path_1;

        //         while (temp_path_1.size())
        //         {
        //             Coord local_1 = temp_path_1.front();
        //             temp_path_1.pop();

        //             map_4[local_1.first][local_1.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 4)
        //     {

        //         auto now_1 = GetNowCoord(1);
        //         auto end_1 = GetTargetCoord(1);

        //         path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
        //         temp_path_1 = path_1;

        //         while (temp_path_1.size())
        //         {
        //             Coord local_1 = temp_path_1.front();
        //             temp_path_1.pop();

        //             map_4[local_1.first][local_1.second] += 1;
        //         }
        //     }

        //     plan_flag_1 = 0;
        // }

        // if (plan_flag_2 == 1) //给编号为2的机器人重新规划路径
        // {
        //     if (robot_level[1] == 1)
        //     {
        //         memcpy(map_1, map_2, sizeof(map_2));
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 2)
        //                 robot_level[i] = 1;
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_2 = GetNowCoord(2);
        //         auto end_2 = GetTargetCoord(2);

        //         path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
        //         temp_path_2 = path_2;

        //         while (temp_path_2.size())
        //         {
        //             Coord local_2 = temp_path_2.front();
        //             temp_path_2.pop();

        //             map_4[local_2.first][local_2.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 2)
        //     {
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_2 = GetNowCoord(2);
        //         auto end_2 = GetTargetCoord(2);

        //         path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
        //         temp_path_2 = path_2;

        //         while (temp_path_2.size())
        //         {
        //             Coord local_2 = temp_path_2.front();
        //             temp_path_2.pop();

        //             map_4[local_2.first][local_2.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 3)
        //     {

        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_2 = GetNowCoord(2);
        //         auto end_2 = GetTargetCoord(2);

        //         path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
        //         temp_path_2 = path_2;

        //         while (temp_path_2.size())
        //         {
        //             Coord local_2 = temp_path_2.front();
        //             temp_path_2.pop();

        //             map_4[local_2.first][local_2.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 4)
        //     {

        //         auto now_2 = GetNowCoord(2);
        //         auto end_2 = GetTargetCoord(2);

        //         path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
        //         temp_path_2 = path_2;

        //         while (temp_path_2.size())
        //         {
        //             Coord local_2 = temp_path_2.front();
        //             temp_path_2.pop();

        //             map_4[local_2.first][local_2.second] += 1;
        //         }
        //     }

        //     plan_flag_2 = 0;
        // }

        // if (plan_flag_3 == 1) //给编号为3的机器人重新规划路径
        // {
        //     if (robot_level[2] == 1)
        //     {
        //         memcpy(map_1, map_2, sizeof(map_2));
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 2)
        //                 robot_level[i] = 1;
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_3 = GetNowCoord(3);
        //         auto end_3 = GetTargetCoord(3);

        //         path_3 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
        //         temp_path_3 = path_3;

        //         while (temp_path_3.size())
        //         {
        //             Coord local_3 = temp_path_3.front();
        //             temp_path_3.pop();

        //             map_4[local_3.first][local_3.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 2)
        //     {
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_3 = GetNowCoord(3);
        //         auto end_3 = GetTargetCoord(3);

        //         path_3 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
        //         temp_path_3 = path_3;

        //         while (temp_path_3.size())
        //         {
        //             Coord local_3 = temp_path_3.front();
        //             temp_path_3.pop();

        //             map_4[local_3.first][local_3.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 3)
        //     {

        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_3 = GetNowCoord(3);
        //         auto end_3 = GetTargetCoord(3);

        //         path_3 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
        //         temp_path_3 = path_3;

        //         while (temp_path_3.size())
        //         {
        //             Coord local_3 = temp_path_3.front();
        //             temp_path_3.pop();

        //             map_4[local_3.first][local_3.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 4)
        //     {

        //         auto now_3 = GetNowCoord(3);
        //         auto end_3 = GetTargetCoord(3);

        //         path_3 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
        //         temp_path_3 = path_3;

        //         while (temp_path_3.size())
        //         {
        //             Coord local_3 = temp_path_3.front();
        //             temp_path_3.pop();

        //             map_4[local_3.first][local_3.second] += 1;
        //         }
        //     }

        //     plan_flag_3 = 0;
        // }

        // if (plan_flag_4 == 1) //给编号为4的机器人重新规划路径
        // {
        //     if (robot_level[2] == 1)
        //     {
        //         memcpy(map_1, map_2, sizeof(map_2));
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 2)
        //                 robot_level[i] = 1;
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_4 = GetNowCoord(4);
        //         auto end_4 = GetTargetCoord(4);

        //         path_4 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));
        //         temp_path_4 = path_4;

        //         while (temp_path_4.size())
        //         {
        //             Coord local_4 = temp_path_4.front();
        //             temp_path_4.pop();

        //             map_4[local_4.first][local_4.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 2)
        //     {
        //         memcpy(map_2, map_3, sizeof(map_3));
        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 3)
        //                 robot_level[i] = 2;
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_4 = GetNowCoord(4);
        //         auto end_4 = GetTargetCoord(4);

        //         path_4 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));
        //         temp_path_4 = path_4;

        //         while (temp_path_4.size())
        //         {
        //             Coord local_4 = temp_path_4.front();
        //             temp_path_4.pop();

        //             map_4[local_4.first][local_4.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 3)
        //     {

        //         memcpy(map_3, map_4, sizeof(map_4));

        //         for (int i = 0; i < 4; i++)
        //         {
        //             if (robot_level[i] == 4)
        //                 robot_level[i] = 3;
        //         }
        //         robot_level[0] = 4;

        //         auto now_4 = GetNowCoord(4);
        //         auto end_4 = GetTargetCoord(4);

        //         path_4 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));
        //         temp_path_4 = path_4;

        //         while (temp_path_4.size())
        //         {
        //             Coord local_4 = temp_path_4.front();
        //             temp_path_4.pop();

        //             map_4[local_4.first][local_4.second] += 1;
        //         }
        //     }

        //     if (robot_level[0] == 4)
        //     {

        //         auto now_4 = GetNowCoord(4);
        //         auto end_4 = GetTargetCoord(4);

        //         path_4 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));
        //         temp_path_4 = path_4;

        //         while (temp_path_4.size())
        //         {
        //             Coord local_4 = temp_path_4.front();
        //             temp_path_4.pop();

        //             map_4[local_4.first][local_4.second] += 1;
        //         }
        //     }

        //     plan_flag_4 = 0;
        // }
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
        // case 3:
        //     coord.x = final_coord_3.first;
        //     coord.y = final_coord_3.second;
        //     coord.pose = 5;
        //     return coord;
        //     break;
        // case 4:
        //     coord.x = final_coord_4.first;
        //     coord.y = final_coord_4.second;
        //     coord.pose = 5;
        //     return coord;
        //     break;
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

    // int plan_flag_1, plan_flag_2;
    // int plan_flag_3, plan_flag_4;
    bool arrive_flag_1, arrive_flag_2;
    bool out_wall;
    // int arrive_flag_3, arrive_flag_4;

    int map_1[num_x][num_y];
    int map_2[num_x][num_y];
    // int map_3[num_x][num_y];
    // int map_4[num_x][num_y];


    Coord final_coord_1, final_coord_2;
    Coord last_coord_2;
    // Coord final_coord_3, final_coord_4;
};

} // namespace pathplan
} // namespace shop

MAIN(shop::pathplan::GlobalPlan, "global_plan_node")
