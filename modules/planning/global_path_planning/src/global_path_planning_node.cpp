#include <queue>
#include <iostream>
#include <string>
#include <cstring>
#include <vector>

#include <global_path_planning_node.h>

#define num_x 10
#define num_y 10

namespace shop{
namespace pathplan{

using namespace std;
typedef pair<int, int> Coord;

class GlobalPlan : public GlobalBase
{
public:
    GlobalPlan(std::string name): GlobalBase(name), INF(999)
    {
        flag = 1;

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
        memcpy(map_3, map, sizeof(map));
        memcpy(map_4, map, sizeof(map));

    }

    void RobotGlobalPlanning()
    {
        if(flag == 1)
        {
            auto now_1 = GetNowCoord(1);
            auto end_1 = GetTargetCoord(1);

            auto now_2 = GetNowCoord(2);
            auto end_2 = GetTargetCoord(2);

            auto now_3 = GetNowCoord(3);
            auto end_3 = GetTargetCoord(3);

            auto now_4 = GetNowCoord(4);
            auto end_4 = GetTargetCoord(4);

            queue path_1 = PathPlanning(Coord(now_1.x, now_1.y), Coord(end_1.x, end_1.y));
            queue path_2 = PathPlanning(Coord(now_2.x, now_2.y), Coord(end_2.x, end_2.y));
            queue path_2 = PathPlanning(Coord(now_3.x, now_3.y), Coord(end_3.x, end_3.y));
            queue path_2 = PathPlanning(Coord(now_4.x, now_4.y), Coord(end_4.x, end_4.y));

            while(path_1.size())
            {
                Coord local_1 = path_1.front();
                path_1.pop();

                map_1[local_1.first][local_1.second] += 1;
            }
            memcpy(map_2, map_1, sizeof(map_1))

            Coord temp, first_coord = path_2.front();
            int counter = 0;
            while(path_2.size())
            {
                Coord local_2 = path_2.front();
                path_2.pop();

                map_2[local_2.first][local_2.second] += 1;
                if(map_2[local_2.first][local_2.second] == 2)
                {
                    if(counter == 0)
                    {
                        temp = local_2;
                    }
                }
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

    queue RecordShortestPath(int x, int y, Coord begin, Coord map_path[num_x][num_y])
    {
        queue que;

        if(x == begin.first && y ==begin.second)
        {
            que.push(begin);
            return que;
        }

        Coord local = map_path[x][y];
        que = RecordShortestPath(local.first, local.second, begin, map_path);
        que.push(local);
        return que
    }

    queue PathPlanning(Coord begin, Coord end)
    {
        queue<Coord> que, path;
        Coord map_path[num_x][num_y];

        int move_x = {0, 0, 1, -1}, move_y = {1, -1, 0, 0};

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


        que.push(begin);
        map[begin.first][begin.second] = 1;
        map_path[begin.first][begin.second] = Coord(-1, -1);

        while(que.size())
        {
            Coord local = que.front();
            que.pop();

            int i = 0;
            for(int i = 0; i<4; i++)
            {
                int temp_x = local.first + move_x[i];
                int temp_y = local.second + move_y[i];

                if(0 <= temp_x && temp_x < num_x && 0 <= temp_y && temp_y < num_y && map[temp_x][temp_y] != 1)
                {
                    que.push(Coord(temp_x, temp_y));
                    map_path[temp_x][temp_y] = local;
                    map[temp_x][temp_y] = 1;
                    if(temp_x == end.first && temp_y == end.second)
                        break;
                }
            }
            if(i != 4)
                break;
        }

        path = RecordShortestPath(path, end.first, end.second, begin, map_path);
        return path;
    }

private:
    const int INF;

    int flag;
    
    int map_1[num_x][num_y];
    int map_2[num_x][num_y];
    int map_3[num_x][num_y];
    int map_4[num_x][num_y];

    Coord final_coord_1, final_coord_2, final_coord_3, final_coord_4;

}





} // namespace pathplan
} // namespace shop
