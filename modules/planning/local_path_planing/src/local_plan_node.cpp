#include <local_path_planing/localplan_baseclass.hpp>
#include <queue>
#include <iostream>

#define num_x 10
#define num_y 10


namespace shop
{
namespace pathplan
{

class LocalPlan : public LocalBase
{
public:
    const int INF = 999;
    int move_x[4], move_y[4];
    //
    int map[num_x][num_y]
    int distance[num_x][num_y];

    LocalPlan()
}

} // namespace pathplan
} // namespace shop