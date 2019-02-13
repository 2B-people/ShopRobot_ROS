#ifndef WORLD_BOARD_H
#define WORLD_BOARD_H

#include <ros/ros.h>
#include <ros/time.h>

#include<string>

#include <common/rrts.h>
#include <common/main_interface.h>
#include <data/SetBool.h>
#include <data/Goods.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

namespace shop
{
namespace decision
{
class WorldBoard : public Blackboard,public shop::common::RRTS
{
  public:
    WorldBoard(std::string name);
    ~WorldBoard();

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer goods_srv_;	
    ros::Publisher coordinate_target_1_pub_;
    ros::Publisher coordinate_target_2_pub_;
    ros::Publisher coordinate_target_3_pub_;
    ros::Publisher coordinate_target_4_pub_;

    bool GoodsCB(data::Goods::Request &req,data::Goods::Response &res);
    
};

} // namespace decision
} // namespace shop

#endif