#ifndef WORLD_BOARD_H
#define WORLD_BOARD_H

#include <ros/ros.h>
#include <ros/time.h>

#include<string>

#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

namespace shop
{
namespace decision
{
class WorldBoard : public Blackboard, shop::common::RRTS
{
  public:
    WorldBoard(std::string name);
    ~WorldBoard();

  private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;	
    
};

} // namespace decision
} // namespace shop

#endif