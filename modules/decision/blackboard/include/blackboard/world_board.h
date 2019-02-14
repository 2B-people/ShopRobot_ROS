#ifndef WORLD_BOARD_H
#define WORLD_BOARD_H

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <stdint.h>

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
class WorldBoard : public Blackboard, public shop::common::RRTS
{
public:
  WorldBoard(std::string name);
  ~WorldBoard();

private:
  ros::NodeHandle nh_;
  
  //server 服务
  ros::ServiceServer goods_write_srv_;
  ros::ServiceServer goods_read_srv_;
  ros::ServiceServer roadblock_write_srv_;
  ros::ServiceServer roadblock_read_srv_;
  ros::ServiceServer target_coordinate_lock_srv_;
  ros::ServiceServer robot1_target_coordinate_write_srv_;
  ros::ServiceServer robot2_target_coordinate_write_srv_;
  ros::ServiceServer robot3_target_coordinate_write_srv_;
  ros::ServiceServer robot4_target_coordinate_write_srv_;
  ros::ServiceServer robot1_target_coordinate_read_srv_;
  ros::ServiceServer robot2_target_coordinate_read_srv_;
  ros::ServiceServer robot3_target_coordinate_read_srv_;
  ros::ServiceServer robot4_target_coordinate_read_srv_;

  //货物读写服务
  bool GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res);
  bool GoodsReadCB(data::Goods::Request &req, data::Goods::Response &res);
  //路障读写服务
  bool RoadblockWirteCB(data::Roadblock::Request &req, data::Roadblock::Response &res);
  bool RoadblockReadCB(data::Roadblock::Request &req, data::Roadblock::Response &res);
  
  //打开目标坐标锁
  bool TargetCoordinateLockCB(data::Coordinate::Request &req, data::Coordinate::Response &res);
  //breif 写入目标坐标
  //注: 调用前要打开对应锁
  bool TargetCoordinateWriteCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);
  //读到目标坐标
  bool TargetCoordinateReadCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);
};

} // namespace decision
} // namespace shop

#endif