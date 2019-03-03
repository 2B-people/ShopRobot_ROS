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
#include <data/ActionName.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <data/ShelfBarrier.h>
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

  ros::ServiceServer robot1_target_actionname_write_srv_;
  ros::ServiceServer robot2_target_actionname_write_srv_;
  ros::ServiceServer robot3_target_actionname_write_srv_;
  ros::ServiceServer robot4_target_actionname_write_srv_;
  ros::ServiceServer robot1_target_actionname_read_srv_;
  ros::ServiceServer robot2_target_actionname_read_srv_;
  ros::ServiceServer robot3_target_actionname_read_srv_;
  ros::ServiceServer robot4_target_actionname_read_srv_;

  ros::ServiceServer a_shelf_barrier_write_srv_;
  ros::ServiceServer a_shelf_barrier_read_srv_;
  ros::ServiceServer b_shelf_barrier_write_srv_;
  ros::ServiceServer b_shelf_barrier_read_srv_;
  ros::ServiceServer c_shelf_barrier_write_srv_;
  ros::ServiceServer c_shelf_barrier_read_srv_;
  ros::ServiceServer d_shelf_barrier_write_srv_;
  ros::ServiceServer d_shelf_barrier_read_srv_;

  bool debug_;

  //货物读写服务
  bool GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res);
  bool GoodsReadCB(data::Goods::Request &req, data::Goods::Response &res);
  //路障读写服务
  bool RoadblockWirteCB(data::Roadblock::Request &req, data::Roadblock::Response &res);
  bool RoadblockReadCB(data::Roadblock::Request &req, data::Roadblock::Response &res);

  //打开目标坐标锁
  bool TargetCoordinateLockCB(data::Coordinate::Request &req, data::Coordinate::Response &res);
  //breif 写入目标坐标
  //注: 调用前要打开对应锁|
  bool TargetCoordinateWriteCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);
  //读到目标坐标
  bool TargetCoordinateReadCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateReadCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);

  bool TargetActionNameWriteCB1(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB2(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB3(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB4(data::ActionName::Request &req, data::ActionName::Response &res);
 
  bool TargetActionNameReadCB1(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameReadCB2(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameReadCB3(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameReadCB4(data::ActionName::Request &req, data::ActionName::Response &res);
  

  //货架障碍物读写服务
  bool AshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool BshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool CshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool DshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool AshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool BshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool CshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool DshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
};

} // namespace decision
} // namespace shop

#endif