#ifndef WORLD_BOARD_H
#define WORLD_BOARD_H

#include <ros/ros.h>
#include <ros/time.h>

#include <string>
#include <stdint.h>

#include <blackboard/black_board.hpp>
#include <blackboard/data_structure.hpp>

#include <common/rrts.h>
#include <common/main_interface.h>

#include <data/Action.h>
#include <data/Cargo.h>
#include <data/Coord.h>
#include <data/RoadblockMsg.h>
#include <data/Barrier.h>
#include <data/Photo.h>

#include <data/PhotoSrv.h>
#include <data/SetBool.h>
#include <data/Goods.h>
#include <data/ActionName.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <data/ShelfBarrier.h>

namespace shop
{
namespace decision
{
class WorldBoard : public Blackboard, public shop::common::RRTS
{
public:
  WorldBoard(std::string name);
  ~WorldBoard();
  void Run();

private:
  ros::NodeHandle nh_;

  //topic
  // 货物
  ros::Publisher goods_pub_;
  ros::Publisher roadblock_pub_;

  ros::Publisher photo_number_pub_;

  //最终坐标
  ros::Publisher robot1_target_coord_pub_;
  ros::Publisher robot2_target_coord_pub_;
  ros::Publisher robot3_target_coord_pub_;
  ros::Publisher robot4_target_coord_pub_;

  // 动作
  ros::Publisher robot1_target_action_pub_;
  ros::Publisher robot2_target_action_pub_;
  ros::Publisher robot3_target_action_pub_;
  ros::Publisher robot4_target_action_pub_;

  // 障碍物a，b，c，d货架
  ros::Publisher a_shelf_barrier_pub_;
  ros::Publisher b_shelf_barrier_pub_;
  ros::Publisher c_shelf_barrier_pub_;
  ros::Publisher d_shelf_barrier_pub_;

  //server 服务

  ros::ServiceServer photo_number_write_srv_;
  // 货物
  ros::ServiceServer goods_write_srv_;
  // ros::ServiceServer goods_read_srv_;

  ros::ServiceServer roadblock_write_srv_;
  // ros::ServiceServer roadblock_read_srv_;

  ros::ServiceServer target_coordinate_lock_srv_;
  ros::ServiceServer robot1_target_coordinate_write_srv_;
  ros::ServiceServer robot2_target_coordinate_write_srv_;
  ros::ServiceServer robot3_target_coordinate_write_srv_;
  ros::ServiceServer robot4_target_coordinate_write_srv_;
  // ros::ServiceServer robot1_target_coordinate_read_srv_;
  // ros::ServiceServer robot2_target_coordinate_read_srv_;
  // ros::ServiceServer robot3_target_coordinate_read_srv_;
  // ros::ServiceServer robot4_target_coordinate_read_srv_;

  ros::ServiceServer robot1_target_actionname_write_srv_;
  ros::ServiceServer robot2_target_actionname_write_srv_;
  ros::ServiceServer robot3_target_actionname_write_srv_;
  ros::ServiceServer robot4_target_actionname_write_srv_;
  // ros::ServiceServer robot1_target_actionname_read_srv_;
  // ros::ServiceServer robot2_target_actionname_read_srv_;
  // ros::ServiceServer robot3_target_actionname_read_srv_;
  // ros::ServiceServer robot4_target_actionname_read_srv_;

  ros::ServiceServer a_shelf_barrier_write_srv_;
  ros::ServiceServer b_shelf_barrier_write_srv_;
  ros::ServiceServer c_shelf_barrier_write_srv_;
  ros::ServiceServer d_shelf_barrier_write_srv_;
  // ros::ServiceServer a_shelf_barrier_read_srv_;
  // ros::ServiceServer b_shelf_barrier_read_srv_;
  // ros::ServiceServer c_shelf_barrier_read_srv_;
  // ros::ServiceServer d_shelf_barrier_read_srv_;

  bool is_debug_;

  bool PhotoNumWirteCB(data::PhotoSrv::Request &req, data::PhotoSrv::Response &res);

  //货物读写服务
  bool GoodsWirteCB(data::Goods::Request &req, data::Goods::Response &res);
  // bool GoodsReadCB(data::Goods::Request &req, data::Goods::Response &res);
  //路障读写服务
  bool RoadblockWirteCB(data::Roadblock::Request &req, data::Roadblock::Response &res);
  // bool RoadblockReadCB(data::Roadblock::Request &req, data::Roadblock::Response &res);

  //打开目标坐标锁
  bool TargetCoordinateLockCB(data::Coordinate::Request &req, data::Coordinate::Response &res);
  //breif 写入目标坐标
  //注: 调用前要打开对应锁|
  bool TargetCoordinateWriteCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  bool TargetCoordinateWriteCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);
  // 重构为topic
  // //读到目标坐标
  // bool TargetCoordinateReadCB1(data::Coordinate::Request &req, data::Coordinate::Response &res);
  // bool TargetCoordinateReadCB2(data::Coordinate::Request &req, data::Coordinate::Response &res);
  // bool TargetCoordinateReadCB3(data::Coordinate::Request &req, data::Coordinate::Response &res);
  // bool TargetCoordinateReadCB4(data::Coordinate::Request &req, data::Coordinate::Response &res);

  //目标动作
  bool TargetActionNameWriteCB1(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB2(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB3(data::ActionName::Request &req, data::ActionName::Response &res);
  bool TargetActionNameWriteCB4(data::ActionName::Request &req, data::ActionName::Response &res);

  // 重构为topic
  // bool TargetActionNameReadCB1(data::ActionName::Request &req, data::ActionName::Response &res);
  // bool TargetActionNameReadCB2(data::ActionName::Request &req, data::ActionName::Response &res);
  // bool TargetActionNameReadCB3(data::ActionName::Request &req, data::ActionName::Response &res);
  // bool TargetActionNameReadCB4(data::ActionName::Request &req, data::ActionName::Response &res);

  //货架障碍物读写服务
  bool AshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool BshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool CshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  bool DshelfWirteCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  // 重构为topic
  // bool AshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  // bool BshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  // bool CshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
  // bool DshelfReadCB(data::ShelfBarrier::Request &req, data::ShelfBarrier::Response &res);
};

} // namespace decision
} // namespace shop

#endif