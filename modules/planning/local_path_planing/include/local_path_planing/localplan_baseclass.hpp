#ifndef LOCALPLAN_BASECLASS_H
#define LOCALPLAN_BASECLASS_H

#include <ros/ros.h>
#include <ros/time.h>

#include <string>

#include <common/rrts.h>
#include <common/main_interface.h>

#include <data/Coord.h>
#include <data/SetBool.h>
#include <data/Goods.h>
#include <data/Roadblock.h>
#include <data/Coordinate.h>
#include <data/ShelfBarrier.h>

namespace shop
{
namespace pathplan
{

class LocalBase : public shop::common::RRTS
{
public:
  LocalBase(std::string name) : shop::common::RRTS(name)
  {
    robot1_coord_now_.x = 10;
    robot1_coord_now_.y = 10;
    robot1_coord_now_.pose = 0;
    robot2_coord_now_.x = 10;
    robot2_coord_now_.y = 10;
    robot2_coord_now_.pose = 0;
    robot3_coord_now_.x = 10;
    robot3_coord_now_.y = 10;
    robot3_coord_now_.pose = 0;
    robot4_coord_now_.x = 10;
    robot4_coord_now_.y = 10;
    robot4_coord_now_.pose = 0;

    robot1_coordinate_now_ = nh_.subscribe<data::Coord>("robot1_web/coord_now", 10, boost::bind(&LocalBase::Robo1CoordNowCB, this, _1));
    robot2_coordinate_now_ = nh_.subscribe<data::Coord>("robot2_web/coord_now", 10, boost::bind(&LocalBase::Robo2CoordNowCB, this, _1));
    robot3_coordinate_now_ = nh_.subscribe<data::Coord>("robot3_web/coord_now", 10, boost::bind(&LocalBase::Robo3CoordNowCB, this, _1));
    robot4_coordinate_now_ = nh_.subscribe<data::Coord>("robot4_web/coord_now", 10, boost::bind(&LocalBase::Robo4CoordNowCB, this, _1));

    goods_write_clt_ = nh_.serviceClient<data::Goods>("shop/goods_write_srv");
    goods_read_clt_ = nh_.serviceClient<data::Goods>("shop/goods_read_srv");

    roadblock_read_clt_ = nh_.serviceClient<data::Roadblock>("shop/roadblock_read_srv");

    a_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/A_shelf_barrier_wirte");
    b_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_wirte");
    c_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_wirte");
    d_shelf_barrier_write_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_wirte");

    a_shelf_barrier_read_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/A_shelf_barrier_read");
    a_shelf_barrier_read_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/B_shelf_barrier_read");
    a_shelf_barrier_read_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/C_shelf_barrier_read");
    a_shelf_barrier_read_clt_ = nh_.serviceClient<data::ShelfBarrier>("shop/D_shelf_barrier_read");

    target_coordinate_lock_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot/coordinate_srv");
    robot1_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot1/target_coordinate_write");
    robot2_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot2/target_coordinate_write");
    robot3_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot3/target_coordinate_write");
    robot4_target_coordinate_write_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot4/target_coordinate_write");

    robot1_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot1/target_actionname_write");
    robot2_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot2/target_actionname_write");
    robot3_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot3/target_actionname_write");
    robot4_target_actionname_write_clt_ = nh_.serviceClient<data::ActionName>("shop/robot4/target_actionname_write");
  }

  virtual ~LocalBase() = default;

  virtual void PlanPlace(uint8_t robot_num) = 0;
  virtual void PlanCarry(uint8_t robot_num) = 0;

  // auto now = GetNowCoord(1);
  // x = now.x;
  data::Coord GetNowCoord(uint8_t robot_num)
  {
    switch (robot_num)
    {
    case 1:
      return robot1_coord_now_;
      break;
    case 2:
      return robot2_coord_now_;
      break;
    case 3:
      return robot3_coord_now_;
      break;
    case 4:
      return robot4_coord_now_;
      break;
    default:
      ROS_ERROR("no robot num in %s in %s", __FUNCTION__, name_.c_str());
      break;
    }
  }

  // //货物类别
  // enum class GoodsName : int8_t
  // {
  //   NONE = 0, //空,放好或者未识别
  //   RED,
  //   BLUE,
  //   GREEN,
  //   SYY,
  //   YLD,
  //   ADG,
  //   XH,
  //   HN,
  //   LH,
  //   WQ,
  //   MF,
  //   TLS,
  // };
  //read 货物类别 按照上诉规则
  int8_t GetGoods(int8_t location)
  {
    data::Goods srv;
    srv.request.location = location;
    goods_read_clt_.call(srv);
    return srv.response.name;
  }

  //直接设置目标货物为空
  void SetGoodsNONE(int8_t location)
  {
    data::Goods srv;
    srv.request.location = location;
    srv.request.name = 0;
    goods_write_clt_.call(srv);
  }

  int8_t GetRoadblock()
  {
    data::Roadblock srv;
    roadblock_read_clt_.call(srv);
    return srv.response.number;
  }

  void SetRobotTargetCoord(int8_t robot_num, int16_t x, int16_t y, int8_t pose)
  {
    data::Coordinate srv;
    srv.request.number = robot_num;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.pose = pose;
    target_coordinate_lock_clt_.call(srv);
    switch (robot_num)
    {
    case 1:
      robot1_target_coordinate_write_clt_.call(srv);
      break;
    case 2:
      robot2_target_coordinate_write_clt_.call(srv);
      break;
    case 3:
      robot3_target_coordinate_write_clt_.call(srv);
      break;
    case 4:
      robot4_target_coordinate_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      return;
      break;
    }
  }

  //shelf num 为a:1,b:2,c:3,d:4
  void GetShelfBarrier(bool *shelf_barrier, int8_t shelf_num)
  {
    data::ShelfBarrier srv;
    switch (shelf_num)
    {
    case 1:
      a_shelf_barrier_read_clt_.call(srv);
      break;
    case 2:
      b_shelf_barrier_read_clt_.call(srv);
      break;
    case 3:
      c_shelf_barrier_read_clt_.call(srv);
      break;
    case 4:
      d_shelf_barrier_read_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
    for (size_t i = 0; i < 12; i++)
    {
      shelf_barrier[i] = srv.response.shelf_barrier_all[i];
    }
  }

  // @breif 使得目标位置为false
  void SetShelfToFalse(int8_t shelf_num, int_8 location)
  {
    data::ShelfBarrier srv;
    srv.request.location = location;
    srv.request.shelf_barrier = false;
    switch (shelf_num)
    {
    case 1:
      a_shelf_barrier_write_clt_.call(srv);
      break;
    case 2:
      b_shelf_barrier_write_clt_.call(srv);
      break;
    case 3:
      c_shelf_barrier_write_clt_.call(srv);
      break;
    case 4:
      d_shelf_barrier_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
  }

  void SetRobotTargetAction(int8_t robot_num, std::string action_name)
  {
    data::AcionName srv;
    srv.request.action_name = action_name;
    switch (robot_num)
    {
    case 1:
      robot1_target_actionname_write_clt_.call(srv);
      break;
    case 2:
      robot2_target_actionname_write_clt_.call(srv);
      break;
    case 3:
      robot3_target_actionname_write_clt_.call(srv);
      break;
    case 4:
      robot4_target_actionname_write_clt_.call(srv);
      break;
    default:
      ROS_ERROR("%s no robot in %s ", name_.c_str(), __FUNCTION__);
      break;
    }
  }

protected:
  data::Coord robot1_coord_now_;
  data::Coord robot2_coord_now_;
  data::Coord robot3_coord_now_;
  data::Coord robot4_coord_now_;

  ros::NodeHandle nh_;
  ros::Subscriber robot1_coordinate_now_;
  ros::Subscriber robot2_coordinate_now_;
  ros::Subscriber robot3_coordinate_now_;
  ros::Subscriber robot4_coordinate_now_;

  ros::ServiceClient goods_write_clt_;
  ros::ServiceClient goods_read_clt_;
  ros::ServiceClient roadblock_read_clt_;

  ros::ServiceClient target_coordinate_lock_clt_;
  ros::ServiceClient robot1_target_coordinate_write_clt_;
  ros::ServiceClient robot2_target_coordinate_write_clt_;
  ros::ServiceClient robot3_target_coordinate_write_clt_;
  ros::ServiceClient robot4_target_coordinate_write_clt_;

  ros::ServiceClient robot1_target_actionname_write_clt_;
  ros::ServiceClient robot2_target_actionname_write_clt_;
  ros::ServiceClient robot3_target_actionname_write_clt_;
  ros::ServiceClient robot4_target_actionname_write_clt_;

  ros::ServiceClient a_shelf_barrier_write_clt_;
  ros::ServiceClient b_shelf_barrier_write_clt_;
  ros::ServiceClient c_shelf_barrier_write_clt_;
  ros::ServiceClient d_shelf_barrier_write_clt_;

  ros::ServiceClient a_shelf_barrier_read_clt_;
  ros::ServiceClient b_shelf_barrier_read_clt_;
  ros::ServiceClient c_shelf_barrier_read_clt_;
  ros::ServiceClient d_shelf_barrier_read_clt_;

  void Robo1CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot1_coord_now_.x = msg->x;
    robot1_coord_now_.y = msg->y;
    robot1_coord_now_.pose = msg->pose;
  }

  void Robo2CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot2_coord_now_.x = msg->x;
    robot2_coord_now_.y = msg->y;
    robot2_coord_now_.pose = msg->pose;
  }

  void Robo3CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot3_coord_now_.x = msg->x;
    robot3_coord_now_.y = msg->y;
    robot3_coord_now_.pose = msg->pose;
  }

  void Robo4CoordNowCB(const data::Coord::ConstPtr &msg)
  {
    robot4_coord_now_.x = msg->x;
    robot4_coord_now_.y = msg->y;
    robot4_coord_now_.pose = msg->pose;
  }
};

} // namespace pathplan
} // namespace shop

#endif