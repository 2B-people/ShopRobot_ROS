#ifndef GLOBAL_PATH_PLANNING_H
#define GLOBAL_PATH_PLANNING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <common/rrts.h>

namespace shop
{
namespace pathplan
{
class GlobalBase : public shop::common::RRTS
{
public:
  GlobalBase(std::string name) : common::RRTS(name, 1)
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

    robot1_coordinate_sub_ = nh_.subscribe<data::Coord>("robot1_web/coord_now", 10, boost::bind(&GlobalBase::Robo1CoordNowCB, this, _1));
    robot2_coordinate_sub_ = nh_.subscribe<data::Coord>("robot1_web/coord_now", 10, boost::bind(&GlobalBase::Robo1CoordNowCB, this, _1));
    robot3_coordinate_sub_ = nh_.subscribe<data::Coord>("robot1_web/coord_now", 10, boost::bind(&GlobalBase::Robo1CoordNowCB, this, _1));
    robot4_coordinate_sub_ = nh_.subscribe<data::Coord>("robot1_web/coord_now", 10, boost::bind(&GlobalBase::Robo1CoordNowCB, this, _1));

    roadblock_read_clt_ = nh_.serviceClient<data::Roadblock>("shop/roadblock_read_srv");

    robot1_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot1/target_coordinate_read");
    robot2_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot2/target_coordinate_read");
    robot3_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot3/target_coordinate_read");
    robot4_target_coordinate_read_clt_ = nh_.serviceClient<data::Coordinate>("shop/robot4/target_coordinate_read");
  }
  virtual ~GlobalBase() = default;

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

  int8_t GetRoadblock()
  {
    data::Roadblock srv;
    roadblock_read_clt_.call(srv);
    return srv.response.number;
  }

  data::Coord GetTargetCoord(uint8_t robot_num)
  {
    data::Coordinate srv;
    data::Coord result;
    switch (robot_num)
    {
    case 1:
      robot1_target_coordinate_read_clt_.call(srv);
      break;
    case 2:
      robot2_target_coordinate_read_clt_.call(srv);
      break;
    case 3:
      robot3_target_coordinate_read_clt_.call(srv);
      break;
    case 4:
      robot4_target_coordinate_read_clt_.call(srv);
      break;
    default:
      ROS_ERROR("no robot num in %s in %s", __FUNCTION__, name_.c_str());
      break;
    }
    result.x = srv.x;
    result.y = srv.y;
    result.pose = result.pose;
    return result;
  }

protected:
  ros::NodeHandle nh_;
  data::Coord robot1_coord_now_;
  data::Coord robot2_coord_now_;
  data::Coord robot3_coord_now_;
  data::Coord robot4_coord_now_;
  ros::Subscriber robot1_coordinate_sub_;
  ros::Subscriber robot2_coordinate_sub_;
  ros::Subscriber robot3_coordinate_sub_;
  ros::Subscriber robot4_coordinate_sub_;
  ros::ServiceClient roadblock_read_clt_;

  ros::ServiceClient robot1_target_coordinate_read_clt_;
  ros::ServiceClient robot2_target_coordinate_read_clt_;
  ros::ServiceClient robot3_target_coordinate_read_clt_;
  ros::ServiceClient robot4_target_coordinate_read_clt_;

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

private:
};

} // namespace pathplan

} // namespace shop

#endif