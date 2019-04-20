#ifndef WEB_SERVER_CALSS_H
#define WEB_SERVER_CALSS_H

//ros
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <actionlib/server/simple_action_server.h>

//cpp std
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <memory>
#include <thread>
#include <unistd.h>

#include "common/log.h"
#include "common/main_interface.h"
#include "common/rrts.h"

//action
#include <data/MoveAction.h>
#include <data/ShopActionAction.h>
#include <data/OpeningAction.h>

//server
#include <data/Goods.h>
#include <data/ShelfBarrier.h>
#include <data/Roadblock.h>
#include <data/ActionName.h>

//msg
#include <data/Action.h>
#include <data/Cargo.h>
#include <data/Coord.h>
#include <data/RoadblockMsg.h>
#include <data/Barrier.h>

namespace shop
{
namespace webserver
{
#define BUFF_MAX 9

typedef actionlib::SimpleActionServer<data::MoveAction> MOVEACTIONSERVER;
typedef actionlib::SimpleActionServer<data::ShopActionAction> SHOPACTIONSERVER;
typedef actionlib::SimpleActionServer<data::OpeningAction> OPENINGACTIONSERVER;

class WebServer : public shop::common::RRTS
{
public:
  WebServer(std::string name);
  ~WebServer();
  void Run(void);
  void Stop();
  void Resume();

private:
  //running state
  bool is_open_;
  bool move_stop_;
  bool shop_stop_;
  bool open_stop_;
  bool go_flag_;
  bool move_flag_;

  //socket
  int client_sockfd_;
  int server_sockfd_;
  int bind_port_;
  bool is_debug_;
  std::string server_addr_;
  struct sockaddr_in my_addr_;     //服务器网络地址结构体
  struct sockaddr_in remote_addr_; //客户端网络地址结构体
  //data
  data::Coord now_coord_;
  data::Coord target_coord_;
  data::Coord cmd_coord_;
  data::Action target_action_;

  // ros
  ros::NodeHandle nh_;
  //action
  MOVEACTIONSERVER move_as_;
  SHOPACTIONSERVER action_as_;
  OPENINGACTIONSERVER opening_as_;
  //service
  ros::ServiceClient roadblock_client_;
  ros::ServiceClient c_shelf_barrier_client_;
  ros::ServiceClient b_shelf_barrier_client_;
  ros::ServiceClient d_shelf_barrier_client_;
  ros::ServiceClient action_client_;
  //publish
  ros::Publisher move_pub_;

  //sub
  ros::Subscriber move_target_sub_;
  ros::Subscriber move_cmd_sub_;
  ros::Subscriber action_sub_;

  //function
  bool InitWeb();
  void MoveExecuteCB(const data::MoveGoal::ConstPtr &goal);
  void ShopExecuteCB(const data::ShopActionGoal::ConstPtr &goal);
  void OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal);

  void MovePreemptCB();
  void ShopPreemptCB();
  void OpenPreemptCB();

  void TargetCoordCB(const data::Coord::ConstPtr &msg);
  void CmdCoordCB(const data::Coord::ConstPtr &msg);
  void TargetActionCB(const data::Action::ConstPtr &msg);

  data::Coord DataToCoord(std::string buf);
  void DataToBarrier(std::string temp);
  std::string CoordToData(data::Coord temp);

  bool Send(std::string temp);
  std::string Recv(void);
};

} // namespace webserver
} // namespace shop

#endif
