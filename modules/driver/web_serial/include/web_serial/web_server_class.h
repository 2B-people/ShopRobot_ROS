#ifndef WEB_SERVER_CALSS_H
#define WEB_SERVER_CALSS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <actionlib/server/simple_action_server.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdint.h>
#include <thread>
#include <unistd.h>

//#include<data/.h>   //msg文件

#include "common/log.h"
#include "common/main_interface.h"
#include "common/rrts.h"

#include <data/MoveAction.h>
#include <data/ShopActionAction.h>
#include <data/OpeningAction.h>

#include <data/Goods.h>
#include <data/ShelfBarrier.h>
#include <data/Roadblock.h>

#include <data/Coord.h>

namespace shop
{
namespace webserver
{
#define BUFF_MAX 100

typedef actionlib::SimpleActionServer<data::MoveAction> MOVEACTIONSERVER;
typedef actionlib::SimpleActionServer<data::ShopActionAction> SHOPACTIONSERVER;
typedef actionlib::SimpleActionServer<data::OpeningAction> OPENINGACTIONSERVER;

class WebServer : public shop::common::RRTS
{
public:
  WebServer(std::string name);
  ~WebServer();
  void Run();
  void Stop();
  void Resume();

private:
  //running state
  bool is_open_;
  bool move_stop_;
  bool shop_stop_;
  bool open_stop_;

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
  // ros
  ros::NodeHandle nh_;
  //action
  MOVEACTIONSERVER move_as_;
  SHOPACTIONSERVER action_as_;
  OPENINGACTIONSERVER opening_as_;
  //service
  ros::ServiceClient roadblock_client_;
  ros::ServiceClient shelf_barrier_client_;
  //publish
  ros::Publisher move_pub_;

  //function
  bool InitWeb();
  void MoveExecuteCB(const data::MoveGoal::ConstPtr &goal);
  void ShopExecuteCB(const data::ShopActionGoal::ConstPtr &goal);
  void OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal);

  void MovePreemptCB();
  void ShopPreemptCB();
  void OpenPreemptCB();

  data::Coord DataToCoord(const char *buf);
  data::ShelfBarrier DataToBarrier(std::string temp);
  std::string CoordToData(data::Coord temp);

  bool Send(std::string temp);
  std::string Recv();
};

} // namespace webserver
} // namespace shop

#endif