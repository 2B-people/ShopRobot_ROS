#ifndef WEB_SERVER_CALSS_H
#define WEB_SERVER_CALSS_H

#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdint.h>
#include <thread>
#include <unistd.h>

//#include<data/.h>   //msg文件

#include "common/log.h"
#include "common/main_interface.h"
#include "common/rrts.h"

#include <data/MoveAction.h>
#include <data/ShopGripAction.h>

#include <data/Goods.h>
#include <data/ShelfBarrier.h>
#include <data/Roadblock.h>

namespace shop{
namespace webserver{

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

  //name
  std::string name_;

  //socket
  uint16_t bind_port_;
  std::string server_addr_;


};

} // namespace webserver
} // namespace shop

#endif