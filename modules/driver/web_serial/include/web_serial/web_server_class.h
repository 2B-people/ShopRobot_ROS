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

namespace shop
{
namespace webserver
{

typedef struct FifoData
{
  int *base;
  int read;
  int front;
} fifo_t_e;

#define FIFOMAX 100

#define SEND 1
#define READ 2

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
  bool stop_read_;
  bool stop_send_;
  bool stop_topic_;

  //name
  std::string name_;

  // fifo
  fifo_t_e send_fifo_p_;
  fifo_t_e read_fifo_P_;
  std::string[FIFOMAX] send_fifo_buff_;
  std::string[FIFOMAX] read_fifo_buff_;

  //thread
  std::thread *read_thread_, *topic_thread_, *send_thread_;

  /*******funtion***********/
  void initWeb();

  void TopicLoop();
  void SendLoop();
  void ReadLoop();

  void AddFifo(std::string data, uint8_t fun);
  void GetReadFifo(std::string *data, uint8_t fun)
};

} // namespace webserver
} // namespace shop

#endif