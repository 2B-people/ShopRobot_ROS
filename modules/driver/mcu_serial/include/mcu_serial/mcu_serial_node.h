#ifndef MCU_SERIAL_NODE_H
#define MCU_SERIAL_NODE_H

#include <ros/ros.h>
#include <serial/serial.h>

#include <stdint.h>
#include <thread>
#include <unistd.h>

#include <data/CarScan.h>
#include <data/Remote.h>
#include <mcu_serial/infantry_info.h>

#include "common/main_interface.h"
#include "common/log.h"
#include "common/rrts.h"

namespace shop {
namespace mcuserial {

typedef struct FifoData {
  int *base;
  int read;
  int front;
} fifo_t_e;

class McuSerial : public shop::common::rrts {
public:
  McuSerial(std::string name);
  ~McuSerial();
  void Run();
  void Stop();
  void Resume();

private:
  void initSerial();

  //TODO topic的回调
  //void controlCB(const geometry_msgs::Twist::ConstPtr &vel);
  void ReceiveLoop();
  void SendPack();

  //running state
  bool is_open_;
  bool stop_read_;
  bool stop_send_;

  //fifo
  fifo_t_e send_fifo_p_;
  fifo_t_e read_fifo_P_;  
  Connect_Typedef send_fifo_[10];
  Connect_Typedef read_fifo_[10];

  float distance_f_, distance_b_, distance_l_, distance_r_;

  //ros
  ros::Publisher scan_pub_, romote_pub_;
  ros::Subscriber sub_cmd_vel_;

  //thread
  std::thread *receive_loop_thread_, *send_loop_thread_;


  //ttyUSB setting
  std::string mcu_port_;
  int mcu_baudrate_;
  serial::Serial ser_;

  void ReceiveDataHandle(uint8_t *data);
  void SendDataHandle(Connect_Typedef co);

  void AddSendFifo(Connect_Typedef co);
  void AddReadFifo(Connect_Typedef co)

  uint16_t DatatoType(uint8_t *buff);
  int32_t DatatoInt(uint8_t *buff);
  float DatatoFloat(uint8_t *buff, int mult);

  void Typetodata(uint16_t type, uint8_t *type);
  void InttoData(uint32_t data,uint8_t *data_type)


};

} // namespace mcuserial
} // namespace shop

#endif