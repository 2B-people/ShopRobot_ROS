#ifndef MCU_SERIAL_NODE_H
#define MCU_SERIAL_NODE_H

#include <ros/duration.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <serial/serial.h>

#include <stdint.h>
#include <string.h>
#include <thread>
#include <unistd.h>

#include <data/CarScan.h>
#include <data/Remote.h>
#include <mcu_serial/infantry_info.h>

#include "common/log.h"
#include "common/main_interface.h"
#include "common/rrts.h"

namespace shop {
namespace mcuserial {

typedef struct FifoData {
  int *base;
  int read;
  int front;
} fifo_t_e;

#define FIFO_BUFF_MAX 20
#define SEND_FRAME_MAX 100

class McuSerial : public shop::common::RRTS {
public:
  McuSerial(std::string name);
  ~McuSerial();
  void Run();
  void Stop();
  void Resume();

private:
  // running state
  bool is_open_;
  bool stop_read_;
  bool stop_send_;
  bool stop_topic_;

  // fifo
  fifo_t_e send_fifo_p_;
  fifo_t_e read_fifo_p_;
  Connect_Typedef send_fifo_buff_[FIFO_BUFF_MAX];
  Connect_Typedef read_fifo_buff_[FIFO_BUFF_MAX];

  // send
  uint8_t send_frames_buff_[SEND_FRAME_MAX];
  uint8_t frames_len_;
  uint8_t frames_type_;
  uint16_t frames_now_;

  // ros
  ros::Publisher scan_pub_, romote_pub_;
  ros::Subscriber sub_cmd_vel_;
  ros::NodeHandle nh_;

  // data
  data::Remote remote_msg_;
  data::CarScan carscan_msg_;
  // thread
  std::thread *read_thread_, *send_loop_thread_, *topic_thread_;

  // ttyUSB setting
  std::string mcu_port_;
  int mcu_baudrate_;
  serial::Serial ser_;

  // void controlCB(const geometry_msgs::Twist::ConstPtr &vel);
  void ReceiveLoop();
  void SendPack();
  void TopicLoop();
  // TODO topic回调函数

  void ReceiveDataHandle(uint8_t len, uint8_t type);
  void SendDataHandle(Connect_Typedef co);
  void ReadDatatoTopicHandle(uint8_t *buff, uint8_t type);

  void initSerial();

  // fifo
  void AddSendFifo(Connect_Typedef co);
  void AddReadFifo(Connect_Typedef co);
  void GetSendFifo(Connect_Typedef *co);
  void GetReadFifo(Connect_Typedef *co);

  // tool
  void SendFramesZero();
  int16_t DatatoInt16(uint8_t *buff);
  int32_t DatatoInt32(uint8_t *buff);
  uint16_t DatatoUint16(uint8_t *buff);
  uint32_t DatatoUint32(uint8_t *buff);
  float DatatoFloat(uint8_t *buff, int mult);
  uint16_t DatatoType(uint8_t *buff);

  void TypetoData(uint8_t len, uint8_t type, uint8_t *data_type);
  void Int16toData(int16_t data, uint8_t *data_type);
  void Int32toData(int32_t data, uint8_t *data_type);
  void Uint16toData(uint16_t data, uint8_t *data_type);
  void Uint32toData(uint32_t data, uint8_t *data_type);
  float FloattoData(float data, int mult, uint8_t *data_type);
};

} // namespace mcuserial
} // namespace shop

#endif