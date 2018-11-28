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

namespace shop {
namespace mcuserial {

class McuSerial : public shop::common::rrts {
public:
  McuSerial(std::string name);
  ~McuSerial();
  void Run();
  void Stop();
  void Resume();

private:
  void initSerial();
  void controlCB(const geometry_msgs::Twist::ConstPtr &vel);
  void ReceiveLoop();
  void SendPack();

  bool is_open_;
  bool stop_read_;
  bool stop_send_;


  ros::Publisher scan_pub_, romote_pub_;
  ros::Subscriber sub_cmd_vel_;

  std::thread *receive_loop_thread_, *send_loop_thread_;

  serial::Serial ser_;
  std::string mcu_port_;
  int mcu_baudrate_;

  void SendDataHandle();
  void SendDataHandle(Connect_Typedef co);
  uint16_t DatatoType(uint8_t *buff);
  int32_t DatatoInt(uint8_t *buff);
  float DatatoFloat(uint8_t *buff, int mult);
};

} // namespace mcuserial
} // namespace shop

#endif