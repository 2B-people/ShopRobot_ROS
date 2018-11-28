#include <mcu_serial/mcu_serial_node.h>

namespace shop {
namespace serial {

McuSerial::McuSerial(std::string name)
    : common::RRTS(name), mcu_port_("/dev/ttyUSB0"), mcu_baudrate_(115200),
      is_open_(false), stop_read_(true), stop_send_(true), {
  is_open_ = true;
  stop_receive_ = false;
  stop_send_ = false;
  scan_pub_ = nh_.advertiseService("scan_data", &data::CanScan, this);
  romote_pub_ = nh_.advertiseService("romote_status", &data::Romote, this);
}

void McuSerial::Run() {
  initSerial();
  receive_loop_thread_ =
      new std::thread(boost::bind(&McuSerial::ReceiveLoop, this));
  sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &McuSerial::controlCB, this);
  send_loop_thread_ = new std::thread(boost::bind(&McuSerial::SendPack, this));
  ros::spin();
}

void McuSerial::Stop() {
  stop_read_ = true;
  stop_send_ = true;
}

void McuSerial::Resume() {
  stop_read_ = false;
  stop_send_ = false;
}

McuSerial::~McuSerial() {
  ROS_WARN("McuSerial class is die!");
  if (receive_loop_thread_ != nullptr) {
    stop_read_ = true;
    receive_loop_thread_->join();
    delete receive_loop_thread_;
  }
  if (send_loop_thread_ != nullptr) {
    stop_send_ = true;
    send_loop_thread_->join();
    delete send_loop_thread_;
  }
  is_open_ = false
}

McuSerial::initSerial() {
  ser_.setPort(mcu_port_);
  ser_.setBaudrate(mcu_baudrate_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  ser_.setTimeout(to);
  ser_.open();

  while (ser_.isOpen() == 0) {
    NOTICE("Warting MCU!!");
  }
  ROS_INFO("MCU is ready!!");
}

// TODO(nqq)  完成接受进程
void McuSerial::ReceiveLoop() {
  while (is_open_ && !stop_read_ && ros::ok()) {
    if (ser_.available()) {
      uint8_t hand[2];
      if (ser_.read(hand, 2) && (hand[0] == 0x89) && (hand[1] == 0x89)) {
        DataHandle();
      }
    }
  }
}

// TODO(nqq)  完成回调函数
void McuSerial::controlCB(const geometry_msgs::Twist::ConstPtr &vel) {
    
}

void McuSerial::SendPack() {
  while (is_open_ && !stop_send_ && ROS::OK()) {
  }
}



void SerialComNode::DataHandle(){      
}


void McuSerial::SendDataHandle(Connect_Typedef co) {}



uint16_t McuSerial::DatatoType(uint8_t *buff) {
  uint16_t type = buff[0];
  type = type << 8;
  type |= buff[1];
  return type;
}

int32_t McuSerial::DatatoInt(uint8_t *buff) {
  int32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++) {
    data = data << 8;
    data |= buff[i];
  }
  return data;
}

float McuSerial::DatatoFloat(uint8_t *buff, int mult) {
  int32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++) {
    data = data << 8;
    data |= buff[i];
  }
  return (float)data / mult;
}

} // namespace serial
} // namespace shop
