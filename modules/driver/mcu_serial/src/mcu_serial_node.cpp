#include <mcu_serial/mcu_serial_node.h>

namespace shop {
namespace serial {

McuSerial::McuSerial(std::string name)
    : common::RRTS(name), mcu_port_("/dev/ttyUSB0"), mcu_baudrate_(115200),
      is_open_(false), stop_read_(true), stop_send_(true), {
  ros::NodeHandle nh_private_("~");

  is_open_ = true;
  stop_read_ = false;
  stop_send_ = false;
  nh_private_.getParam("Prot", mcu_port_);
  nh_private_.getParam("Baudrate", mcu_baudrate_);
  distance_f_ = 0;
  distance_b_ = 0;
  distance_l_ = 0;
  distance_r_ = 0;
  send_fifo_p_.front = send_fifo_p_.read = 0;
  for (size_t i = 0; i < 10; i++) {
    send_fifo_[i].type = 0;
    send_fifo_[i].data = 0;
  }
  scan_pub_ = nh_.advertiseService("scan_data", &data::CanScan, this);
  romote_pub_ = nh_.advertiseService("romote_status", &data::Romote, this);
}

void McuSerial::Run() {
  initSerial();
  receive_loop_thread_ =
      new std::thread(boost::bind(&McuSerial::ReceiveLoop, this));
  // TODO topic回调函数
  // sub_cmd_vel_ = nh_.subscribe("cmd_vel", 1, &McuSerial::controlCB, this);
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
  is_open_ = false;
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

void McuSerial::ReceiveLoop() {
  while (is_open_ && !stop_read_ && ros::ok()) {
    if (ser_.available()) {
      uint8_t hand[2];
      if (ser_.read(hand, 2) && (hand[0] == 0x89) && (hand[1] == 0x89)) {
        uint8_t buff[8];
        ser_.read(buff, 8);
        ReceiveDataHandle(buff);
      }
    }
  }
}

// TODO(nqq)  完成回调函数



void McuSerial::SendPack() {
  Connect_Typedef co;
  while (is_open_ && !stop_send_ && ROS::OK()) {
    co.type = send_fifo_[send_fifo_p_.front].type;
    co.data = send_fifo_[send_fifo_p_.front].data;
    send_fifo_[send_fifo_p_.front].type = 0;
    send_fifo_[send_fifo_p_.front].data = 0;
    if (co.type != 0) {
      SendDataHandle(co);
    }
    send_fifo_p_.front = (send_fifo_p_.front + 1) % 10;
  }
}

void SerialComNode::ReceiveDataHandle(uint8_t *data) {
  uint16_t type = DatatoType(data);
  int32_t data_int = DatatoInt(&data[2]);
  float data_float = DatatoFloat(&data[2], 100);
  switch (type) {
  case MSG_remote_ch0:

    break;
  case MSG_remote_ch1:

    break;
  case MSG_remote_ch2:

    break;
  case MSG_remote_ch3:

    break;
  case MSG_remote_s1:

    break;
  case MSG_remote_s2:

    break;
  default:
    ROS_WARN("happen data error")
  }
}

void McuSerial::SendDataHandle(Connect_Typedef co) {
  uint8_t buff[10];
  buff[0] = buff[1] = 0x89;
  buff[8] = buff[9] = 0xfe;
  Typetodata(co.type,*buff[2]);
  InttoData(co.data,*buff[4]);
  ser_.write(buff,10);
}

void McuSerial::AddSendFifo(Connect_Typedef co) {
  if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front) {
    ROS_WARN("Fifo is full!!")
    sleep(100);
    if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front) {
      send_fifo_[send_fifo_p_.read].type = co.type;
      send_fifo_[send_fifo_p_.read].data = co.data;
      send_fifo_p_.read = (send_fifo_p_.read + 1) % MAX;
    }
  } else {
    send_fifo_[send_fifo_p_.read].type = co.type;
    send_fifo_[send_fifo_p_.read].data = co.data;
    send_fifo_p_.read = (send_fifo_p_.read + 1) % MAX;
  }
}


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

void McuSerial::TypetoData(uint16_t type, uint8_t *data_type) {
  data_type[0] = (uint8_t)type & 0x00FF;
  data_type[1] = (uint8_t)(type >> 8) & 0x00FF;
}

void McuSerial::InttoData(uint32_t data, uint8_t *data_type) {
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

} // namespace serial
} // namespace shop

MAIN("mcu_serial",shop::serial::McuSerial)

