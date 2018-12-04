#include <mcu_serial/mcu_serial_node.h>

namespace shop {
namespace serial {

McuSerial::McuSerial(std::string name)
    : common::RRTS(name), mcu_port_("/dev/ttyUSB0"), mcu_baudrate_(115200),
      is_open_(false), stop_read_(true), stop_send_(true), stop_topic_(true) {
  ros::NodeHandle nh_private_("~");

  is_open_ = true;
  stop_read_ = false;
  stop_send_ = false;
  stop_topic_ = false;

  send_fifo_p_.read = 0;
  send_fifo_p_.front = 0;
  read_fifo_p_.read = 0;
  read_fifo_p_.front = 0;

  for (size_t i = 0; i < BUFF_MAX; i++)
  {
    send_buff_[i].type = 0;
    send_buff_[i].data_16 = 0;
    send_buff_[i].data_32 = 0;
    send_buff_[i].data_16_u = 0;
    send_buff_[i].data_32_u = 0;
    read_buff_[i].type = 0;
    read_buff_[i].data_16 = 0;
    read_buff_[i].data_32 = 0;
    read_buff_[i].data_16_u = 0;
    read_buff_[i].data_32_u = 0;
  }
  
  nh_private_.getParam("Prot", mcu_port_);
  nh_private_.getParam("Baudrate", mcu_baudrate_);

  scan_pub_ = nh_.advertiseService("scan_data", &data::CanScan, this);
  romote_pub_ = nh_.advertiseService("romote_status", &data::Romote, this);
}

void McuSerial::Run() {
  initSerial();
  read_thread_ = new std::thread(boost::bind(&McuSerial::ReceiveLoop, this));
  topic_thread_ = new std::thread(boost::bind(&McuSerial::TopicLoop, this));
  send_loop_thread_ = new std::thread(boost::bind(&McuSerial::SendPack, this));

  // TODO topic回调函数
  romote_public_ = n.createTimer(ros::Duration(0.1), McuSerial::RomoteCB);
  scan_public_ = n.createTimer(ros::Duration(0.1), McuSerial::ScanCB);

  ros::spin();
}

void McuSerial::Stop() {
  stop_read_ = true;
  stop_topic_ = true;
  stop_send_ = true;
}

void McuSerial::Resume() {
  stop_read_ = false;
  stop_topic_ = false;
  stop_send_ = false;
}

McuSerial::~McuSerial() {
  ROS_WARN("McuSerial class is die!");
  if (read_thread_ != nullptr) {
    stop_read_ = true;
    read_thread_->join();
    delete read_thread_;
  }
  if (send_loop_thread_ != nullptr) {
    stop_send_ = true;
    send_loop_thread_->join();
    delete send_loop_thread_;
  }
  if (topic_thread_ != nullptr) {
    stop_topic_ = true;
    topic_thread_->join();
    delete topic_thread_;
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
  ROS_INFO("ReceiveLoop is ready!");
  while (is_open_ && !stop_read_ && ros::ok()) {
    if (ser_.available()) {
      uint8_t hand[3];
      if (ser_.read(hand, 2) && (hand[0] == MSGHANDL) &&
          (hand[1] == MSGHANDW)) {
        uint8_t len = hand[3] & 0x0F;
        uint8_t type = (hand[3] >> 4) & 0x0F;
        ReceiveDataHandle(len, type);
      }
    }
  }
}

// TODO(nqq)  完成回调函数
void McuSerial::RomoteCB(const ros::TimerEvent &) {
  ros::Time current_time = ros::Time::now();
  scan_pub_.publish(scan_pub_);
  
}
void McuSerial::ScanCB(const ros::TimerEvent &) {
  romote_pub_.publish(romote_pub_);
}

void McuSerial::TopicLoop() {
  ros::Time now_time = ros::Time::now();
  ros::Duration current_time;
  ROS_INFO("TopicLoop is ready!");
  Connect_Typedef co;
  while (is_open_ && !stop_topic_ && ROS::OK()) {
    co.type = read_fifo_[read_fifo_p_.front].type;
    co.data_16 = read_fifo_[read_fifo_p_.front].data_16;
    co.data_32 = read_fifo_[read_fifo_p_.front].data_32;
    co.data_16_u = read_fifo_[read_fifo_p_.front].data_16_u;
    co.data_32_u = read_fifo_[read_fifo_p_.front].data_32_u;
    read_fifo_[read_fifo_p_.front].type = 0;
    read_fifo_[read_fifo_p_.front].data_16 = 0;
    read_fifo_[read_fifo_p_.front].data_32 = 0;
    read_fifo_[read_fifo_p_.front].data_16_u = 0;
    read_fifo_[read_fifo_p_.front].data_32_u = 0;
    read_fifo_p_.front = (read_fifo_p_.front + 1) % BUFF_MAX;

    switch (co.type) {
    case MSG_DISTANCE_F:
      scan_pub_.distance_f = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_B:
      scan_pub_.distance_b = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_L:
      scan_pub_.distance_l = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_R:
      scan_pub_.distance_r = (float)(co.data_32 / 100);
      break;
    case MSG_REMOTE_CH0:
      romote_pub_.ch0 = co.data_16_u;
      break;
    case MSG_REMOTE_CH1:
      romote_pub_.ch1 = co.data_16_u;
      break;
    case MSG_REMOTE_CH2:
      romote_pub_.ch2 = co.data_16_u;
      break;
    case MSG_REMOTE_CH3:
      romote_pub_.ch3 = co.data_16_u;
      break;
    case MSG_REMOTE_S1:
      romote_pub_.s1 = co.data_16_u;
      break;
    case MSG_REMOTE_S2:
      romote_pub_.s2 = co.data_16_u;
      break;

    default:
      ROS_WARN("Warn:no msg ,please debug in mcu");
    }
  }
}

void McuSerial::SendPack() {
  Connect_Typedef co;
  ROS_INFO("SendPack is ready!");
  while (is_open_ && !stop_send_ && ROS::OK()) {
    co.type = send_fifo_[send_fifo_p_.front].type;
    co.data_16 = send_fifo_[send_fifo_p_.front].data_16;
    co.data_32 = send_fifo_[send_fifo_p_.front].data_32;
    co.data_16_u = send_fifo_[send_fifo_p_.front].data_16_u;
    co.data_32_u = send_fifo_[send_fifo_p_.front].data_32_u;
    send_fifo_[send_fifo_p_.front].type = 0;
    send_fifo_[send_fifo_p_.front].data_16 = 0;
    send_fifo_[send_fifo_p_.front].data_32 = 0;
    send_fifo_[send_fifo_p_.front].data_16_u = 0;
    send_fifo_[send_fifo_p_.front].data_32_u = 0;
    send_fifo_.front = (send_fifo_p_.front + 1) % BUFF_MAX;
  }
}

void SerialComNode::ReceiveDataHandle(uint8_t len, uint8_t type) {
  uint8_t buff[len * 5 + 2];
  ser_.read(buff, (len * 5 + 2));
  for (size_t i = 0; i < len; i++) {
    ReadDatatoTopicHandle(&buff[i * 5]);
  }
  if (buff[len * 5] != 0x55 || buff[len * 5 + 1] != 0x39) {
    ROS_WARN("warn: msg is error ,mcu's bug");
  }
}

//重写
void McuSerial::SendDataHandle(Connect_Typedef co) {}

void McuSerial::ReadDatatoTopicHandle(uint8_t *buff, uint8_t type) {
  Connect_Typedef co;
  co.type = buff[0];
  switch (type) {
  case INT16_T_TYPE:
    co.data_16 = DatatoInt16(&buff[1]);
    co.data_32 = 0;
    co.data_16_u = 0;
    co.data_32_u = 0;
    break;
  case INT32_T_TYPE:
    co.data_32 = DatatoInt32(&buff[1]);
    co.data_16 = 0;
    co.data_16_u = 0;
    co.data_32_u = 0;
    break;
  case UINT16_T_TYPE:
    co.data_16_u = DatatoUint16(&buff[1]);
    co.data_16 = 0;
    co.data_32 = 0;
    co.data_32_u = 0;
    break;
  case UINT32_T_TYPE:
    co.data_32_u = DatatoUint32(&buff[1]);
    co.data_16 = 0;
    co.data_32 = 0;
    co.data_16_u = 0;
    break;
  default:
    ROS_WARN("warn: msg is error ,mcu's bug");
  }

  AddReadFifo(co);
}

void McuSerial::AddSendFifo(Connect_Typedef co) {
  if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front) {
    ROS_WARN("Fifo is full!!")
    sleep(100);
    if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front) {
      send_fifo_[send_fifo_p_.read].type = co.type;
      send_fifo_[read_fifo_p_.read].data_16 = co.data_16;
      send_fifo_[read_fifo_p_.read].data_32 = co.data_32;
      send_fifo_[read_fifo_p_.read].data_16_u = co.data_16_u;
      send_fifo_[read_fifo_p_.read].data_32_u = co.data_32_u;
      send_fifo_p_.read = (send_fifo_p_.read + 1) % BUFF_MAX;
    }
  } else {
    send_fifo_[send_fifo_p_.read].type = co.type;
    send_fifo_[read_fifo_p_.read].data_16 = co.data_16;
    send_fifo_[read_fifo_p_.read].data_32 = co.data_32;
    send_fifo_[read_fifo_p_.read].data_16_u = co.data_16_u;
    send_fifo_[read_fifo_p_.read].data_32_u = co.data_32_u;
    send_fifo_p_.read = (send_fifo_p_.read + 1) % BUFF_MAX;
  }
}

void McuSerial::AddReadFifo(Connect_Typedef co) {
  if ((read_fifo_p_.read + 1) % 10 == read_fifo_p_.front) {
    ROS_WARN("Fifo is full!!")
    sleep(100);
    if ((read_fifo_p_.read + 1) % 10 == read_fifo_p_.front) {
      read_fifo_[read_fifo_p_.read].type = co.type;
      read_fifo_[read_fifo_p_.read].data_16 = co.data_16;
      read_fifo_[read_fifo_p_.read].data_32 = co.data_32;
      read_fifo_[read_fifo_p_.read].data_16_u = co.data_16_u;
      read_fifo_[read_fifo_p_.read].data_32_u = co.data_32_u;
      read_fifo_p_.read = (read_fifo_p_.read + 1) % BUFF_MAX;
    }
  } else {
    read_fifo_[read_fifo_p_.read].type = co.type;
    read_fifo_[read_fifo_p_.read].data_16 = co.data_16;
    read_fifo_[read_fifo_p_.read].data_32 = co.data_32;
    read_fifo_[read_fifo_p_.read].data_16_u = co.data_16_u;
    read_fifo_[read_fifo_p_.read].data_32_u = co.data_32_u;
    read_fifo_p_.read = (read_fifo_p_.read + 1) % BUFF_MAX;
  }
}

/***************************************************/
uint16_t McuSerial::DatatoType(uint8_t *buff) {
  uint16_t type = buff[0];
  type = type << 8;
  type |= buff[1];
  return type;
}

int16_t McuSerial::DatatoInt16(uint8_t *buff) {
  int16_t data;
  data = buff[0];
  data = data << 8;
  data |= buff[i];

  return data;
}

int32_t McuSerial::DatatoInt32(uint8_t *buff) {
  int32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++) {
    data = data << 8;
    data |= buff[i];
  }
  return data;
}

uint16_t McuSerial::DatatoUint16(uint8_t *buff) {
  uint16_t data;
  data = buff[0];
  data = data << 8;
  data |= buff[i];

  return data;
}

uint32_t McuSerial::DatatoUint32(uint8_t *buff) {
  uin32_t data;
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

void Int16toData(int16_t data, uint8_t *data_type) {
  data_type[0] = (uint8_t)type & 0x00FF;
  data_type[1] = (uint8_t)(type >> 8) & 0x00FF;
}

void Int32toData(int32_t data, uint8_t *data_type) {
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

void Uint16toData(uint16_t data, uint8_t *data_type) {
  data_type[0] = (uint8_t)type & 0x00FF;
  data_type[1] = (uint8_t)(type >> 8) & 0x00FF;
}

void Uint32toData(uint32_t data, uint8_t *data_type) {
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

float FloattoData(float data_float, int mult, uint8_t *data_type) {
  uin32_t data = (uin32_t)(data_float * mult);
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

} // namespace serial
} // namespace shop

MAIN("mcu_serial", shop::serial::McuSerial)
