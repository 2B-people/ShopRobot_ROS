#include <mcu_serial/mcu_serial_node.h>

namespace shop
{
namespace mcuserial
{

McuSerial::McuSerial(std::string name)
    : common::RRTS(name), mcu_port_("/dev/ttyUSB0"), mcu_baudrate_(115200),
      is_open_(false), stop_read_(true), stop_send_(true), stop_topic_(true)
{
  ros::NodeHandle nh_private_("~");

  //启动标志
  is_open_ = true;
  stop_read_ = false;
  stop_send_ = false;
  stop_topic_ = false;

  //Frames to zero
  SendFramesZero();

  //fifo指针
  send_fifo_p_.read = 0;
  send_fifo_p_.front = 0;
  read_fifo_p_.read = 0;
  read_fifo_p_.front = 0;

  //fifo zero
  for (size_t i = 0; i < FIFO_BUFF_MAX; i++)
  {
    send_fifo_buff_[i].type = 0;
    send_fifo_buff_[i].data_16 = 0;
    send_fifo_buff_[i].data_32 = 0;
    send_fifo_buff_[i].data_16_u = 0;
    send_fifo_buff_[i].data_32_u = 0;
    read_fifo_buff_[i].type = 0;
    read_fifo_buff_[i].data_16 = 0;
    read_fifo_buff_[i].data_32 = 0;
    read_fifo_buff_[i].data_16_u = 0;
    read_fifo_buff_[i].data_32_u = 0;
  }

  //serial init
  nh_private_.getParam("McuSerialProt", mcu_port_);
  nh_private_.getParam("McuSerialBaudrate", mcu_baudrate_);

  //publish init
  scan_pub_ = nh_.advertise<data::Remote>("remote",30);
  romote_pub_ = nh_.advertise<data::CarScan>("carscan",30);
}

void McuSerial::Run()
{
  initSerial();

  read_thread_ = new std::thread(boost::bind(&McuSerial::ReceiveLoop, this));
  topic_thread_ = new std::thread(boost::bind(&McuSerial::TopicLoop, this));
  send_loop_thread_ = new std::thread(boost::bind(&McuSerial::SendPack, this));

  // TODO topic回调函数

  ros::spin();
}

void McuSerial::Stop()
{
  stop_read_ = true;
  stop_topic_ = true;
  stop_send_ = true;
}

void McuSerial::Resume()
{
  stop_read_ = false;
  stop_topic_ = false;
  stop_send_ = false;
}

McuSerial::~McuSerial()
{
  ROS_WARN("McuSerial class is die!");
  if (read_thread_ != nullptr)
  {
    stop_read_ = true;
    read_thread_->join();
    delete read_thread_;
  }
  if (send_loop_thread_ != nullptr)
  {
    stop_send_ = true;
    send_loop_thread_->join();
    delete send_loop_thread_;
  }
  if (topic_thread_ != nullptr)
  {
    stop_topic_ = true;
    topic_thread_->join();
    delete topic_thread_;
  }
  is_open_ = false;
}

// init serial
void McuSerial::initSerial()
{
  ser_.setPort(mcu_port_);
  ser_.setBaudrate(mcu_baudrate_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  ser_.setTimeout(to);
  ser_.open();

  while (ser_.isOpen() == 0)
  {
    NOTICE("Warting MCU!!");
  }
  ROS_INFO("MCU is ready!!");
}

// TODO(nqq)  完成回调函数

//接受进程
void McuSerial::ReceiveLoop()
{
  ROS_INFO("ReceiveLoop is ready!");
  while (is_open_ && !stop_read_ && ros::ok())
  {
    if (ser_.available())
    {
      uint8_t hand[3];
      if (ser_.read(hand, 2) && (hand[0] == MSG_HAND_L) &&
          (hand[1] == MSG_HAND_W))
      {
        uint8_t len = hand[3] & 0x0F;
        uint8_t type = (hand[3] >> 4) & 0x0F;
        ReceiveDataHandle(len, type);
      }
    }
  }
}

//发布进程
void McuSerial::TopicLoop()
{
  ros::Time now_time = ros::Time::now();
  ros::Duration current_time;
  ROS_INFO("TopicLoop is ready!");
  Connect_Typedef co;
  while (is_open_ && !stop_topic_ && ros::ok())
  {
    GetReadFifo(&co);

    switch (co.type)
    {
    case MSG_DISTANCE_F:
      carscan_msg_.distance_f = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_B:
      carscan_msg_.distance_b = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_L:
      carscan_msg_.distance_l = (float)(co.data_32 / 100);
      break;
    case MSG_DISTANCE_R:
      carscan_msg_.distance_r = (float)(co.data_32 / 100);
      scan_pub_.publish(carscan_msg_);
      break;
    case MSG_REMOTE_CH0:
      remote_msg_.ch0 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;
    case MSG_REMOTE_CH1:
      remote_msg_.ch1 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;
    case MSG_REMOTE_CH2:
      remote_msg_.ch2 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;
    case MSG_REMOTE_CH3:
      remote_msg_.ch3 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;
    case MSG_REMOTE_S1:
      remote_msg_.s1 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;
    case MSG_REMOTE_S2:
      remote_msg_.s2 = co.data_16_u;
      romote_pub_.publish(remote_msg_);
      break;

    default:
      ROS_WARN("Warn:no msg ,please debug in mcu");
    }
  }
}

//tx通信进程
void McuSerial::SendPack()
{
  Connect_Typedef co;
  ROS_INFO("SendPack is ready!");
  while (is_open_ && !stop_send_ && ros::ok())
  {
    //发送句柄
    GetSendFifo(&co);
    if (co.type != 0)
    {
      SendDataHandle(co);
    }
  }
}

void McuSerial::ReceiveDataHandle(uint8_t len, uint8_t type)
{
  uint8_t buff[len * 5 + 2];
  ser_.read(buff, (len * 5 + 2));
  for (size_t i = 0; i < len; i++)
  {
    ReadDatatoTopicHandle(&buff[i * 5], type);
  }
  if (buff[len * 5] != 0x55 || buff[len * 5 + 1] != 0x39)
  {
    ROS_WARN("warn: msg is error ,mcu's bug");
  }
}

void McuSerial::SendDataHandle(Connect_Typedef co)
{
  uint16_t frames_type_size;
  if (frames_len_ != 0)
  {
    if (send_frames_buff_[0] != MSG_HAND_L)
    {
      send_frames_buff_[0] = MSG_HAND_L;
      send_frames_buff_[1] = MSG_HAND_W;
      TypetoData(frames_len_, frames_type_, &send_frames_buff_[2]);
      frames_now_ = 3;
    }
    else
    {
      send_frames_buff_[frames_now_] = co.type;
      frames_now_++;
      switch (frames_type_)
      {
      case INT16_T_TYPE:
        Int16toData(co.data_16, &send_frames_buff_[frames_now_]);
        frames_now_ += 2;
        frames_type_size = 3;
        break;
      case INT32_T_TYPE:
        Int32toData(co.data_32, &send_frames_buff_[frames_now_]);
        frames_now_ += 4;
        frames_type_size = 5;
        break;
      case UINT16_T_TYPE:
        Uint16toData(co.data_16_u, &send_frames_buff_[frames_now_]);
        frames_type_size = 3;
        frames_now_ += 2;
        break;
      case UINT32_T_TYPE:
        Uint32toData(co.data_32_u, &send_frames_buff_[frames_now_]);
        frames_type_size = 5;
        frames_now_ += 4;
        break;
      default:
        ROS_WARN("type is error!");
        break;
      }

      if (frames_now_ == 2 + frames_type_size * frames_len_)
      {
        send_frames_buff_[++frames_now_] = MSG_END_L;
        send_frames_buff_[++frames_now_] = MSG_END_W;
        ser_.write(send_frames_buff_, frames_now_);

        SendFramesZero();
      }
    }
  }
}

void McuSerial::ReadDatatoTopicHandle(uint8_t *buff, uint8_t type)
{
  Connect_Typedef co;
  co.type = buff[0];
  switch (type)
  {
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

void McuSerial::AddSendFifo(Connect_Typedef co)
{
  if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
  {
    ROS_WARN("Fifo is full!!");
    ros::Duration(0.05).sleep();
    if ((send_fifo_p_.read + 1) % 10 == send_fifo_p_.front)
    {
      send_fifo_buff_[send_fifo_p_.read].type = co.type;
      send_fifo_buff_[send_fifo_p_.read].data_16 = co.data_16;
      send_fifo_buff_[send_fifo_p_.read].data_32 = co.data_32;
      send_fifo_buff_[send_fifo_p_.read].data_16_u = co.data_16_u;
      send_fifo_buff_[send_fifo_p_.read].data_32_u = co.data_32_u;
      send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFO_BUFF_MAX;
    }
  }
  else
  {
    send_fifo_buff_[send_fifo_p_.read].type = co.type;
    send_fifo_buff_[send_fifo_p_.read].data_16 = co.data_16;
    send_fifo_buff_[send_fifo_p_.read].data_32 = co.data_32;
    send_fifo_buff_[send_fifo_p_.read].data_16_u = co.data_16_u;
    send_fifo_buff_[send_fifo_p_.read].data_32_u = co.data_32_u;
    send_fifo_p_.read = (send_fifo_p_.read + 1) % FIFO_BUFF_MAX;
  }
}

void McuSerial::AddReadFifo(Connect_Typedef co)
{
  if ((read_fifo_p_.read + 1) % 10 == read_fifo_p_.front)
  {
    ROS_WARN("Fifo is full!!");
    ros::Duration(0.05).sleep();
    if ((read_fifo_p_.read + 1) % 10 == read_fifo_p_.front)
    {
      read_fifo_buff_[read_fifo_p_.read].type = co.type;
      read_fifo_buff_[read_fifo_p_.read].data_16 = co.data_16;
      read_fifo_buff_[read_fifo_p_.read].data_32 = co.data_32;
      read_fifo_buff_[read_fifo_p_.read].data_16_u = co.data_16_u;
      read_fifo_buff_[read_fifo_p_.read].data_32_u = co.data_32_u;
      read_fifo_p_.read = (read_fifo_p_.read + 1) % FIFO_BUFF_MAX;
    }
  }
  else
  {
    read_fifo_buff_[read_fifo_p_.read].type = co.type;
    read_fifo_buff_[read_fifo_p_.read].data_16 = co.data_16;
    read_fifo_buff_[read_fifo_p_.read].data_32 = co.data_32;
    read_fifo_buff_[read_fifo_p_.read].data_16_u = co.data_16_u;
    read_fifo_buff_[read_fifo_p_.read].data_32_u = co.data_32_u;
    read_fifo_p_.read = (read_fifo_p_.read + 1) % FIFO_BUFF_MAX;
  }
}

void McuSerial::GetReadFifo(Connect_Typedef *co)
{
  co->type = read_fifo_buff_[read_fifo_p_.front].type;
  co->data_16 = read_fifo_buff_[read_fifo_p_.front].data_16;
  co->data_32 = read_fifo_buff_[read_fifo_p_.front].data_32;
  co->data_16_u = read_fifo_buff_[read_fifo_p_.front].data_16_u;
  co->data_32_u = read_fifo_buff_[read_fifo_p_.front].data_32_u;
  read_fifo_buff_[read_fifo_p_.front].type = 0;
  read_fifo_buff_[read_fifo_p_.front].data_16 = 0;
  read_fifo_buff_[read_fifo_p_.front].data_32 = 0;
  read_fifo_buff_[read_fifo_p_.front].data_16_u = 0;
  read_fifo_buff_[read_fifo_p_.front].data_32_u = 0;
  read_fifo_p_.front = (read_fifo_p_.front + 1) % FIFO_BUFF_MAX;
}

void McuSerial::GetSendFifo(Connect_Typedef *co)
{
  co->type = send_fifo_buff_[send_fifo_p_.front].type;
  co->data_16 = send_fifo_buff_[send_fifo_p_.front].data_16;
  co->data_32 = send_fifo_buff_[send_fifo_p_.front].data_32;
  co->data_16_u = send_fifo_buff_[send_fifo_p_.front].data_16_u;
  co->data_32_u = send_fifo_buff_[send_fifo_p_.front].data_32_u;
  send_fifo_buff_[send_fifo_p_.front].type = 0;
  send_fifo_buff_[send_fifo_p_.front].data_16 = 0;
  send_fifo_buff_[send_fifo_p_.front].data_32 = 0;
  send_fifo_buff_[send_fifo_p_.front].data_16_u = 0;
  send_fifo_buff_[send_fifo_p_.front].data_32_u = 0;
  send_fifo_p_.front = (send_fifo_p_.front + 1) % FIFO_BUFF_MAX;
}
/***************************************************/

uint16_t McuSerial::DatatoType(uint8_t *buff)
{
  uint16_t type = buff[0];
  type = type << 8;
  type |= buff[1];
  return type;
}

int16_t McuSerial::DatatoInt16(uint8_t *buff)
{
  int16_t data;
  data = buff[0];
  data = data << 8;
  data |= buff[1];

  return data;
}

int32_t McuSerial::DatatoInt32(uint8_t *buff)
{
  int32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++)
  {
    data = data << 8;
    data |= buff[i];
  }
  return data;
}

uint16_t McuSerial::DatatoUint16(uint8_t *buff)
{
  uint16_t data;
  data = buff[0];
  data = data << 8;
  data |= buff[1];

  return data;
}

uint32_t McuSerial::DatatoUint32(uint8_t *buff)
{
  uint32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++)
  {
    data = data << 8;
    data |= buff[i];
  }
  return data;
}

float McuSerial::DatatoFloat(uint8_t *buff, int mult)
{
  int32_t data;
  data = buff[0];
  for (int i = 1; i < 4; i++)
  {
    data = data << 8;
    data |= buff[i];
  }
  return (float)data / mult;
}

void McuSerial::TypetoData(uint8_t len, uint8_t type, uint8_t *data_type)
{
  uint8_t data;
  data = type;
  data = data << 4;
  data |= len;
  *data_type = data;
}

void McuSerial::Int16toData(int16_t data, uint8_t *data_type)
{
  data_type[0] = (uint8_t)data & 0x00FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x00FF;
}

void McuSerial::Int32toData(int32_t data, uint8_t *data_type)
{
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

void McuSerial::Uint16toData(uint16_t data, uint8_t *data_type)
{
  data_type[0] = (uint8_t)data & 0x00FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x00FF;
}

void McuSerial::Uint32toData(uint32_t data, uint8_t *data_type)
{
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

float McuSerial::FloattoData(float data_float, int mult, uint8_t *data_type)
{
  uint32_t data = (uint32_t)(data_float * mult);
  data_type[0] = (uint8_t)data & 0x000000FF;
  data_type[1] = (uint8_t)(data >> 8) & 0x000000FF;
  data_type[2] = (uint8_t)(data >> 16) & 0x000000FF;
  data_type[3] = (uint8_t)(data >> 24) & 0x000000FF;
}

void McuSerial::SendFramesZero()
{
  for (size_t i = 0; i < SEND_FRAME_MAX; i++)
  {
    send_frames_buff_[i] = 0;
  }
  frames_len_ = 0;
  frames_type_ = 0;
  frames_now_ = 0;
}

} // namespace mcuserial
} // namespace shop

MAIN(shop::mcuserial::McuSerial,"mcu_serial")
