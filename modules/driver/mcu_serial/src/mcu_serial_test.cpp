#include <stdint.h>

#include <data/SerialTest.h>
#include <ros/ros.h>
#include <serial/serial.h>

#define USBPIN "/dev/ttyUSB0"
#define USBBT 115200

serial::Serial ser;

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "serial_example_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<data::SerialTest>("vel_x", 10);

  try
  {
    ser.setPort(USBPIN);
    ser.setBaudrate(USBBT);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  }
  catch (const std::exception &e)
  {
    ROS_ERROR("Unable to open port");
    return -1;
  }

  while (ser.isOpen() == 0)
  {
    ROS_INFO("Serial Init");
  }
  ROS_INFO("success");

  uint8_t send_msg = 0x0a;
  ser.write(&send_msg, 1);
  ROS_INFO("is send");

  while (1)
  {
    if (ser.available())
    {
      uint8_t hand;
      ser.read(&hand, 1);
      if (hand == 0x0b)
      {
        ROS_INFO("read : %d", hand);
      }
      ros::spinOnce();
    }
  }
  return 0;
}
