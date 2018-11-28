#include <stdint.h>

#include <data/SerialTest.h>
#include <ros/ros.h>
#include <serial/serial.h>

#define USBPIN "/dev/ttyUSB0"
#define USBBT 115200

serial::Serial ser;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "serial_example_node");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<data::SerialTest>("vel_x", 10);

  try {
    ser.setPort(USBPIN);
    ser.setBaudrate(USBBT);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (const std::exception &e) {
    ROS_ERROR("Unable to open port");
    return -1;
  }

  while (ser.isOpen() == 0) {
    ROS_INFO("Serial Init");
  }

  while (1) {
    if (ser.available()) {
      // uint8_t hand[2];
      // ser.read(hand, 2);
      // if (hand[0] == 0x89 && hand[1] == 0x89) {
      //   uint8_t data[8];
      //   ser.read(data, 8);
      //   if (data[0] == 0x00 && data[1] == 0x01)
      //     for (int i = 2; i < 8; i++) {
      //       ROS_INFO("read : %d", data[i]);
      //       data::SerialTest msg;
      //       for (int i = 0; i < 8; i++) {
      //         msg.data[i] = data[i];
      //       }

      //       pub.publish(msg);
      //     }
      //   else
      //     ROS_INFO("not is msg");
      // }
      uint8_t hand[2];
      if (ser.read(hand, 2) && (hand[0] == 0x89) && (hand[1] == 0x89)) {
        printf("hello world");
      }
      printf("1\n");
    }
    ros::spinOnce();
  }
  return 0;
}
