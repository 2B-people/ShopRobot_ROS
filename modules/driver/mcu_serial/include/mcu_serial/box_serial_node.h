#ifndef BOX_SERIAL_NODE_H
#define BOX_SERIAL_NODE_H

//ros
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <actionlib/server/simple_action_server.h>

//serial
#include <serial/serial.h>

#include <string>
#include <cstring>
#include <stdint.h>
#include <stdlib.h>
#include <memory>
#include <thread>
#include <unistd.h>


#include <data/OpeningAction.h>

#include "common/log.h"
#include "common/main_interface.h"
#include "common/rrts.h"
namespace shop
{
namespace mcu_serial
{

typedef actionlib::SimpleActionServer<data::OpeningAction> OPENINGACTIONSERVER;

class BoxSerial: public shop::common::RRTS
{
public:
    BoxSerial(std::string name);
    ~BoxSerial();

private:
    bool is_open_;

    // ros
    ros::NodeHandle nh_;

    // action
    OPENINGACTIONSERVER opening_as_;

    // ttyUSB setting
    std::string mcu_port_;
    int mcu_baudrate_;
    serial::Serial ser_;

    void initSerial();

    void OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal);
};

} // namespace mcuserial

} // namespace shop

#endif
