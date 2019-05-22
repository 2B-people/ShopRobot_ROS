#include <mcu_serial/box_serial_node.h>

namespace shop
{
namespace mcu_serial
{

BoxSerial::BoxSerial(std::string name) : common::RRTS(name, 1), is_open_(false)
, opening_as_(nh_, ("shop/opencar/opening_action"), boost::bind(&BoxSerial::OpeningExecuteCB, this, _1), false)
,mcu_port_("/dev/car")
{
    // ros::NodeHandle nh_private_("~");
    is_open_ = true;
    initSerial();
    opening_as_.start();
}

BoxSerial::~BoxSerial()
{
    ROS_INFO("aaaaaaa");
}

void BoxSerial::initSerial()
{
    ser_.setPort(mcu_port_);
    ser_.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();

    while (ser_.isOpen() == 0)
    {
        NOTICE("Warting MCU!!");
    }
    ROS_INFO("MCU is ready!!");
}

void BoxSerial::OpeningExecuteCB(const data::OpeningGoal::ConstPtr &goal)
{
    data::OpeningFeedback feedback;
    data::OpeningResult result;

    ROS_INFO("opencar is run");

    uint8_t send_msg = 0x0a;
    ser_.write(&send_msg, 1);
    ROS_INFO("is send");

    while (ros::ok() && is_open_)
    {
        if (ser_.available())
        {
            uint8_t hand;
            ser_.read(&hand, 1);
            if (hand == 0x0b)
            {
                ROS_INFO("read : %d", hand);
                break;
            }
        }
    }
    result.success_flag = true;
    opening_as_.setSucceeded(result);
    return;
}

} // namespace mcu_serial
} // namespace shop

MAIN(shop::mcu_serial::BoxSerial,"opencar")
