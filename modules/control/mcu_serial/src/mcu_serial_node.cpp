#include <mcu_serial/mcu_serial_node.h>

namespace shop
{
namespace serial
{

McuSerial::McuSerial(std::string name) : common::RRTS(name),
{
}

McuSerial::~McuSerial()
{
    
}


McuSerial::initSerial()
{
    ser_.setPort();
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
} // namespace serial
} // namespace shop