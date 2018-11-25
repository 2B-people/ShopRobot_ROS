#ifndef MCU_SERIAL_NODE_H
#define MCU_SERIAL_NODE_H

namespace shop {
namespace serial {
class McuSerial : public shop::common::rrts {
public:
  McuSerial(std::string name);
  ~McuSerial();

<<<<<<< HEAD:modules/control/mcu_serial/include/mcu_serial/mcu_serial_node.h
private:
  serial::Serial ser_;
  void initSerial();
};
} // namespace serial
=======
        private:
        serial::Serial ser_;
        void initSerial();
        
    };
}
>>>>>>> 784c40bf908acf5f4d8272bba8e6c0e18e955b93:modules/driver/mcu_serial/include/mcu_serial/mcu_serial_node.h
} // namespace shop

#endif