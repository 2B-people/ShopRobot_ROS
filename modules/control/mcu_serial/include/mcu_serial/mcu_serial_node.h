#ifndef MCU_SERIAL_NODE_H
#define MCU_SERIAL_NODE_H

namespace shop
{
namespace serial
{
    class McuSerial:public common::rrts{
        public:
        McuSerial(std::string name);
        ~McuSerial();

        private:
        serial::Serial ser_;
        void initSerial();
        virtual void Run();
        
    };
}
} // namespace shop

#endif