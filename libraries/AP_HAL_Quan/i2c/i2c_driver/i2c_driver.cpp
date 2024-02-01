

#include "i2c_driver.hpp"

uint8_t     Quan::i2c_driver::m_device_bus_address = 0U; 
const char * Quan::i2c_driver::m_device_name = nullptr;

bool Quan::i2c_driver::get_bus()
{

   if(!Quan::i2c_periph::bus_released()) {
      hal.console->write("i2c reg based driver : bus not released\n");
      return false;
   }

   if (!Quan::wait_for_bus_free_ms(100)){return false;}
   // i2c bus may still be busy after bus was released
   if(!Quan::i2c_periph::get_bus()){
      hal.console->write("i2c reg based driver : failed to acquire bus\n");
      return false;
   }
   return true;
}

bool Quan::i2c_driver::install_device(const char* name, uint8_t address)
{
   if (! get_bus()){
      hal.console->write("get bus fail in install device\n");
      return false;
   }
   set_device_name(name);
   set_device_address(address);
   return true;
}
