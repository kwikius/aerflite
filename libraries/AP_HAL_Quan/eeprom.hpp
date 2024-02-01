#ifndef STM32F4_TEST_M24M01_HPP_INCLUDED
#define STM32F4_TEST_M24M01_HPP_INCLUDED

#include <AP_HAL_Quan/i2c/i2c_driver/i2c_eeprom_driver.hpp>
#include "FreeRTOS.h"
#include <cstdio>
#include <task.h>
#include <semphr.h>

namespace Quan{

   struct eeprom_info {

      struct eeprom_m24m01{ 
         static constexpr uint8_t      get_device_address()   { return 0b10100000;}
         static constexpr const char * get_device_name()      { return "M24M01 eeprom";}
         static constexpr uint32_t     get_memory_size_bytes(){ return 128U * 1024U;}
         static constexpr uint32_t     get_page_size_bytes()   { return 256U;}
         static constexpr Quan::i2c_driver::millis_type get_write_cycle_time_ms() { return Quan::i2c_driver::millis_type{5U};}
      };
#if 0
      struct eeprom_24lc128{
         static constexpr uint8_t      get_device_address()   { return 0b10100000;}
         static constexpr const char * get_device_name()      { return "24LC128 eeprom";}
         static constexpr uint32_t     get_memory_size_bytes(){ return 16U * 1024U;}
         static constexpr uint32_t     get_page_size_bytes()   { return 64U;}
         static constexpr Quan::i2c_driver::millis_type get_write_cycle_time_ms() { return Quan::i2c_driver::millis_type{5U};}
      };
#endif

   };

   typedef Quan::i2c_eeprom_driver<Quan::eeprom_info::eeprom_m24m01> eeprom;

   struct eeprom_read_msg{
      uint8_t* mcu_address;
      uint32_t eeprom_address;
      uint32_t num_elements ;
      eeprom_read_msg(void* mcu_address_in,uint32_t eeprom_address_in, uint32_t num_elements_in)
      :mcu_address{static_cast<uint8_t*>(mcu_address_in)}, eeprom_address{eeprom_address_in},num_elements{num_elements_in}{}
      eeprom_read_msg():mcu_address{nullptr},eeprom_address{0},num_elements{0}{}
   } __attribute__ ((packed));

   
   struct eeprom_write_msg{
      static constexpr uint8_t buffer_length = 32U;
      struct {
         uint32_t eeprom_address : 24;
         uint8_t num_elements    :  8;
      };
      uint8_t  data[buffer_length];
      eeprom_write_msg( const void* mcu_address, uint32_t eeprom_address_in, uint8_t num_elements_in)
      : eeprom_address{eeprom_address_in},num_elements{num_elements_in}
      {
         for (uint8_t i = 0; i < buffer_length; ++i){
            data[i] = static_cast<uint8_t const*>(mcu_address)[i];
         }
      }
      eeprom_write_msg()
      : eeprom_address{0},num_elements{0}{}
      
   } __attribute__ ((packed)); 


   static_assert(sizeof(eeprom_write_msg) == 36U,"unexpected size");

   QueueHandle_t get_eeprom_read_handle();

   QueueHandle_t get_eeprom_write_handle();

   SemaphoreHandle_t   get_read_complete_semaphore();

   TimerHandle_t get_eeprom_timer_handle();

   bool setup_eeprom();
   bool eeprom_service_write_buffer();
   bool eeprom_service_read_requests();
   
}// Quan

#endif // STM32F4_TEST_M24M01_HPP_INCLUDED
