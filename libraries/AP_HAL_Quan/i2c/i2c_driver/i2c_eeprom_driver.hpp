#ifndef STM32F4_TEST_I2C_MEMORY_DRIVER_HPP_INCLUDED
#define STM32F4_TEST_I2C_MEMORY_DRIVER_HPP_INCLUDED

#include <cstdint>
#include "FreeRTOS.h"
#include "timers.h"
#include "i2c_driver.hpp"

namespace Quan{

   struct i2c_eeprom_driver_base : i2c_driver {

      // return the absolute system time when write will complete
      // note that i2c bus is available to other devices during this time
      // as long as the bus is free
  
      // true if a write to this eeprom is in progress
      static bool write_in_progress() { return m_write_in_progress;}

      // blocks the thread
      static bool wait_for_write_complete();

      static TickType_t get_write_time_left_ms();

      // return how long the write takes from end of the actual i2c write communication  
      static TickType_t get_write_cycle_time_ms() { return m_write_cycle_time_ms;}
      
      // return the size of a memory page
      // only one page can be read or written at at time
      static uint32_t get_page_size_bytes() {return m_page_size_bytes;}

      // return The total memory size of the eeprom
      static uint32_t get_memory_size_bytes() { return m_memory_size_bytes;}

      // return true if a potential write is in one page
      static bool check_in_page( uint32_t address, uint32_t len, uint32_t page_size)
      {
         return ( address / page_size) == (( address + (len -1) ) / page_size);
      }

      // one shot timer
      static void  timer_callback( TimerHandle_t xTimer ) { m_write_in_progress = false;}
   protected :
      // call i2c_bus_free first
      // install this device as the current i2c bus owner
      // return true if the device was installed
      
      static bool install_device(
            const char* name, 
            uint8_t address,
            uint32_t memory_size,
            uint32_t page_size,
            TickType_t write_cycle_time_ms
      );

   /*
   sets up and starts i2c register based read.
   return true if the read was setup and started correctly
   After returning, the end of the transaction is signified 
   by i2c::bus_released returning true. 
   i2c::bus_released should be polled and if not true within
   a reasonable time an error should be considered to have occurred
   */
      static bool ll_read(uint32_t start_address_in,uint8_t  * data_out,uint32_t len);

   /*
   sets up and starts i2c register based write.
   return true if the write was setup and started correctly
   After returning, the end of the transaction is signified 
   by i2c::bus_released returning true. 
   i2c::bus_released should be polled and if not true within
   a reasonable time an error should be considered to have occurred
   */
      static bool ll_write(uint32_t address_in, uint8_t const * data_in, uint32_t len);

   private:
     // static TickType_t get_write_start_time_ms() { return m_write_start_time_ms_ms;}
      static void set_new_write_start_time_from_isr(BaseType_t & hprtw);
      static void set_write_cycle_time_ms(TickType_t t) { m_write_cycle_time_ms = t;}
      static void set_memory_size_bytes( uint32_t bytes){ m_memory_size_bytes = bytes;}
      static void set_page_size_bytes( uint32_t bytes) { m_page_size_bytes = bytes;}
      
      static TickType_t          m_write_cycle_time_ms;
      static volatile bool       m_write_in_progress;
      static volatile TickType_t m_write_start_time_ms;
      
      static uint32_t          m_memory_size_bytes ;
      static uint32_t          m_page_size_bytes;
      i2c_eeprom_driver_base() = delete;
      i2c_eeprom_driver_base(i2c_eeprom_driver_base const &) = delete;
      i2c_eeprom_driver_base& operator= (i2c_eeprom_driver_base const & ) = delete;

      // return true if the bus was acquired
      // The bus may not be acquired if it is being used by another device
      // or there is an eeprom write in progress
      // check i2c::bus_free() to see if bus is available first
      // since we are running all the clients in one thread
      // The result of bus_free should be reliable. If bus_free() is true
      // then get_bus() shouldnt fail
     // static bool get_bus();

      static void on_write_start_sent();
      static void on_write_device_address_sent();
      static void on_write_data_address_hi_sent();
      static void on_write_data_address_lo_sent();
   #if defined QUAN_I2C_TX_DMA
      static void on_write_dma_transfer_complete();
   #else
      static void on_writing_data();
   #endif
      static void on_write_last_byte_transfer_complete();
      static void on_write_error();
   //---------read irq handlers --------------

      static void on_read_start_sent();
      static void on_read_device_address_sent();
      static void on_read_data_address_hi_sent();
      static void on_read_data_address_lo_sent();
      static void on_read_repeated_start_sent();
      static void on_read_device_read_address_sent();
      static void on_read_dma_transfer_complete();
   #if !defined QUAN_I2C_RX_DMA
      static void on_read_multi_byte_handler();
   #endif
      static void on_read_single_byte_handler();
      static void on_read_error();
   #if !defined QUAN_I2C_TX_DMA
      static uint32_t        m_data_length;
   #endif
      union data_ptr_type{
         uint8_t *         read_ptr; 
         uint8_t  const *  write_ptr;
      };
      static data_ptr_type   m_data;
   #if !defined QUAN_I2C_RX_DMA
         static uint32_t m_data_idx;
         static uint32_t m_bytes_left;
   #endif
   
      static uint8_t         m_data_address[2]; 
      static uint8_t         m_data_address_hi;
     
   };

   /*
      The template ID parameter adds the specific device info
   */

    template <typename ID>
    struct i2c_eeprom_driver : i2c_eeprom_driver_base{

      static bool read(uint32_t start_address_in,uint8_t* data_out, uint32_t len);
     /*
        read at most one page
        requires start address and start address + len must both be in the same page
        Since this is a private function this is not checked
     */
     static bool read_page(uint32_t start_address_in, uint8_t * data_out, uint32_t len)
     {
         return install_device(
               ID::get_device_name(),
               ID::get_device_address(),
               ID::get_memory_size_bytes(),
               ID::get_page_size_bytes(),
               ID::get_write_cycle_time_ms()
         ) && 
         ll_read(start_address_in,data_out,len);
     }

    static bool write(uint32_t start_address_in, uint8_t const * data_in, uint32_t len);

     /*
        write at most one page
        so start address and start address + len must both be in the same page
        Since this is a private function this is not checked
     */
   private:
     static bool write_page(uint32_t start_address_in, uint8_t const * data_in, uint32_t len)
     {
         return install_device(
            ID::get_device_name(), 
            ID::get_device_address(),
            ID::get_memory_size_bytes(),
            ID::get_page_size_bytes(),
            ID::get_write_cycle_time_ms()
         ) && 
          ll_write(start_address_in,data_in,len);
     }
   };

} //Quan

#endif // STM32F4_TEST_I2C_MEMORY_DRIVER_HPP_INCLUDED
