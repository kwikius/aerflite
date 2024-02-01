#ifndef STM32_SIMPLE_READ_WRITE_DRIVER_HPP_INCLUDED
#define STM32_SIMPLE_READ_WRITE_DRIVER_HPP_INCLUDED

#include <cstdint>
#include "i2c_driver.hpp"

//extern "C" bool is_valid_heap_memory(void * p);

namespace Quan{

   struct i2c_read_write_driver_base : i2c_driver{
 
   protected :
    
   /*
   sets up and starts i2c  read.
   return true if the read was setup and started correctly
   After returning, the end of the transaction is signified 
   by i2c::bus_released returning true. 
   i2c::bus_released should be polled and if not true within
   a reasonable time an error should be considered to have occurred 
   */
      static bool ll_read(uint8_t * data, uint32_t len);

   /*
   sets up and starts simple write.
   return true if the write was setup and started correctly
   After returning, the end of the transaction is signified 
   by i2c::bus_released returning true. 
   i2c::bus_released should be polled and if not true within
   a reasonable time an error should be considered to have occurred
   */
      static bool ll_write(uint8_t const * data , uint32_t len);

     private:
      // read handlers
      static void on_read_start_sent();
      static void on_read_device_address_sent();
     // static void on_read_reg_index_sent();
     // static void on_read_repeated_start_sent();
     // static void on_read_device_read_address_sent();
      static void on_read_single_byte_handler();
      static void on_read_multi_byte_handler();
      static void on_read_dma_transfer_complete();
      static void on_read_error();
      // write handlers
      static void on_write_start_sent();
      static void on_write_device_address_sent();
   //   static void on_write_reg_index_sent();
      static void on_writing_data();
    //  static void on_write_last_byte_transfer_complete();
      static void on_write_error();

      // save space but respect constness of
      // input when writing to the device
      union data_ptr_type{
         uint8_t *         read_ptr; 
         uint8_t  const *  write_ptr;
        // uint8_t           value_to_write;
      };

      static data_ptr_type m_data;
      static uint32_t m_data_length;
#if !defined QUAN_I2C_RX_DMA
      static uint32_t m_data_idx;
      static uint32_t m_bytes_left;
#endif
     // static uint8_t  m_register_index; 
   };

   template <typename ID> 
   struct i2c_read_write_driver : i2c_read_write_driver_base{

      // use i2c::bus_free() to check if bus is free first
      static bool read(uint8_t * data, uint32_t len)
      {
         return install_device(ID::get_device_name(), ID::get_device_address()) && 
            ll_read(data,len);
      }

       // use Quan::i2c_periph::bus_free() to check if bus is free first
      static bool write(uint8_t const * data, uint32_t len)
      {
         return install_device(ID::get_device_name(), ID::get_device_address()) && 
            ll_write(data,len);
      }
   }; 

}//Quan

#endif // STM32_SIMPLE_READ_WRITE_DRIVER_HPP_INCLUDED

