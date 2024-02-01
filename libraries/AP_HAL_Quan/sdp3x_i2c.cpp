
#include "FreeRTOS.h"
#include <task.h>
#include "i2c_task.hpp"
#include "sdp3x_i2c.hpp"
#include <quan/pressure.hpp>
#include <quan/temperature.hpp>

#if (CONFIG_HAL_BOARD == HAL_BOARD_QUAN)  && (defined QUAN_MIXER_TRANQUILITY)

#if defined QUAN_I2C_DEBUG
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_debug.hpp>
#endif

extern const AP_HAL::HAL& hal;

namespace {

   // input: data must be a non null pointer to an array of data to be crced of length num_elements
   // output: the calculated crc
   uint8_t sdp3x_crc(const uint8_t * data, uint32_t num_elements)
   {

      uint8_t constexpr init_crc_value = 0xFF;
      uint8_t constexpr polynomial = 0x31;

      uint8_t crc_value = init_crc_value;

      for (uint32_t i = 0U; i < num_elements; ++i) {
         crc_value ^= data[i];
         for (uint8_t j = 0U; j < 8U; ++j) {
            crc_value = ((crc_value & 0x80) == 0U) 
            ? (crc_value << 1U)
            : (crc_value << 1U) ^ polynomial;
         }
      }

      return crc_value;
   }

   uint8_t __attribute__ ((section (".dma_memory"))) result_values[9] ;
}

namespace Quan{

  bool sdp3_init()
  {
    // write first read product id command
    uint8_t ar[18] = {(sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF00) >> 8U,sdp3x_i2c_0::cmd::read_product_id[0] & 0xFF};
    if (! sdp3x_i2c_0::write(ar,2)){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3x write cmd 0 failed\n");
       show_i2c_sr1_flags();
#endif
       return false;
    }
    if (! Quan::wait_for_bus_free_ms(2)){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3x couldnt get bus after cmd 0 write\n");
       show_i2c_sr1_flags();
#endif
       return false;
    }

    // write second read product id command
    ar[0] = (sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF00) >> 8U;
    ar[1] = sdp3x_i2c_0::cmd::read_product_id[1] & 0xFF;
    if (! sdp3x_i2c_0::write(ar,2)){
#if defined QUAN_I2C_DEBUG
      hal.console->printf("sdp3x write cmd 1 failed\n");
      show_i2c_sr1_flags();
#endif
      return false;
    }
    if (! Quan::wait_for_bus_free_ms(2)){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3x couldnt get bus after cmd 1 write\n");
#endif
       return false;
    }

    // read the data
    if (!sdp3x_i2c_0::read(ar,18)){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3_x read id failed\n");
       show_i2c_sr1_flags();
#endif
       return false;
    }
    if (! Quan::wait_for_bus_free_ms(10)){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3x couldnt get bus after id read\n");
#endif
       return false;
    }
   
    //check crc of result ( 2 bytes data are followed by one byte of crc)
    bool crc_good = true;
    for (uint32_t i = 0U; i < 6U; ++i){
       crc_good &= sdp3x_crc(&ar[3U * i],2) == ar[3U * i + 2U];
    }

    if (!crc_good){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("crc failed\n");
#endif
       return false;
    }
    // get the id. 
    uint32_t const sdp3x_id = (ar[0] << 24U) | (ar[1] << 16U) | (ar[3] << 8U) | ( ar[4]) ;
    
    //N.B for compare Ignore the last 8 bits, which are a revison number and subject to change
    uint32_t constexpr sdp31_id = 0x03010100;

    if ( (sdp3x_id & 0xFFFFFFF0) != sdp31_id){
#if defined QUAN_I2C_DEBUG
       hal.console->printf("sdp3x invalid id\n");
#endif
       return false;
    }
    hal.console->printf("sdp31 i2c airspeed sensor detected\n");

    ar[0] = (sdp3x_i2c_0::cmd::start_continuous_diff_pressure_average & 0xFF00) >> 8U;
    ar[1] = sdp3x_i2c_0::cmd::start_continuous_diff_pressure_average & 0xFF;
    
    if (! sdp3x_i2c_0::write(ar,2)){
#if defined QUAN_I2C_DEBUG
      hal.console->printf("sdp31 write  measurement mode failed\n");
#endif
      return false;
    }

    if (! Quan::wait_for_bus_free_ms(2)){
#if defined QUAN_I2C_DEBUG
      hal.console->printf("sdp3x couldnt get bus after measurement mode write\n");
#endif
      return false;
    }
    // datasheet specfies a 8 ms delay after measurement command before any read
    hal.scheduler->delay(10);

    return true;
    
  }

 // start a read of the airspeed data
// n.b could shorten this to just read first values, since the last part is a scaling constant
   bool sdp3_start_read()
   {
     // uint8_t ar[9];
      if (!sdp3x_i2c_0::read(result_values,9)){
#if defined QUAN_I2C_DEBUG
         hal.console->printf("sdp3_x read failed\n");
         show_i2c_sr1_flags();
#endif
         return false;
      }
       return true;
   }

   bool sdp3_calculate()
   {
      // check crc
      for (uint8_t i = 0U; i < 3U; ++i){
         if ( sdp3x_crc(&result_values[3U * i],2) != result_values[3U * i + 2U]){
#if defined QUAN_I2C_DEBUG
            hal.console->printf("sdp3x read invalid crc\n");
#endif
            return false;
         }
      }

      union {
         uint8_t ar[2];
         int16_t result;
      } u;

      u.ar[0] = result_values[1U];
      u.ar[1] = result_values[0U];
      quan::pressure::Pa const differential_pressure{ u.result / 60.f };

      u.ar[0] = result_values[4U];
      u.ar[1] = result_values[3U];
      quan::temperature::K const temperature{ u.result / 200.f + 273.15f };

            // overwrite the queue with new values
      Quan::detail::airspeed_args args;
      args.differential_pressure = differential_pressure;
      args.temperature = temperature;
      xQueueOverwrite(get_airspeed_queue_handle(),&args);

      return true;
   }

}

#endif



