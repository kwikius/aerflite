#ifndef APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED
#define APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include <quan/pressure.hpp>
#include <quan/temperature.hpp>
#include <quan/time.hpp>
#include <quan/magnetic_flux_density.hpp>
#include <quan/three_d/vect.hpp>

#if !defined QUAN_AERFLITE_BOARD
#include "I2CDriver.h"
#endif

namespace Quan{ 

   void create_i2c_task();
   QueueHandle_t get_compass_queue_handle();
   QueueHandle_t get_baro_queue_handle();

#if defined QUAN_MIXER_TRANQUILITY
   QueueHandle_t get_airspeed_queue_handle();
#endif
   
   namespace detail{

      struct baro_args{
         quan::pressure_<float>::Pa         pressure;
         quan::temperature_<float>::K       temperature;
      }; 

      struct compass_args{
         quan::three_d::vect<quan::magnetic_flux_density_<float>::milli_gauss> field;
         uint32_t time_us;
      };

      struct compass_gain{
         quan::three_d::vect<float> field;
      };

#if defined QUAN_MIXER_TRANQUILITY
      struct airspeed_args{
         quan::pressure_<float>::Pa         differential_pressure;
         quan::temperature_<float>::K       temperature;
      }; 

#endif
   }

#if defined QUAN_AERFLITE_BOARD
   uint32_t * get_i2c_task_notify();
   TaskHandle_t   get_i2c_task_handle();
   QueueHandle_t get_compass_gain_handle();

   void set_gains(quan::three_d::vect<float> const & g);

   bool setup_compass();
   bool compass_request_conversion();
   bool compass_start_read();
   bool compass_calculate();

   bool setup_baro();
   bool baro_request_conversion();
   bool baro_start_read();
   bool baro_calculate();

#if defined QUAN_MIXER_TRANQUILITY
   bool sdp3_init();
   bool sdp3_start_read();
   bool sdp3_calculate();
#endif

   
#endif
}

#endif //CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // APM_QUANTRACKER_I2C_TASK_HPP_INCLUDED
