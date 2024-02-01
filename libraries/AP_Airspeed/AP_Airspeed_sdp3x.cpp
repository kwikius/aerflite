
#include <AP_HAL/AP_HAL.h>

#if (CONFIG_HAL_BOARD == HAL_BOARD_QUAN)  && (defined QUAN_MIXER_TRANQUILITY)

#include "AP_Airspeed_sdp3x.hpp"
#include <AP_HAL_Quan/i2c_task.hpp>

#include <task.h>

#include <AP_Baro/AP_Baro.h>
#include <quan/pressure.hpp>
#include <quan/temperature.hpp>
#include <quan/constants/constant.hpp>

extern const AP_HAL::HAL& hal;

namespace {
   AP_Airspeed_sdp3x airspeed_driver;
}

template <> AP_Airspeed_Backend * connect_airspeed_driver<Quan::tag_board>(AP_Airspeed & airspeed)
{
 // give time for sensor to stabilise
   uint32_t now = AP_HAL::millis();
   if ( now < 1000){
         hal.scheduler->delay(1000 - now);
   }
   airspeed_driver.connect(Quan::get_airspeed_queue_handle());
   return &airspeed_driver;
}

namespace {

   float constexpr m_c = 4.79e-7f ;  // kg.s-1 
   float constexpr delta_p_c = 101.f; // Pa
   //    float constexpr hose_length = 0.1f;  // m
   //    float constexpr hose_inner_dia = 0.002f; // m
    // = hose_length / (hose_inner_dia ^4)
   float constexpr hose_constant = 7499999832.0f;

    // return effective pressure
   quan::pressure::Pa get_sdp3x_pressure( quan::pressure::Pa const & delta_p_sensor, quan::temperature::K t_deg_K)
   {
       quan::pressure::Pa p_abs_Pa{get_barometer().get_pressure()};
       quan::pressure::bar p_abs_bar = p_abs_Pa;

       float const n_air =  (18.205f + 0.0484f * (t_deg_K.numeric_value() - 293.15f )) * 1.e-6f; //( Pa.s-1)

       float const p_air = (1.1885f * p_abs_bar.numeric_value()) * 293.15f / t_deg_K.numeric_value(); // kg.m-3

       float const c1 = -64.f/ quan::constant_<float>::pi * n_air/ p_air * m_c / delta_p_sensor.numeric_value() * 
            ( safe_sqrt(1.f + 8.f * delta_p_sensor.numeric_value()/delta_p_c) - 1.f);

      // float const epsilon = c1 * hose_length / quan::pow<4>(hose_inner_dia); 
       float const epsilon = c1 * hose_constant; 

       return delta_p_sensor / (1.f + epsilon);
   }

}

void AP_Airspeed_sdp3x::update()
{
   Quan::detail::airspeed_args args;

   if ( (m_hQueue != NULL) && ( xQueueReceive(m_hQueue, &args,0) == pdTRUE) ) {
      m_temperature = args.temperature;
      m_diff_pressure = get_sdp3x_pressure(args.differential_pressure,args.temperature);
   }
}


#endif