

#include <AP_HAL/AP_HAL.h>

#if (CONFIG_HAL_BOARD == HAL_BOARD_QUAN)  && (!defined QUAN_MIXER_TRANQUILITY)

#include "AP_Airspeed_Quan.h"

extern const AP_HAL::HAL& hal;

namespace {
   AP_Airspeed_Quan airspeed_driver;
}

template <> AP_Airspeed_Backend * connect_airspeed_driver<Quan::tag_board>(AP_Airspeed & airspeed)
{
 // give time for sensor to stabilise
   uint32_t now = AP_HAL::millis();
   if ( now < 5000){
         hal.scheduler->delay(5000 - now);
   }
   return &airspeed_driver;
}

void AP_Airspeed_Quan::update(){}
// MPXV7002
bool AP_Airspeed_Quan::get_differential_pressure(float &pressure)const
{

//  sensitivity  = 1 e-3 Volts/ Pa
//
//  result Pa  = V / sensitivity
//
//  Pa = v_adc  * 1000
// In fact should be offset at VCC/2 but think that offset is taken care of externally
#if defined QUAN_AERFLITE_BOARD
   static constexpr uint8_t airspeed_adc_channel = 4;
#else
   static constexpr uint8_t airspeed_adc_channel = 2;
#endif
   pressure = hal.analogin->channel(airspeed_adc_channel)->voltage_average_ratiometric() * 1000.f;
   return true;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
