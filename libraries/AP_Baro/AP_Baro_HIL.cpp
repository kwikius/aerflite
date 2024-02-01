/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Baro.h"
#include "AP_Baro_HIL.h"

extern const AP_HAL::HAL& hal;

namespace {
   AP_Baro_HIL baro_driver;
}

template<> AP_baro_driver * connect_baro_driver<HALSITL::tag_board>(AP_Baro & baro)
{
   return baro_driver.connect(baro);
}

AP_baro_driver* AP_Baro_HIL::connect(AP_Baro& baro)
{
   m_baro = &baro;
   m_instance = baro.register_sensor();
   return this;
}

AP_Baro_HIL::AP_Baro_HIL() 
:m_baro{nullptr}, m_instance {static_cast<uint8_t>(-1)}
{}

// Read the sensor
void AP_Baro_HIL::update(void)const
{
   if ( m_baro != nullptr){

      float pressure_sum = 0.0;
      float temperature_sum = 0.0;
      uint32_t sum_count = 0;

      while (m_baro->_hil.press_buffer.is_empty() == false){
         float pressure = 0.0;
         m_baro->_hil.press_buffer.pop_front(pressure);
         pressure_sum += pressure; // Pressure in Pascals

         float temperature = 0.0;
         m_baro->_hil.temp_buffer.pop_front(temperature);
         temperature_sum += temperature; // degrees celcius

         ++sum_count;
      }

      if (sum_count > 0) {
         pressure_sum /= sum_count;
         temperature_sum /= sum_count;
         m_baro->set_sensor_instance(m_instance, pressure_sum, temperature_sum);
      }
   }
}

#endif
