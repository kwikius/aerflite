
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Baro_Quan.h"
#include <task.h>

namespace {
   AP_Baro_Quan baro_driver;
}

template<> AP_baro_driver * connect_baro_driver<Quan::tag_board>(AP_Baro & baro)
{
   return baro_driver.connect(baro);
}

AP_Baro_Quan::AP_Baro_Quan(): 
m_baro{nullptr},m_hQueue{NULL},m_instance{static_cast<uint8_t>(-1)}
{}

AP_baro_driver* AP_Baro_Quan::connect(AP_Baro & baro)
{
   auto const inst = baro.register_sensor();
   if ( inst < baro.num_instances()){
      m_instance = inst;
      m_hQueue = Quan::get_baro_queue_handle();
      m_baro = &baro;
   }
   return this;
}

void AP_Baro_Quan::copy_to_frontend(quan::pressure_<float>::Pa const & pressure, quan::temperature_<float>::K const & temperature)const
{
    if (m_baro != nullptr) {
        m_baro->set_sensor_instance(m_instance, pressure.numeric_value(), temperature.numeric_value() - 273.15f);
    }
}

void AP_Baro_Quan::update()const
{
   Quan::detail::baro_args args;
   // receive should be available in 1/5th sec!
   if ( (m_hQueue != NULL) && ( xQueueReceive(m_hQueue, &args,0) == pdTRUE) ) {
      copy_to_frontend(args.pressure,args.temperature);
   }
}

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN



