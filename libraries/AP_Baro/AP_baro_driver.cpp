/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#include "AP_baro_driver.h"
#include "AP_Baro.h"

extern const AP_HAL::HAL& hal;

// constructor
AP_baro_driver::AP_baro_driver() 
{}

/*
  copy latest data to the frontend from a backend

  N.B no mapping betwwen sensor instances to backends so wtf?
 */
//void AP_baro_driver::_copy_to_frontend(uint8_t instance, float pressure, float temperature)
//{
//    if ( (m_frontend != nullptr) && (instance < m_frontend->_num_sensors)) {
//       m_frontend->sensors[instance].pressure = pressure;
//       m_frontend->sensors[instance].temperature = temperature;
//       m_frontend->sensors[instance].last_update_ms = AP_HAL::millis();
//    }
//}
