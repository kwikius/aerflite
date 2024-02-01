/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       AP_Compass_HIL.cpp - HIL backend for AP_Compass
 *
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_Compass_HIL.h"

namespace {
  AP_Compass_HIL compass_driver;
}

template <> AP_Compass_Backend * connect_compass_driver<HALSITL::tag_board>(Compass& compass)
{
   return compass_driver.connect(compass);
}

extern const AP_HAL::HAL& hal;

// constructor
AP_Compass_HIL::AP_Compass_HIL()
{
   memset(_compass_instance, 0, sizeof(_compass_instance));
}

// detect the sensor
AP_Compass_HIL *AP_Compass_HIL::connect(Compass &compass)
{
   this->set_compass(compass);
   for (uint8_t i=0; i<HIL_NUM_COMPASSES; i++) {
        _compass_instance[i] = compass.register_compass();
   }
   compass._setup_earth_field();
   return this;
}

void AP_Compass_HIL::update()
{
    for (uint8_t i=0; i < ARRAY_SIZE(_compass_instance); i++) {
        if (m_compass->_hil.healthy[i]) {
            uint8_t compass_instance = _compass_instance[i];
            Vector3f field = m_compass->_hil.field[compass_instance];
            rotate_field(field, compass_instance);
            publish_raw_field(field, AP_HAL::micros(), compass_instance);
            correct_field(field, compass_instance);
            publish_filtered_field(field, compass_instance);
        }
    }
}
#endif


