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
 *   analog airspeed driver
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Airspeed/AP_Airspeed_analog.h>

namespace {

   AP_Airspeed_Analog airspeed_driver;
}

extern const AP_HAL::HAL& hal;

template <> AP_Airspeed_Backend * connect_airspeed_driver<HALSITL::tag_board>(AP_Airspeed & airspeed)
{
   airspeed_driver.set_source( hal.analogin->channel(airspeed.get_pin()));
   return &airspeed_driver;
}

void AP_Airspeed_Analog::update()
{
   ;
}

// scaling for 3DR analog airspeed sensor
#define VOLTS_TO_PASCAL 819

// read the airspeed sensor
bool AP_Airspeed_Analog::get_differential_pressure(float &pressure)const
{
   if (_source != nullptr) {
      pressure = _source->voltage_average_ratiometric() * VOLTS_TO_PASCAL;
      return true;
   }else{
      return false;
   }
}

void AP_Airspeed_Analog::set_source(AP_HAL::AnalogSource * source)
{
   _source = source;
}

#endif


