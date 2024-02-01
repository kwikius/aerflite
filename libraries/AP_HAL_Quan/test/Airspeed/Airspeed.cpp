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
 *   Airspeed.pde - airspeed example sketch
 *
 */

#include <AP_HAL/AP_HAL.h>

#include <AP_Airspeed/AP_Airspeed.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>

void on_telemetry_transmitted()
{
}

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    draw_text("Quan APM Airspeed Test",pos);
}

#endif


static AP_Vehicle::FixedWing aparm;

AP_Airspeed airspeed(aparm);

void setup()
{
    hal.console->println("ArduPilot Airspeed library test");
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_PIN", 65);
#endif
    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_ENABLE", 1);
    AP_Param::set_object_value(&airspeed, airspeed.var_info, "_USE", 1);

    airspeed.init();
    airspeed.calibrate(false);
}

void loop(void)
{
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
     hal.scheduler->delay(100);
#else
    static uint32_t timer;
    if((AP_HAL::millis() - timer) > 100) {
        timer = AP_HAL::millis();
#endif
        airspeed.read();
        hal.console->printf("airspeed %.2f healthy=%u\n", static_cast<double>(airspeed.get_airspeed()), airspeed.healthy());
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    }
    hal.scheduler->delay(1);
#endif
}

AP_HAL_MAIN();
