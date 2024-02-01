/*
 *       Example of AP_BattMonitor library
 *       Code by DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <AP_BattMonitor/AP_BattMonitor.h>
#include <quantracker/osd/osd.hpp>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   AP_BattMonitor battery_mon;

   float voltage_V = 0.f;
   float current_A = 0.f;

}

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-150,70};
    draw_text("Battery monitor test",pos);

    pos.y -= 40;
    draw_text<100>(pos,"Batt voltage = % 8.3f V",static_cast<double>(voltage_V));

    pos.y -= 20;
    draw_text<100>(pos,"Batt current = % 8.3f A",static_cast<double>(current_A));
      
}

void setup() {
    hal.console->println("Battery monitor library test");

    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_VOLT_PIN", 3);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info, "_CURR_PIN", 2);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info,"_VOLT_MULT", 4.092f);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info,"_AMP_OFFSET", 0.0f);
    AP_Param::set_object_value(&battery_mon, battery_mon.var_info,"_AMP_PERVOLT", 16.67f);
 
    battery_mon.init();

    hal.scheduler->delay(1000);
}

namespace {
   uint32_t counter =0;
}

void loop()
{
    hal.scheduler->delay(100);

    battery_mon.read();

    voltage_V = battery_mon.voltage();
    current_A = battery_mon.current_amps();
    
    // display output at 1hz
    if (++counter >= 10) {
        counter = 0;
        hal.console->printf("\nVoltage: %.2f V \tCurrent: %.2f A \tTotCurr:%.2f mAh",
			    static_cast<double>(voltage_V),
			    static_cast<double>(current_A),
             static_cast<double>(battery_mon.current_total_mah())
       );
    }
}

AP_HAL_MAIN();

#endif 
