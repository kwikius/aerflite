/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_BATTMONITOR_ANALOG_H
#define AP_BATTMONITOR_ANALOG_H

#include <AP_HAL/AnalogIn.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_Backend.h"

// default pins and dividers
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_QUAN
  #if defined QUAN_AERFLITE_BOARD
     # define AP_BATT_VOLT_PIN                  3
     # define AP_BATT_CURR_PIN                  2
     // 39K high side
     // 7k5 low side
     # define AP_BATT_VOLTDIVIDER_DEFAULT       4.092f
  #else
     # define AP_BATT_VOLT_PIN                  0
     # define AP_BATT_CURR_PIN                  1
     // using a 33 K res low side and 270 k high side
     // but the voltage is scaled as if it was 5V   
   # define AP_BATT_VOLTDIVIDER_DEFAULT       6.06f
 #endif
// current sensor same for both boards
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  16.67f
#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#endif

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, uint8_t instance, AP_BattMonitor::BattMonitor_State &mon_state);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read();

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
};

#endif  // AP_BATTMONITOR_ANALOG_H
