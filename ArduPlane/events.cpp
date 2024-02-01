// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

//void Plane::low_battery_event(void)
//{
//    if (in_low_battery_failsafe) {
//        return;
//    }
//    gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Low battery %.2fV used %.0f mAh",
//                      (double)battery.voltage(), (double)battery.current_total_mah());
//    if (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
//        flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
//    	set_mode(FlightMode::RTL);
//    	aparm.thrust_cruise.load();
//    }
//    in_low_battery_failsafe = true;
//    AP_Notify::flags.failsafe_battery = true;
//}


