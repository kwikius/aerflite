#ifndef AP_OSD_ENQUEUE_H_INCLUDED
#define AP_OSD_ENQUEUE_H_INCLUDED

#include "FreeRTOS.h"
#include <queue.h>
#include <quan/three_d/vect.hpp>

#include "AP_OSD.h"

namespace AP_OSD{ namespace enqueue{

   // the sender inits when ready to start sending
   void initialise();

   // x = pitch in degrees, y = roll in degrees, z = yaw in degress
   bool attitude(quan::three_d::vect<float> const & attitude_in);

   // see AP_GPS::GPS_Status
   bool gps_status(gps_info_t info);
   // x == lat in deg10e7, y = lon in deg10e7 , z = alt in cm
   bool gps_location(quan::three_d::vect<int32_t> const & in);
  // x == lat in deg10e7, y = lon in deg10e7 , z = alt in cm
   bool home_location(quan::three_d::vect<int32_t> const & in);
 // airspeed in m/s
   bool airspeed(float m_per_s);
   
   bool battery(quan::three_d::vect<float> const & in); // Voltage, Current, MaH
   bool battery_low_voltage(float low_voltage);
   bool system_status(AP_OSD::system_status_t status); // status, numsats
   bool rc_inputs_0_to_5(uint16_t * arr, uint8_t n);
   bool rc_inputs_6_to_11(uint16_t * arr, uint8_t n);
   bool rc_inputs_12_to_17(uint16_t * arr, uint8_t n);
   bool control_mode( uint8_t value);  // see Arduplane/defines.h
   bool baro_alt(float const & value);
   bool rcin_failsafe(bool value);
}}

#endif // AP_OSD_ENQUEUE_H_INCLUDED
