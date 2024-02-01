#ifndef APM_QUANTRACKER_OSD_MESSAGE_HPP_INCLUDED1
#define APM_QUANTRACKER_OSD_MESSAGE_HPP_INCLUDED1

#include <quan/three_d/vect.hpp>
#include "FreeRTOS.h"
#include <AP_GPS/AP_GPS.h>
#include <queue.h>

namespace Quan{
   
   bool osd_send_attitude(quan::three_d::vect<float> const & attitude_in);
   bool osd_send_drift(quan::three_d::vect<float> const & drift_in);
   bool osd_send_heading(float heading_in);
   bool osd_send_gps_fix_type(uint8_t status);

   void osd_init();

}


#endif // APM_QUANTRACKER_OSD_MESSAGE_HPP_INCLUDED1
