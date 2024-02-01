
#include <cstdio>
#include "AP_OSD_dequeue.h"
#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>
#include <quan/uav/get_bearing_and_distance.hpp>

namespace{

   void get_attitude(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      typedef quan::angle_<float>::deg deg;
      info.attitude 
         = quan::uav::osd::attitude_type{
               deg{msg.value.vect3df.z}, // yaw
               deg{msg.value.vect3df.x}, // pitch
               deg{msg.value.vect3df.y}  // roll
         };
   }

   void get_gps_status(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.gps_status = msg.value.gps_info.status;
      info.gps_num_sats = msg.value.gps_info.num_sats;
      info.ground_speed = quan::velocity_<float>::m_per_s{msg.value.gps_info.ground_speed_m_per_s};
      info.ground_course = quan::angle_<float>::deg{msg.value.gps_info.ground_course_cd / 100.f};
   }

   void get_gps_location(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      quan::angle_<int32_t>::deg10e7 lat{msg.value.vect3di32.x};
      quan::angle_<int32_t>::deg10e7 lon{msg.value.vect3di32.y};
      quan::length_<int32_t>::cm     alt{msg.value.vect3di32.z};
      info.aircraft_position = quan::uav::osd::position_type{lat,lon,alt};
   }

   void get_home_location(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      quan::angle_<int32_t>::deg10e7 lat{msg.value.vect3di32.x};
      quan::angle_<int32_t>::deg10e7 lon{msg.value.vect3di32.y};
      quan::length_<int32_t>::cm     alt{msg.value.vect3di32.z};
      info.home_position = quan::uav::osd::position_type{lat,lon,alt};
      info.home_is_set = true;
   }

   void get_airspeed(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.airspeed = quan::velocity_<float>::m_per_s{msg.value.f};
   }

   void get_battery(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.battery_voltage = quan::voltage_<float>::V{msg.value.vect3df.x};
      info.battery_current = quan::current_<float>::A{msg.value.vect3df.y};
      info.battery_mAh_consumed = quan::charge_<float>::mA_h{msg.value.vect3df.z};
   }

   void get_battery_low_voltage(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.battery_low_voltage = quan::voltage_<float>::V{msg.value.f};
   }

   void get_system_status(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
      info.system_status = msg.value.sys_status;
   }

   void get_rcin_0_to_5(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[i] = msg.value.u16_array6[i];
        }
   }

   void get_rcin_6_to_11(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[6 + i] = msg.value.u16_array6[i];
        }
   }

   void get_rcin_12_to_17(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
        for ( uint8_t i = 0; i < 6; ++i){
           info.rc_in_channels[12 + i] = msg.value.u16_array6[i];
        }
   }

   void get_control_mode(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
       info.control_mode = msg.value.u8;
   }

   void get_baro_alt(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
       info.baro_alt = quan::length_<float>::m{msg.value.f};
   }

   void get_rcin_failsafe(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info)
   {
       info.in_rcin_failsafe = msg.value.b;
   }
   
   typedef void(*fun_ptr)(AP_OSD::osd_message_t const & msg, AP_OSD::dequeue::osd_info_t & info);

   // order must match enums
   fun_ptr funs[] = {
      get_attitude,        //   0
      get_gps_status,      //   1 
      get_gps_location,    //   2
      get_home_location,   //   3
      get_airspeed,        //   4
      get_battery,         //   5
      get_system_status,   //   6
      get_rcin_0_to_5,     //   7
      get_rcin_6_to_11,    //   8
      get_rcin_12_to_17,   //   9
      get_control_mode,    //  10
      get_baro_alt,        //  11
      get_battery_low_voltage,  // 12
      get_rcin_failsafe          //13
   };

   QueueHandle_t osd_queue = nullptr;

} // namespace
   
// called by osd thread to get latest data
void AP_OSD::dequeue::read_stream(AP_OSD::dequeue::osd_info_t& info)
{
   if ( osd_queue != nullptr){
      AP_OSD::osd_message_t msg;
      while ( xQueueReceive(osd_queue,&msg,0) == pdTRUE){
         uint32_t const id = static_cast<uint32_t>(msg.id);
         if ( id < static_cast<uint32_t>(AP_OSD::msgID::max_messages)){
             fun_ptr fun = funs[id];
             fun(msg,info);
         }
      }
   }
}

void AP_OSD::dequeue::update(AP_OSD::dequeue::osd_info_t& info)
{
   // recalc distance to home and bearing
   // It is necessary to convert from deg10e7 int to deg float
   // to prevent overflow and incorrect results
   typedef quan::uav::position<quan::angle::deg,quan::length::mm> local_pos_type;
   local_pos_type const home_position = info.home_position;
   local_pos_type const aircraft_position = info.aircraft_position;

   quan::uav::get_bearing_and_distance(
      home_position,aircraft_position,
      info.bearing_to_home,info.distance_from_home
   );
}

namespace AP_OSD { namespace dequeue {namespace detail{

   QueueHandle_t  initialise()
   {
      osd_queue = xQueueCreate(30,sizeof(osd_message_t));
      return osd_queue;
   }
}}}

