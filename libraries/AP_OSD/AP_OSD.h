#ifndef AP_OSD_H_INCLUDED
#define AP_OSD_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <quan/three_d/vect.hpp>
#include <quan/angle.hpp>

namespace AP_OSD{

   /*
       add loading eeprom
       demo servos
   */

   enum class system_status_t {
      starting = 0
     , in_cli
     , initialising
     , loading_eeprom_params
     , demo_servos
     , running
   };

   struct gps_info_t{
        float   ground_speed_m_per_s;   
        int32_t ground_course_cd;
        uint8_t status;
        uint8_t num_sats;
   };

   // union for sending different data types
   union osd_data_t{
      
      osd_data_t(){}
      quan::three_d::vect<float>      vect3df;
      quan::three_d::vect<uint32_t> vect3du32;
      quan::three_d::vect<int32_t>  vect3di32;
      uint8_t                byte_array12[12];
      int16_t                   i16_array6[6];
      uint16_t                  u16_array6[6];
      float                                 f;
      uint8_t                              u8;
      int8_t                               i8;
      uint16_t                            u16;
      int16_t                             i16;
      int32_t                             i32;
      uint32_t                            u32;
      bool                                  b;
      system_status_t              sys_status;
      gps_info_t                     gps_info;
   };
   // ID to identify what is in the message
   enum class msgID{
      attitude,                         // 0 
      gps_status,                       // 1
      gps_location,                     // 2  
      home_location,                    // 3
      airspeed,                         // 4
      battery,                          // 5
      system_status,                    // 6
      rcin_0_to_5,                      // 7
      rcin_6_to_11,                     // 8
      rcin_12_to_17,                    // 9
      control_mode,                     // 10
      baro_alt,                         // 11
      battery_low_voltage,              // 12
      rcin_failsafe,                     //13
      max_messages
   };

   // the message object sent by copy on the queue
   struct osd_message_t{
      osd_message_t(){}
      msgID id;
      osd_data_t value;
   };

   struct OSD_params{
      OSD_params();
      quan::angle_<float>::deg artifical_horizon_pitch_adjustment;
      int32_t viewing_distance_px; 
      quan::three_d::vect<int32_t> battery_pos;
      quan::three_d::vect<int32_t> gps_pos;
      quan::three_d::vect<int32_t> control_mode_pos;
      quan::three_d::vect<int32_t> airspeed_pos;
      quan::three_d::vect<int32_t> homeinfo_pos;
      quan::three_d::vect<int32_t> alt_scale_pos;
   };

   namespace dequeue{
      struct osd_info_t;
   }

   void draw_artificial_horizon(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_compass(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_batteries(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_gps(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_control_mode(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_airspeed(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_homeinfo(dequeue::osd_info_t const &,OSD_params const & osd);
   void draw_altitude_scale(dequeue::osd_info_t const &,OSD_params const & osd);
}

#endif // AP_OSD_H_INCLUDED
