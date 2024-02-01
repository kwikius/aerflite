/*
 Copyright (c) 2003-2014 Andy Little.

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see http://www.gnu.org/licenses./
 */


#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <quan/where.hpp>
#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstdlib>
#include <quantracker/osd/telemetry_transmitter.hpp>
#include <quan/tracker/zapp4/position.hpp>
#include <quan/tracker/zapp4/gps_status.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_OSD/fonts.hpp>
#include <AP_GPS/AP_GPS.h>
#include "Plane.h"

namespace{

   AP_OSD::dequeue::osd_info_t info;
   AP_OSD::OSD_params osd;
#if defined QUAN_AERFLITE_BOARD
   AP_HAL::UARTDriver * telem_port = nullptr;
   uint32_t last_telem_time_ms = 0U; 
   uint32_t telem_loop_time_ms = 150; 
   // make the timing of telem updates somewhat random
   // This means that hopefully it won't sync with other 
   // 433 MHz radio signals
   void update_loop_time_ms()
   {
     telem_loop_time_ms = 100 + (rand() % 100);
   }

   constexpr char version_string []= "AerFlite OSD V1.1";
#else
   constexpr char version_string []= "AerfQuan OSD V1.1";
#endif
   
}

namespace Quan{
   AP_OSD::dequeue::osd_info_t const & get_osd_info();
}

AP_OSD::dequeue::osd_info_t const & Quan::get_osd_info()
   {return info;}

namespace {

   void do_startup_screen()
   {
      quan::uav::osd::pxp_type pos {-150,20};
      quan::uav::osd::draw_text(version_string,pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return x 3 for CLI",pos,Quan::FontID::MWOSD); 
   }

   // ideally hand over to osd
   // need a way for osd to take over
   void do_cli()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("CLI setup",pos,Quan::FontID::MWOSD); 
   }

   void do_initialising()
   {

      quan::uav::osd::pxp_type pos {-150,20};
      quan::uav::osd::draw_text("initialising",pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Press return x 3 for CLI",pos,Quan::FontID::MWOSD); 
#if defined QUAN_AERFLITE_BOARD
      if ( telem_port == nullptr){
         telem_port = hal.uartD; // 3.3V port uart6
         if ( telem_port != nullptr){
            telem_port->begin(115200, 100,100);
         }
         //srand (AP_HAL::millis());
      }
#endif

   } 

   void do_loading_eeprom_params()
   {
      quan::uav::osd::pxp_type pos {-150,20};
      quan::uav::osd::draw_text("Loading params",pos); 
      pos.y -= 20;
      quan::uav::osd::draw_text("Please wait...",pos,Quan::FontID::MWOSD); 
   }

   void do_demo_servos()
   {
      quan::uav::osd::pxp_type pos {-150,20};
      quan::uav::osd::draw_text("Demo servos",pos); 
   }

   void do_running()
   {
     AP_OSD::draw_artificial_horizon(info,osd);
     AP_OSD::draw_compass(info,osd);
     AP_OSD::draw_batteries(info,osd);
     AP_OSD::draw_gps(info,osd);
     AP_OSD::draw_control_mode(info,osd);
     AP_OSD::draw_airspeed(info,osd);
     AP_OSD::draw_homeinfo(info,osd);
     AP_OSD::draw_altitude_scale(info,osd);
   }

   void do_unknown()
   {
      quan::uav::osd::pxp_type pos {-100,20};
      quan::uav::osd::draw_text("unknown system state - report",pos); 
   }
   
}
 
void quan::uav::osd::on_draw() 
{ 
   AP_OSD::dequeue::read_stream(info);

   AP_OSD::dequeue::update(info);

   switch(Quan::get_osd_info().system_status){
     case AP_OSD::system_status_t::starting:
         do_startup_screen();
         break;
     case AP_OSD::system_status_t::in_cli:
         do_cli();
         break;
     case AP_OSD::system_status_t::demo_servos:
         do_demo_servos();
         break;
     case AP_OSD::system_status_t::loading_eeprom_params:
         do_loading_eeprom_params();
         break;
     case AP_OSD::system_status_t::initialising:
         do_initialising();
         break;
     case AP_OSD::system_status_t::running:
         do_running();
         break;
     default:
         do_unknown();
         break;
   }
    
}

namespace{

   void transmit_gps_data()
   {
      quan::uav::osd::norm_position_type norm_pos;
      {
         vTaskSuspendAll();
          norm_pos.lat = info.aircraft_position.lat;
          norm_pos.lon = info.aircraft_position.lon;
          norm_pos.alt = info.aircraft_position.alt;
         xTaskResumeAll();
      }
      uint8_t encoded [19];
      
      quan::tracker::zapp4::encode_position(norm_pos,encoded);
      write_telemetry_data((const char*)encoded,19);

#if defined QUAN_AERFLITE_BOARD
//write to uart for rf modem
      auto const time_now_ms = AP_HAL::millis();
      if ( (time_now_ms - last_telem_time_ms) >= telem_loop_time_ms){
         last_telem_time_ms = time_now_ms;
         update_loop_time_ms();
         if ( telem_port != nullptr){
            telem_port->write(encoded,19);
            telem_port->write(static_cast<uint8_t>(0U)); // for end of frame
         }
      }
#endif
   }

   void transmit_gps_status(quan::tracker::zapp4::gps_status_t gps_status)
   {
      uint8_t encoded[11];
      quan::tracker::zapp4::encode_gps_status(gps_status,encoded);
      write_telemetry_data((const char*)encoded,11);
#if defined QUAN_AERFLITE_BOARD
//write to uart for rf modem
//      auto const time_now_ms = AP_HAL::millis();
//      if ( (time_now_ms - last_telem_time_ms) >= telem_loop_time_ms){
//         last_telem_time_ms = time_now_ms;
//         update_loop_time_ms();
//         if ( telem_port != nullptr){
//            telem_port->write(encoded,11);
//         }
//      }
#endif
   }
}


// called in context of sytem transmit telemetry task
void on_telemetry_transmitted()
{

/*
    if GPS is in a good state just send data, else send status. If it isnt syatem startup that is bad
    since it probably means the GPS has lost its lock or is dead!
*/
   switch ( info.gps_status ){

      case AP_GPS::GPS_OK_FIX_3D:      
      case AP_GPS::GPS_OK_FIX_3D_DGPS: 
      case AP_GPS::GPS_OK_FIX_3D_RTK:
         transmit_gps_data();
         break;
      case AP_GPS::NO_GPS:
         transmit_gps_status(quan::tracker::zapp4::gps_status_t::no_gps);
         break;
      case AP_GPS::NO_FIX:
         transmit_gps_status(quan::tracker::zapp4::gps_status_t::no_fix);
         break;
      case AP_GPS::GPS_OK_FIX_2D:
         transmit_gps_status(quan::tracker::zapp4::gps_status_t::fix_2d);
         break;
      default:
         transmit_gps_status(quan::tracker::zapp4::gps_status_t::unknown);
         break;
      
   }
   
}

#endif  // #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
