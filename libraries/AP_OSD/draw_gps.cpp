
#include <cstdio>
#include <cmath>
#include <quan/min.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_GPS/AP_GPS.h>
#include <quan/uav/osd/api.hpp>
#include "bitmaps.hpp"
#include "fonts.hpp"

/*
  while not had a good fix from startup
   then just show GPS status num sats
  else
   if fix bad then flashing 
   else just show lat lon 
  
*/

using namespace quan::uav::osd;

void AP_OSD::draw_gps(dequeue::osd_info_t const & info,OSD_params const & osd)
{

   bool valid_fix  = false;
   const char* gps_status_txt = "No GPS";

   switch(info.gps_status){
      case AP_GPS::NO_GPS:
         break;
      case AP_GPS::NO_FIX:
         gps_status_txt = "No Fix";
         break;
      case AP_GPS::GPS_OK_FIX_2D:
         gps_status_txt = "2D Fix";
         break;
      case AP_GPS::GPS_OK_FIX_3D:
         gps_status_txt = "3D Fix";
         valid_fix = true;
         break;
      case AP_GPS::GPS_OK_FIX_3D_DGPS:
         gps_status_txt = "3D Fix dgps";
          valid_fix = true;
         break;
      case AP_GPS::GPS_OK_FIX_3D_RTK:
         gps_status_txt = "3D Fix RTK";
          valid_fix = true;
         break;
      default:
         break;
   }

   pxp_type pos = {osd.gps_pos.x,(( get_video_mode() == video_mode::pal)?osd.gps_pos.y:osd.gps_pos.z)};
   font_ptr font = get_font(Quan::FontID::MWOSD);
   auto font_size = get_size(font);
   char buf[100];
   if ( valid_fix){
      sprintf(buf,"Lon:%10.5f",static_cast<double>(info.aircraft_position.lon.numeric_value() * 1.e-7f));
      draw_text(buf,pos,font);
      pos.y += font_size.y;
      sprintf(buf,"Lat:%10.5f",static_cast<double>(info.aircraft_position.lat.numeric_value() * 1.e-7f));
      draw_text(buf,pos,font);
   }else{
      sprintf(buf,"%u sats",static_cast<unsigned>(info.gps_num_sats));
      draw_text(buf,pos,font);
      pos.y += font_size.y;
      sprintf(buf,"GPS:%s",gps_status_txt);
      draw_text(buf,pos,font);
   }
  
}

