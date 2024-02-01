
#include <cstdio>

#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_GPS/AP_GPS.h>

namespace {

   AP_OSD::dequeue::osd_info_t aircraft_info;

   void do_gps(quan::uav::osd::pxp_type const & pos);

}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
    AP_OSD::dequeue::read_stream(aircraft_info);

    pxp_type pos{-150,110};
    
    do_gps(pos);
    pos.y -= 20;

    char buf [100];

    sprintf(buf,"pitch   = % 8.3f deg",static_cast<double>(aircraft_info.attitude.pitch.numeric_value()));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"roll    = % 8.3f deg",static_cast<double>(aircraft_info.attitude.roll.numeric_value()));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"yaw     = % 8.3f deg",static_cast<double>(aircraft_info.attitude.yaw.numeric_value()));
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"lat     = % 8.3f deg",static_cast<double>(aircraft_info.aircraft_position.lat.numeric_value())* 1e-7);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"lon     = % 8.3f deg",static_cast<double>(aircraft_info.aircraft_position.lon.numeric_value())*1e-7);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"alt     = % 8.3f m",static_cast<double>(aircraft_info.aircraft_position.alt.numeric_value()) * 1e-3);
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"baroalt = % 8.3f m",static_cast<double>(aircraft_info.baro_alt.numeric_value()) );
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"airspd  = % 8.3f m/s",static_cast<double>(aircraft_info.airspeed.numeric_value()) );
    draw_text(buf,pos);
    pos.y -= 20;

    sprintf(buf,"Batt V  = % 8.3f V",static_cast<double>(aircraft_info.battery_voltage.numeric_value()) );
    draw_text(buf,pos);
    pos.y -= 20;
   
    sprintf(buf,"Batt I  = % 8.3f A",static_cast<double>(aircraft_info.battery_current.numeric_value()) );
    draw_text(buf,pos);
    pos.y -= 20;
      

}

namespace {

   void do_gps(quan::uav::osd::pxp_type const & pos)
   {
      using quan::uav::osd::draw_text;
      switch ( static_cast<AP_GPS::GPS_Status>(aircraft_info.gps_status) ){
          case AP_GPS::NO_GPS:
            draw_text("No GPS",pos);
            break;
          case AP_GPS::NO_FIX:
            draw_text("No Fix",pos);
            break;
          case AP_GPS::GPS_OK_FIX_2D:
            draw_text("2D Fix",pos);
            break;
          case AP_GPS::GPS_OK_FIX_3D:
            draw_text("3D Fix",pos);
            break;
          case AP_GPS::GPS_OK_FIX_3D_DGPS:
            draw_text("3D Fix dgps",pos);
            break;
          case AP_GPS::GPS_OK_FIX_3D_RTK:
            draw_text("3D Fix RTK",pos);
            break;
          default:
            draw_text("GPS state out of range",pos);
            break;
      }
   }
}

