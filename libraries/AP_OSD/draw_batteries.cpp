
#include <cstdio>
#include <cmath>
#include <quan/min.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <quan/uav/osd/api.hpp>
#include "bitmaps.hpp"
#include "fonts.hpp"

using namespace quan::uav::osd;

namespace {
   uint8_t low_battery_count = 0;
   bool    low_battery_text_on = false;
}

void AP_OSD::draw_batteries(dequeue::osd_info_t const & info,OSD_params const & osd)
{
   pxp_type  pos = 
      {osd.battery_pos.x,
      (( get_video_mode() == video_mode::pal)?osd.battery_pos.y:osd.battery_pos.z)};

   font_ptr font = get_font(Quan::FontID::MWOSD);
   if ( font){
      size_type font_size = get_size(font);
      size_type text_size = {font_size.x * 5, font_size.y * 2};
      // text left bottom == 0,0
     
      size_type const border = {2,2};

      pxp_type p1 = pxp_type{-border.x, text_size.y + border.y} ;
      pxp_type p2 = pxp_type{ text_size.x + border.x, p1.y} ;
      pxp_type p3 = pxp_type{ p2.x, p2.y - (5 + border.y) } ;
      pxp_type p4 = pxp_type{ p3.x + 4 , p3.y } ;
      pxp_type p5 = pxp_type{ p4.x, 5 - border.y} ;
      pxp_type p6 = pxp_type{ p3.x,p5.y} ;
      pxp_type p7 = pxp_type{ p6.x, -border.y} ;
      pxp_type p8 = pxp_type{ p1.x, p7.y} ;

      p1 += pos;
      p2 += pos;
      p3 += pos;
      p4 += pos;
      p5 += pos;
      p6 += pos;
      p7 += pos;
      p8 += pos;

      draw_line(p1,p2,colour_type::white);
      draw_line(p2,p3,colour_type::white);
      draw_line(p3,p4,colour_type::white);
      draw_line(p4,p5,colour_type::white);
      draw_line(p5,p6,colour_type::white);
      draw_line(p6,p7,colour_type::white);
      draw_line(p7,p8,colour_type::white);
      draw_line(p8,p1,colour_type::white);

      p1 += pxp_type{-1,1};
      p2 += pxp_type{1,1};
      p3 += pxp_type{1,1};
      p4 += pxp_type{1,1};
      p5 += pxp_type{1,-1};
      p6 += pxp_type{1,-1};
      p7 += pxp_type{1,-1};
      p8 += pxp_type{-1,-1};
      
      draw_line(p1,p2,colour_type::black);
      draw_line(p2,p3,colour_type::black);
      draw_line(p3,p4,colour_type::black);
      draw_line(p4,p5,colour_type::black);
      draw_line(p5,p6,colour_type::black);
      draw_line(p6,p7,colour_type::black);
      draw_line(p7,p8,colour_type::black);
      draw_line(p8,p1,colour_type::black);

      char buf [30];

      sprintf(buf,"%4.1fV",static_cast<double>(info.battery_voltage.numeric_value()));
      draw_text(buf,pos,font);
      pos.y += font_size.y;
      sprintf(buf,"%4.1fA",static_cast<double>(info.battery_current.numeric_value()));
      draw_text(buf,pos,font);

      if ( info.battery_voltage < info.battery_low_voltage){
         if ( low_battery_text_on == false){
            if ( ++low_battery_count < 33){
               low_battery_count = 0;
               low_battery_text_on = true;
            }
         }else{  // show text
            if ( ++low_battery_count < 67){
               pos.y += 2 * font_size.y;
               pos.x -= 7 * font_size.x;
               draw_text("Low Battery",pos,font);
            }else{
               low_battery_count = 0;
               low_battery_text_on = false;
            }
         }
      }else{
         low_battery_count = 0;
         low_battery_text_on = false;
      }
   }
}