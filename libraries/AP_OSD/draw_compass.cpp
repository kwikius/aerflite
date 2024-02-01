
#include <quan/uav/osd/api.hpp>
#include <quan/two_d/rotation.hpp>
#include <quan/uav/osd/get_aircraft_heading.hpp>
#include <quan/uav/osd/get_aircraft_position.hpp>
#include <quan/uav/osd/get_home_position.hpp>
#include <quan/uav/get_bearing.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include "bitmaps.hpp"
#include "fonts.hpp"
//#include "symbology.hpp"
//#include "osd.hpp"

using namespace quan::uav::osd;

void AP_OSD::draw_compass (dequeue::osd_info_t const & info,AP_OSD::OSD_params const & osd) 
{
   angle_type const &heading = info.attitude.yaw;

   angle_type const & home_bearing = info.bearing_to_home;

   quan::two_d::rotation const rotate {heading}; 
  // pxp_type const pos = get_osd_compass_position();
   pxp_type const pos{0,-115};
   int constexpr radius = 25;
   draw_circle(radius + 2,pos, colour_type::black);
   // should prob make this a bitmap for speed
   // since its constant
   for (int i = 0; i < 16; ++i) {
      constexpr auto offset = angle_type {360.f / 32.f};
      angle_type const basic_angle = angle_type { (360.f * i) / 16.f};
      angle_type const start_angle = basic_angle - offset;
      angle_type const end_angle = basic_angle + offset;
      colour_type col 
         = (i & 1) 
         ? colour_type::white 
         : colour_type::black;
      draw_arc (radius + 1,pos, start_angle, end_angle,  col);
      draw_arc (radius,pos, start_angle, end_angle,  col);
   }
   draw_circle (radius - 1, pos,  colour_type::black);

   bitmap_ptr home_arrow = get_bitmap(Quan::BitmapID::home_arrow);
   if (home_arrow) {
      size_type const vect = get_size(home_arrow) / 2;
      draw_bitmap(home_arrow, pos, vect, heading - home_bearing);
   }
   
   int32_t cir_rad = 20;
   for ( int32_t i = -2; i < 3; ++i){
      color_type ncol_type
      = (( i == -2)  || (i == 2))
      ?  colour_type::white
      :  colour_type::black
      ;
      draw_arc(
         cir_rad+i,
         pos,
         angle_type{22.5} + heading,
         angle_type{90-22.5} + heading,
         ncol_type
      );

      draw_arc(
         cir_rad+i,
         pos,
         angle_type{90 + 22.5} + heading,
         angle_type{180 - 22.5} + heading,
         ncol_type
      );

      color_type ncol_type1
      = (( i == -2)  || (i == 2))
      ?  colour_type::black
      :  colour_type::white
      ;
       
      draw_arc(
         cir_rad+i,
         pos,
         angle_type{180 + 22.5} + heading,
         angle_type{270 - 22.5} + heading,
         ncol_type1
      );

      draw_arc(
         cir_rad+i,
         pos,
         angle_type{270 + 22.5} + heading,
         angle_type{360 - 22.5} + heading,
         ncol_type1
      );
   }

   font_ptr font = get_font(Quan::FontID::MWOSD);
   if (font) {
      size_type const char_size = get_size(font );
      pxp_type const  char_offset = - char_size / 2;
      constexpr auto font_radius = 19;
      constexpr char compass_char[] {'N', 'S', 'E', 'W'};
      constexpr pxp_type compass_vector[] {
         {0, font_radius}
         , {0, - font_radius}
         , {font_radius, 0}
         , { -font_radius, 0}
      };
      for (size_t i = 0; i < 4; ++i) {
         bitmap_ptr char_bmp = get_char(font,compass_char[i]);
         if (char_bmp) {
            auto const char_pos = rotate (compass_vector[i]);
            pxp_type const char_pos_i = pos + char_pos + char_offset ;
            draw_bitmap (char_bmp,char_pos_i);
         }
      }
   }
}
