
#include <quan/uav/osd/api.hpp>

#include <quan/constrain.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>
#include <AP_OSD/fonts.hpp>


using namespace quan::uav::osd;
namespace {
   QUAN_QUANTITY_LITERAL(length,m)
   constexpr auto granularity = 1_m;
   constexpr auto num_bars = 16;
   constexpr auto gran_scale = 10 / 1_m; // 10 px per m
}

void AP_OSD::draw_altitude_scale(dequeue::osd_info_t const & info,OSD_params const & osd)
{
   auto const font_size = get_size( get_font(Quan::FontID::MWOSD));
   quan::length::m const aircraft_altitude = info.aircraft_position.alt - info.home_position.alt;

   pxp_type pos{ osd.alt_scale_pos.x, (( quan::uav::osd::get_video_mode() == quan::uav::osd::video_mode::pal)
       ?osd.alt_scale_pos.y
       :osd.alt_scale_pos.z)
   };

   auto const max_alt_bar = static_cast<int32_t>( (aircraft_altitude  + (num_bars/2U) *granularity) / granularity) * granularity;

   for ( auto i = 0; i <= num_bars; ++i){
      auto const cur_alt_bar = max_alt_bar - (granularity * i);
      auto const int_height = static_cast<int>(cur_alt_bar/1_m);
      auto const y_pos = (cur_alt_bar - aircraft_altitude) * gran_scale;
      auto line_len = 0;
      if ( (int_height % 5) == 0){
         if ( (int_height % 10) == 0){
              line_len = 20;
              pxp_type p1 {pos.x, pos.y + y_pos - font_size.y/2};
              draw_text<100,Quan::FontID::MWOSD>(p1,"% 6i\n", int_height);
         }else{
             line_len = 15;
         }
      }else{
         line_len = 10;
      }
      auto const tab_pos_x = font_size.x * 6;
      pxp_type p1{pos.x + tab_pos_x + 20 - line_len,pos.y + y_pos +1};
      pxp_type p2 = p1 + pxp_type{line_len,0};
      draw_line(p1,p2,colour_type::black);
      p1 -= pxp_type{0,1};
      p2 -= pxp_type{0,1}; 
      draw_line(p1,p2,colour_type::white);
      p1 -= pxp_type{0,1};
      p2 -= pxp_type{0,1}; 
      draw_line(p1,p2,colour_type::black);
   }

   pxp_type p1{pos.x,pos.y + font_size.y/2};
   pxp_type p2{pos.x + font_size.x * 6 + 20,pos.y - font_size.y/2};

   // clear a box for current altitude
   draw_box(p1,p2,colour_type::transparent,true);

   p1 += pxp_type{20-2,2};
   p2 += pxp_type{23,-2};

   draw_box(p1,p2,colour_type::white,false);
   // put current altitude in middle of scale
   pxp_type p3{pos.x+20,pos.y -font_size.y/2};
   draw_text<100,Quan::FontID::MWOSD>(p3,"% 6i\n", static_cast<int32_t>(aircraft_altitude.numeric_value() + 0.5f));
    
}
