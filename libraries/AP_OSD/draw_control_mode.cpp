#include <quan/uav/osd/api.hpp>

#include <AP_OSD/AP_OSD_dequeue.h>
#include "bitmaps.hpp"
#include "fonts.hpp"

namespace {

/*
enum FlightMode {
    MANUAL        = 0,
    CIRCLE        = 1,
    STABILIZE     = 2,
    TRAINING      = 3,
    ACRO          = 4,
    FLY_BY_WIRE_A = 5,
    FLY_BY_WIRE_B = 6,
    CRUISE        = 7,
    AUTOTUNE      = 8,
    AUTO          = 10,
    RTL           = 11,
    LOITER        = 12,
    GUIDED        = 15,
    INITIALISING  = 16
};
*/

   constexpr char strings [] [7] = {
      "MANUAL"
      ,"CIRCLE"   
      ,"STABIL"    
      ,"TRAING"     
      ,"ACRO  "        
      ,"FLYBWA"
      ,"FLYBWB"
      ,"CRUISE"       
      ,"AUTUNE" 
      ,"UNDEF1"    
      ,"AUTO  "        
      ,"RETL  "           
      ,"LOITER"  
      ,"UNDEF2"
      ,"UNDEF3"
      ,"GUIDED"      
      ,"INITIA"  
   };

}

namespace {
   uint8_t rcin_failsafe_count = 0;
   bool    failsafe_text_on = false;
}

using namespace quan::uav::osd;

void AP_OSD::draw_control_mode(dequeue::osd_info_t const & info,OSD_params const & osd)
{
   pxp_type pos = 
      {osd.control_mode_pos.x,
      (( get_video_mode() == video_mode::pal)
         ?osd.control_mode_pos.y
         :osd.control_mode_pos.z)
      };
   uint32_t strid = info.control_mode;
   if ( strid < (sizeof(strings) / sizeof(strings[0])) ){
      draw_text(strings[strid],pos,Quan::FontID::MWOSD);
   }

   if ( info.in_rcin_failsafe){
      if ( failsafe_text_on == false){
         if ( ++rcin_failsafe_count < 33){
            rcin_failsafe_count = 0;
            failsafe_text_on = true;
         }
      }else{  // show text
         if ( ++rcin_failsafe_count < 67){
            font_ptr font = get_font(Quan::FontID::MWOSD);
            size_type font_size = get_size(font);
            pos.y += 3 * font_size.y;
            pos.x -=  font_size.x;
            draw_text("No RC Input",pos,font);
         }else{
            rcin_failsafe_count = 0;
            failsafe_text_on = false;
         }
      }
   }else{
         rcin_failsafe_count = 0;
         failsafe_text_on = false;
   }

}