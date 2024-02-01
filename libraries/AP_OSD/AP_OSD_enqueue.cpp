#include "AP_OSD_enqueue.h"
#include <quan/min.hpp>

namespace AP_OSD{namespace dequeue{namespace detail{
   QueueHandle_t initialise();
}}}

// called by the sender when ready to start sending
namespace {
   // note this is a duplicate handle
   QueueHandle_t osd_queue = nullptr;
}

void AP_OSD::enqueue::initialise()
{
   osd_queue = AP_OSD::dequeue::detail::initialise();
}

namespace {

   bool queue_ready_for_msg()
   {
      return ( (osd_queue != nullptr) && (uxQueueSpacesAvailable(osd_queue) != 0) );
   }

   bool put_message( AP_OSD::msgID id, quan::three_d::vect<float> const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.vect3df = v; 
          xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }

#if 0
// not used atm
   bool put_message( AP_OSD::msgID id, quan::three_d::vect<uint32_t> const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.vect3du32 = v; 
          xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }
#endif
   bool put_message( AP_OSD::msgID id, quan::three_d::vect<int32_t> const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.vect3di32 = v; 
          xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }

   bool put_message( AP_OSD::msgID id, float const & v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.f = v;  
          xQueueSendToBack(osd_queue,&msg,0);
          return true;
      }else{
         return false;
      }
   }

   bool put_message( AP_OSD::msgID id, uint8_t v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.u8 = v;  
          xQueueSendToBack(osd_queue,&msg,0);
          return true;
      }else{
         return false;
      }
   }

   bool put_message( AP_OSD::msgID id, bool v)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          msg.value.b = v;  
          xQueueSendToBack(osd_queue,&msg,0);
          return true;
      }else{
         return false;
      }
   }




   bool put_message(AP_OSD::msgID id, uint16_t * ar, uint8_t n_in)
   {
      if ( queue_ready_for_msg()){
          AP_OSD::osd_message_t msg;
          msg.id = id;
          uint8_t const n = quan::min(n_in,6);
          for ( uint8_t i = 0; i < 6; ++i){
            uint16_t const val_in = (i < n)? ar[i] : 0;
            msg.value.u16_array6[i] = val_in;
          }
          xQueueSendToBack(osd_queue,&msg,0);
          return true;
      }else{
         return false;
      }
   }

}

bool AP_OSD::enqueue::rcin_failsafe(bool value)
{
   return put_message(AP_OSD::msgID::rcin_failsafe,value);
}
   
bool AP_OSD::enqueue::attitude(quan::three_d::vect<float> const & in)
{
   return put_message(AP_OSD::msgID::attitude,in);
}

bool AP_OSD::enqueue::baro_alt(float const & baro_alt_m)
{
   return put_message(AP_OSD::msgID::baro_alt,baro_alt_m );
}

bool AP_OSD::enqueue::gps_status(AP_OSD::gps_info_t in)
{
   if ( queue_ready_for_msg()){
      AP_OSD::osd_message_t msg;
      msg.id = AP_OSD::msgID::gps_status;
      msg.value.gps_info = in;
      xQueueSendToBack(osd_queue,&msg,0);
      return true;
   }else{
      return false;
   }
}

bool AP_OSD::enqueue::gps_location(quan::three_d::vect<int32_t> const & in)
{
  return put_message(AP_OSD::msgID::gps_location,in);
}

bool AP_OSD::enqueue::home_location(quan::three_d::vect<int32_t> const & in)
{
   return put_message(AP_OSD::msgID::home_location,in);
}

bool AP_OSD::enqueue::airspeed(float in)
{
   return put_message(AP_OSD::msgID::airspeed,in);
}

bool AP_OSD::enqueue::battery_low_voltage(float in)
{
   return put_message(AP_OSD::msgID::battery_low_voltage,in);
}

bool AP_OSD::enqueue::battery(quan::three_d::vect<float> const & in)
{
   return put_message(AP_OSD::msgID::battery,in);
}

bool AP_OSD::enqueue::system_status(AP_OSD::system_status_t status)
{
   if ( queue_ready_for_msg()){
       AP_OSD::osd_message_t msg;
       msg.id = AP_OSD::msgID::system_status;
       msg.value.sys_status = status;  
       xQueueSendToBack(osd_queue,&msg,0);
       return true;
   }else{
      return false;
   }
}

bool AP_OSD::enqueue::rc_inputs_0_to_5(uint16_t * ar, uint8_t n)
{
    return put_message(AP_OSD::msgID::rcin_0_to_5,ar,n);
}

bool AP_OSD::enqueue::rc_inputs_6_to_11(uint16_t * ar, uint8_t n)
{
    return put_message(AP_OSD::msgID::rcin_6_to_11,ar,n);
}

bool AP_OSD::enqueue::rc_inputs_12_to_17(uint16_t * ar, uint8_t n)
{
   return put_message(AP_OSD::msgID::rcin_12_to_17,ar,n);
}

/*
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
*/

bool AP_OSD::enqueue::control_mode(uint8_t mode)
{
   if ( mode < 17){
      return put_message(AP_OSD::msgID::control_mode,mode);
   }else{
      return false;
   }
}

