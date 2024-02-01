
#include <cstdio>
#include "osd_message.hpp"
#include <quantracker/osd/osd.hpp>
#include <task.h>

#include <cstring>
#include <stm32f4xx.h>
#include <quan/uav/osd/api.hpp>

namespace{

   QueueHandle_t osd_queue = nullptr;

   enum class msgID{
      heading , // float
      drift ,  // vect3df;
      attitude // vect3df
   };

   // different msg types
   union osd_data_t{
      osd_data_t(){}
      quan::three_d::vect<float> vect3df;
      float f;
   };

   // the message ent on the queue
   struct osd_message_t{
      osd_message_t(){}
      msgID id;
      osd_data_t value;
   };

   // structure for use by osd in its thread
   struct osd_info_t{
      quan::three_d::vect<float>  attitude;
      quan::three_d::vect<float> drift;
      float  heading;
   } osd_info;

   // calee by osd thread to get latest data
   void get_osd_data()
   {
      if ( osd_queue != nullptr){
         osd_message_t msg;
    
         while ( xQueueReceive(osd_queue,&msg,0) == pdTRUE){
            switch (msg.id){
               case msgID::heading:
                  osd_info.heading = msg.value.f;
                  break;
               case msgID::attitude:
                  osd_info.attitude = msg.value.vect3df;
                  break;
               case msgID::drift:
                  osd_info.drift = msg.value.vect3df;
                  break;
               default:
                  break;
            }
         }
      }
   }
}

namespace Quan{

   void osd_init()
   {
      osd_info.attitude= {0.f,0.f,0.f};
      osd_info.drift = {0.f,0.f,0.f};
      osd_info.heading = 0.f;
      osd_queue = xQueueCreate(10,sizeof(osd_message_t));
   }
   
   bool osd_send_attitude(quan::three_d::vect<float> const & attitude_in)
   {
      if (osd_queue == nullptr) {return false;}
      if ( uxQueueSpacesAvailable(osd_queue) != 0 ){
         osd_message_t msg;
         msg.id = msgID::attitude;
         msg.value.vect3df = attitude_in;
         xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }

   bool osd_send_drift(quan::three_d::vect<float> const & drift_in)
   {
       if (osd_queue == nullptr) {return false;}
       if ( uxQueueSpacesAvailable(osd_queue) != 0 ){
         osd_message_t msg;
         msg.id = msgID::drift;
         msg.value.vect3df = drift_in;
         xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }

   bool osd_send_heading(float heading_in)
   {
       if (osd_queue == nullptr) {return false;}
       if ( uxQueueSpacesAvailable(osd_queue) != 0 ){
         osd_message_t msg;
         msg.id = msgID::heading;
         msg.value.f = heading_in;
         xQueueSendToBack(osd_queue,&msg,0);
         return true;
      }else{
         return false;
      }
   }
}

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    get_osd_data();
    pxp_type pos{-150,100};
    draw_text("Quan APM AHRS Test",pos);
    pos.y -= 20;
    if (osd_queue != nullptr){
       char buf [100];

       sprintf(buf,"pitch   = %8.3f",static_cast<double>(osd_info.attitude.x));
       draw_text(buf,pos);
       pos.y -= 20;

       sprintf(buf,"roll    = %8.3f",static_cast<double>(osd_info.attitude.y));
       draw_text(buf,pos);
       pos.y -= 20;

       sprintf(buf,"yaw     = %8.3f",static_cast<double>(osd_info.attitude.z));
       draw_text(buf,pos);
       pos.y -= 20;

       sprintf(buf,"heading = %8.3f",static_cast<double>(osd_info.heading));
       draw_text(buf,pos);
    }
   
}

