
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include "FreeRTOS.h"
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include "eeprom.hpp"

/*
   for eeprom read commands, the calling thread (e.g usually for example ArduPlane app)
   is blocked  until the read is completed
   Read can then be of "unlimited" size.
   While a read is progressing the other i2c tasks will be blocked though.
   The read queue consists of one element
   When the read is completed the eeprom(i2c ) task sends a read complete  semaphore
   That signifies the memory the read was done to is now valid and unblocks the calling task


   for eeprom writes
   A  queue is used and the data to write is copied to the queue, so the calling thread isnt blocked
   unless the queue becomes full
   Each buffer element represents a write of a certain fixed max number of bytes

*/

using AP_HAL::panic;
using AP_HAL::millis;

extern const AP_HAL::HAL& hal;

namespace {

   QueueHandle_t      eeprom_read_handle = nullptr;

   QueueHandle_t      eeprom_write_handle = nullptr;

   SemaphoreHandle_t  eeprom_read_complete_semaphore = nullptr;

   TimerHandle_t      h_eeprom_timer = nullptr;

}

namespace Quan{

   /*
      return
         0 on  nothing done,
         1 on  successful read
        -1 on  fail
   */
   bool eeprom_service_read_requests()
   {
      eeprom_read_msg msg;
      if ( xQueueReceive(eeprom_read_handle,&msg,0) == pdTRUE){
          bool result = eeprom::read(msg.eeprom_address,msg.mcu_address, msg.num_elements);
          if (result ){
            xSemaphoreGive(eeprom_read_complete_semaphore);
            return true;
          }else{
            hal.console->write("eeprom : service read to queue failed");
            return false;
          }
      }else{
         return true;
      }
  }

}//Quan

namespace {

   Quan::eeprom_write_msg __attribute__ ((section (".dma_memory"))) m_eeprom_write_msg;
}

namespace Quan{

   bool eeprom_service_write_buffer()
   {
      auto const start_time = millis();
      for(;;){
         eeprom_read_msg read_msg;
         if ( xQueuePeek(eeprom_read_handle,&read_msg,0) == pdTRUE ){
            // if there is a read message quit and deal with it
            break;
         }
         auto const up_time = millis() - start_time;
         if (up_time < 20) {
            if ( xQueueReceive(eeprom_write_handle,&m_eeprom_write_msg,up_time) == pdTRUE){
               bool result = eeprom::write(
                  m_eeprom_write_msg.eeprom_address,
                  m_eeprom_write_msg.data,
                  m_eeprom_write_msg.num_elements
               );
               if (!result){
                  AP_HAL::panic("eeprom : write from queue failed");
                  return false;
               }
            }
         }else{
            break;
         }
      }
      return true;
   }

   QueueHandle_t get_eeprom_read_handle()
   {
      if ( eeprom_read_handle == nullptr){
         panic("Requesting null eeprom read handle\n");
      }
      return eeprom_read_handle;
   }

   QueueHandle_t get_eeprom_write_handle()
   {
      if ( eeprom_write_handle == nullptr){
         panic("Requesting null eeprom write handle\n");
      }
      return eeprom_write_handle;
   }

   SemaphoreHandle_t get_read_complete_semaphore()
   {
      if ( eeprom_read_complete_semaphore == nullptr){
         panic("Requesting null eeprom read complete semaphore\n");
      }
      return eeprom_read_complete_semaphore;
   }

   TimerHandle_t get_eeprom_timer_handle()
   {
      if ( h_eeprom_timer != nullptr){
         return h_eeprom_timer;
      }else{
         panic("Requesting null eeprom timer handle\n");
      }
   }

   bool setup_eeprom()
   {
      // for reading block till it returns so only 1 element
      constexpr uint32_t read_msg_size = sizeof(Quan::eeprom_read_msg);
      eeprom_read_handle = xQueueCreate(1,read_msg_size);

      // for writing dont block if possible so use a reasonable size queue
      // allow 1k mem
      // with a queue element size of 36 bytes , gives 28 elements
      // that can be written before the queue blocks
      constexpr uint32_t write_msg_size = sizeof(Quan::eeprom_write_msg);
      eeprom_write_handle = xQueueCreate(1024/write_msg_size,write_msg_size);

      eeprom_read_complete_semaphore = xSemaphoreCreateBinary();

      static constexpr TickType_t init_ee_timer_period = 5U;
      static constexpr BaseType_t reload_false = pdFALSE;
      h_eeprom_timer = xTimerCreate(
         "ee_timer"
         ,init_ee_timer_period
         ,reload_false          // This makes a "one shot" timer
         ,(void*) 0             // The timer id
         ,&i2c_eeprom_driver_base::timer_callback
      );

      return (eeprom_read_handle != nullptr)
      && (eeprom_write_handle != nullptr)
      && (eeprom_read_complete_semaphore != nullptr)
      && (h_eeprom_timer != nullptr) ;
   }
}
