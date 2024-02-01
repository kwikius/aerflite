
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include "i2c_task.hpp"
#include "eeprom.hpp"
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_periph.hpp>

#include <AP_HAL_Quan/i2c/i2c_driver/i2c_periph.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_eeprom_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_register_based_driver.cpp>
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_read_write_driver.cpp>

#if defined QUAN_I2C_DEBUG
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_debug.cpp>
#endif

using AP_HAL::millis;
extern const AP_HAL::HAL& hal;

namespace {

   // queue for sending baro data to apm thread
   QueueHandle_t hBaroQueue = nullptr;
   // queue for sending compass data to apm thread
   QueueHandle_t hCompassQueue = nullptr;

   QueueHandle_t hCompassGainQueue = nullptr;

#if defined QUAN_MIXER_TRANQUILITY
   QueueHandle_t hAirspeedQueue = nullptr;
#endif

   void wait_for_power_up();

   struct task{
      task(const char* name_in, uint32_t begin_ms, uint32_t len_ms, bool (*pfn)())
      :m_name{name_in}, m_begin_ms{begin_ms}, m_task_length_ms{len_ms}, m_pfn{pfn}{}

      bool run()
      {
        return m_pfn();
      }

      uint32_t get_begin_ms() const {return m_begin_ms;}
      uint32_t get_length_ms() const { return m_task_length_ms;}
      const char* get_name() const {return m_name;}
     private:
         const char* const m_name;
         uint32_t const m_begin_ms;
         uint32_t const m_task_length_ms;
         bool (* const m_pfn)();
   };

   void i2c_task(void* params)
   {
      // need to wait for 5V on the external sensor voltage regs
      wait_for_power_up();

      hal.console->printf("starting i2c task\n");

#if defined QUAN_MIXER_TRANQUILITY
      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) || (hAirspeedQueue == nullptr) ){
#else
      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) ){
#endif
        AP_HAL::panic("create FreeRTOS queues failed in I2c task\n");
      }

      Quan::i2c_periph::init();

#if defined QUAN_MIXER_TRANQUILITY
      if ( !Quan::sdp3_init()){
          AP_HAL::panic("sdp3x init failed");
      }
#endif

      if (!Quan::setup_baro()) {
         AP_HAL::panic("baro setup failed");
      }

      if (!Quan::setup_compass()) {
         AP_HAL::panic("compass setup failed");
      }

      uint32_t looptime_ms = 50U;
      TickType_t previous_waketime_ms = xTaskGetTickCount();


      /*
        i2c task duration is dependent on the I2C clock speed, which is limited to
         the speed of the slowest densor on the bus.
         If there is an external sensor, the clock speed should be limited to 100 kHz.
         TODO. Could add

      */

      task tasks [] = {
           // task name, task loop begin time ms, task duration ms, task function
          {"baro : request conversion"    ,  1 ,   1, Quan::baro_request_conversion}
         ,{"compass : request conversion" ,  2 ,   1, Quan::compass_request_conversion}
#if defined QUAN_MIXER_TRANQUILITY
         ,{"airspeed : start read"        ,  3,    2, Quan::sdp3_start_read}
         ,{"airspeed : calculate"         ,  7,    2, Quan::sdp3_calculate}
#endif
         ,{"compass : start_read"         ,  9,   2, Quan::compass_start_read}
         ,{"compass_calculate"            ,  14 ,  1, Quan::compass_calculate}
         ,{"eeprom : serv write buffer"   ,  15 ,  25, Quan::eeprom_service_write_buffer}
         ,{"baro : start read"            ,  45 ,  2, Quan::baro_start_read}
         ,{"baro : calculate"             ,  48 ,  1, Quan::baro_calculate}
      };

      constexpr uint32_t num_tasks = sizeof(tasks)/ sizeof(task);
#if defined QUAN_I2C_DEBUG
      bool succeeded = true;
#endif
    //TODO.
 // On bus error etc. clear the error and continue
      for (;;){
#if defined QUAN_I2C_DEBUG
         Quan::reset_i2c_sr1_flags_index();
#endif
         auto const loop_start_ms = millis();
         for ( uint32_t i = 0; i < num_tasks ; ++i){
            // do any eeprom reads in a timely fashion,
            // but allow the start conv requests to be started first
#if defined QUAN_MIXER_TRANQUILITY
            static constexpr uint32_t first_tasks_started = 2U;
#else
            static constexpr uint32_t first_tasks_started = 1U;
#endif
            if ( i > first_tasks_started) {// now the baro and compass request have been started

               if( ! Quan::eeprom_service_read_requests()){
#if defined QUAN_I2C_DEBUG
                  succeeded = false;
                  break;
#else
                  Quan::i2c_periph::init();
#endif
              }
            }
            task & t = tasks[i];
            auto const time_since_task_started = millis() - loop_start_ms;
            if ( time_since_task_started < t.get_begin_ms()  ){
              vTaskDelay( t.get_begin_ms() - time_since_task_started);
            }

            if ( !t.run()){
               hal.console->printf("i2c task %s failed\n",t.get_name());
#if defined QUAN_I2C_DEBUG
               succeeded = false;
               break;
#else
               Quan::i2c_periph::init();
#if defined QUAN_MIXER_TRANQUILITY
               Quan::sdp3_init();
#endif
#endif
            }
         }
#if defined QUAN_I2C_DEBUG
         if (succeeded) {
#endif
         vTaskDelayUntil(&previous_waketime_ms, looptime_ms);
#if defined QUAN_I2C_DEBUG
         }else{
            break;
         }
#endif
      }  // ~task loop
#if defined QUAN_I2C_DEBUG
  // ----------------- get here on fail
   // todo we actually need to try to recover
      if ( Quan::i2c_periph::has_errored()){
         hal.console->printf("NOTE:--- i2c has flagged errored ---\n");
      }else{
         hal.console->printf("--- i2c has failed but no errors flagged ---\n");
      }

      //read the captured interrupt entry flags
      Quan::show_i2c_sr1_flags();

         // show we are still running
      uint32_t count = 0;
      for(uint32_t i = 0; i < 120;++i){
         vTaskDelayUntil(&previous_waketime_ms,looptime_ms);
         hal.console->printf("*");
         if (++count == 40){
            count = 0;
            hal.console->printf("\n");
         }
      }
      // do nothing
      for(;;){
         vTaskDelayUntil(&previous_waketime_ms,looptime_ms);
      }
#endif
   }

   void wait_for_power_up()
   {
      uint32_t const now_ms = AP_HAL::millis();
      if( now_ms < 200){ // should be board_startup_time
         hal.scheduler->delay(200 - now_ms);
      }
   }

   TaskHandle_t task_handle;
   void * dummy_params;

}

namespace Quan {

   TaskHandle_t get_i2c_task_handle() { return task_handle;}

   void create_i2c_task()
   {

      hBaroQueue = xQueueCreate(1,sizeof(Quan::detail::baro_args));
      hCompassQueue = xQueueCreate(1,sizeof(Quan::detail::compass_args));
      hCompassGainQueue = xQueueCreate(1,sizeof(Quan::detail::compass_gain));

#if defined QUAN_MIXER_TRANQUILITY
      hAirspeedQueue = xQueueCreate(1,sizeof(Quan::detail::airspeed_args));
#endif

      if (! Quan::setup_eeprom()){
         AP_HAL::panic("eeprom setup failed");
      }

      xTaskCreate(
         i2c_task,"I2C_task",
         1000,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
         & task_handle
      ) ;
   }

   QueueHandle_t get_compass_queue_handle()
   {
      if ( hCompassQueue == nullptr){
         panic("Requesting null compass queue handle\n");
      }
      return hCompassQueue;
   }

   QueueHandle_t get_baro_queue_handle()
   {
      if ( hBaroQueue == nullptr){
         panic("Requesting null baro queue handle\n");
      }
      return hBaroQueue;
   }

   QueueHandle_t get_compass_gain_handle()
   {
      if ( hCompassGainQueue == nullptr){
         panic("Requesting null compass gain queue handle\n");
      }
      return hCompassGainQueue;
   }

#if defined QUAN_MIXER_TRANQUILITY
 QueueHandle_t get_airspeed_queue_handle()
   {
      if ( hAirspeedQueue == nullptr){
         panic("Requesting null airspeed handle\n");
      }
      return hAirspeedQueue;
   }
#endif

}

#endif // #if defined QUAN_AERFLITE_BOARD
#endif // AP_HAL_QUAN
