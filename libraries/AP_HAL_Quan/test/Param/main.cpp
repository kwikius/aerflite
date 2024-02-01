
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_HAL_Quan/Storage.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <quan/min.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
  test of basic storage function
*/

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

  // constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint32_t ee_addr = 1253;

   char buffer[100U];

   AP_Float  float_param;

   struct test_task_t{

      test_task_t(): m_count{false}{}

      void fun()
      { 
          if ( m_count == false){

            float_param.set(1.2345f);
            if ( float_param.save()){
               hal.console->printf("save %f successful\n",static_cast<double>(float_param.get()));
            }else{
               hal.console->printf("save failed\n");
            }
            float_param.set(4.5678f);
            hal.console->printf("value now %f\n",static_cast<double>(float_param.get()));
            m_count = true;
          }else{

             if ( float_param.load()){
               hal.console->printf("param load ok got %f\n",static_cast<double>(float_param.get()));
             }else{
              hal.console->printf("load failed\n");
             }
             m_count = false;
          }
      };

      void init()
      {
         // hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
         // hal.gpio->write(red_led_pin,pin_off);
          
      }
   private:
      bool m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{

   
 	hal.console->printf("Quan APM Param test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);

   AP_Param::setup();
//   test_task.init();
//
   test_task.fun();
   test_task.fun();
}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Param test",{-150,50});
}

namespace {

   TickType_t prev_wake_time= 0; 
 
}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,500); 

   hal.gpio->toggle(test_pin);
   
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_uartC = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )


