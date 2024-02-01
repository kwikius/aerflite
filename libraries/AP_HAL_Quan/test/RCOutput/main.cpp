
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
   Test of the RC Output
   makes the outputs go end to end at different rates
*/

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
using AP_HAL::millis;

namespace {

   constexpr uint8_t red_led_pin = 1U;
   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint8_t num_rc_out_channels = 7;

   constexpr uint16_t max_pulsewidth_us = 2100;
   constexpr uint16_t min_pulsewidth_us = 900;

   enum class direction : bool {decreasing = false, increasing = true};
   constexpr direction increasing = direction::increasing;
   constexpr direction decreasing = direction::decreasing;

   struct pwm_params_t{
      direction dir;             
      uint32_t last_update_ms;   // last update time for this servo
      uint16_t value_us;         // the pwm value in us
      uint32_t const delta_ms;   // chnge in pulsewidth each update
      uint16_t const incr_us;    // time between updates over which the incr occurs
   };

   pwm_params_t pwm_params[num_rc_out_channels] = {
       {increasing, 0, 1500, 10,  9}
      ,{decreasing, 0, 1500, 11, 10}
      ,{increasing, 0, 1500, 26, 20}
      ,{increasing, 0, 1500, 43, 50}
      ,{decreasing, 0, 1500, 90,  5}
      ,{increasing, 0, 1500,  8, 20}
      ,{decreasing, 0, 1500,  30, 20}
   };

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {

         for (uint8_t i=0; i < num_rc_out_channels; ++i){
            pwm_params_t& pwm = pwm_params[i];
            if ( (millis() - pwm.last_update_ms) > pwm.delta_ms ){
               pwm.last_update_ms = millis();
               if (pwm.dir == increasing){
                  if (pwm.value_us < max_pulsewidth_us){
                     hal.rcout->write(i,pwm.value_us);
                     pwm.value_us += pwm.incr_us;
                  }else{
                     pwm.value_us = max_pulsewidth_us;
                     hal.rcout->write(i,pwm.value_us);
                     pwm.dir = decreasing;
                  }
               }else{ // dir decreasing
                   if (pwm.value_us > min_pulsewidth_us){
                     hal.rcout->write(i,pwm.value_us);
                     pwm.value_us -= pwm.incr_us;
                  }else{
                     pwm.value_us = min_pulsewidth_us;
                     hal.rcout->write(i,pwm.value_us);
                     pwm.dir = increasing;
                  }
               }
            }
         }
         // this is to check we are running
         if (++m_count == 500){
            m_count = 0;
            hal.gpio->toggle(red_led_pin);
         }
      };

      void init()
      {
          for (uint8_t i =0; i < num_rc_out_channels; ++i){
             hal.rcout->enable_ch(i);
          }
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
      }
   private:
      uint32_t m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM RC Output test\n");
   test_task.init();
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM RC Output test",{-140,50});
}

void on_telemetry_transmitted()
{
}

namespace {

}
// called forever in apm_task
void loop() 
{
    hal.scheduler->delay(1);
    test_task.fun();
}

namespace {

   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_uartA = true;
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_rcout = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )



