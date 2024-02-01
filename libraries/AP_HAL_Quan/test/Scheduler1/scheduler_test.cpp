
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>

/*
   Scheduler test
*/

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   struct test_task_t{

      test_task_t(): m_count{0}{}

      void fun()
      {
          if (++m_count == 500){
            hal.gpio->toggle(red_led_pin);
            m_count = 0;
          }
      };

      void init()
      {
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
   const char text[] = "Quan APM Scheduler1 test\n";
 	hal.console->write((uint8_t const*)text,strlen(text));
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Sched Timer test",{-140,50});
}

void on_telemetry_transmitted()
{
}


namespace {
   uint64_t next_event = 10U;
}
// called forever in apm_task
void loop() 
{
   int32_t const now = AP_HAL::millis();
   if ( next_event <= now ){
      hal.gpio->toggle(test_pin);
      next_event = now + 10U;
   }
}

AP_HAL_MAIN();
