
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {
   constexpr uint8_t heartbeat_led = 1U;
   constexpr uint8_t orange_led =  2U;
   constexpr uint8_t green_led = 3U;

   constexpr uint8_t led_off = 0U;
   constexpr uint8_t led_on = 1U;

   constexpr char text[] = "Quan APM GPIO test\n";
}

// called once at startup of apm task
void setup() 
{
   // test gpio
   for ( uint8_t i = 1; i < 4; ++i){
      hal.gpio->pinMode(i,HAL_GPIO_OUTPUT);
      hal.gpio->write(i,led_off);
   }

   hal.scheduler->delay(1000);
	hal.uartB->write((uint8_t const*)text,strlen(text));
   
}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   quan::uav::osd::draw_text(text,{-140,50}); 
}

namespace{
   TickType_t prev_wake_time= 0; 

   uint32_t red_count = 0;
   uint32_t green_count = 0;
   uint32_t orange_count = 0;
}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,1); 

   if ( ++red_count == 200 ){
      red_count = 0;
      hal.gpio->toggle(heartbeat_led);
   } 
   if (++orange_count == 330){
      orange_count = 0;
      hal.gpio->toggle(orange_led);
   }

   if ( ++green_count == 500){
      green_count = 0;
      hal.gpio->toggle(green_led);
   }
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartB = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif


