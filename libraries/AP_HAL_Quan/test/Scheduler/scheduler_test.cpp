
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <task.h>
#include <cstring>

#include <quantracker/osd/osd.hpp>
#include <task.h>

// uncomment to start OSD as well

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

namespace {

   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;
   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;
}

// called once at startup of apm task
void setup() 
{
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);

   const char text[] = "Quan APM Scheduler test\n";

	hal.console->write((uint8_t const*)text,strlen(text));

}

void quan::uav::osd::on_draw() 
{ 
  draw_text("Quan APM Scheduler test",{-100,50});
}

void loop() 
{
   hal.gpio->write(test_pin,pin_on);
   hal.scheduler->delay_microseconds(300);
   hal.gpio->write(test_pin,pin_off);
   hal.scheduler->delay_microseconds(200);
}

AP_HAL_MAIN();


