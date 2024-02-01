

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// called once in apm_task before loop
void setup() 
{
   // test gpio
   hal.gpio->pinMode(1,HAL_GPIO_OUTPUT);
   hal.gpio->write(1,1);

   const char text[] = "Hi from Apm Quan Scheduler\n";
	hal.console->write((uint8_t const*)text,strlen(text));
}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   quan::uav::osd::draw_text("Quan APM UartDriver test",{-100,0}); 
}
namespace {
   TickType_t prev_wake_time= 0; 
}

// called forever
void loop() 
{
     vTaskDelayUntil(&prev_wake_time,20); 
     asm volatile("nop":::); 
}

AP_HAL_MAIN();

