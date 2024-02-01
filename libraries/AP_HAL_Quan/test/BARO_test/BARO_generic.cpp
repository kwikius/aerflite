/*
  generic Baro driver test
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_Baro/AP_Baro.h>
#include <quantracker/osd/osd.hpp>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static AP_Baro barometer;

void setup()
{
   for ( uint8_t i = 1 ; i < 4; ++i){
       hal.gpio->pinMode(i,HAL_GPIO_OUTPUT);
       hal.gpio->write(i,0);
   }
   hal.console->printf("Barometer library test %lu\n", AP_HAL::millis());

   hal.scheduler->delay(1000);

   barometer.init();
   hal.console->printf("Calibrating baro\n");
   barometer.calibrate();
   hal.console->printf("Done setup\n");
}

namespace {

   float altitude  = 0.f;
};

void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-160,50};
    char buf[100];
    sprintf(buf,"alt = % 6.2f", static_cast<double>(altitude));
    draw_text(buf,pos);
}

namespace {
   TickType_t prev_wake_time= 0; 

   uint32_t lp_count = 0;
}

void loop()
{
   vTaskDelayUntil(&prev_wake_time,50); 

   barometer.update();
   altitude = barometer.get_altitude();
   if (!barometer.healthy()) {
      hal.console->println("baro not healthy");
      return;
   }
   if (++lp_count == 2 ){
      lp_count = 0;
      hal.console->printf("Pressure: % 10.3f ",static_cast<double>(barometer.get_pressure()));
      hal.console->printf(" Temperature: % 6.3f",static_cast<double>(barometer.get_temperature()));
      hal.console->printf(" Altitude: % 6.2f", static_cast<double>(altitude));
      hal.console->printf(" climb= % 5.2f\n",static_cast<double>(barometer.get_climb_rate()));
   }

}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif
