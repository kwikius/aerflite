/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <quantracker/osd/osd.hpp>
#include <quan/min.hpp>
#include <quan/max.hpp>
#include <AP_OSD/fonts.hpp>
#include <cstdio>

#if !defined QUAN_MIXER_TRANQUILITY
#error only for tranquility mixer
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

  AP_Airspeed airspeed(50U);

  AP_HAL::UARTDriver * uart = nullptr;
}

void setup() {
    
    uart = hal.uartA;
    uart->begin(115200);

    uart->println("SDP3x library test");

    hal.scheduler->delay(1000);

    airspeed.init();

}

namespace {
  uint32_t count = 0;
}

void quan::uav::osd::on_draw() 
{ 
    draw_text("SDP3x lib Test",{-140,100},Quan::FontID::MWOSD);
}

void on_telemetry_transmitted(){}

void loop()
{

   hal.scheduler->delay(20);

   airspeed.update();

   if (++count == 10U){
      count = 0U;
      float temp;
      airspeed.get_temperature(temp);
      hal.console->printf("airspeed = %f, temp = %f\n"
         ,static_cast<double>(airspeed.get_airspeed())
         ,static_cast<double>(temp)
      );
   }
   
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

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

