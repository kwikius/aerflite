
// Test for AP_GPS_AUTO

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

#include <AP_GPS/AP_GPS.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>

#include <AP_Notify/AP_BoardLED.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include <quantracker/osd/osd.hpp>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// create board led object
AP_BoardLED board_led;

// This example uses GPS system. Create it.
AP_GPS gps;

// Serial manager is needed for UART comunications
AP_SerialManager serial_manager;

void setup()
{
    hal.console->println("GPS AUTO library test");

    // Initialise the leds
    board_led.init();

    // Initialize the UART for GPS system
    serial_manager.init();
    gps.init(NULL, serial_manager);
}

namespace {

    AP_GPS::GPS_Status df_gps_status = AP_GPS::NO_GPS;

    uint8_t led_update_count = 0;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
/*
  NO_GPS = 0,             ///< No GPS connected/detected
        NO_FIX = 1,             ///< Receiving valid GPS messages but no lock
        GPS_OK_FIX_2D = 2,      ///< Receiving valid messages and 2D lock
        GPS_OK_FIX_3D = 3,      ///< Receiving valid messages and 3D lock
        GPS_OK_FIX_3D_DGPS = 4, ///< Receiving valid messages and 3D lock with differential improvements
        GPS_OK_FIX_3D_RTK = 5,  ///<
*/
   pxp_type pos{-140,50};
   switch ( df_gps_status){
       case AP_GPS::NO_GPS:
         draw_text("No GPS",pos);
         break;
       case AP_GPS::NO_FIX:
         draw_text("No Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_2D:
         draw_text("2D Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D:
         draw_text("3D Fix",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D_DGPS:
         draw_text("3D Fix dgps",pos);
         break;
       case AP_GPS::GPS_OK_FIX_3D_RTK:
         draw_text("3D Fix RTK",pos);
         break;
       default:
         draw_text("GPS state out of range",pos);
         break;
   }
   
}
#endif

void loop()
{
    static uint32_t last_msg_ms;

    // Update GPS state based on possible bytes received from the module.
    gps.update();

    // If new GPS data is received, output it's contents to the console
    // Here we rely on the time of the message in GPS class and the time of last message
    // saved in static variable last_msg_ms. When new message is received, the time
    // in GPS class will be updated.
    if (last_msg_ms != gps.last_message_time_ms()) {
        // Reset the time of message
        last_msg_ms = gps.last_message_time_ms();

        // Acquire location
        const Location &loc = gps.location();

        // Print the contents of message
        hal.console->print("Lat: ");
        print_latlon(hal.console, loc.lat);
        hal.console->print(" Lon: ");
        print_latlon(hal.console, loc.lng);
        hal.console->printf(" Alt: %.2fm GSP: %.2fm/s CoG: %d SAT: %d TIM: %u/%lu STATUS: %u\n",
                            static_cast<double>(loc.alt * 0.01),
                            static_cast<double>(gps.ground_speed()),
                            (int)gps.ground_course_cd() / 100,
                            gps.num_sats(),
                            gps.time_week(),
                            (unsigned long)gps.time_week_ms(),
                            gps.status());

        df_gps_status = gps.status();
       
    }
    if ( ++led_update_count ==2){
       led_update_count = 0;
       board_led.update();
    }
    // Delay for 10 mS will give us 100 Hz invocation rate
    hal.scheduler->delay(10);
}

// Register above functions in HAL board level
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
