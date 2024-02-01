// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_UserInteract_Stream.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <stm32f4xx.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#endif

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_InertialSensor ins;

static void display_offsets_and_scaling();
static void run_test();
static void run_calibration();

void setup(void)
{
    hal.gpio->pinMode(1,1); // pin1 output led
    hal.gpio->write(1,0);  // low
    hal.console->println("AP_InertialSensor startup...");
    ins.init(AP_InertialSensor::RATE_50HZ);
    // display initial values
    display_offsets_and_scaling();

    hal.console->println("Complete. Reading:");

}

void loop(void)
{
    int16_t user_input;

    hal.console->println();
    hal.console->println(
    "Menu:\r\n"
    "    c calibrate accelerometers\r\n"
    "    d) display offsets and scaling\r\n"
    "    l) level (capture offsets from level)\r\n"
    "    t) test\r\n"
    "    r) reboot");

    // wait for user input
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while( hal.console->available() ) {
        user_input = hal.console->read();

        if( user_input == 'c' || user_input == 'C' ) {
            run_calibration();
            display_offsets_and_scaling();
        }

        if( user_input == 'd' || user_input == 'D' ) {
            display_offsets_and_scaling();
        }

        if( user_input == 't' || user_input == 'T' ) {
            run_test();
        }

        if( user_input == 'r' || user_input == 'R' ) {
			hal.scheduler->reboot(false);
        }
    }
}

static void run_calibration()
{
    float roll_trim, pitch_trim;
    // clear off any other characters (like line feeds,etc)
    while( hal.console->available() ) {
        hal.console->read();
    }


    AP_InertialSensor_UserInteractStream interact(hal.console);
   ins.calibrate_accel(&interact, roll_trim, pitch_trim);
}

static void display_offsets_and_scaling()
{
    Vector3f accel_offsets = ins.get_accel_offsets();
    Vector3f accel_scale = ins.get_accel_scale();
    Vector3f gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf(
            "\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    static_cast<double>(accel_offsets.x),
                    static_cast<double>(accel_offsets.y),
                    static_cast<double>(accel_offsets.z)
    );
    hal.console->printf(
            "Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    static_cast<double>(accel_scale.x),
                    static_cast<double>(accel_scale.y),
                    static_cast<double>(accel_scale.z)
    );
    hal.console->printf(
            "Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                    static_cast<double>(gyro_offsets.x),
                    static_cast<double>(gyro_offsets.y),
                    static_cast<double>(gyro_offsets.z)
    );
}

static void run_test()
{
   uint8_t counter = 0;
   uint32_t last = AP_HAL::millis();
   // flush any user input
   while( hal.console->available() ) {
      hal.console->read();
   }
   // loop as long as user does not press a key
   while( !hal.console->available() ) {
      ins.update();
      uint32_t const now = AP_HAL::millis();
      uint32_t const dt = now - last;
      last = now;
      if (++counter == 50) {
         counter = 0;
         Vector3f accel = ins.get_accel();
         Vector3f gyro = ins.get_gyro();

         hal.console->printf("dt = %6lu ,acc[% 7.2f,% 7.2f,% 7.2f] , gyr[% 7.2f,% 7.2f,% 7.2f]\n", 
            dt,
            static_cast<double>(accel.x),static_cast<double>(accel.y), static_cast<double>(accel.z), 
            static_cast<double>(gyro.x),  static_cast<double>(gyro.y), static_cast<double>(gyro.z)
         );
      }
   }

   // clear user input
   while( hal.console->available() ) {
      hal.console->read();
   }
}

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM INS test",{-140,50});
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_uartA = true;
    //  flags.init_analogin = true;
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_spi = true;

      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )
#else
AP_HAL_MAIN();
#endif
