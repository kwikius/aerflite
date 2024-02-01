
#include <AP_HAL/AP_HAL.h>
#include <quan_matters/test/ap_test.hpp>
//this allows access to private functions
// for easier testing
#define QUAN_PUBLIC_PRIVATE_MEMBERS
#include <RC_Channel/RC_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>

// do something on osd to check its running ok
void quan::uav::osd::on_draw() 
{ 
    pxp_type pos{-140,50};
    draw_text("Quan APM RC Channel Test",pos);
}
void on_telemetry_transmitted(){}

#endif

RC_Channel chan{0,0,RC_Channel::channel_type::angle,1500};

void test1 ()
{
   chan.set_joystick_input_usec(1000);
   QUAN_CHECK(chan.get_joystick_in_usec() == 1000);
   QUAN_CHECK(chan.norm_input() == -1.f);
   hal.console->printf("norm_input = %f\n",static_cast<double>(chan.norm_input()));
   QUAN_CHECK(chan.get_control_in() == -4500);

   QUAN_CHECK(chan.get_output_usec() == 1500);
   QUAN_CHECK(chan.get_temp_out() == 0);

   chan.set_reversed(true);
   chan.set_joystick_input_usec(1000);

   QUAN_CHECK(chan.get_joystick_in_usec() == 2000);
   QUAN_CHECK(chan.norm_input() == 1.f);
   hal.console->printf("norm_input = %f\n",static_cast<double>(chan.norm_input()));
   QUAN_CHECK(chan.get_control_in() == 4500);

   QUAN_CHECK(chan.get_output_usec() == 1500);
   QUAN_CHECK(chan.get_temp_out() == 0);
}

void setup()
{
    
    hal.console->printf("ArduPilot RC Channel library test");

    test1();

    EPILOGUE;

}

void loop(void)
{
    
}

int errors=0;

AP_HAL_MAIN();
