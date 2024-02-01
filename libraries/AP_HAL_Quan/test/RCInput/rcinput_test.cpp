
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <quan/min.hpp>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
   Test of the RC Input
   shows positions of all inputs in microseconds

   also blinks heartbeat LED at 2 Hz
*/

//const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint8_t max_input_channels = 16U;
   uint16_t stick_inputs[max_input_channels];
   uint8_t num_rc_in_channels = 0;
   uint8_t constexpr num_rc_out_channels = 6;

   struct test_task_t{

      test_task_t(): m_led_count{0}{}

      // call at 100 Hz
      void fun()
      {
         if ( hal.rcin->new_input()){
            num_rc_in_channels = quan::min(hal.rcin->num_channels(),max_input_channels);
            if ( num_rc_in_channels > 0){
               for ( uint8_t i = 0U; i < num_rc_in_channels; ++i){
                  stick_inputs[i] = hal.rcin->read(i);
                  hal.rcout->write(i,stick_inputs[i]);
                  hal.console->printf("rc in ch[%d] = %u usec\n",i,static_cast<unsigned int>(stick_inputs[i]));
               }
            }else{
               hal.console->printf("no input channels\n");
            }
         }

         if (++m_led_count == 50){
            m_led_count = 0;
            hal.gpio->toggle(red_led_pin);
         }
      };

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);

          for (auto & v: stick_inputs){ v = 0;}


          for (uint8_t i =0; i < num_rc_out_channels; ++i){
             hal.rcout->enable_ch(i);
          }
      }
   private:
      uint32_t m_led_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM RC Input test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
   
   test_task.init();
}

namespace {

   constexpr int32_t stick_y_pos = 0;
   constexpr int32_t stick_box_size = 100;
   constexpr int32_t box_x_sep = 20;

   // max pos = +-50 
   // max pulse diff = +=-500
   constexpr float stick_scale = 0.1;

   constexpr quan::uav::osd::pxp_type left_stick_cen{-(stick_box_size + box_x_sep)/2, stick_y_pos};
   constexpr quan::uav::osd::pxp_type right_stick_cen{(stick_box_size + box_x_sep)/2, stick_y_pos};
}

void quan::uav::osd::on_draw() 
{ 
   
   draw_box(
        left_stick_cen - pxp_type{stick_box_size,stick_box_size}/2
      , left_stick_cen + pxp_type{stick_box_size,stick_box_size}/2 
      , colour_type::white,false);

   uint32_t channels[4];
   for (uint8_t i = 0; i < 4; ++i){
     channels[i] = static_cast<int32_t>((stick_inputs[i] - 1500) * stick_scale);
   }
   draw_circle(10,left_stick_cen + pxp_type{channels[3],channels[2]},color_type::white);

   draw_box(
        right_stick_cen - pxp_type{stick_box_size,stick_box_size}/2
      , right_stick_cen + pxp_type{stick_box_size,stick_box_size}/2 
      , colour_type::white,false);

   draw_circle(10,right_stick_cen + pxp_type{channels[0],-channels[1]},color_type::white);
   
}

void on_telemetry_transmitted()
{
}

// called forever in apm_task
void loop() 
{
   hal.scheduler->delay(10);
   test_task.fun();
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_uartA = true;
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_rcin = true;
      flags.init_rcout = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )
