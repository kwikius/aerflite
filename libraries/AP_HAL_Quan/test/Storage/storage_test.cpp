
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_HAL_Quan/Storage.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL/utility/functor.h>

#include <quantracker/osd/osd.hpp>
#include <quan/min.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

/*
  test of basic storage function
*/

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

  // constexpr uint8_t red_led_pin = 1U;
   // Pin2 == PC14
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint32_t ee_addr = 1253;

   char constexpr text [] = "Simple Mixer \n"
"# anything global must be a constant ...\n"
"# type is deduced from the initialiser expression\n"
"# There are 3 types integer, float bool\n"
"# floats are differentiated by including a decimal point and fractional part\n"
"# bools are either true or false\n"
"# There is NO conversion between the different types\n"
"#  x = 1 ; // x is an integer\n"
"#  y = 1.0 ; // y is a float\n"
"#  z1 = x + x ; // OK z1 is an integer\n"
"#  z2 = x + y ; // Error : x and y are different types\n"
"#  b = true;   // b is a bool\n"
"#  c = b && x ;   // Error :  b and x are different types\n"
"#  d = b && !b ; // ok ( c is false)\n"
"#  w = x != 0  ; // w is bool \n"
"\n"
"pitch_gain = 500.0; # pitch gain surprisingly ( Just here to test comments)\n";

char buffer[100U];

   struct test_task_t{

      test_task_t(): m_count{false}{}

      void fun()
      { 
          uint32_t len = strlen(text) +1;
          if ( m_count == false){
             hal.storage->write_block(ee_addr,text,len);
             m_count = true;
          }else{
             hal.console->printf("-------------------------\n");

 /*
   The data is buffered in a queue and takes a while to be written to eeprom
   so, before reading it, check that it has been flushed
*/
             Quan::wait_for_eeprom_write_queue_flushed();
             
/*
      read the data to a small buffer a bit at a time
*/
             uint32_t len1 = len;
             uint32_t ee_addr1 = ee_addr;
             while (len1 > 0U){
                uint32_t bytes_to_read = quan::min(100U,len1);
                hal.storage->read_block(buffer,ee_addr1 ,bytes_to_read);
                // we also need to beware of overrunning the uart buffer
                while (hal.console->tx_pending() ){asm volatile ("nop":::);}
                for ( uint32_t i= 0; i < bytes_to_read; ++i){
                  hal.console->printf("%c",(((char const*)buffer)[i]));
                }
                ee_addr1 += bytes_to_read;
                len1 -= bytes_to_read;
             }
             hal.console->printf("\n\n-------------------------\n");
             m_count = false;
          }
      };

      void init()
      {
         // hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
         // hal.gpio->write(red_led_pin,pin_off);
          
      }
   private:
      bool m_count ;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM Storage test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
//   test_task.init();
//
   test_task.fun();
   test_task.fun();
}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM Storage test",{-140,50});
}

namespace {

   TickType_t prev_wake_time= 0; 
 

}
// called forever in apm_task
void loop() 
{
   vTaskDelayUntil(&prev_wake_time,500); 

   hal.gpio->toggle(test_pin);
   
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


