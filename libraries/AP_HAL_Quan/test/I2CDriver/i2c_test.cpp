
#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD
#error the quan aerflite board doesnt expose the i2c driver
#endif
#endif

#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Param/AP_Param.h>
#include <StorageManager/StorageManager.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL/utility/functor.h>
#include <AP_Math/vector3.h>

#include <quantracker/osd/osd.hpp>
#include <task.h>
#include <cstring>
#include <stm32f4xx.h>

#pragma message  "need to disable the i2c_task in quan scheduler.cpp for this to work atm"
const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   // connect a HMC5883 mag and MS5611 baro to test

   constexpr uint8_t red_led_pin = 1U;
 
   constexpr uint8_t test_pin = 2U;

   constexpr uint8_t pin_off = 0U;
   constexpr uint8_t pin_on = 1U;

   constexpr uint8_t mag_addr = 0x1E;
   constexpr uint8_t mag_modereg =  0x02;
   uint8_t mag_msb_x = 0x03;
   constexpr uint8_t mag_single_measurement = 0x01;
   uint8_t values [6] = {0,0,0,0,0,0};
   constexpr uint8_t baro_addr = 0x77;
   constexpr uint8_t ConfigRegA = 0x00;
   static uint8_t baro_start_pressure_conv = 0x48;
   static uint8_t baro_read_adc = 0x00;

   bool alt_method = false;
   
   int16_t convert_to_int16(uint8_t * d)
   {
      union{
         uint8_t in[2] ;
         int16_t out;
      }u;
      u.in[0] = d[1];
      u.in[1] = d[0];
      return u.out;
   }

   void copy_new_values(Vector3<int> & result_out)
   {
      result_out.x = convert_to_int16(values);
      result_out.y = convert_to_int16(values + 4);
      result_out.z = convert_to_int16(values + 2);
   }

   struct test_task_t{

      enum baro_state_t{ idle, conv_requested};
   
      test_task_t(): m_count{0}, m_baro_count{0}, m_baro_state{idle}{}

      void fun()
      {
//         if (++m_count == 500){
//            m_count = 0;

            hal.gpio->toggle(red_led_pin);

            auto * sem = hal.i2c->get_semaphore();
            
            if ( sem && sem->take_nonblocking() ){

//               if (  hal.i2c->writeRegister(mag_addr,mag_modereg,mag_single_measurement) != 0){
//                  sem->give();
//                  return; // should report on error
//               }
//
//               if (!alt_method){
//                  if ( hal.i2c->write(mag_addr,1,&mag_msb_x) !=0 ){
//                     sem->give();
//                     return;
//                  }
//                  if ( hal.i2c->read(mag_addr,6,values) != 0){
//                     sem->give();
//                     return;
//                  }
//               }else{
//                  if ( hal.i2c->readRegisters(mag_addr,mag_msb_x,6,values) != 0){
//                     sem->give();
//                     return;
//                  }
//               }
//               alt_method = ! alt_method;
               //-------------------
               if (hal.i2c->writeRegister(mag_addr,ConfigRegA,(0x03 << 5) | ( 0x06 << 2) | 0x10 ) != 0 ){
                  hal.console->printf("writeRegister failed\n");
                  sem->give();
                  return;
               }
               uint8_t test_ret;
               if ( hal.i2c->readRegister(mag_addr,ConfigRegA,&test_ret) != 0){
                  sem->give();
                  return;
               }
               if ( test_ret !=  ((0x03 << 5) | ( 0x06 << 2) | 0x10 )  ){
                  hal.console->printf("readRegister failed\n");
                  sem->give();
                  return;
               }
              // hal.console->printf("configreg A is %u\n", static_cast<unsigned int>(test_ret));
               hal.console->printf("read mag success\n");
//               Vector3<int> result;
//               copy_new_values(result);
//               hal.console->printf("mag result = [%d, %d,%d]\n",result.x,result.y,result.z);
               // baro test --------------------
               if ( m_baro_state == idle){
                  // request a conversion
                  if ( hal.i2c->write(baro_addr,1,&baro_start_pressure_conv) != 0){
                      sem->give();
                     return;
                  }
                  m_baro_state = conv_requested;
                  m_baro_count = 0;
               }else{ // baro not idle doing a conv
                  // after 10 ms can read
                  if ( ++m_baro_count == 10){
                    // uint8_t arr [3];
                     if (  hal.i2c->readRegisters(baro_addr,baro_read_adc,3,values) != 0){
                        sem->give();
                        return;
                     }
                     hal.console->printf("read baro success\n");
                     uint32_t value = (values[0] << 16) | ( values[1] << 8) | ( values[2]);
                     hal.console->printf("baro result = %u\n",static_cast<unsigned int>(value));
                     m_baro_state = idle;
                  }
               }
               sem->give();
            }else{
               hal.console->printf("couldnt get semaphore\n");
            }
        // }
      }

      void init()
      {
          hal.gpio->pinMode(red_led_pin,HAL_GPIO_OUTPUT);
          hal.gpio->write(red_led_pin,pin_off);
         // hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&test_task_t::fun, void));
      }
   private:
      uint32_t m_count ;
      uint32_t m_baro_count;
      baro_state_t m_baro_state;
   } test_task;

}

// called once after init of hal before startup of apm task
void setup() 
{
 	hal.console->printf("Quan APM I2C test\n");
   hal.gpio->pinMode(test_pin,HAL_GPIO_OUTPUT);
   hal.gpio->write(test_pin,pin_off);
}

void on_telemetry_transmitted()
{
}

void quan::uav::osd::on_draw() 
{ 
   draw_text("Quan APM I2C test",{-140,50});
}

namespace {
   constexpr TickType_t interval = 500U;
   TickType_t wakeup_time;
}
// called forever in apm_task
// so make the loop so that it yields

void loop() 
{
  vTaskDelayUntil(&wakeup_time,interval);
  test_task.fun();
 
}

#if 1
AP_HAL_MAIN();
#else
void create_apm_task();
void create_timer_task();

extern "C" {
   int main (void) 
   {
      osd_setup(); 
      create_draw_task(); 
      create_apm_task(); 
      vTaskStartScheduler (); 
   }
}

namespace { 
   char dummy_param = 0; 
   TaskHandle_t task_handle = NULL; 
   void apm_task(void * params) 
   { 
      hal.init(0, NULL);
      setup();
      hal.scheduler->system_initialized(); 
      test_task.init();
      for(;;){ 
         loop(); 
      } 
   } 
} 

void create_apm_task() 
{ 
  xTaskCreate( 
      apm_task,"apm task", 
      5000, 
      &dummy_param, 
      tskIDLE_PRIORITY + 1, 
      &task_handle 
  ); 
}
#endif

