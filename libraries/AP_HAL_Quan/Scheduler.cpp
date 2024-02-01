

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include "Scheduler.h"
#include <cstdio>
#include <task.h>
#include <semphr.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/max.hpp>
#include "i2c_task.hpp"
#include <AP_HAL/system.h>

using namespace Quan;

extern const AP_HAL::HAL& hal;

namespace {

   // TODO replace this with tim5 which is 32 bit
   // 32 bit usec timer will overflow once every 71 mins
   // can then use 4 channels for usec delays possibly on different threads
   // e.g sleep the thread and then wake from interrupt
   typedef quan::stm32::tim13 usec_timer;
   void setup_usec_timer()
   {
      quan::stm32::module_enable<usec_timer>();
      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<usec_timer>();
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");
      usec_timer::get()->psc = psc;
      usec_timer::get()->cnt = 0;
      usec_timer::get()->arr = 0xffff;
      usec_timer::get()->sr = 0;
      usec_timer::get()->dier.setbit<0>(); //(UIE)  

      NVIC_SetPriority(TIM8_UP_TIM13_IRQn,14);
      NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
   }

   void start_usec_timer()
   {
      usec_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }
   
   // overflow after  approx 9 years
   volatile uint32_t timer_micros_ovflo_count = 0U;
 
} // namespace


extern "C" void TIM8_UP_TIM13_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

extern "C" void TIM8_UP_TIM13_IRQHandler()
{
   if ( usec_timer::get()->sr & (1 << 0) ) {// UIF
      usec_timer::get()->sr = 0;
      ++ timer_micros_ovflo_count;
   }
}

QuanScheduler::QuanScheduler()
{}
//void create_adc_task();
// called in HAL_Quan::init( int argc, char * const * argv)
// after console and GPIO inited
void QuanScheduler::init(void* )
{
   setup_usec_timer();

   start_usec_timer();
}

namespace{
   AP_HAL::Proc m_delay_callback = nullptr;
   uint16_t m_delay_callback_min_delay_ms = 0;
}

void QuanScheduler::delay(uint16_t delay_length_ms)
{
   uint64_t const end_of_delay_ms  = AP_HAL::millis64() + delay_length_ms;
   for (;;){
      uint64_t const time_now_ms = AP_HAL::millis64();
      if ( m_delay_callback && (( time_now_ms + m_delay_callback_min_delay_ms) < end_of_delay_ms)){
         m_delay_callback();
         if ( AP_HAL::millis64() < end_of_delay_ms){
            vTaskDelay(1);
         }else{
            break; // end of delay
         }
      }else{
         if ( time_now_ms < end_of_delay_ms){
            vTaskDelay(end_of_delay_ms - time_now_ms );
         }
         break; // end of delay
      }
   }
}


uint64_t AP_HAL::micros64() 
{
   vTaskSuspendAll();
   uint32_t const hi1 = timer_micros_ovflo_count;
   uint16_t const lo1 = usec_timer::get()->cnt;
   uint32_t const hi2 = timer_micros_ovflo_count;
   if ( hi2 == hi1){
      xTaskResumeAll();
      return (static_cast<uint64_t>(hi1) << 16U) | lo1; 
   }else{
      uint16_t const lo2 = usec_timer::get()->cnt;
      xTaskResumeAll();
      return (static_cast<uint64_t>(hi2) << 16U) | lo2; 
   }
}

uint64_t AP_HAL::millis64() 
{
   return micros64() / 1000ULL;
}

uint32_t AP_HAL::millis() {
    return millis64();
}

uint32_t AP_HAL::micros() {
    return micros64();
}

 // delay longer than 1 ms will in fact actively yield to other tasks
void QuanScheduler::delay_microseconds(uint16_t us)
{
   uint64_t const end_of_delay_us = AP_HAL::micros64() + us;
   uint64_t const delay_ms = us / 1000;
   // yields
   if ( delay_ms > 0){
      delay(delay_ms);
   }
   // not going critical here. yield away!
   while (AP_HAL::micros64() < end_of_delay_us){ 
      asm volatile ("nop":::);
   }
}

/*
a function to do useful stuff during the delay function
 maybe be null
 called during Plane::init_ardupilot fun
 to run Mavlink output fun
 TODO .. Remove this 
*/
void QuanScheduler::register_delay_callback(AP_HAL::Proc pfn,
            uint16_t min_time_ms)
{
   m_delay_callback = pfn;
   m_delay_callback_min_delay_ms = min_time_ms;
}

namespace{
     AP_HAL::MemberProc new_scheduler_timer_task_proc_in = nullptr;
}
void QuanScheduler::register_timer_process(AP_HAL::MemberProc mp)
{
    hal.console->printf("QuanScheduler::register_timer_process called\n");
//   if ( (mp == nullptr) == false){
//      new_scheduler_timer_task_proc_in = mp;
//      if (xQueueSendToBack(scheduler_timer_task_message_queue,&new_scheduler_timer_task_proc_in,2) == errQUEUE_FULL){
//          hal.console->printf("failed to add new scheduler_timer_task proc\n");
//      }
//   }
}

//"not supported on AVR" so wont bother yet
void QuanScheduler::register_io_process(AP_HAL::MemberProc k)
{
   hal.console->printf("QuanScheduler::register_io_process called\n");

}

// TODO register a function to call on failsafe ( eg. WDT timeout)
// and sort failsafe watchdog etc
// do this asynchronously
void QuanScheduler::register_timer_failsafe(AP_HAL::Proc, uint32_t period_us)
{}

// request 
void QuanScheduler::suspend_timer_procs()
{
//   request_timer_procs_suspended = true;
//   while (!timer_procs_suspended){
//      delay(1);
//   }
}

void QuanScheduler::resume_timer_procs()
{
//    request_timer_procs_suspended = false;
//    while (timer_procs_suspended){
//      delay(1);
//   }
}

// call from interrupt?
bool QuanScheduler::in_timerprocess() 
{
    return false; // m_in_timer_process; 
}

namespace {
  bool m_system_initialised = false;
}

bool QuanScheduler::system_initializing() 
{
    return ! m_system_initialised;
}

void QuanScheduler::system_initialized()
{
   m_system_initialised = true;
}

void AP_HAL::panic(const char *errormsg,...) 
{
    vTaskSuspendAll();
    va_list args;
    va_start(args, errormsg);
    char buf[256];
    int n = vsprintf(buf, errormsg, args);
    va_end(args);
    while (hal.console->tx_pending()) {asm volatile ("nop":::);}
    hal.console->write((uint8_t const *)buf,n);
    hal.console->printf("\n");
    for(;;){ ;;}
}

// TODO wdt
// no bootloader
void QuanScheduler::reboot(bool hold_in_bootloader) 
{
    NVIC_SystemReset();
   // for(;;);
}
