
#ifndef __AP_HAL_QUAN_SCHEDULER_H__
#define __AP_HAL_QUAN_SCHEDULER_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>

class Quan::QuanScheduler final : public AP_HAL::Scheduler {
public:
    QuanScheduler();
    void     init(void* machtnichts);
    void     delay(uint16_t ms);
    void     delay_microseconds(uint16_t us);
    void     register_delay_callback(AP_HAL::Proc,uint16_t min_time_ms);
    void     register_timer_process(AP_HAL::MemberProc);
    void     register_io_process(AP_HAL::MemberProc);
    void     suspend_timer_procs();
    void     resume_timer_procs();
    bool     in_timerprocess();
    void     register_timer_failsafe(AP_HAL::Proc, uint32_t period_us);
    bool     system_initializing();
    void     system_initialized();

  //  void     panic(const char* errormsg,...);
    void     reboot(bool hold_in_bootloader);
};

#endif // __AP_HAL_QUAN_SCHEDULER_H__
