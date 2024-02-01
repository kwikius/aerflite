
#ifndef __AP_HAL_QUAN_SEMAPHORE_H__
#define __AP_HAL_QUAN_SEMAPHORE_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include "FreeRTOS.h"
#include <semphr.h>

struct Quan::QuanSemaphore final : public AP_HAL::Semaphore {
public:
    QuanSemaphore() ;
    bool give();
    bool take(uint32_t timeout_ms) ; 
    bool take_nonblocking();
    void init();
private:
    SemaphoreHandle_t m_mutex_handle;
};

#endif // __AP_HAL_QUAN_SEMAPHORE_H__
