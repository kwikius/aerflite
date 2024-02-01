
#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

Quan::QuanSemaphore::QuanSemaphore()
:  m_mutex_handle{nullptr}{}

void Quan::QuanSemaphore::init()
{
   if(m_mutex_handle == nullptr){
      m_mutex_handle = xSemaphoreCreateMutex(); 
      if(m_mutex_handle == nullptr){ 
         AP_HAL::panic("create semaphore failed");
      }
   }
}

bool Quan::QuanSemaphore::give()
{ 
   return xSemaphoreGive(m_mutex_handle) == pdTRUE;
}

bool Quan::QuanSemaphore::take(uint32_t timeout_ms) 
{ 
   return xSemaphoreTake(m_mutex_handle,timeout_ms) == pdTRUE;
}
bool Quan::QuanSemaphore::take_nonblocking() 
{ return this->take(0);}

