
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD
#if defined QUAN_I2C_DEBUG

#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/i2c_task.hpp>

#include "i2c_debug.hpp"

using AP_HAL::millis;
extern const AP_HAL::HAL& hal;

namespace {

   struct irq_info{
      irq_info(){}
      irq_info(const char* name,uint32_t flags):m_name{name},m_flags{flags}{}
      const char * m_name;
      uint32_t     m_flags;
   };

   irq_info infos[100];
   uint32_t flags_idx = 0;

   uint32_t incr_flags()
   { 
      uint32_t const result = flags_idx ;
      flags_idx = (flags_idx + 1) % 100;
      return result;
   }

   void show_sr1_flags( uint32_t flags){

      if (flags & (1<<0)){
          hal.console->printf("sb |");
      }

      if(flags & (1<<1) ){
          hal.console->printf("addr |");
      }
      
      if(flags & (1<<2) ){
          hal.console->printf("btf |");
      }

      if(flags & (1<<4) ){
          hal.console->printf("stopf |");
      }

      if(flags & (1<<6) ){
          hal.console->printf("rxne |");
      }

      if(flags & (1<<7) ){
          hal.console->printf("txe |");
      }

      if(flags & (1<<8) ){
          hal.console->printf("berr |");
      }

      if(flags & (1<<9) ){
          hal.console->printf("arlo |");
      }

      if(flags & (1 << 10) ){
          hal.console->printf("af |");
      }
      if(flags & (1 << 10) ){
          hal.console->printf("ovr |");
      }

      hal.console->printf("\n");

   }

}

void Quan::capture_i2c_sr1_flags(const char* name,uint32_t flags)
{
   infos[incr_flags()] = {name,flags};
}

void Quan::show_i2c_sr1_flags()
{

   for (uint32_t idx = 0; idx < flags_idx; ++idx){
      while (hal.console->tx_pending() ){asm volatile ("nop":::);}
      hal.console->printf("flags[%s] = %lX\n",infos[idx].m_name,infos[idx].m_flags);
      show_sr1_flags(infos[idx].m_flags);
   }

   hal.console->printf("----------------\n");
}

namespace{
   bool m_want_flags_index_reset = false;
}

    bool Quan::want_flags_index_reset() {return m_want_flags_index_reset;}
    void Quan::set_want_flags_index_reset(bool b){m_want_flags_index_reset = b;}

void Quan::reset_i2c_sr1_flags_index()
{
   flags_idx = 0;
}

#endif // #if defined QUAN_I2C_DEBUG
#endif // #if defined QUAN_AERFLITE_BOARD
#endif // AP_HAL_QUAN
