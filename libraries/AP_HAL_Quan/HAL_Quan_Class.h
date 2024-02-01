
#ifndef __AP_HAL_QUAN_CLASS_H__
#define __AP_HAL_QUAN_CLASS_H__

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_HAL_Quan_Namespace.h"

class HAL_Quan final : public AP_HAL::HAL {
public:
    HAL_Quan();

   void run(void * params) const;
   // flags to start main app
   static constexpr uint32_t start_main = 0b111111111111;
   union start_flags{
      struct{
         bool  init_gpio      : 1;
         bool  init_uartA     : 1;
         bool  init_uartB     : 1;
         bool  init_uartC     : 1;
         bool  init_uartD     : 1;
         bool  init_spi       : 1;
         bool  init_rcin      : 1;
         bool  init_rcout     : 1;
         bool  init_i2c       : 1;
         bool  init_analogin  : 1;
         bool  init_scheduler : 1;
         bool  init_util      : 1;
         uint32_t             : 20; 
      };
      uint32_t value;
      start_flags(uint32_t value_in): value{value_in}{}
     
   };

};

#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // __AP_HAL_QUAN_CLASS_H__

