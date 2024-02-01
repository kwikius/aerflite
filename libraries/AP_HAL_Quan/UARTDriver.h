
#ifndef __AP_HAL_QUAN_UARTDRIVER_H__
#define __AP_HAL_QUAN_UARTDRIVER_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>

namespace Quan{
   template <uint32_t I>
   AP_HAL::UARTDriver * get_serial_port();

  // void set_console_irq_mode(bool b);

#if ! defined QUAN_AERFLITE_BOARD
   // val true for inverting false for non-inverting
   void set_usart3_tx_inverting( bool val);
#endif
}

#endif
