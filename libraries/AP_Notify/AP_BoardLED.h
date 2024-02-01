/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __AP_HAL_BOARDLED_H__
#define __AP_HAL_BOARDLED_H__

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "NotifyDevice.h"

#define HIGH 1
#define LOW 0

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
// heartbeat led PB12 pin 33
 # define HAL_GPIO_A_LED_PIN        1
//  PC14
 # define HAL_GPIO_B_LED_PIN        2
//  PC15
 # define HAL_GPIO_C_LED_PIN        3
 # define HAL_GPIO_LED_ON           HIGH
 # define HAL_GPIO_LED_OFF          LOW
#else
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
#error "Unknown board type in AP_Notify"
#endif
#endif

class AP_BoardLED: public NotifyDevice
{
public:
    // initialise the LED driver
    bool init(void);

    // should be called at 50Hz
    void update(void);

private:
    // counter incremented at 50Hz
    uint8_t _counter;
};

#endif // __AP_HAL_BOARDLED_H__
