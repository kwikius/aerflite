
#ifndef __AP_HAL_QUAN_NAMESPACE_H__
#define __AP_HAL_QUAN_NAMESPACE_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

/* While not strictly required, names inside the Quan namespace are prefixed
 * with Quan for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Quan {
    class QuanStorage;
    class QuanGPIO;
    class QuanDigitalSource;
    class QuanRCInput;
    class QuanRCOutput;
    struct QuanSemaphore;
    class QuanScheduler;
    class QuanUtil;
    class QuanPrivateMember;
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // __AP_HAL_QUAN_NAMESPACE_H__

