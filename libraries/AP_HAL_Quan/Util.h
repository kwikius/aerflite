
#ifndef __AP_HAL_QUAN_UTIL_H__
#define __AP_HAL_QUAN_UTIL_H__

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Quan_Namespace.h"

class Quan::QuanUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
};

#endif // __AP_HAL_QUAN_UTIL_H__
