
#pragma once

#include "i2c_periph.hpp"

#if defined QUAN_I2C_DEBUG

namespace Quan{

    void capture_i2c_sr1_flags(const char* name,uint32_t flags);
    void reset_i2c_sr1_flags_index();
    bool want_flags_index_reset();
    void set_want_flags_index_reset(bool b);
    void show_i2c_sr1_flags();
}

#endif

