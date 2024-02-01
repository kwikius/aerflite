
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
#include <assert.h>
#endif

#include "HAL.h"

namespace AP_HAL {

HAL::FunCallbacks::FunCallbacks(void (*setup_fun)(void), void (*loop_fun)(void))
    : _setup(setup_fun)
    , _loop(loop_fun)
{
#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
    assert(setup_fun);
    assert(loop_fun);
#else
  // flash error led if fail
#endif
}

}
