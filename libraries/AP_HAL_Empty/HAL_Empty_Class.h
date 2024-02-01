#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_HAL_Empty_Namespace.h"
#include "PrivateMember.h"

class HAL_Empty : public AP_HAL::HAL {
public:
    HAL_Empty();
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    void run(void*) const override;
#else
    void run(int argc, char * const argv[], Callbacks* callbacks) const override;
#endif
private:
    Empty::EmptyPrivateMember *_member;
};
