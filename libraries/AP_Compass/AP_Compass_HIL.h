/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_HIL_H
#define AP_Compass_HIL_H

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "Compass.h"

#define HIL_NUM_COMPASSES 2

class AP_Compass_HIL final: public AP_Compass_Backend{
public:
    AP_Compass_HIL();
    AP_Compass_HIL*   connect(Compass & compass);
    void              update() override;
private:
    uint8_t     _compass_instance[HIL_NUM_COMPASSES];
};

#endif

#endif
