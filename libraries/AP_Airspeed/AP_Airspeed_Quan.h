#ifndef AP_AIRSPEED_QUAN_H_INCLUDED
#define AP_AIRSPEED_QUAN_H_INCLUDED

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Quan : public AP_Airspeed_Backend {
public:
    // constructor
    AP_Airspeed_Quan() {}

    void update() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure)const;

    bool get_temperature(float &temperature)const { return false;}
};


#endif

#endif // AP_AIRSPEED_QUAN_H_INCLUDED
