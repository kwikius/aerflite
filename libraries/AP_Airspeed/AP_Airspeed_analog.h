/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_AIRSPEED_ANALOG_H__
#define __AP_AIRSPEED_ANALOG_H__

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <AP_Param/AP_Param.h>
#include <AP_HAL/AnalogIn.h>
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_Analog final : public  AP_Airspeed_Backend
{
public:
    AP_Airspeed_Analog() : 
        _source(NULL)
    {}

    void update();
    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure)const;

    // temperature not available via analog backend
    bool get_temperature(float &temperature) const { return false; }
     
    void set_source(AP_HAL::AnalogSource * source);
private:
    AP_HAL::AnalogSource *_source;
};

#endif

#endif // __AP_AIRSPEED_ANALOG_H__
