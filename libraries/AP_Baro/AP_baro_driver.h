/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_BACKEND_H__
#define __AP_BARO_BACKEND_H__

#include "AP_Baro.h"

class AP_baro_driver
{
protected:
    AP_baro_driver();
public:
    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update()const = 0;

    virtual ~AP_baro_driver(){}
};

#endif // __AP_BARO_BACKEND_H__
