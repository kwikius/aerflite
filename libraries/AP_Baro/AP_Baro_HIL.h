/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
  dummy backend for HIL (and SITL). This doesn't actually need to do
  any work, as setHIL() is in the frontend
 */

#ifndef __AP_BARO_HIL_H__
#define __AP_BARO_HIL_H__

#include "AP_baro_driver.h"

class AP_Baro_HIL final : public AP_baro_driver
{
public:
    AP_Baro_HIL();
    void update()const override;
    AP_baro_driver* connect(AP_Baro & baro);
private:
    uint8_t m_instance;
    AP_Baro * m_baro;
};

#endif //  __AP_BARO_HIL_H__
