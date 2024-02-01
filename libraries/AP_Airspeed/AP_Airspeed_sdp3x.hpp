#ifndef AERFPILOT_LIBRARIES_AIRSPEED_AP_AIRSPEED_SDP3X_HPP_INCLUDED
#define AERFPILOT_LIBRARIES_AIRSPEED_AP_AIRSPEED_SDP3X_HPP_INCLUDED

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"

#include <AP_HAL_Quan/i2c_task.hpp>

#include <quan/pressure.hpp>
#include <quan/temperature.hpp>

class AP_Airspeed_sdp3x : public AP_Airspeed_Backend {
public:
    // constructor
    AP_Airspeed_sdp3x(): m_hQueue{nullptr} {}

    void connect(QueueHandle_t handle) { m_hQueue = handle;}

    void update() override;

    // return the current differential_pressure in Pascal
    bool get_differential_pressure(float &pressure)const { pressure = m_diff_pressure.numeric_value(); return true;}

    // return the current temperature in degrees C
    bool get_temperature(float &temperature)const {temperature = m_temperature.numeric_value() - 273.15f; return true;}

    private:

     quan::pressure::Pa m_diff_pressure;
     quan::temperature::K m_temperature;

     QueueHandle_t m_hQueue;
};


#endif

#endif // AERFPILOT_LIBRARIES_AIRSPEED_AP_AIRSPEED_SDP3X_HPP_INCLUDED
