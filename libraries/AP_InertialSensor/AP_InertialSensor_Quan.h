/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef AP_INERTIAL_SENSOR_QUAN_H_INCLUDED
#define AP_INERTIAL_SENSOR_QUAN_H_INCLUDED

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_Quan : public AP_InertialSensor_Backend{
public:
    AP_InertialSensor_Quan(AP_InertialSensor &imu);
    bool update();
    static AP_InertialSensor_Backend *detect(AP_InertialSensor &imu);

private:

   uint8_t m_accel_id;  //
   uint8_t m_gyro_id;

};

#endif // CONFIG_HAL_BOARD
#endif // AP_INERTIAL_SENSOR_QUAN_H_INCLUDED
