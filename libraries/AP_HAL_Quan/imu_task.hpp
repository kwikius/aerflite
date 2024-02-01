#ifndef APM_QUANTRACKER_IMU_TASK_HPP_INCLUDED
#define APM_QUANTRACKER_IMU_TASK_HPP_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <AP_InertialSensor/AP_InertialSensor.h>
#include "FreeRTOS.h"

#include <AP_Math/vector3_volatile.h>

namespace Quan{

   bool wait_for_imu_sample(uint32_t wait_usecs);
   bool update_ins(Vector3f & accel,Vector3f & gyro);

   namespace detail{
      void inertial_sensor_setup(uint16_t sample_rate_Hz, uint8_t acc_cutoff_Hz, uint8_t gyro_cutoff_Hz);
   }
   
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // APM_QUANTRACKER_IMU_TASK_HPP_INCLUDED
