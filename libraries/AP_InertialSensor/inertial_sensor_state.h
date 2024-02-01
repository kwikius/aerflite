#ifndef AERFPILOT_LIBRARIES_AP_INERTIAL_SENSOR_INERTIAL_SENSOR_STATE_H_INCLUDED
#define AERFPILOT_LIBRARIES_AP_INERTIAL_SENSOR_INERTIAL_SENSOR_STATE_H_INCLUDED

#include <stdint.h>
#include <AP_Math/AP_Math.h>
#include <Filter/LowPassFilter.h>
#include <Filter/LowPassFilter2p.h>

struct accel_state_t{
   accel_state_t():
      clip_count(0),
      error_count(0),
      delta_velocity_dt(0),
      delta_velocity_acc_dt(0),
      max_abs_offsets(3.5),
      raw_sample_rate(0),
      new_data(false),
      healthy(false),
      delta_velocity_valid(false)
   {}

   void offset_and_scale(Vector3f &accel) const
   {
      Vector3f const v0 = accel - offset;
      Vector3f const & sc = scale.get() ;
      accel = Vector3f(v0.x * sc.x, v0.y * sc.y, v0.z * sc.z) ;
   }

   Vector3f current_value; // final calculated value of last iteration
   Vector3f filtered_value;
   LowPassFilter2pVector3f filter;

   Vector3f delta_velocity;
   Vector3f delta_velocity_acc;     // delta velocity accumulator

   AP_Vector3f scale;
   AP_Vector3f offset;

   uint32_t clip_count;
   uint32_t error_count;
   float delta_velocity_dt;
   float delta_velocity_acc_dt;  // time accumulator for delta velocity accumulator
   // accelerometer max absolute offsets to be used for calibration
   float max_abs_offsets;
   // accelerometer and gyro raw sample rate in units of Hz
   uint16_t raw_sample_rate;
   // combine as flags ?
   bool new_data;
   bool healthy;
   bool delta_velocity_valid;
};

struct gyro_state_t{
   gyro_state_t ():
      error_count(0),
      raw_sample_rate(0),
      delta_angle_valid(false),
      new_data(false),
      healthy(false),
      cal_ok(true)
   {}

   void apply_offset(Vector3f &gyro) const
   {
      gyro -= offset;
    }
   // Most recent gyro reading
   Vector3f current_value;
   LowPassFilter2pVector3f filter;
   Vector3f  filtered_value;
   AP_Vector3f offset;
   Vector3f delta_angle;
   Vector3f last_delta_angle;
   Vector3f last_raw_value;
   Vector3f delta_angle_acc;
   uint32_t error_count;
   uint16_t raw_sample_rate;
   bool delta_angle_valid;
   bool new_data;
   bool healthy;
   bool cal_ok;
};

#endif // AERFPILOT_LIBRARIES_AP_INERTIAL_SENSOR_INERTIAL_SENSOR_STATE_H_INCLUDED
