/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "AP_InertialSensor_SITL.h"
#include <SITL/SITL.h>
#include <quan/time.hpp>

template <>
AP_InertialSensor_Backend * connect_inertial_sensor_driver<HALSITL::tag_board>(AP_InertialSensor & imu)
{
   return AP_InertialSensor_SITL::detect(imu);
}

const extern AP_HAL::HAL& hal;

AP_InertialSensor_SITL::AP_InertialSensor_SITL(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu, AP_PRODUCT_ID_NONE)
{
}

/*
  detect the sensor
 */
AP_InertialSensor_Backend *AP_InertialSensor_SITL::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_SITL *sensor = new AP_InertialSensor_SITL(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init_sensor()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

bool AP_InertialSensor_SITL::init_sensor(void)
{
    sitl = (SITL::SITL *)AP_Param::find_object("SIM_");
    if (sitl == nullptr) {
        return false;
    }

    // grab the used instances
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        gyro_instance[i] = _imu.register_gyro(sitl->update_rate_hz);
        accel_instance[i] = _imu.register_accel(sitl->update_rate_hz);
    }

    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_SITL::timer_update, void));
    return true;
}

namespace {
  QUAN_QUANTITY_LITERAL(acceleration,m_per_s2);
  QUAN_QUANTITY_LITERAL(time,s);
}

void AP_InertialSensor_SITL::timer_update(void)
{
   // minimum noise levels are 2 bits, but averaged over many
   // samples, giving around 0.01 m/s/s
   using acc = quan::acceleration_<float>;
   acc::m_per_s2 accel_noise = 0.01_m_per_s2;
   acc::m_per_s2 accel2_noise = 0.01_m_per_s2;

   using rad_per_s = quan::reciprocal_time_<
      quan::angle_<float>::rad
   >::per_s ;

   using deg_per_s = quan::reciprocal_time_<
      quan::angle_<float>::deg
   >::per_s ;

   // minimum gyro noise is also less than 1 bit
   rad_per_s gyro_noise = quan::angle_<float>::deg{0.04f} / 1_s;
   if (sitl->motors_on) {
      // add extra noise when the motors are on
      accel_noise += acc::m_per_s2{sitl->accel_noise.get()};
      accel2_noise += acc::m_per_s2{sitl->accel2_noise.get()};
      gyro_noise += quan::angle_<float>::deg{sitl->gyro_noise.get()} / 1_s;
   }

    // get accel bias (add only to first accelerometer)
    Vector3f accel_bias = sitl->accel_bias.get();

    acc::m_per_s2 xAccel1 = sitl->state.xAccel + accel_noise * rand_float() + acc::m_per_s2{accel_bias.x};
    acc::m_per_s2 yAccel1 = sitl->state.yAccel + accel_noise * rand_float() + acc::m_per_s2{accel_bias.y};
    acc::m_per_s2 zAccel1 = sitl->state.zAccel + accel_noise * rand_float() + acc::m_per_s2{accel_bias.z};

    acc::m_per_s2 xAccel2 = sitl->state.xAccel + accel2_noise * rand_float();
    acc::m_per_s2 yAccel2 = sitl->state.yAccel + accel2_noise * rand_float();
    acc::m_per_s2 zAccel2 = sitl->state.zAccel + accel2_noise * rand_float();

    if (fabsf(sitl->accel_fail) > 1.0e-6f) {
        xAccel1.set_numeric_value<acc::m_per_s2>(sitl->accel_fail);
        yAccel1.set_numeric_value<acc::m_per_s2>(sitl->accel_fail);
        zAccel1.set_numeric_value<acc::m_per_s2>(sitl->accel_fail);
    }

    Vector3f accel0 = Vector3f(xAccel1.numeric_value(), yAccel1.numeric_value(), zAccel1.numeric_value()) + _imu.get_accel_offsets(0);
    Vector3f accel1 = Vector3f(xAccel2.numeric_value(), yAccel2.numeric_value(), zAccel2.numeric_value()) + _imu.get_accel_offsets(1);

    _notify_new_accel_raw_sample(accel_instance[0], accel0);
    _notify_new_accel_raw_sample(accel_instance[1], accel1);

    rad_per_s p = sitl->state.rollRate + quan::angle_<float>::rad{gyro_drift()} / 1_s;
    rad_per_s q = sitl->state.pitchRate + quan::angle_<float>::rad{gyro_drift()}/ 1_s;
    rad_per_s r = sitl->state.yawRate + quan::angle_<float>::rad{gyro_drift()}/ 1_s;

    rad_per_s p1 = p + gyro_noise * rand_float();
    rad_per_s q1 = q + gyro_noise * rand_float();
    rad_per_s r1 = r + gyro_noise * rand_float();

    rad_per_s p2 = p + gyro_noise * rand_float();
    rad_per_s q2 = q + gyro_noise * rand_float();
    rad_per_s r2 = r + gyro_noise * rand_float();

    Vector3f gyro0 = Vector3f(p1.numeric_value(), q1.numeric_value(), r1.numeric_value()) + _imu.get_gyro_offsets(0);
    Vector3f gyro1 = Vector3f(p2.numeric_value(), q2.numeric_value(), r2.numeric_value()) + _imu.get_gyro_offsets(1);

    _notify_new_gyro_raw_sample(gyro_instance[0], gyro0);
    _notify_new_gyro_raw_sample(gyro_instance[1], gyro1);
}

// generate a random float between -1 and 1
float AP_InertialSensor_SITL::rand_float(void)
{
    return ((((unsigned)random()) % 2000000) - 1.0e6) / 1.0e6;
}

float AP_InertialSensor_SITL::gyro_drift(void)
{
    if (sitl->drift_speed == 0.0f ||
        sitl->drift_time == 0.0f) {
        return 0;
    }
    double period  = sitl->drift_time * 2;
    double minutes = fmod(AP_HAL::micros64() / 60.0e6, period);
    if (minutes < period/2) {
        return minutes * ToRad(sitl->drift_speed);
    }
    return (period - minutes) * ToRad(sitl->drift_speed);

}


bool AP_InertialSensor_SITL::update(void)
{
    for (uint8_t i=0; i<INS_SITL_INSTANCES; i++) {
        update_accel(accel_instance[i]);
        update_gyro(gyro_instance[i]);
    }
    return true;
}

#endif // HAL_BOARD_SITL
