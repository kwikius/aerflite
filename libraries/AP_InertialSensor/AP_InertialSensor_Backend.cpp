/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_Backend.h"
#include <DataFlash/DataFlash.h>

const extern AP_HAL::HAL& hal;

AP_InertialSensor_Backend::AP_InertialSensor_Backend(AP_InertialSensor &imu, int16_t product_id_in) :
    _imu(imu),
    _product_id(product_id_in)
{}

void AP_InertialSensor_Backend::_rotate_and_correct_accel(uint8_t instance, Vector3f &accel) const
{
    // apply offsets and then scaling
    _imu.m_ins_state[instance].accel.offset_and_scale(accel);
    // then rotate to body frame
    accel.rotate(_imu._board_orientation);
}

void AP_InertialSensor_Backend::_rotate_and_correct_gyro(uint8_t instance, Vector3f &gyro_value)
{
    // apply offset
    _imu.m_ins_state[instance].gyro.apply_offset(gyro_value);
    // rotate to body frame
    gyro_value.rotate(_imu._board_orientation);
}

/*
  rotate gyro vector and add the gyro offset
 */
void AP_InertialSensor_Backend::_publish_gyro(uint8_t instance, const Vector3f &gyro)
{
   auto & gyro_state = _imu.m_ins_state[instance].gyro;
   gyro_state.current_value = gyro;
   gyro_state.healthy = true;

   if (gyro_state.raw_sample_rate > 0) {
      // publish delta angle
      gyro_state.delta_angle = gyro_state.delta_angle_acc;
      gyro_state.delta_angle_valid = true;
   }
}

void AP_InertialSensor_Backend::_notify_new_gyro_raw_sample(uint8_t instance,
                                                            const Vector3f &gyro,
                                                            uint64_t sample_us)
{
    //float dt;
    auto & gyro_state = _imu.m_ins_state[instance].gyro;

    if (gyro_state.raw_sample_rate <= 0) {
        return;
    }

    float const dt = 1.0f / gyro_state.raw_sample_rate;

    // compute delta angle
    Vector3f delta_angle = (gyro + gyro_state.last_raw_value) * 0.5f * dt;

    // compute coning correction
    // see page 26 of:
    // Tian et al (2010) Three-loop Integration of GPS and Strapdown INS with Coning and Sculling Compensation
    // Available: http://www.sage.unsw.edu.au/snap/publications/tian_etal2010b.pdf
    // see also examples/coning.py
    Vector3f delta_coning = (gyro_state.delta_angle_acc +
                             gyro_state.last_delta_angle * (1.0f / 6.0f));
    delta_coning = delta_coning % delta_angle;
    delta_coning *= 0.5f;

    // integrate delta angle accumulator
    // the angles and coning corrections are accumulated separately in the
    // referenced paper, but in simulation little difference was found between
    // integrating together and integrating separately (see examples/coning.py)
    gyro_state.delta_angle_acc += delta_angle + delta_coning;

    // save previous delta angle for coning correction
    gyro_state.last_delta_angle = delta_angle;
    gyro_state.last_raw_value = gyro;

    gyro_state.filtered_value = gyro_state.filter.apply(gyro);
    if (gyro_state.filtered_value.is_nan() || gyro_state.filtered_value.is_inf()) {
        gyro_state.filter.reset();
    }
    gyro_state.new_data = true;

    DataFlash_Class *dataflash = get_dataflash();
    if (dataflash != NULL) {
        uint64_t now = AP_HAL::micros64();
        struct log_GYRO pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_GYR1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            GyrX      : gyro.x,
            GyrY      : gyro.y,
            GyrZ      : gyro.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
    }
}

/*
  rotate accel vector, scale and add the accel offset
 */
void AP_InertialSensor_Backend::_publish_accel(uint8_t instance, const Vector3f &accel)
{

    auto & accel_state = _imu.m_ins_state[instance].accel;

    accel_state.current_value = accel;
    accel_state.healthy = true;

    if (accel_state.raw_sample_rate <= 0) {
        return;
    }

    // publish delta velocity
    accel_state.delta_velocity = accel_state.delta_velocity_acc;
    accel_state.delta_velocity_dt = accel_state.delta_velocity_acc_dt;
    accel_state.delta_velocity_valid = true;
}

void AP_InertialSensor_Backend::_notify_new_accel_raw_sample(uint8_t instance,
                                                             const Vector3f &accel,
                                                             uint64_t sample_us)
{
   auto& accel_state = _imu.m_ins_state[instance].accel;

   if (accel_state.raw_sample_rate <= 0) {
      return;
   }

   float const dt = 1.0f / accel_state.raw_sample_rate;

   _imu.calc_vibration_and_clipping(instance, accel, dt);

   // delta velocity
   accel_state.delta_velocity_acc += accel * dt;
   accel_state.delta_velocity_acc_dt += dt;

   accel_state.filtered_value = accel_state.filter.apply(accel);
   if(accel_state.filtered_value.is_nan() || accel_state.filtered_value.is_inf() ){
      accel_state.filter.reset();
   }

   accel_state.new_data = true;

   DataFlash_Class *dataflash = get_dataflash();
   if (dataflash != NULL) {
        uint64_t now = AP_HAL::micros64();
        struct log_ACCEL pkt = {
            LOG_PACKET_HEADER_INIT((uint8_t)(LOG_ACC1_MSG+instance)),
            time_us   : now,
            sample_us : sample_us?sample_us:now,
            AccX      : accel.x,
            AccY      : accel.y,
            AccZ      : accel.z
        };
        dataflash->WriteBlock(&pkt, sizeof(pkt));
   }
}

void AP_InertialSensor_Backend::_set_accel_max_abs_offset(uint8_t instance,
                                                          float max_offset)
{
    _imu.m_ins_state[instance].accel.max_abs_offsets = max_offset;
}

// set accelerometer error_count
void AP_InertialSensor_Backend::_set_accel_error_count(uint8_t instance, uint32_t error_count)
{
    _imu.m_ins_state[instance].accel.error_count = error_count;
}

// set gyro error_count
void AP_InertialSensor_Backend::_set_gyro_error_count(uint8_t instance, uint32_t error_count)
{
    _imu.m_ins_state[instance].gyro.error_count = error_count;
}

/*
  publish a temperature value for an instance
 */
void AP_InertialSensor_Backend::_publish_temperature(uint8_t instance, float temperature)
{
    _imu.m_ins_state[instance].temperature = temperature;
}

/*
  common gyro update function for all backends
 */
void AP_InertialSensor_Backend::update_gyro(uint8_t instance)
{
    hal.scheduler->suspend_timer_procs();

    auto & gyro_state = _imu.m_ins_state[instance].gyro;

    if (gyro_state.new_data) {
        _publish_gyro(instance, gyro_state.filtered_value);
        gyro_state.new_data = false;
    }

    // possibly update filter frequency
    if (_last_gyro_filter_hz[instance] != _gyro_filter_cutoff()) {
        gyro_state.filter.set_cutoff_frequency(_gyro_raw_sample_rate(instance), _gyro_filter_cutoff());
        _last_gyro_filter_hz[instance] = _gyro_filter_cutoff();
    }

    hal.scheduler->resume_timer_procs();
}

/*
  common accel update function for all backends
 */
void AP_InertialSensor_Backend::update_accel(uint8_t instance)
{
    hal.scheduler->suspend_timer_procs();

    auto & accel_state = _imu.m_ins_state[instance].accel;

    if (accel_state.new_data){
        _publish_accel(instance, accel_state.filtered_value);
        accel_state.new_data = false;
    }

    // possibly update filter frequency
    if (_last_accel_filter_hz[instance] != _accel_filter_cutoff()) {
        accel_state.filter.set_cutoff_frequency(_accel_raw_sample_rate(instance), _accel_filter_cutoff());
        _last_accel_filter_hz[instance] = _accel_filter_cutoff();
    }

    hal.scheduler->resume_timer_procs();
}
