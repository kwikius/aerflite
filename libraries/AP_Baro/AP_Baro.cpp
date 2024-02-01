/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 *       APM_Baro.cpp - barometer driver
 *
 */

#include <quan/min.hpp>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include "AP_Baro.h"
#include "AP_baro_driver.h"

// maximum number of sensor instances
//#define BARO_MAX_INSTANCES 3

// maximum number of drivers. Note that a single driver can provide
// multiple sensor instances
//#define BARO_MAX_DRIVERS 2

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Baro::var_info[] = {
    // NOTE: Index numbers 0 and 1 were for the old integer
    // ground temperature and pressure

    // @Param: ABS_PRESS
    // @DisplayName: Absolute Pressure
    // @Description: calibrated ground pressure in Pascals
    // @Units: pascals
    // @Increment: 1
    AP_GROUPINFO("ABS_PRESS", 2, AP_Baro, sensors[0].ground_pressure, 0),

    // @Param: TEMP
    // @DisplayName: ground temperature
    // @Description: calibrated ground temperature in degrees Celsius
    // @Units: degrees celsius
    // @Increment: 1
    AP_GROUPINFO("TEMP", 3, AP_Baro, sensors[0].ground_temperature, 0),

    // index 4 reserved for old AP_Int8 version in legacy FRAM
    //AP_GROUPINFO("ALT_OFFSET", 4, AP_Baro, _alt_offset, 0),

    // @Param: ALT_OFFSET
    // @DisplayName: altitude offset
    // @Description: altitude offset in meters added to barometric altitude. This is used to allow for automatic adjustment of the base barometric altitude by a ground station equipped with a barometer. The value is added to the barometric altitude read by the aircraft. It is automatically reset to 0 when the barometer is calibrated on each reboot or when a preflight calibration is performed.
    // @Units: meters
    // @Increment: 0.1
    AP_GROUPINFO("ALT_OFFSET", 5, AP_Baro, _alt_offset, 0),

    // @Param: PRIMARY
    // @DisplayName: Primary barometer
    // @Description: This selects which barometer will be the primary if multiple barometers are found
    // @Values: 0:FirstBaro,1:2ndBaro,2:3rdBaro
    AP_GROUPINFO("PRIMARY", 6, AP_Baro, _primary_baro, 0),
    
    AP_GROUPEND
};

/*
  AP_Baro constructor
 */
AP_Baro::AP_Baro() :
        _num_drivers(0),
        _num_sensors(0),
        _primary(0),
        _last_altitude_EAS2TAS(0.0f),
        _EAS2TAS(0.0f),
        _external_temperature(0.0f),
        _last_external_temperature_ms(0),
        _hil_mode(false)
{
    memset(sensors, 0, sizeof(sensors));

    AP_Param::setup_object_defaults(this, var_info);
}

// calibrate the barometer. This must be called at least once before
// the altitude() or climb_rate() interfaces can be used
void AP_Baro::calibrate()
{
    // reset the altitude offset when we calibrate. The altitude
    // offset is supposed to be for within a flight
   // hal.console->printf("set and save\n");
    _alt_offset.set_and_save(0);

    // start by assuming all sensors are calibrated (for healthy() test)
   // hal.console->printf("num sensors = %u\n",static_cast<unsigned int>( _num_sensors));
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].calibrated = true;
        sensors[i].alt_ok = true;
    }

    // let the barometer settle for a full second after startup
    // the MS5611 reads quite a long way off for the first second,
    // leading to about 1m of error if we don't wait
    hal.console->printf("health checking\n");
    for (uint8_t i = 0; i < 10; i++) {
        uint32_t tstart = AP_HAL::millis();
        do {
           // hal.console->printf("call update\n");
            update();
           // hal.console->printf("update done\n");
            if (AP_HAL::millis() - tstart > 500) {
                AP_HAL::panic("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [2]\r\n");
            }
            hal.scheduler->delay(10);
        } while (!healthy());
       //  hal.console->printf("done do\n");
        hal.scheduler->delay(100);
    }
   // hal.console->printf("health check done\n");
    // now average over 5 values for the ground pressure and
    // temperature settings
    float sum_pressure[m_max_baro_instances] = {0};
    float sum_temperature[m_max_baro_instances] = {0};
    uint8_t count[m_max_baro_instances] = {0};
    const uint8_t num_samples = 5;
     //hal.console->printf("averaging\n");
    for (uint8_t c = 0; c < num_samples; c++) {
        uint32_t tstart = AP_HAL::millis();
        do {
            update();
            if (AP_HAL::millis() - tstart > 500) {
                AP_HAL::panic("PANIC: AP_Baro::read unsuccessful "
                        "for more than 500ms in AP_Baro::calibrate [3]\r\n");
            }
        } while (!healthy());
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                sum_pressure[i] += sensors[i].pressure;
                sum_temperature[i] += sensors[i].temperature;
                count[i] += 1;
            }
        }
        hal.scheduler->delay(100);
    }
    // hal.console->printf("set and save 2\n");
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (count[i] == 0) {
            sensors[i].calibrated = false;
        } else {
            sensors[i].ground_pressure.set_and_save(sum_pressure[i] / count[i]);
            sensors[i].ground_temperature.set_and_save(sum_temperature[i] / count[i]);
        }
    }

    // panic if all sensors are not calibrated
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].calibrated) {
            return;
        }
    }
    AP_HAL::panic("AP_Baro: all sensors uncalibrated");
}

/*
   update the barometer calibration
   this updates the baro ground calibration to the current values. It
   can be used before arming to keep the baro well calibrated
*/
void AP_Baro::update_calibration()
{
    for (uint8_t i=0; i<_num_sensors; i++) {
        if (healthy(i)) {
            sensors[i].ground_pressure.set(get_pressure(i));
        }
        float last_temperature = sensors[i].ground_temperature;
        sensors[i].ground_temperature.set(get_calibration_temperature(i));

        // don't notify the GCS too rapidly or we flood the link
        uint32_t now = AP_HAL::millis();
        if (now - _last_notify_ms > 10000) {
            sensors[i].ground_pressure.notify();
            sensors[i].ground_temperature.notify();
            _last_notify_ms = now;
        }
        if (fabsf(last_temperature - sensors[i].ground_temperature) > 3) {
            // reset _EAS2TAS to force it to recalculate. This happens
            // when a digital airspeed sensor comes online
            _EAS2TAS = 0;
        }
    }
}

// return altitude difference in meters between current pressure and a
// given base_pressure in Pascal
float AP_Baro::get_altitude_difference(float base_pressure, float pressure) const
{
   // float ret;
    float const temp    = get_ground_temperature() + 273.15f;
    float const scaling = pressure / base_pressure;

    // This is an exact calculation that is within +-2.5m of the standard
    // atmosphere tables in the troposphere (up to 11,000 m amsl).
    return 153.8462f * temp * (1.0f - expf(0.190259f * logf(scaling)));

   // return ret;
}


// return current scale factor that converts from equivalent to true airspeed
// valid for altitudes up to 10km AMSL
// assumes standard atmosphere lapse rate
// Fixme This is a mess. Split into  get and calc funs
// if altitude difference < 100 m do nothing much
// therefore only need updating every 1 s or so?
float AP_Baro::get_EAS2TAS(void)
{
    float altitude = get_altitude();
    if ((fabsf(altitude - _last_altitude_EAS2TAS) < 100.0f) && !is_zero(_EAS2TAS)) {
        // not enough change to require re-calculating
        return _EAS2TAS;
    }

    float tempK = get_calibration_temperature() + 273.15f - 0.0065f * altitude;
    _EAS2TAS = safe_sqrt(1.225f / ((float)get_pressure() / (287.26f * tempK)));
    _last_altitude_EAS2TAS = altitude;
    return _EAS2TAS;
}

// return air density / sea level density - decreases as altitude climbs
float AP_Baro::get_air_density_ratio(void)
{
    float eas2tas = get_EAS2TAS();
    if (eas2tas > 0.0f) {
        return 1.0f/(sq(get_EAS2TAS()));
    } else {
        return 1.0f;
    }
}

// return current climb_rate estimeate relative to time that calibrate()
// was called. Returns climb rate in meters/s, positive means up
// note that this relies on read() being called regularly to get new data
float AP_Baro::get_climb_rate(void)
{
    // we use a 7 point derivative filter on the climb rate. This seems
    // to produce somewhat reasonable results on real hardware
// Fixme This is a mess. Split into  get and calc funs
    return _climb_rate_filter.slope() * 1.0e3f;
}


/*
  set external temperature to be used for calibration (degrees C)
 */
void AP_Baro::set_external_temperature(float temperature)
{
    _external_temperature = temperature;
    _last_external_temperature_ms = AP_HAL::millis();
}

/*
  get the temperature in degrees C to be used for calibration purposes
 */
float AP_Baro::get_calibration_temperature(uint8_t instance) const
{
    // if we have a recent external temperature then use it
    if (_last_external_temperature_ms != 0 && AP_HAL::millis() - _last_external_temperature_ms < 10000) {
        return _external_temperature;
    }
    // if we don't have an external temperature then use the minimum
    // of the barometer temperature and 25 degrees C. The reason for
    // not just using the baro temperature is it tends to read high,
    // often 30 degrees above the actual temperature. That means the
    // EAS2TAS tends to be off by quite a large margin

    return quan::min(get_temperature(instance),25.f);
  
}


/*
  initialise the barometer object, loading backend drivers
 */
void AP_Baro::init(void)
{
   drivers[0] = connect_baro_driver<AP_HAL::board>(*this);
   if (drivers[0] == NULL) {
      AP_HAL::panic("Baro: unable to initialise driver");
   }
   _num_drivers = 1;
}


/*
  call update on all drivers
 */
void AP_Baro::update(void)
{
    if (!_hil_mode) {
        for (uint8_t i=0; i<_num_drivers; i++) {
            drivers[i]->update();
        }
    }

    // consider a sensor as healthy if it has had an update in the
    // last 0.5 seconds
    uint32_t now = AP_HAL::millis();
    for (uint8_t i=0; i<_num_sensors; i++) {
        sensors[i].healthy = (now - sensors[i].last_update_ms < 500) && !is_zero(sensors[i].pressure);
    }

    using std::isnan;
    using std::isinf;

    for (uint8_t i=0; i<_num_sensors; i++) {
        if (sensors[i].healthy) {
            // update altitude calculation
            if (is_zero(sensors[i].ground_pressure)) {
                sensors[i].ground_pressure = sensors[i].pressure;
            }
            float altitude = get_altitude_difference(sensors[i].ground_pressure, sensors[i].pressure);
            // sanity check altitude
            sensors[i].alt_ok = !(isnan(altitude) || isinf(altitude));
            if (sensors[i].alt_ok) {
                sensors[i].altitude = altitude + _alt_offset;
            }
        }
    }

    // ensure the climb rate filter is updated
    if (healthy()) {
        _climb_rate_filter.update(get_altitude(), get_last_update());
    }

    // choose primary sensor
    if (_primary_baro >= 0 && _primary_baro < _num_sensors && healthy(_primary_baro)) {
        _primary = _primary_baro;
    } else {
        _primary = 0;
        for (uint8_t i=0; i<_num_sensors; i++) {
            if (healthy(i)) {
                _primary = i;
                break;
            }
        }
    }
}

/* register a new sensor, claiming a sensor slot. If we are out of
   slots it will panic
*/
uint8_t AP_Baro::register_sensor(void)
{
    if (_num_sensors >= m_max_baro_instances) {
        AP_HAL::panic("Too many barometers");
    }
    return _num_sensors++;
}


/*
  check if all barometers are healthy
 */
bool AP_Baro::all_healthy(void) const
{
     for (uint8_t i=0; i<_num_sensors; i++) {
         if (!healthy(i)) {
             return false;
         }
     }
     return _num_sensors > 0;
}

// ==========================================================================
// based on tables.cpp from http://www.pdas.com/atmosdownload.html

namespace {
   /* 
      Compute the temperature, density, and pressure in the standard atmosphere
      Correct to 20 km.  Only approximate thereafter.
   */
   void compute_atmosphere(
      const float alt,                           // geometric altitude, km.
      float& sigma,                   // density/sea-level standard density
      float& delta,                 // pressure/sea-level standard pressure
      float& theta)           // temperature/sea-level standard temperature
   {
       const float REARTH = 6369.0f;        // radius of the Earth (km)
       const float GMR    = 34.163195f;     // gas constant
       float h=alt*REARTH/(alt+REARTH);     // geometric to geopotential altitude

       if (h < 11.0f) {
           // Troposphere
           theta=(288.15f-6.5f*h)/288.15f;
           delta=powf(theta, GMR/6.5f);
       } else {
           // Stratosphere
           theta=216.65f/288.15f;
           delta=0.2233611f*expf(-GMR*(h-11.0f)/216.65f);
       }

       sigma = delta/theta;
   }
}

/*
  convert an altitude in meters above sea level to a presssure and temperature
 */
void AP_Baro::setHIL(float altitude_msl)
{
    float sigma, delta, theta;
    const float p0 = 101325;

    compute_atmosphere(altitude_msl*0.001f, sigma, delta, theta);
    float p = p0 * delta;
    float T = 303.16f * theta - 273.16f; // Assume 30 degrees at sea level - converted to degrees Kelvin

    setHIL(0, p, T);
}

/*
  set HIL pressure and temperature for an instance
 */
void AP_Baro::setHIL(uint8_t instance, float pressure, float temperature)
{
    if (instance >= _num_sensors) {
        // invalid
        return;
    }
    _hil.press_buffer.push_back(pressure);
    _hil.temp_buffer.push_back(temperature);
}

