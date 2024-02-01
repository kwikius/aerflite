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
 *   APM_Airspeed.cpp - airspeed (pitot) driver
 */


#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <AP_Baro/AP_Baro.h>
#include "AP_Airspeed.h"
#include "AP_Airspeed_Backend.h"

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
 #define ARSPD_DEFAULT_PIN 1
#elif CONFIG_HAL_BOARD == HAL_BOARD_QUAN
  #if defined QUAN_AERFLITE_BOARD
    #define ARSPD_DEFAULT_PIN 4
  #else
    #define ARSPD_DEFAULT_PIN 2
  #endif
#else
 #define ARSPD_DEFAULT_PIN 0
#endif

// table of user settable parameters
const AP_Param::GroupInfo AP_Airspeed::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Airspeed enable
    // @Description: enable airspeed sensor
    // @Values: 0:Disable,1:Enable
    AP_GROUPINFO("ENABLE",    0, AP_Airspeed, _enable, 1),

    // @Param: USE
    // @DisplayName: Airspeed use
    // @Description: use airspeed for flight control
    // @Values: 1:Use,0:Don't Use
    AP_GROUPINFO("USE",    1, AP_Airspeed, _use, 0),

    // @Param: OFFSET
    // @DisplayName: Airspeed offset
    // @Description: Airspeed calibration offset in Pascals to subtract from sensor reading
    // passed to the backend
    // @Increment: 0.1
    AP_GROUPINFO("OFFSET", 2, AP_Airspeed, _offset, 0),

    // @Param: RATIO
    // @DisplayName: Airspeed ratio
    // @Description: Airspeed calibration ratio
    // @Increment: 0.1
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    // using differential pressure in Pa
    // V = sqrt( 2 * dP / rho) where dP is differential pressure in Pa
    // and rho is density in kg.m[-3]
    // rho == 1.225 kg.m[-3] at sea level and 15 c
    // a better formula would take into account
    // altitude and air temperature
    AP_GROUPINFO("RATIO",  3, AP_Airspeed, _ratio, 1.633),
#else
    AP_GROUPINFO("RATIO",  3, AP_Airspeed, _ratio, 1.9936f),
#endif
    // @Param: PIN
    // @DisplayName: Airspeed pin
    // @Description: The analog pin number that the airspeed sensor is connected to. Set this to 0..9 for the APM2 analog pins. Set to 64 on an APM1 for the dedicated airspeed port on the end of the board. Set to 11 on PX4 for the analog airspeed port. Set to 15 on the Pixhawk for the analog airspeed port. Set to 65 on the PX4 or Pixhawk for an EagleTree or MEAS I2C airspeed sensor.
    // @User: Advanced
    AP_GROUPINFO("PIN",  4, AP_Airspeed, _pin, ARSPD_DEFAULT_PIN),

    // @Param: AUTOCAL
    // @DisplayName: Automatic airspeed ratio calibration
    // @Description: If this is enabled then the APM will automatically adjust the ARSPD_RATIO during flight, based upon an estimation filter using ground speed and true airspeed.
    // The automatic calibration will save the new ratio to EEPROM every 2 minutes if it changes by more than 5%. 
    // This option should be enabled for a calibration flight then disabled again when calibration is complete. 
    // Leaving it enabled all the time is not recommended.
    // @User: Advanced
    AP_GROUPINFO("AUTOCAL",  5, AP_Airspeed, _autocal, 0),

    // @Param: TUBE_ORDER
    // @DisplayName: Control pitot tube order
    // @Description: This parameter allows you to control whether the order in which the tubes
    // are attached to your pitot tube matters. 
    // If you set this to 0 then the top connector on the sensor needs to be the dynamic pressure.
    // If set to 1 then the bottom connector needs to be the dynamic pressure. 
    // If set to 2 (the default) then the airspeed driver will accept either order. 
    // The reason you may wish to specify the order is it will allow your airspeed sensor to detect 
    // if the aircraft it receiving excessive pressure on the static port, which would otherwise 
    // be seen as a positive airspeed.
    // @User: Advanced
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   AP_GROUPINFO("TUBE_ORDER",  6, AP_Airspeed, _tube_order, 0),
#else
    AP_GROUPINFO("TUBE_ORDER",  6, AP_Airspeed, _tube_order, 2),
#endif
    // @Param: SKIP_CAL
    // @DisplayName: Skip airspeed calibration on startup
    // @Description: This parameter allows you to skip airspeed offset calibration on startup, 
    // instead using the offset from the last calibration. 
    // This may be desirable if the offset variance between flights for your sensor 
    //is low and you want to avoid having to cover the pitot tube on each boot.
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("SKIP_CAL",  7, AP_Airspeed, _skip_cal, 0),

    AP_GROUPEND
};


void AP_Airspeed::init()
{
    m_backend = connect_airspeed_driver<AP_HAL::board>(*this);
    _last_pressure = 0;
    _calibration.init(_ratio);
    _last_saved_ratio = _ratio;
    _counter = 0;
}

// read the airspeed sensor
// FixMe!
// in fact returns the pressure but with an offset

float AP_Airspeed::get_pressure(void)
{
    
    if (!_enable) {
        return 0.f;
    }
    if (_hil_set) {
        _healthy = true;
        return _hil_pressure;
    }
    
    if (m_backend != nullptr){
        float pressure = 0.f;
       _healthy = m_backend->get_differential_pressure(pressure);

        return pressure;
    }else{
       return 0.f;
    }
}

// get a temperature reading if possible
bool AP_Airspeed::get_temperature(float &temperature)const
{
    if (!_enable || (m_backend == nullptr)) {
        return false;
    }
    return m_backend->get_temperature(temperature);

}

// calibrate the airspeed. This must be called at least once before
// the get_airspeed() interface can be used
void AP_Airspeed::calibrate(bool in_startup)
{
    if (!_enable) {
        return;
    }
#if defined(QUAN_MIXER_TRANQUILITY)
    _airspeed = 0;
    _raw_airspeed = 0;
    _offset.set_and_save(0);
    return;
#else

    float sum = 0;
    uint8_t count = 0;

    if (in_startup && _skip_cal) {
        return;
    }
    // discard first reading

    get_pressure();
    for (uint8_t i = 0; i < 10; i++) {
        hal.scheduler->delay(100);
        float p = get_pressure();
        if (_healthy) {
            sum += p;
            count++;
        }
    }
    if (count == 0) {
        // unhealthy sensor
        hal.console->println("Airspeed sensor unhealthy");
        _offset.set(0);
        return;
    }
    float raw = sum/count;
    _offset.set_and_save(raw);
    _airspeed = 0;
    _raw_airspeed = 0;

#endif

}

// update the airspeed sensor
void AP_Airspeed::update(void)
{
    if (!_enable) {
        return;
    }
    m_backend->update();

#if (defined QUAN_MIXER_TRANQUILITY)
    // no offset
    float airspeed_pressure = get_pressure() ;
#else
    float airspeed_pressure = get_pressure() - _offset;
#endif
    // remember raw pressure for logging
    _raw_pressure  = airspeed_pressure;

    /*
      we support different pitot tube setups so user can choose if
      they want to be able to detect pressure on the static port
     */
    switch ((enum pitot_tube_order)_tube_order.get()) {
    case PITOT_TUBE_ORDER_NEGATIVE:
        airspeed_pressure = -airspeed_pressure;
        // no break
    case PITOT_TUBE_ORDER_POSITIVE:
        if (airspeed_pressure < -32.f) {
            // we're reading more than about -8m/s. The user probably has
            // the ports the wrong way around
            _healthy = false;
        }
        break;
    case PITOT_TUBE_ORDER_AUTO:
    default:
        airspeed_pressure = fabsf(airspeed_pressure);
        break;
    }
    airspeed_pressure       = max(airspeed_pressure, 0.f);
    _last_pressure          = airspeed_pressure;

#if (defined QUAN_MIXER_TRANQUILITY)
    // V = sqrt( 2 * dP / rho) where dP is differential pressure in Pa
    // and rho is density in kg.m[-3]
    // rho == 1.225 kg.m[-3] at sea level and 15 c
   //  constexpr float rho = 1.225;
     constexpr float gas_constant = 287.058f; // J.kg-1.K-1
     float const abs_pressure = get_barometer().get_pressure(); // Pa
     float temperature ;  // K
     get_temperature(temperature); // use this rather than baro temperature, since it is off the board
     temperature += 273.13f; // to Kelvin
     float const rho = abs_pressure/(gas_constant * temperature);
    _raw_airspeed           = sqrtf( airspeed_pressure * 2.f/rho  );
#else
     _raw_airspeed           = sqrtf( airspeed_pressure * _ratio  );
#endif
    _airspeed               = 0.7f * _airspeed  +  0.3f * _raw_airspeed;
    _last_update_ms         = AP_HAL::millis();
}

void AP_Airspeed::setHIL(float airspeed, float diff_pressure, float temperature)
{
    _raw_airspeed = airspeed;
    _airspeed = airspeed;
    _last_pressure = diff_pressure;
    _last_update_ms = AP_HAL::millis();    
    _hil_pressure = diff_pressure;
    _hil_set = true;
    _healthy = true;
}

// log airspeed calibration data to MAVLink
void AP_Airspeed::log_mavlink_send(mavlink_channel_t chan, const Vector3f &vground)const
{
    mavlink_msg_airspeed_autocal_send(chan,
                                      vground.x,
                                      vground.y,
                                      vground.z,
                                      get_differential_pressure(),
                                      _EAS2TAS,
                                      _ratio.get(),
                                      _calibration.state.x,
                                      _calibration.state.y,
                                      _calibration.state.z,
                                      _calibration.P.a.x,
                                      _calibration.P.b.y,
                                      _calibration.P.c.z);
}

