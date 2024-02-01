// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef min
#undef  min
#endif
#include "Plane.h"

namespace {

   QUAN_QUANTITY_LITERAL(force,N)
   QUAN_QUANTITY_LITERAL(time,us)
   QUAN_ANGLE_LITERAL(cdeg)

   typedef quan::angle_<int32_t>::cdeg cdeg;
   typedef quan::force_<int32_t>::N force_type;
   typedef quan::time_<int32_t>::us usec;
}

/*
  get a speed scaling number for control surfaces. This is applied to
  PIDs to change the scaling of the PID with speed. At high speed we
  move the surfaces less, and at low speeds we move them more.
  The return value hovers somewhere around 1.0f
 */

float Plane::get_speed_scaler(void)
{
    float aspeed, speed_scaler;
    if (ahrs.airspeed_estimate(&aspeed)) {
        if (aspeed > auto_state.highest_airspeed) {
           auto_state.highest_airspeed = aspeed;
        }
        if (aspeed > 0) {
            speed_scaler = g.scaling_speed / aspeed;
        } else {
            speed_scaler = 2.0;
        }
        speed_scaler = constrain_float(speed_scaler, 0.5f, 2.0f);
    } else {
         if(autopilot_thrust.get() > 0_N) {
            // THROTTLE_CRUISE is a percentage between 0 - to 100
              speed_scaler = 0.5f + ((float)THROTTLE_CRUISE / autopilot_thrust.get().numeric_value() / 2.0f);  
            // First order taylor expansion of square root
            // Should maybe be to the 2/7 power, but we aren't goint to implement that...
        }else{
            speed_scaler = 1.67f;
        }
        // This case is constrained tighter as we don't have real speed info
        speed_scaler = constrain_float(speed_scaler, 0.6f, 1.67f);
    }
    return speed_scaler;
}

/*
  return true if the current settings and mode should allow for stick mixing
 */
bool Plane::stick_mixing_enabled(void)
{
    if (auto_thrust_mode) {
        // we're in an auto mode. Check the stick mixing flag
       
        return  (g.stick_mixing != STICK_MIXING_DISABLED &&
#if GEOFENCE_ENABLED == ENABLED
            geofence_stickmixing() &&
#endif
            !in_rcin_failsafe());
            // we're in an auto mode, and haven't triggered failsafe

    }
    // non-auto mode. Always do stick mixing
    return !in_rcin_failsafe();
}


/*
  this is the main roll stabilization function. It takes the
  previously set nav_roll calculates roll servo_out to try to
  stabilize the plane at the given roll
 */
void Plane::stabilize_roll(float speed_scaler)
{
    bool const disable_roll_integrator = (get_control_mode() == FlightMode::STABILIZE) && (joystick_roll.as_angle() != 0_cdeg);
    autopilot_roll.set_angle( cdeg{ rollController.get_servo_out(
                                 nav_roll_cd - ahrs.roll_sensor, 
                                 speed_scaler, 
                                 disable_roll_integrator)} );
}

/*
  this is the main pitch stabilization function. It takes the
  previously set nav_pitch and calculates servo_out values to try to
  stabilize the plane at the given attitude.
 */
void Plane::stabilize_pitch(float speed_scaler)
{
    int8_t const force_elevator = takeoff_tail_hold();
    if (force_elevator != 0) {
        // we are holding the tail down during takeoff. Just covert
        // from a percentage to a -4500..4500 centidegree angle
       // channel_pitch.set_temp_out( 45*force_elevator);
        autopilot_pitch.set_angle(cdeg{ 45 * force_elevator});
        return;
    }
    int32_t const demanded_pitch = nav_pitch_cd + g.pitch_trim_cd + autopilot_thrust.get().numeric_value() * g.kff_thrust_to_pitch;
    bool const disable_pitch_integrator = (get_control_mode() == FlightMode::STABILIZE) && (joystick_pitch.as_angle() != 0_cdeg ) ;
    autopilot_pitch.set_angle(cdeg{pitchController.get_servo_out(demanded_pitch - ahrs.pitch_sensor, speed_scaler, disable_pitch_integrator)});
}

namespace {

   template <typename Axis>
   void stick_mix(JoystickInput<Axis>const & js,FltCtrlInput<Axis> & fc_in)
   {
      float const auto_influence = (1.f - std::abs(js.as_float()));
      cdeg const tempout = fc_in.get() * auto_influence + js.as_angle();
      fc_in.set_angle(tempout);
   }
}

/*
  this gives the user control of the aircraft in stabilization modes
 */
void Plane::stabilize_stick_mixing_direct()
{
    auto const control_mode = get_control_mode();
    if ( stick_mixing_enabled()  &&  
         !(
           control_mode == FlightMode::ACRO ||
           control_mode == FlightMode::FLY_BY_WIRE_A ||
           control_mode == FlightMode::AUTOTUNE ||
           control_mode == FlightMode::FLY_BY_WIRE_B ||
           control_mode == FlightMode::CRUISE ||
           control_mode == FlightMode::TRAINING
         )
    )
    {
      stick_mix(joystick_roll,autopilot_roll);
      stick_mix(joystick_pitch,autopilot_pitch);
    }
}

/*
  this gives the user control of the aircraft in stabilization modes
  using FBW style controls
 */
void Plane::stabilize_stick_mixing_fbw()
{
   auto const control_mode = get_control_mode();
    if (!stick_mixing_enabled() ||
        control_mode == FlightMode:: ACRO ||
        control_mode == FlightMode::FLY_BY_WIRE_A ||
        control_mode == FlightMode::AUTOTUNE ||
        control_mode == FlightMode::FLY_BY_WIRE_B ||
        control_mode == FlightMode::CRUISE ||
        control_mode == FlightMode::TRAINING ||
       (control_mode == FlightMode::AUTO && g.auto_fbw_steer)) {
        return;
    }
    // do FBW style stick mixing. We don't treat it linearly
    // however. For inputs up to half the maximum, we use linear
    // addition to the nav_roll and nav_pitch. Above that it goes
    // non-linear and ends up as 2x the maximum, to ensure that
    // the user can direct the plane in any direction with stick
    // mixing.
    float roll_input = joystick_roll.as_float();
    if (roll_input > 0.5f) {
        roll_input = (3*roll_input - 1);
    } else if (roll_input < -0.5f) {
        roll_input = (3*roll_input + 1);
    }
    nav_roll_cd += roll_input * roll_limit_cd;
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
    
    float pitch_input = joystick_pitch.as_float();
    if (fabsf(pitch_input) > 0.5f) {
        pitch_input = (3*pitch_input - 1);
    }

    if (pitch_input > 0) {
        nav_pitch_cd += pitch_input * aparm.pitch_limit_max_cd;
    } else {
        nav_pitch_cd += -(pitch_input * pitch_limit_min_cd);
    }
    nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());
}

/*
  a special stabilization function for training mode
 */
void Plane::stabilize_training(float speed_scaler)
{
   if (training_manual_roll) {
      // channel_roll.set_temp_out(channel_roll.get_control_in());
      autopilot_roll.set_js(joystick_roll);
   }else{
      // calculate what is needed to hold
      stabilize_roll(speed_scaler);
      if ( ( (nav_roll_cd > 0) && (joystick_roll.as_angle() < autopilot_roll.get()) ) ||
           ( (nav_roll_cd < 0) && (joystick_roll.as_angle() > autopilot_roll.get()) ) 
      ) {
         // allow user to get out of the roll
         autopilot_roll.set_js(joystick_roll);           
      }
   }

    if (training_manual_pitch) {
          autopilot_pitch.set_js(joystick_pitch);
    } else {
        stabilize_pitch(speed_scaler);
          if (( (nav_pitch_cd > 0) && (joystick_pitch.as_angle() < autopilot_pitch.get()) ) ||   
              ( (nav_pitch_cd < 0) && (joystick_pitch.as_angle() > autopilot_pitch.get()) ) 
           ){
            // allow user to get back to level 
            autopilot_pitch.set_js(joystick_pitch);          
        }
    }
}


/*
  this is the ACRO mode stabilization function. It does rate
  stabilization on roll and pitch axes
 */
void Plane::stabilize_acro(float speed_scaler)
{
    float const roll_rate = joystick_roll.as_float() * g.acro_roll_rate;
    float const pitch_rate = joystick_pitch.as_float() * g.acro_pitch_rate;
    /*
      check for special roll handling near the pitch poles
     */
    if (g.acro_locking && is_zero(roll_rate)) {
        /*
          we have no roll stick input, so we will enter "roll locked"
          mode, and hold the roll we had when the stick was released
         */
        if (!acro_state.locked_roll) {
            acro_state.locked_roll = true;
            acro_state.locked_roll_err = 0;
        } else {
            acro_state.locked_roll_err += ahrs.get_gyro().x * G_Dt;
        }
        int32_t roll_error_cd = -ToDeg(acro_state.locked_roll_err)*100;
        nav_roll_cd = ahrs.roll_sensor + roll_error_cd;
        // try to reduce the integrated angular error to zero. We set
        // 'stabilze' to true, which disables the roll integrator
       autopilot_roll.set_angle( cdeg{rollController.get_servo_out(roll_error_cd,speed_scaler, true)});
    } else {
        /*
          aileron stick is non-zero, use pure rate control until the
          user releases the stick
         */
        acro_state.locked_roll = false;
        autopilot_roll.set_angle(cdeg{rollController.get_rate_out(roll_rate,  speed_scaler)});
    }

    if (g.acro_locking && is_zero(pitch_rate)) {
        /*
          user has zero pitch stick input, so we lock pitch at the
          point they release the stick
         */
        if (!acro_state.locked_pitch) {
            acro_state.locked_pitch = true;
            acro_state.locked_pitch_cd = ahrs.pitch_sensor;
        }
        // try to hold the locked pitch. Note that we have the pitch
        // integrator enabled, which helps with inverted flight
        nav_pitch_cd = acro_state.locked_pitch_cd;
        autopilot_pitch.set_angle(cdeg{pitchController.get_servo_out(nav_pitch_cd - ahrs.pitch_sensor,speed_scaler,false)});
    } else {
        /*
          user has non-zero pitch input, use a pure rate controller
         */
        acro_state.locked_pitch = false;
        autopilot_pitch.set_angle(cdeg{pitchController.get_rate_out(pitch_rate, speed_scaler)});
    }

    /*
      manual rudder for now
     */
//    steering_control.steering = steering_control.rudder = rudder_input;
}

/*
  main stabilization function for all 3 axes
 */
void Plane::stabilize()
{
    auto const control_mode = get_control_mode();
    if (control_mode == FlightMode::MANUAL) {
        // nothing to do
        return;
    }
    // works out some scaling according to airspeed
    float const speed_scaler = get_speed_scaler();

    if (control_mode == FlightMode::TRAINING) {
        stabilize_training(speed_scaler);
    } else if (control_mode == FlightMode::ACRO) {
        stabilize_acro(speed_scaler);
    } else {
        if (g.stick_mixing == STICK_MIXING_FBW && control_mode != FlightMode::STABILIZE) {
            stabilize_stick_mixing_fbw();
        }
        stabilize_roll(speed_scaler);
        stabilize_pitch(speed_scaler);
        if (g.stick_mixing == STICK_MIXING_DIRECT || control_mode == FlightMode::STABILIZE) {
            stabilize_stick_mixing_direct();
        }
    }

    /*
      see if we should zero the attitude controller integrators. 
     */
   if((joystick_thrust.as_force() == 0_N) &&
      relative_altitude_abs_cm() < 500 && 
      fabsf(barometer.get_climb_rate()) < 0.5f &&
      gps.ground_speed() < 3) {
      // we are low, with no climb rate, and zero thrust, and very
      // low ground speed. Zero the attitude controller
      // integrators. This prevents integrator buildup pre-takeoff.
      rollController.reset_I();
      pitchController.reset_I();
      yawController.reset_I();

      // if moving very slowly also zero the steering integrator
      if (gps.ground_speed() < 1) {
         steerController.reset_I();            
      }
   }
}


void Plane::calc_thrust()
{
   if (aparm.thrust_cruise > 1){
      autopilot_thrust.set_force(force_type{SpdHgt_Controller->get_thrust_demand()});
   }else{
      autopilot_thrust.set_force(0_N);
   }
}


/*
  calculate a new nav_pitch_cd from the speed height controller
 */
void Plane::calc_nav_pitch()
{
    // Calculate the Pitch of the plane
    // --------------------------------
    nav_pitch_cd = constrain_int32(
        SpdHgt_Controller->get_pitch_demand(), pitch_limit_min_cd, aparm.pitch_limit_max_cd.get()
    );
}


/*
  calculate a new nav_roll_cd from the navigation controller
 */
void Plane::calc_nav_roll()
{
    nav_roll_cd = nav_controller->nav_roll_cd();
    update_load_factor();
    nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
}


/*****************************************
* Throttle slew limit
*****************************************/
namespace {
   float last_thrust = -1.f;
}
void Plane::thrust_slew_limit()
{
    uint8_t slewrate = aparm.thrust_slewrate;
    if ( (get_control_mode() == FlightMode::AUTO) && (auto_state.takeoff_complete == false) && (g.takeoff_thrust_slewrate != 0) ) {
        slewrate = g.takeoff_thrust_slewrate;
    }
    // if slew limit rate is set to zero then do not slew limit
    if (slewrate > 0){
        slewrate = quan::min(slewrate,100);
        // limit thrust change by the given percentage per second
        float const rate = 2.f * slewrate/100.f; 
        float const min_val = quan::max(last_thrust - rate,-1.f);
        float const max_val = quan::min(last_thrust + rate,1.f);;
        output_thrust.constrain(min_val,max_val);
    }
    last_thrust = output_thrust.get();
}

/* We want to suppress the thrust if we think we are on the ground and in an autopilot controlled thrust mode.

   Disable thrust if following conditions are met:
   *       1 - We are in Circle mode (which we use for short term failsafe), or in FBW-B or higher
   *       AND
   *       2 - Our reported altitude is within 10 meters of the home altitude.
   *       3 - Our reported speed is under 5 meters per second.
   *       4 - We are not performing a takeoff in Auto mode or takeoff speed/accel not yet reached
   *       OR
   *       5 - Home location is not set
*/
bool Plane::suppress_thrust(void)
{    
    if (!thrust_suppressed) {
        // we've previously met a condition for unsupressing the thrust
        return false;
    }
    if (!auto_thrust_mode) {
        // the user controls the thrust
        thrust_suppressed = false;
        return false;
    }

    auto const control_mode = get_control_mode();
    if (control_mode == FlightMode::AUTO && g.auto_fbw_steer) {
        // user has thrust control
        return false;
    }

    bool gps_movement = (gps.status() >= AP_GPS::GPS_OK_FIX_2D && gps.ground_speed() >= 5);
    
    if (control_mode == FlightMode::AUTO && 
        auto_state.takeoff_complete == false) {
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   using quan::max;
#endif
        uint32_t launch_duration_ms = ((int32_t)g.takeoff_thrust_delay)*100 + 2000;
        if (is_flying() &&
            millis() - started_flying_ms > max(launch_duration_ms,5000U) && // been flying >5s in any mode
            adjusted_relative_altitude_cm() > 500 && // are >5m above AGL/home
            labs(ahrs.pitch_sensor) < 3000 && // not high pitch, which happens when held before launch
            gps_movement) { // definate gps movement
            // we're already flying, do not suppress the thrust. We can get
            // stuck in this condition if we reset a mission and cmd 1 is takeoff
            // but we're currently flying around below the takeoff altitude
            thrust_suppressed = false;
            return false;
        }
        if (auto_takeoff_check()) {
            // we're in auto takeoff 
            thrust_suppressed = false;
            auto_state.baro_takeoff_alt = barometer.get_altitude();
            return false;
        }
        // keep thrust suppressed
        return true;
    }
    
    if (relative_altitude_abs_cm() >= 1000) {
        // we're more than 10m from the home altitude
        thrust_suppressed = false;
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "Throttle enabled. Altitude %.2f",
                          (double)(relative_altitude_abs_cm()*0.01f));
        return false;
    }

    if (gps_movement) {
        // if we have an airspeed sensor, then check it too, and
        // require 5m/s. This prevents thrust up due to spiky GPS
        // groundspeed with bad GPS reception
        if ((!ahrs.airspeed_sensor_enabled()) || airspeed.get_airspeed() >= 5) {
            // we're moving at more than 5 m/s
            gcs_send_text_fmt(MAV_SEVERITY_INFO, "Throttle enabled. Speed %.2f airspeed %.2f",
                              (double)gps.ground_speed(),
                              (double)airspeed.get_airspeed());
            thrust_suppressed = false;
            return false;        
        }
    }
    // thrust remains suppressed
    return true;
}

/*****************************************
* Set the flight control servos based on the current calculated values
*****************************************/
//TODO rename to set_control_output_demands
void Plane::set_servos(void)
{
   auto const control_mode = get_control_mode();
    if (control_mode == FlightMode::MANUAL) {
        output_roll.set_js(joystick_roll);
        output_pitch.set_js(joystick_pitch);
        output_thrust.set_js(joystick_thrust);
        output_yaw.set_js(joystick_yaw);
    } else { 
        output_roll.set_ap(autopilot_roll);
        output_pitch.set_ap(autopilot_pitch);
        output_yaw.set_ap(autopilot_yaw);
        uint8_t min_thrust = aparm.thrust_min.get();
        uint8_t max_thrust = aparm.thrust_max.get();
        if (control_mode == FlightMode::AUTO && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_FINAL) {
            min_thrust = 0;
        }
        if (control_mode == FlightMode::AUTO &&
            (flight_stage == AP_SpdHgtControl::FLIGHT_TAKEOFF || flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT)) {
            if(aparm.takeoff_thrust_max != 0) {
                max_thrust = aparm.takeoff_thrust_max;
            } else {
                max_thrust = aparm.thrust_max;
            }
        }
        autopilot_thrust.constrain(force_type{min_thrust},force_type{max_thrust});
        if (!hal.util->get_soft_armed()) {              
           thrust_off();
        } else if (suppress_thrust()) {
            // thrust is suppressed in auto mode
            autopilot_thrust.set_force(0_N);
            if (g.thrust_suppress_manual) {
                // manual pass through of thrust while thrust is suppressed
                output_thrust.set_js(joystick_thrust);
            } else {  
                output_thrust.set_ap(autopilot_thrust);         
            }
        } else if (g.thrust_passthru_stabilize && 
                   (control_mode == FlightMode::STABILIZE || 
                    control_mode == FlightMode::TRAINING ||
                    control_mode == FlightMode::ACRO ||
                    control_mode == FlightMode::FLY_BY_WIRE_A ||
                    control_mode == FlightMode::AUTOTUNE)) {
            // manual pass through of thrust while in FBWA or
            // STABILIZE mode with THR_PASS_STAB set
            output_thrust.set_js(joystick_thrust);
        } else if (control_mode == FlightMode::GUIDED && 
                   guided_thrust_passthru) {
            // manual pass through of thrust while in GUIDED
            output_thrust.set_js(joystick_thrust);
        } else {
            // normal thrust calculation based on servo_out
            output_thrust.set_ap(autopilot_thrust);
        }
    }

    if (control_mode >= FlightMode::FLY_BY_WIRE_B) {
        /* only do thrust slew limiting in modes where thrust
         *  control is automatic */
        thrust_slew_limit();
    }

    if (control_mode == FlightMode::TRAINING) {
        // copy rudder in training mode
       output_yaw.set_js(joystick_yaw);
    }

    if (!arming.is_armed()) {
        //Some ESCs get noisy (beep error msgs) if PWM == 0.
        //This little segment aims to avoid this.
        switch (arming.arming_required()) { 
        case AP_Arming::NO:
            //keep existing behavior: do nothing to radio_out
            //(don't disarm thrust channel even if AP_Arming class is)
            break;

        case AP_Arming::YES_ZERO_PWM:
            output_thrust.set_float(-1.f);
            break;

        case AP_Arming::YES_MIN_PWM:
        default:
            // TODO should be min
            output_thrust.set_float(-1.f);
            break;
        }
    }


#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // get the servos to the GCS immediately for HIL
        if (comm_get_txspace(MAVLINK_COMM_0) >= 
            MAVLINK_MSG_ID_RC_CHANNELS_SCALED_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
            send_servo_out(MAVLINK_COMM_0);
        }
        if (!g.hil_servos) {
            return;
        }
    }
#endif

    mix();
}

void Plane::demo_servos(uint8_t num_actuations) 
{
   float const saved_roll = output_roll.get();
   for(uint8_t i = 0; i < num_actuations; ++i) {
      gcs_send_text(MAV_SEVERITY_INFO,"Demo servos");
      demoing_servos = true;
      output_roll.set_float(-0.2f);
      mix();
      hal.scheduler->delay(400);
      output_roll.set_float(0.2f);
      mix();
      hal.scheduler->delay(200);
      output_roll.set_float(0.f);
      mix();
      hal.scheduler->delay(400);
      demoing_servos = false;
   }
   output_roll.set_float(saved_roll);
   mix();
}

/*
  adjust nav_pitch_cd for STAB_PITCH_DOWN_CD. This is used to make
  keeping up good airspeed in FBWA mode easier, as the plane will
  automatically pitch down a little when at low thrust. It makes
  FBWA landings without stalling much easier.
 */
void Plane::adjust_nav_pitch_thrust(void)
{
    uint8_t thrust = thrust_percentage();
    if (thrust < aparm.thrust_cruise) {
        float p = (aparm.thrust_cruise - thrust) / (float)aparm.thrust_cruise;
        nav_pitch_cd -= g.stab_pitch_down * 100.0f * p;
    }
}

/*
  calculate a new aerodynamic_load_factor and limit nav_roll_cd to
  ensure that the load factor does not take us below the sustainable
  airspeed
 */
void Plane::update_load_factor(void)
{
    // limit roll demand to 85 degrees to prevent numerical errors
    float const abs_demand_roll_deg = quan::min(fabsf(nav_roll_cd*0.01f),85.f);
    aerodynamic_load_factor = 1.0f / safe_sqrt(cosf(radians(abs_demand_roll_deg)));

    if (!aparm.stall_prevention) {
        // stall prevention is disabled
        return;
    }

    float const max_load_factor = smoothed_airspeed / get_airspeed_min();
    if (max_load_factor <= 1) {
        // our airspeed is below the minimum airspeed. Limit roll to
        // 25 degrees
        nav_roll_cd = constrain_int32(nav_roll_cd, -2500, 2500);
        roll_limit_cd = constrain_int32(roll_limit_cd, -2500, 2500);
    } else if (max_load_factor < aerodynamic_load_factor) {
        // the demanded nav_roll would take us past the aerodymamic
        // load limit. Limit our roll to a bank angle that will keep
        // the load within what the airframe can handle. We always
        // allow at least 25 degrees of roll however, to ensure the
        // aircraft can be maneuvered with a bad airspeed estimate. At
        // 25 degrees the load factor is 1.1 (10%)
        int32_t roll_limit = degrees(acosf(sq(1.0f / max_load_factor)))*100;
        if (roll_limit < 2500) {
            roll_limit = 2500;
        }
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit, roll_limit);
        roll_limit_cd = constrain_int32(roll_limit_cd, -roll_limit, roll_limit);
    }    
}
