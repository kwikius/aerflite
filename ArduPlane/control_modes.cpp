/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif

namespace {

  // return a valid flight mode index or 255 if out of range
   uint8_t pulse_position_to_rcin_control_mode_index(uint16_t pulsewidth)
   {
      if (pulsewidth < 900U)  return 255U; // Error condition

      if (pulsewidth < 1230U) return 0U;
      if (pulsewidth < 1360U) return 1U;
      if (pulsewidth < 1490U) return 2U;
      if (pulsewidth < 1620U) return 3U;
      if (pulsewidth < 1750U) return 4U;   
      if (pulsewidth < 2200U) return 5U;

      return 255U;   // Error condition
   }

   uint8_t constexpr switch_bounce_max = 3;
   uint8_t switch_bounce_count = 0;
   FlightMode rcin_control_mode = FlightMode::RTL;
   FlightMode maybe_rcin_control_mode = rcin_control_mode;

}

/*
    Return current requested flight mode according to the control switch
*/
FlightMode Plane::readControlSwitch(void)const
{
   uint16_t const pulsewidth = hal.rcin->read(g.flight_mode_channel - 1);                                         
   uint8_t const control_mode_idx = pulse_position_to_rcin_control_mode_index(pulsewidth);

   FlightMode const new_rcin_control_mode = 
      (control_mode_idx < 6) 
         ? static_cast<FlightMode>(this->flight_modes[control_mode_idx].get()) 
         : FlightMode::RTL;
   
   if ( new_rcin_control_mode != rcin_control_mode){           // rcin_control_mode pulse has changed
     
      if (new_rcin_control_mode == maybe_rcin_control_mode){   // consistent new value
         if (switch_bounce_count == switch_bounce_max){        // over n cycles
            rcin_control_mode = maybe_rcin_control_mode;       // then switch to new value
            switch_bounce_count = 0;                           // and reset the state machine
         }else{ // consistent change but not reached n cycles, keep counting consistent change
            ++ switch_bounce_count;    
         }
      }else{                                                  // changed value not consistent with previous changed value
         if ( switch_bounce_count == 0U){                     // but OK if this is a new detection sequence
            maybe_rcin_control_mode = new_rcin_control_mode;  
            ++ switch_bounce_count;
         }else{                                               // otherwise reset the state machine
            maybe_rcin_control_mode = rcin_control_mode;
            switch_bounce_count = 0U;
         }
      }
   }
   // convert to control mode now
   return rcin_control_mode;
}

FlightMode Plane::get_previous_mode() {
    return m_previous_mode; 
}

void Plane::set_mode(enum FlightMode mode)
{
    auto const cur_control_mode = get_control_mode();
    if(cur_control_mode == mode) {
        // don't switch modes if we are already in the correct mode.
        return;
    }

/*
    We should do this on a special command only
    if(g.auto_trim > 0 && control_mode == MANUAL){
        trim_control_surfaces();
    }
*/
    // perform any cleanup required for prev flight mode
    exit_mode(cur_control_mode);

    // cancel inverted flight
    auto_state.inverted_flight = false;

    // don't cross-track when starting a mission
    auto_state.next_wp_no_crosstrack = true;

    // reset landing check
    auto_state.checked_for_autoland = false;

    // reset go around command
    auto_state.commanded_go_around = false;

    // zero locked course
    steer_state.locked_course_err = 0;

    // reset crash detection
    crash_state.is_crashed = false;

    // set mode
    m_previous_mode = cur_control_mode;
    m_control_mode = mode;

    if (get_previous_mode() == FlightMode::AUTOTUNE && get_control_mode() != FlightMode::AUTOTUNE) {
        // restore last gains
        autotune_restore();
    }

    // zero initial pitch and highest airspeed on mode change
    auto_state.highest_airspeed = 0;
    auto_state.initial_pitch_cd = ahrs.pitch_sensor;

    // disable taildrag takeoff on mode change
    auto_state.fbwa_tdrag_takeoff_mode = false;

    // start with previous WP at current location
    prev_WP_loc = current_loc;

    // new mode means new loiter
    loiter.start_time_ms = 0;

    auto const new_control_mode = get_control_mode();
    switch(new_control_mode) {
       case FlightMode::INITIALISING:
           auto_thrust_mode = true;
           break;

       case FlightMode::MANUAL:
          //     hal.console->printf("Manual activated\n");
       case FlightMode::STABILIZE:
       case FlightMode::TRAINING:
       case FlightMode::FLY_BY_WIRE_A:
           auto_thrust_mode = false;
           break;

       case FlightMode::AUTOTUNE:
           auto_thrust_mode = false;
           autotune_start();
           break;

       case FlightMode::ACRO:
           auto_thrust_mode = false;
           acro_state.locked_roll = false;
           acro_state.locked_pitch = false;
           break;

       case FlightMode::CRUISE:
           auto_thrust_mode = true;
           cruise_state.locked_heading = false;
           cruise_state.lock_timer_ms = 0;
           set_target_altitude_current();
           break;

       case FlightMode::FLY_BY_WIRE_B:
           auto_thrust_mode = true;
           set_target_altitude_current();
           break;

       case FlightMode::CIRCLE:
           // the altitude to circle at is taken from the current altitude
           auto_thrust_mode = true;
           next_WP_loc.alt = current_loc.alt;
           break;

       case FlightMode::AUTO:
        //   hal.console->printf("Auto activated\n");
           auto_thrust_mode = true;
           next_WP_loc = prev_WP_loc = current_loc;
           // start or resume the mission, based on MIS_AUTORESET
           mission.start_or_resume();
           break;

       case FlightMode::RTL:
           // hal.console->printf("RTL activated\n");
           auto_thrust_mode = true;
           prev_WP_loc = current_loc;
           do_RTL();
           break;

       case FlightMode::LOITER:
           auto_thrust_mode = true;
           do_loiter_at_location();
           break;

       case FlightMode::GUIDED:
           auto_thrust_mode = true;
           guided_thrust_passthru = false;
           /*
             when entering guided mode we set the target as the current
             location. This matches the behaviour of the copter code
           */
           guided_WP_loc = current_loc;
           set_guided_WP();
           break;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::control_mode(static_cast<uint8_t>(new_control_mode));
#endif 

    // start with thrust suppressed in auto_thrust modes
    thrust_suppressed = auto_thrust_mode;

    if (should_log(MASK_LOG_MODE)){
        DataFlash.Log_Write_Mode(static_cast<uint8_t>(new_control_mode));
    }
    // reset attitude integrators on mode change
    rollController.reset_I();
    pitchController.reset_I();
    yawController.reset_I();    
    steerController.reset_I();    
}

/*
  set_mode() wrapper for MAVLink SET_MODE
 */
bool Plane::mavlink_set_mode(uint8_t mode)
{
    switch (FlightMode const flight_mode = static_cast<FlightMode>(mode)) {
       case FlightMode::MANUAL:
       case FlightMode::CIRCLE:
       case FlightMode::STABILIZE:
       case FlightMode::TRAINING:
       case FlightMode::ACRO:
       case FlightMode::FLY_BY_WIRE_A:
       case FlightMode::AUTOTUNE:
       case FlightMode::FLY_BY_WIRE_B:
       case FlightMode::CRUISE:
       case FlightMode::GUIDED:
       case FlightMode::AUTO:
       case FlightMode::RTL:
       case FlightMode::LOITER:
           set_mode(flight_mode);
           return true;
       default:
           return false;
    }
}

// exit_mode - perform any cleanup required when leaving a flight mode
void Plane::exit_mode(FlightMode mode)
{
    // stop mission when we leave auto
    if (mode == FlightMode::AUTO) {
        if (mission.state() == AP_Mission::MISSION_RUNNING) {
            mission.stop();

            if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND)
            {
                restart_landing_sequence();
            }
        }
        auto_state.started_flying_in_auto_ms = 0;
    }
}
