
/*
   Andy Little 2015 - 2018
 
   Authors   Andrew Tridgell, Doug Weibel, Jose Julio, Jordi Munoz, Jason Short, Randy Mackay, Pat Hickey, John Arne Birkeland, Olivier Adler, Amilcar Lucas, Gregory Fletcher, Paul Riseborough, Brandon Jones, Jon Challinger
   Thanks to:  Chris Anderson, Michael Oborne, Paul Mather, Bill Premerlani, James Cohen, JB from rotorFX, Automatik, Fefenin, Peter Meister, Remzibi, Yury Smirnov, Sandro Benigno, Max Levine, Roberto Navoni, Lorenz Meier, Yury MonZon

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

#include "Plane.h"
#include <quan/min.hpp>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif

#define SCHED_TASK(func, _interval_ticks, _max_time_micros) {\
    .function = FUNCTOR_BIND(&plane, &Plane::func, void),\
    AP_SCHEDULER_NAME_INITIALIZER(func)\
    .interval_ticks = _interval_ticks,\
    .max_time_micros = _max_time_micros,\
}

/*
  scheduler table - all regular tasks are listed here, along with how
  often they should be called (in 20ms units) and the maximum time
  they are expected to take (in microseconds)
 */

const AP_Scheduler::Task Plane::scheduler_tasks[] = {
    SCHED_TASK(read_radio,              1,    700),
 //   SCHED_TASK(check_short_failsafe,    1,   1000),
    SCHED_TASK(ahrs_update,             1,   6400),
    SCHED_TASK(update_speed_height,     1,   1600),
    SCHED_TASK(update_flight_mode,      1,   1400),
    SCHED_TASK(stabilize,               1,   3500),
    SCHED_TASK(set_servos,              1,   1600),

 //   SCHED_TASK(read_control_switch,     7,   1000),
    SCHED_TASK(gcs_retry_deferred,      1,   1000),

    SCHED_TASK(update_GPS_50Hz,         1,   2500),
    SCHED_TASK(update_GPS_10Hz,         5,   2500),

    SCHED_TASK(navigate,                5,   3000),
    SCHED_TASK(update_compass,          5,   1200),
    SCHED_TASK(update_airspeed,         5,   1200),
    SCHED_TASK(update_alt,              5,   3400),
///----------------------------------------
    SCHED_TASK(adjust_altitude_target,  5,   1000),

    SCHED_TASK(gcs_update,              1,   1700),
    SCHED_TASK(gcs_data_stream_send,    1,   3000),
    SCHED_TASK(read_battery,            5,   1000),
  //  SCHED_TASK(compass_accumulate,      1,   1500),
    SCHED_TASK(update_notify,           1,    300),
    SCHED_TASK(compass_cal_update,      1,    100),
#if OPTFLOW == ENABLED
    SCHED_TASK(update_optical_flow,     1,    500),
#endif
    SCHED_TASK(one_second_loop,        50,   1000),
  //  SCHED_TASK(check_long_failsafe,    15,   1000),
    SCHED_TASK(read_receiver_rssi,      5,   1000),
    SCHED_TASK(airspeed_ratio_update,  50,   1000),
    SCHED_TASK(update_is_flying_5Hz,   10,    100),
    SCHED_TASK(adsb_update,            50,    500),
};

namespace {

   QUAN_QUANTITY_LITERAL(force,N)
   QUAN_ANGLE_LITERAL(cdeg)
}
// called at start of apm task so task has started
void Plane::setup() 
{
 #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::initialise();
#endif
    cliSerial = hal.console;

    AP_Param::setup_sketch_defaults();

    if (! create_mixer()){
      cliSerial->printf("create mixer failed\n");
      while (1) {asm volatile ("nop":::);}
    }

    AP_Notify::flags.failsafe_battery = false;

    notify.init(false);

    rssi.init();

    init_ardupilot();

    scheduler.init(&scheduler_tasks[0], ARRAY_SIZE(scheduler_tasks));
}

void Plane::loop()
{
    ins.wait_for_sample();

    uint32_t const timer = micros();
    delta_us_fast_loop  = timer - fast_loopTimer_us;
    G_Dt                = delta_us_fast_loop * 1.0e-6f;

    if (delta_us_fast_loop > G_Dt_max && fast_loopTimer_us != 0) {
        G_Dt_max = delta_us_fast_loop;
    }

    if (delta_us_fast_loop < G_Dt_min || G_Dt_min == 0) {
        G_Dt_min = delta_us_fast_loop;
    }
    fast_loopTimer_us = timer;
    ++mainLoop_count;
    scheduler.tick();
    uint32_t const remaining = quan::min( (timer + 20000U) - micros(),19500U);
    scheduler.run(remaining);
}

// update AHRS system
void Plane::ahrs_update()
{
    hal.util->set_soft_armed(arming.is_armed() &&
                   hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // update hil before AHRS update
        gcs_update();
    }
#endif

   ahrs.update();
   #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   AP_OSD::enqueue::attitude({ToDeg(ahrs.pitch),ToDeg(ahrs.roll),ToDeg(ahrs.yaw)});
   #endif

    if (should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_IMU)) {
        Log_Write_IMU();
        DataFlash.Log_Write_IMUDT(ins);
    }

    // calculate a scaled roll limit based on current pitch
    roll_limit_cd = g.roll_limit_cd * cosf(ahrs.pitch);
    pitch_limit_min_cd = aparm.pitch_limit_min_cd * fabsf(cosf(ahrs.roll));

    // update the summed gyro used for ground steering and
    // auto-takeoff. Dot product of DCM.c with gyro vector gives earth
    // frame yaw rate
    steer_state.locked_course_err += ahrs.get_yaw_rate_earth() * G_Dt;
    steer_state.locked_course_err = wrap_PI(steer_state.locked_course_err);
}

/*
  update 50Hz speed/height controller
 */
void Plane::update_speed_height(void)
{
    if (auto_thrust_mode) {
	    // Call TECS 50Hz update. Note that we call this regardless of
	    // thrust suppressed, as this needs to be running for
	    // takeoff detection
        SpdHgt_Controller->update_50hz(tecs_hgt_afe());
    }
}

/*
  read and update compass
 */
void Plane::update_compass(void)
{
    if (g.compass_enabled && compass.update()) {
        ahrs.set_compass(&compass);
        compass.learn_offsets();
        if (should_log(MASK_LOG_COMPASS)) {
           DataFlash.Log_Write_Compass(compass);
        }
    } else {
        ahrs.set_compass(NULL);
    }
}

/*
  if the compass is enabled then try to accumulate a reading
 */
//void Plane::compass_accumulate(void)
//{
//    if (g.compass_enabled) {
//        compass.accumulate();
//    }    
//}

/*
  do 10Hz logging
 */
void Plane::update_logging1(void)
{
    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_ATTITUDE_FAST)) {
        Log_Write_Attitude();
    }

    if (should_log(MASK_LOG_ATTITUDE_MED) && !should_log(MASK_LOG_IMU))
        Log_Write_IMU();
}

/*
  do 10Hz logging - part2
 */
void Plane::update_logging2(void)
{
    if (should_log(MASK_LOG_CTUN))
        Log_Write_Control_Tuning();
    
    if (should_log(MASK_LOG_NTUN))
        Log_Write_Nav_Tuning();

    if (should_log(MASK_LOG_RC))
        Log_Write_RC();

    if (should_log(MASK_LOG_IMU))
        DataFlash.Log_Write_Vibration(ins);
}

void Plane::one_second_loop()
{
    if (should_log(MASK_LOG_CURRENT))
        Log_Write_Current();

    // send a heartbeat
    gcs_send_message(MSG_HEARTBEAT);

    // make it possible to change control channel ordering at runtime
   // set_control_channels();

    // make it possible to change orientation at runtime
    ahrs.set_orientation();

    // sync MAVLink system ID
    mavlink_system.sysid = g.sysid_this_mav;

   // update_aux();

    // update notify flags
    AP_Notify::flags.pre_arm_check = arming.pre_arm_checks(false);
    AP_Notify::flags.pre_arm_gps_check = true;
    AP_Notify::flags.armed = arming.is_armed() || arming.arming_required() == AP_Arming::NO;

    crash_detection_update();

#if AP_TERRAIN_AVAILABLE
    if (should_log(MASK_LOG_GPS)) {
        terrain.log_terrain_data(DataFlash);
    }
#endif
    // piggyback the status log entry on the MODE log entry flag
    if (should_log(MASK_LOG_MODE)) {
        Log_Write_Status();
    }

    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
}

void Plane::log_perf_info()
{
    if (scheduler.debug() != 0) {
        gcs_send_text_fmt(MAV_SEVERITY_INFO, "G_Dt_max=%lu G_Dt_min=%lu\n",
                          (unsigned long)G_Dt_max, 
                          (unsigned long)G_Dt_min);
    }

    if (should_log(MASK_LOG_PM)) {
        Log_Write_Performance();
    }

    G_Dt_max = 0;
    G_Dt_min = 0;
    resetPerfData();
}

void Plane::compass_save()
{
    if (g.compass_enabled) {
        compass.save_offsets();
    }
}

void Plane::terrain_update(void)
{
#if AP_TERRAIN_AVAILABLE
    terrain.update();
#endif
}


void Plane::dataflash_periodic(void)
{
    DataFlash.periodic_tasks();
}

/*
  once a second update the airspeed calibration ratio
 */
void Plane::airspeed_ratio_update(void)
{
    if (!airspeed.enabled() ||
        gps.status() < AP_GPS::GPS_OK_FIX_3D ||
        gps.ground_speed() < 4) {
        // don't calibrate when not moving
        return;        
    }
    if (airspeed.get_airspeed() < aparm.airspeed_min && 
        gps.ground_speed() < (uint32_t)aparm.airspeed_min) {
        // don't calibrate when flying below the minimum airspeed. We
        // check both airspeed and ground speed to catch cases where
        // the airspeed ratio is way too low, which could lead to it
        // never coming up again
        return;
    }
    if (labs(ahrs.roll_sensor) > roll_limit_cd ||
        ahrs.pitch_sensor > aparm.pitch_limit_max_cd ||
        ahrs.pitch_sensor < pitch_limit_min_cd) {
        // don't calibrate when going beyond normal flight envelope
        return;
    }
    const Vector3f &vg = gps.velocity();
    airspeed.update_calibration(vg);
    gcs_send_airspeed_calibration(vg);
}


/*
  read the GPS and update position
 */
void Plane::update_GPS_50Hz(void)
{
    static uint32_t last_gps_reading[GPS_MAX_INSTANCES];
    gps.update();

    for (uint8_t i=0; i<gps.num_sensors(); i++) {
        if (gps.last_message_time_ms(i) != last_gps_reading[i]) {
            last_gps_reading[i] = gps.last_message_time_ms(i);
            if (should_log(MASK_LOG_GPS)) {
                Log_Write_GPS(i);
            }
        }
    }
}

/*
  read update GPS position - 10Hz update
 */
void Plane::update_GPS_10Hz(void)
{
    // get position from AHRS
    have_position = ahrs.get_position(current_loc);

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

    AP_OSD::gps_info_t gps_info;
    gps_info.ground_speed_m_per_s   = gps.ground_speed();
    gps_info.ground_course_cd       = gps.ground_course_cd();
    gps_info.num_sats               = gps.num_sats();
    gps_info.status                 = gps.status();

    AP_OSD::enqueue::gps_status(gps_info);
    if ( have_position){
      AP_OSD::enqueue::gps_location({gps.location().lat,gps.location().lng,gps.location().alt});
    }

#endif

    static uint32_t last_gps_msg_ms;
    if (gps.last_message_time_ms() != last_gps_msg_ms && gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
        last_gps_msg_ms = gps.last_message_time_ms();

        if (ground_start_count > 1) {
            ground_start_count--;
        } else if (ground_start_count == 1) {
            // We countdown N number of good GPS fixes
            // so that the altitude is more accurate
            // -------------------------------------
            if (current_loc.lat == 0) {
                ground_start_count = 5;

            } else {
                init_home();

                // set system clock for log timestamps
                hal.util->set_system_clock(gps.time_epoch_usec());

                if (g.compass_enabled) {
                    // Set compass declination automatically
                    const Location &loc = gps.location();
                    compass.set_initial_location(loc.lat, loc.lng);
                }
                ground_start_count = 0;
            }
        }
#if GEOFENCE_ENABLED == ENABLED
        // see if we've breached the geo-fence
        geofence_check(false);
 #endif
        if (!hal.util->get_soft_armed()) {
            update_home();
        }
        // update wind estimate
        ahrs.estimate_wind();
    }

    calc_gndspeed_undershoot();
}

/*
  main handling for AUTO mode
 */
void Plane::handle_auto_mode(void)
{
    uint8_t nav_cmd_id;

    // we should be either running a mission or RTLing home
    if (mission.state() == AP_Mission::MISSION_RUNNING) {
        nav_cmd_id = mission.get_current_nav_cmd().id;
    }else{
        nav_cmd_id = auto_rtl_command.id;
    }

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT)) {
        takeoff_calc_roll();
        takeoff_calc_pitch();
        calc_thrust();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        calc_nav_roll();
        calc_nav_pitch();
        
        if (auto_state.land_complete) {
            // during final approach constrain roll to the range
            // allowed for level flight
            nav_roll_cd = constrain_int32(nav_roll_cd, -g.level_roll_limit*100UL, g.level_roll_limit*100UL);
        }
        calc_thrust();
        
        if (auto_state.land_complete) {
            // we are in the final stage of a landing - force
            // zero thrust
           autopilot_thrust.set_force(0_N);
        }
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            steer_state.hold_course_cd = -1;
        }
        auto_state.land_complete = false;
        calc_nav_roll();
        calc_nav_pitch();
        calc_thrust();
    }
}

/*
  main flight mode dependent update code 
  updates nav_roll_cd and nav_pitch_cd
  and updates thrust directly
 */
void Plane::update_flight_mode(void)
{
    FlightMode effective_mode = get_control_mode();
    if (effective_mode == FlightMode::AUTO && g.auto_fbw_steer) {
        effective_mode = FlightMode::FLY_BY_WIRE_A;
    }

    if (effective_mode != FlightMode::AUTO) {
        // hold_course is only used in takeoff and landing
        steer_state.hold_course_cd = -1;
    }

    switch (effective_mode)  {
    case FlightMode::AUTO:
        handle_auto_mode();
        break;

    case FlightMode::RTL:
    case FlightMode::LOITER:
    case FlightMode::GUIDED:
        calc_nav_roll();
        calc_nav_pitch();
        calc_thrust();
        break;
        
    case FlightMode::TRAINING: {
        training_manual_roll = false;
        training_manual_pitch = false;
        update_load_factor();
        
        // if the roll is past the set roll limit, then
        // we set target roll to the limit
        if (ahrs.roll_sensor >= roll_limit_cd) {
            nav_roll_cd = roll_limit_cd;
        } else if (ahrs.roll_sensor <= -roll_limit_cd) {
            nav_roll_cd = -roll_limit_cd;                
        } else {
            training_manual_roll = true;
            nav_roll_cd = 0;
        }
        
        // if the pitch is past the set pitch limits, then
        // we set target pitch to the limit
        if (ahrs.pitch_sensor >= aparm.pitch_limit_max_cd) {
            nav_pitch_cd = aparm.pitch_limit_max_cd;
        } else if (ahrs.pitch_sensor <= pitch_limit_min_cd) {
            nav_pitch_cd = pitch_limit_min_cd;
        } else {
            training_manual_pitch = true;
            nav_pitch_cd = 0;
        }
        break;
    }

    case FlightMode::ACRO: {
        // handle locked/unlocked control
        if (acro_state.locked_roll) {
            nav_roll_cd = acro_state.locked_roll_err;
        } else {
            nav_roll_cd = ahrs.roll_sensor;
        }
        if (acro_state.locked_pitch) {
            nav_pitch_cd = acro_state.locked_pitch_cd;
        } else {
            nav_pitch_cd = ahrs.pitch_sensor;
        }
        break;
    }

    case FlightMode::AUTOTUNE:
    case FlightMode::FLY_BY_WIRE_A: {
        // set nav_roll and nav_pitch using sticks
        nav_roll_cd = joystick_roll.as_float() * roll_limit_cd;
        update_load_factor();
        float const pitch_input = joystick_pitch.as_float();
        if (pitch_input > 0) {
            nav_pitch_cd = pitch_input * aparm.pitch_limit_max_cd;
        } else {
            nav_pitch_cd = -(pitch_input * pitch_limit_min_cd);
        }
        adjust_nav_pitch_thrust();
        nav_pitch_cd = constrain_int32(nav_pitch_cd, pitch_limit_min_cd, aparm.pitch_limit_max_cd.get());

//        if (failsafe.ch3_failsafe && g.short_fs_action == 2) {
//            // FBWA failsafe glide
//            nav_roll_cd = 0;
//            nav_pitch_cd = 0;
//            autopilot_thrust.set_force(0_N);
//        }
        if (g.fbwa_tdrag_chan > 0) {
            // check for the user enabling FBWA taildrag takeoff mode
            bool tdrag_mode = (hal.rcin->read(g.fbwa_tdrag_chan-1) > 1700);
            if (tdrag_mode && !auto_state.fbwa_tdrag_takeoff_mode) {
                if (auto_state.highest_airspeed < g.takeoff_tdrag_speed1) {
                    auto_state.fbwa_tdrag_takeoff_mode = true;
                    gcs_send_text(MAV_SEVERITY_WARNING, "FBWA tdrag mode");
                }
            }
        }
        break;
    }

    case FlightMode::FLY_BY_WIRE_B:
        // Thanks to Yury MonZon for the altitude limit code!
        nav_roll_cd = joystick_roll.as_float() * roll_limit_cd;
        nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
        update_load_factor();
        update_fbwb_speed_height();
        break;
        
    case FlightMode::CRUISE:
        /*
          in CRUISE mode we use the navigation code to control
          roll when heading is locked. Heading becomes unlocked on
          any aileron or rudder input
        */

        if( (joystick_roll.as_angle() != 0_cdeg )  || 
            (joystick_yaw.as_angle() != 0_cdeg ) ){
            cruise_state.locked_heading = false;
            cruise_state.lock_timer_ms = 0;
        }
        if (!cruise_state.locked_heading) {
            nav_roll_cd = joystick_roll.as_float() * roll_limit_cd;
            nav_roll_cd = constrain_int32(nav_roll_cd, -roll_limit_cd, roll_limit_cd);
            update_load_factor();
        } else {
            calc_nav_roll();
        }
        update_fbwb_speed_height();
        break;
        
    case FlightMode::STABILIZE:
        nav_roll_cd = 0;
        nav_pitch_cd = 0;
        // thrust is passthrough
        break;
        
    case FlightMode::CIRCLE:
        // we have no GPS installed and have lost radio contact
        // or we just want to fly around in a gentle circle w/o GPS,
        // holding altitude at the altitude we set when we
        // switched into the mode
        nav_roll_cd = roll_limit_cd / 3;
        update_load_factor();
        calc_nav_pitch();
        calc_thrust();
        break;

    case FlightMode::MANUAL:
         autopilot_roll.set_js(joystick_roll);
         autopilot_pitch.set_js(joystick_pitch);
         autopilot_yaw.set_js(joystick_yaw);
        break;
        
    case FlightMode::INITIALISING:
        // handled elsewhere
        break;
    }
}

void Plane::update_navigation()
{
    // wp_distance is in ACTUAL meters, not the *100 meters we get from the GPS
    // ------------------------------------------------------------------------

    // distance and bearing calcs only
    switch(get_control_mode()) {
    case FlightMode::AUTO:
        update_commands();
        break;
            
    case FlightMode::RTL:
        if (g.rtl_autoland == 1 &&
            !auto_state.checked_for_autoland &&
            nav_controller->reached_loiter_target() && 
            labs(altitude_error_cm) < 1000) {
            // we've reached the RTL point, see if we have a landing sequence
            jump_to_landing_sequence();

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        else if (g.rtl_autoland == 2 &&
            !auto_state.checked_for_autoland) {
            // Go directly to the landing sequence
            jump_to_landing_sequence();

            // prevent running the expensive jump_to_landing_sequence
            // on every loop
            auto_state.checked_for_autoland = true;
        }
        // fall through to LOITER
    case FlightMode::LOITER:
    case FlightMode::GUIDED:
        // allow loiter direction to be changed in flight
        if (g.loiter_radius < 0) {
            loiter.direction = -1;
        } else {
            loiter.direction = 1;
        }
        update_loiter();
        break;

    case FlightMode::CRUISE:
        update_cruise();
        break;

    case FlightMode::MANUAL:
    case FlightMode::STABILIZE:
    case FlightMode::TRAINING:
    case FlightMode::INITIALISING:
    case FlightMode::ACRO:
    case FlightMode::FLY_BY_WIRE_A:
    case FlightMode::AUTOTUNE:
    case FlightMode::FLY_BY_WIRE_B:
    case FlightMode::CIRCLE:
        // nothing to do
        break;
    }
}

/*
  set the flight stage
 */
void Plane::set_flight_stage(AP_SpdHgtControl::FlightStage fs) 
{
    if (fs == flight_stage) {
        return;
    }

    switch (fs) {
    case AP_SpdHgtControl::FLIGHT_LAND_APPROACH:
#if GEOFENCE_ENABLED == ENABLED 
        if (g.fence_autoenable == 1) {
            if (! geofence_set_enabled(false, AUTO_TOGGLED)) {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence failed (autodisable)");
            } else {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Fence disabled (autodisable)");
            }
        } else if (g.fence_autoenable == 2) {
            if (! geofence_set_floor_enabled(false)) {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Disable fence floor failed (autodisable)");
            } else {
                gcs_send_text(MAV_SEVERITY_NOTICE, "Fence floor disabled (auto disable)");
            }
        }
#endif
        break;

    case AP_SpdHgtControl::FLIGHT_LAND_ABORT:
        gcs_send_text_fmt(MAV_SEVERITY_NOTICE, "Landing aborted via thrust. Climbing to %dm", auto_state.takeoff_altitude_rel_cm/100);
        break;

    case AP_SpdHgtControl::FLIGHT_LAND_FINAL:
    case AP_SpdHgtControl::FLIGHT_NORMAL:
    case AP_SpdHgtControl::FLIGHT_TAKEOFF:
        break;
    }
    flight_stage = fs;
}

void Plane::update_alt()
{
    barometer.update();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }

    // calculate the sink rate.
    float sink_rate;
    Vector3f vel;
    if (ahrs.get_velocity_NED(vel)) {
        sink_rate = vel.z;
    } else if (gps.status() >= AP_GPS::GPS_OK_FIX_3D && gps.have_vertical_velocity()) {
        sink_rate = gps.velocity().z;
    } else {
        sink_rate = -barometer.get_climb_rate();        
    }

    // low pass the sink rate to take some of the noise out
    auto_state.sink_rate = 0.8f * auto_state.sink_rate + 0.2f*sink_rate;
#if GEOFENCE_ENABLED == ENABLED
    geofence_check(true);
#endif

    update_flight_stage();
}

/*
  recalculate the flight_stage
 */
void Plane::update_flight_stage(void)
{
    // Update the speed & height controller states
    if (auto_thrust_mode && !thrust_suppressed) {        
        if (get_control_mode() == FlightMode::AUTO) {
            if (auto_state.takeoff_complete == false) {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_TAKEOFF);
            } else if (mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {
                if ((g.land_abort_thrust_enable && joystick_thrust.as_force() > 95_N) ||
                        flight_stage == AP_SpdHgtControl::FLIGHT_LAND_ABORT){
                    // abort mode is sticky, it must complete while executing NAV_LAND
                    set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_ABORT);
                } else if (auto_state.land_complete == true) {
                    set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_FINAL);
                } else if (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
                    float path_progress = location_path_proportion(current_loc, prev_WP_loc, next_WP_loc);
                    bool lined_up = abs(nav_controller->bearing_error_cd()) < 1000;
                    bool below_prev_WP = current_loc.alt < prev_WP_loc.alt;
                    if ((path_progress > 0.15f && lined_up && below_prev_WP) || path_progress > 0.5f) {
                        set_flight_stage(AP_SpdHgtControl::FLIGHT_LAND_APPROACH);
                    } else {
                        set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);                        
                    }
                }

            } else {
                set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
            }
        } else {
            set_flight_stage(AP_SpdHgtControl::FLIGHT_NORMAL);
        }

        SpdHgt_Controller->update_pitch_thrust(relative_target_altitude_cm(),
                                                 target_airspeed_cm,
                                                 flight_stage,
                                                 auto_state.takeoff_pitch_cd,
                                                 thrust_nudge,
                                                 tecs_hgt_afe(),
                                                 aerodynamic_load_factor);
        if (should_log(MASK_LOG_TECS)) {
            Log_Write_TECS_Tuning();
        }
    }
    // tell AHRS the airspeed to true airspeed ratio
    airspeed.set_EAS2TAS(barometer.get_EAS2TAS());
}

#if OPTFLOW == ENABLED
// called at 50hz
void Plane::update_optical_flow(void)
{
    static uint32_t last_of_update = 0;

    // exit immediately if not enabled
    if (!optflow.enabled()) {
        return;
    }

    // read from sensor
    optflow.update();

    // write to log and send to EKF if new data has arrived
    if (optflow.last_update() != last_of_update) {
        last_of_update = optflow.last_update();
        uint8_t flowQuality = optflow.quality();
        Vector2f flowRate = optflow.flowRate();
        Vector2f bodyRate = optflow.bodyRate();
        ahrs.writeOptFlowMeas(flowQuality, flowRate, bodyRate, last_of_update);
        Log_Write_Optflow();
    }
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
void setup(void);
void loop(void);

void setup(void)
{
    plane.setup();
}
void loop(void)
{
    plane.loop();
}

AP_HAL_MAIN();
#else
AP_HAL_MAIN_CALLBACKS(&plane);
#endif

