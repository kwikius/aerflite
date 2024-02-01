// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif

/*****************************************************************************
*   The init_ardupilot function processes everything we need for an in - air restart
*        We will determine later if we are actually on the ground and process a
*        ground start in that case.
*
*****************************************************************************/

#if CLI_ENABLED == ENABLED

// This is the help function
int8_t Plane::main_menu_help(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Commands:\n"
                         "  logs        log readback/setup mode\n"
                         "  setup       setup mode\n"
                         "  test        test mode\n"
                         "  reboot      reboot to flight mode\n"
                         "\n");
    return(0);
}

// Command/function table for the top-level menu.
static const struct Menu::command main_menu_commands[] = {
//   command		function called
//   =======        ===============
    {"logs",        MENU_FUNC(process_logs)},
    {"setup",       MENU_FUNC(setup_mode)},
    {"test",        MENU_FUNC(test_mode)},
    {"reboot",      MENU_FUNC(reboot_board)},
    {"help",        MENU_FUNC(main_menu_help)},
};

// Create the top-level menu object.
MENU(main_menu, THISFIRMWARE, main_menu_commands);

int8_t Plane::reboot_board(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->reboot(false);
    return 0;
}

// the user wants the CLI. It never exits
void Plane::run_cli(AP_HAL::UARTDriver *port)
{

    // disable the failsafe code in the CLI
    hal.scheduler->register_timer_failsafe(NULL,1);

    // disable the mavlink delay callback
    hal.scheduler->register_delay_callback(NULL, 5);

    cliSerial = port;
    Menu::set_port(port);
    port->set_blocking_writes(true);
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::in_cli);
#endif 
    while (1) {
        main_menu.run();
    }
}

#endif // CLI_ENABLED


static void mavlink_delay_cb_static()
{
    plane.mavlink_delay_cb();
}

void Plane::init_ardupilot()
{
    serial_manager.init_console();

    cliSerial->printf("\n\nInit " FIRMWARE_STRING
                         "\n\nFree RAM: %lu\n",
                        static_cast<unsigned long>(hal.util->available_memory()));

    // loading parameteers
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::loading_eeprom_params);
#endif
    load_parameters();
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::initialising);
#endif
    // parameters loaded

 #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    // load startup params to OSD
    AP_OSD::enqueue::battery_low_voltage(g.fs_batt_voltage.get());
#endif

#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        // set sensors to HIL mode
        ins.set_hil_mode();
        compass.set_hil_mode();
        barometer.set_hil_mode();
    }
#endif

    // initialise serial ports
    serial_manager.init();

    // allow servo set on all channels except first 4
//    ServoRelayEvents.set_channel_mask(0xFFF0);

   // set_control_channels();

    // keep a record of how many resets have happened. This can be
    // used to detect in-flight resets
    g.num_resets.set_and_save(g.num_resets+1);

    // init baro before we start the GCS, so that the CLI baro test works
    barometer.init();

    // initialise rangefinder
    init_rangefinder();

    // initialise battery monitoring
    battery.init();

    // init the GCS
    gcs[0].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_Console, 0);

    // we start by assuming USB connected, as we initialed the serial
    // port with SERIAL0_BAUD. check_usb_mux() fixes this if need be.    
    usb_connected = true;
    check_usb_mux();

   // set up telemetry
   for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; ++i){
       gcs[i].setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i -1);
   }

    // setup frsky

    mavlink_system.sysid = g.sysid_this_mav;

#if LOGGING_ENABLED == ENABLED
    log_init();
#endif

    // initialise airspeed sensor
    airspeed.init();

    if (g.compass_enabled==true) {
        bool compass_ok = compass.init() && compass.update();
#if HIL_SUPPORT
        compass_ok = true;
#endif
        if (!compass_ok) {
            cliSerial->println("Compass initialisation failed!");
            g.compass_enabled = false;
        } else {
            ahrs.set_compass(&compass);
        }
    }
    
#if OPTFLOW == ENABLED
    // make optflow available to libraries
    ahrs.set_optflow(&optflow);
#endif

    // Register mavlink_delay_cb, which will run anytime you have
    // more than 5ms remaining in your call to hal.scheduler->delay
    hal.scheduler->register_delay_callback(mavlink_delay_cb_static, 5);

    // give AHRS the airspeed sensor
    ahrs.set_airspeed(&airspeed);

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
   gps.init(NULL, serial_manager);
    // GPS Initialization
#else
    gps.init(&DataFlash, serial_manager);
#endif

    init_rc_in();               // sets up rc channels from radio
    init_rc_out();              // sets up the timer libs

//    relay.init();

#if FENCE_TRIGGERED_PIN > 0
    hal.gpio->pinMode(FENCE_TRIGGERED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->write(FENCE_TRIGGERED_PIN, 0);
#endif

    /*
     *  setup the 'main loop is dead' check. Note that this relies on
     *  the RC library being initialised.
        DONT use this it isnt robust. TODO better mechanism
     */
    //hal.scheduler->register_timer_failsafe(failsafe_check_static, 1000);

#if CLI_ENABLED == ENABLED
    if (g.cli_enabled == 1) {
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        cliSerial->println(msg);
#if MAVLINK_COMM_NUM_BUFFERS > 1
        if (gcs[1].initialised && (gcs[1].get_uart() != NULL)) {
            gcs[1].get_uart()->println(msg);
        }
#endif
#if MAVLINK_COMM_NUM_BUFFERS > 2
        if (num_gcs > 2 && gcs[2].initialised && (gcs[2].get_uart() != NULL)) {
            gcs[2].get_uart()->println(msg);
        }
#endif
    }

#endif // CLI_ENABLED

    init_capabilities();

    startup_ground();

//    // choose the nav controller
    set_nav_controller();
//
    set_mode((FlightMode)g.initial_mode.get());
//
//    // set the correct flight mode
//    // ---------------------------
  //  reset_control_switch();

 //    read_control_switch();

    // initialise sensor
#if OPTFLOW == ENABLED
    optflow.init();
#endif

}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void Plane::startup_ground(void)
{

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::initialising);
#endif
    set_mode(FlightMode::INITIALISING);

    gcs_send_text(MAV_SEVERITY_INFO,"<startup_ground> Ground start");

#if (GROUND_START_DELAY > 0)
    gcs_send_text(MAV_SEVERITY_NOTICE,"<startup_ground> With delay");
    delay(GROUND_START_DELAY * 1000);
#endif

    // Makes the servos wiggle
    // step 1 = 1 wiggle
    // -----------------------
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        demo_servos(1);
    }

    //INS ground start
    //------------------------
    //
    startup_INS_ground();

    // read the radio to set trims
    // ---------------------------
    if (g.trim_rc_at_start != 0) {
      //  trim_radio();
       bool result = plane.setup_joystick_trims();
       if (!result){
          gcs_send_text(MAV_SEVERITY_INFO,"<startup_ground> Setup joystick trims failed");
       }
    }

    // Save the settings for in-air restart
    // ------------------------------------
    //save_EEPROM_groundstart();

    // initialise mission library
    mission.init();

    // Makes the servos wiggle - 3 times signals ready to fly
    // -----------------------
    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::demo_servos);
#endif
        demo_servos(3);
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::initialising);
#endif
    }

    // reset last heartbeat time, so we don't trigger failsafe on slow
    // startup
    last_mavlink_heartbeat_ms = millis();

    // we don't want writes to the serial port to cause us to pause
    // mid-flight, so set the serial ports non-blocking once we are
    // ready to fly
    serial_manager.set_blocking_writes_all(false);

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    ins.set_raw_logging(false);
    ins.set_dataflash(NULL);
#else
    ins.set_raw_logging(should_log(MASK_LOG_IMU_RAW));
    ins.set_dataflash(&DataFlash);    
#endif
    gcs_send_text(MAV_SEVERITY_INFO,"\n\n Ready to FLY.");

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
    AP_OSD::enqueue::system_status(AP_OSD::system_status_t::running);
#endif
//    for(;;){
//
//      vTaskDelay(100);
//    }
}



//void Plane::check_long_failsafe()
//{
//    uint32_t tnow = millis();
//    // only act on changes
//    // -------------------
//    if(failsafe.state != FAILSAFE_LONG && failsafe.state != FAILSAFE_GCS && flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
//            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH) {
//        if (failsafe.state == FAILSAFE_SHORT &&
//                   (tnow - failsafe.ch3_timer_ms) > g.long_fs_timeout*1000) {
//            failsafe_long_on_event(FAILSAFE_LONG);
//        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_AUTO && get_control_mode() == AUTO &&
//                   last_mavlink_heartbeat_ms != 0 &&
//                   (tnow - last_mavlink_heartbeat_ms) > g.long_fs_timeout*1000) {
//            failsafe_long_on_event(FAILSAFE_GCS);
//        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HEARTBEAT &&
//                   last_mavlink_heartbeat_ms != 0 &&
//                   (tnow - last_mavlink_heartbeat_ms) > g.long_fs_timeout*1000) {
//            failsafe_long_on_event(FAILSAFE_GCS);
//        } else if (g.gcs_heartbeat_fs_enabled == GCS_FAILSAFE_HB_RSSI && 
//                   gcs[0].last_radio_status_remrssi_ms != 0 &&
//                   (tnow - gcs[0].last_radio_status_remrssi_ms) > g.long_fs_timeout*1000) {
//            failsafe_long_on_event(FAILSAFE_GCS);
//        }
//    } else {
//        // We do not change state but allow for user to change mode
//        if (failsafe.state == FAILSAFE_GCS && 
//            (tnow - last_mavlink_heartbeat_ms) < g.short_fs_timeout*1000) {
//            failsafe.state = FAILSAFE_NONE;
//        } else if (failsafe.state == FAILSAFE_LONG && 
//                   !failsafe.ch3_failsafe) {
//            failsafe.state = FAILSAFE_NONE;
//        }
//    }
//}

//void Plane::check_short_failsafe()
//{
//    // only act on changes
//    // -------------------
//    if(failsafe.state == FAILSAFE_NONE && (flight_stage != AP_SpdHgtControl::FLIGHT_LAND_FINAL &&
//            flight_stage != AP_SpdHgtControl::FLIGHT_LAND_APPROACH)) {
//        if(failsafe.ch3_failsafe) { // The condition is checked and the flag ch3_failsafe is set in radio.pde
//            failsafe_short_on_event(FAILSAFE_SHORT);
//        }
//    }
//
//    if(failsafe.state == FAILSAFE_SHORT) {
//        if(!failsafe.ch3_failsafe) {
//            failsafe_short_off_event();
//        }
//    }
//}


void Plane::startup_INS_ground(void)
{
#if HIL_SUPPORT
    if (g.hil_mode == 1) {
        while (barometer.get_last_update() == 0) {
            // the barometer begins updating when we get the first
            // HIL_STATE message
            gcs_send_text(MAV_SEVERITY_WARNING, "Waiting for first HIL_STATE message");
            hal.scheduler->delay(1000);
        }
    }
#endif

    if (ins.gyro_calibration_timing() != AP_InertialSensor::GYRO_CAL_NEVER) {
        gcs_send_text(MAV_SEVERITY_ALERT, "Beginning INS calibration. Do not move plane");
        hal.scheduler->delay(100);
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_vehicle_class(AHRS_VEHICLE_FIXED_WING);
    ahrs.set_wind_estimation(true);

    ins.init(ins_sample_rate);
    ahrs.reset();

    // read Baro pressure at ground
    //-----------------------------
    init_barometer();

    if (airspeed.enabled()) {
        // initialize airspeed sensor
        // --------------------------
        zero_airspeed(true);
    } else {
        gcs_send_text(MAV_SEVERITY_WARNING,"No airspeed");
    }
}

// updates the status of the notify objects
// should be called at 50hz
void Plane::update_notify()
{
    notify.update();
}

void Plane::resetPerfData(void) 
{
    mainLoop_count                  = 0;
    G_Dt_max                        = 0;
    G_Dt_min                        = 0;
    perf_mon_timer                  = millis();
}


void Plane::check_usb_mux(void)
{
    bool usb_check = hal.gpio->usb_connected();
    if (usb_check == usb_connected) {
        return;
    }

    // the user has switched to/from the telemetry port
    usb_connected = usb_check;
}


void Plane::print_flight_mode(AP_HAL::BetterStream *port, uint8_t mode)
{
    FlightMode const flight_mode = static_cast<FlightMode>(mode);
    switch (flight_mode) {
    case FlightMode::MANUAL:
        port->print("Manual");
        break;
    case FlightMode::CIRCLE:
        port->print("Circle");
        break;
    case FlightMode::STABILIZE:
        port->print("Stabilize");
        break;
    case FlightMode::TRAINING:
        port->print("Training");
        break;
    case FlightMode::ACRO:
        port->print("ACRO");
        break;
    case FlightMode::FLY_BY_WIRE_A:
        port->print("FBW_A");
        break;
    case FlightMode::AUTOTUNE:
        port->print("AUTOTUNE");
        break;
    case FlightMode::FLY_BY_WIRE_B:
        port->print("FBW_B");
        break;
    case FlightMode::CRUISE:
        port->print("CRUISE");
        break;
    case FlightMode::AUTO:
        port->print("AUTO");
        break;
    case FlightMode::RTL:
        port->print("RTL");
        break;
    case FlightMode::LOITER:
        port->print("Loiter");
        break;
    case FlightMode::GUIDED:
        port->print("Guided");
        break;
    default:
        port->printf("Mode(%u)", (unsigned)mode);
        break;
    }
}

#if CLI_ENABLED == ENABLED
void Plane::print_comma(void)
{
    cliSerial->print(",");
}
#endif

/*
  write to a servo
  in usec units
 */
//void Plane::servo_write(uint8_t ch, uint16_t pwm)
//{
//#if HIL_SUPPORT
//    if (g.hil_mode==1 && !g.hil_servos) {
//        if (ch < 8) {
//            RC_Channel::rc_channel(ch)->set_output_usec(pwm);
//        }
//        return;
//    }
//#endif
//    hal.rcout->enable_ch(ch);
//    hal.rcout->write(ch, pwm);
//}

/*
  should we log a message type now?
 */
bool Plane::should_log(uint32_t mask)
{
#if LOGGING_ENABLED == ENABLED
    if (!(mask & g.log_bitmask) || in_mavlink_delay) {
        return false;
    }
    bool ret = hal.util->get_soft_armed() || (g.log_bitmask & MASK_LOG_WHEN_DISARMED) != 0;
    if (ret && !DataFlash.logging_started() && !in_log_download) {
        // we have to set in_mavlink_delay to prevent logging while
        // writing headers
        start_logging();
    }
    return ret;
#else
    return false;
#endif
}

/*
  send FrSky telemetry. Should be called at 5Hz by scheduler
 */



/*
  return thrust percentage from 0 to 100
 */
uint8_t Plane::thrust_percentage()const
{
    // to get the real thrust we need to use norm_output() which
    // returns a number from -1 to 1.
    //return constrain_int16(50*(channel_thrust.norm_output()+1), 0, 100);
    return constrain_int16(50*(output_thrust.get()+1), 0, 100);
}

/*
  update AHRS soft arm state and log as needed
 */
void Plane::change_arm_state(void)
{
    Log_Arm_Disarm();
    hal.util->set_soft_armed(arming.is_armed() &&
                             hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
}

/*
  arm motors
 */
bool Plane::arm_motors(AP_Arming::ArmingMethod method)
{
    if (!arming.arm(method)) {
        return false;
    }

    // only log if arming was successful
   // channel_thrust.enable_out();
    output_thrust.enable();
    change_arm_state();
    return true;
}

/*
  disarm motors
 */
bool Plane::disarm_motors(void)
{
    if (!arming.disarm()) {
        return false;
    }
    if (arming.arming_required() == AP_Arming::YES_ZERO_PWM) {
      //  channel_thrust.disable_out(); 
       output_thrust.disable(); 
    }
    if (get_control_mode() != FlightMode::AUTO) {
        // reset the mission on disarm if we are not in auto
        mission.reset();
    }

    // suppress the thrust in auto-thrust modes
    thrust_suppressed = auto_thrust_mode;
    
    //only log if disarming was successful
    change_arm_state();

    return true;
}
