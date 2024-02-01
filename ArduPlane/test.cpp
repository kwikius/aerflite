// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

#if CLI_ENABLED == ENABLED

// Creates a constant array of structs representing menu options
// and stores them in Flash memory, not RAM.
// User enters the string in the console to call the functions on the right.
// See class Menu in AP_Common for implementation details
static const struct Menu::command test_menu_commands[] = {
    {"pwm",                 MENU_FUNC(test_radio_pwm)},
    {"radio",               MENU_FUNC(test_radio)},
  //  {"passthru",            MENU_FUNC(test_passthru)},
    {"failsafe",            MENU_FUNC(test_failsafe)},
   // {"relay",               MENU_FUNC(test_relay)},
    {"waypoints",           MENU_FUNC(test_wp)},
    {"xbee",                MENU_FUNC(test_xbee)},
    {"modeswitch",          MENU_FUNC(test_modeswitch)},

    // Tests below here are for hardware sensors only present
    // when real sensors are attached or they are emulated
    {"gps",                 MENU_FUNC(test_gps)},
    {"ins",                 MENU_FUNC(test_ins)},
    {"airspeed",            MENU_FUNC(test_airspeed)},
    {"airpressure",         MENU_FUNC(test_pressure)},
    {"compass",             MENU_FUNC(test_mag)},
    {"logging",             MENU_FUNC(test_logging)},
};

// A Macro to create the Menu
MENU(test_menu, "test", test_menu_commands);

int8_t Plane::test_mode(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Test Mode\n\n");
    test_menu.run();
    return 0;
}

void Plane::print_hit_enter()
{
    cliSerial->printf("Hit Enter to exit.\n\n");
}

int8_t Plane::test_radio_pwm(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);

    while(1) {
        hal.scheduler->delay(20);

        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        cliSerial->printf("roll: %3u pitch: %3u yaw: %3u thrust: %3u\n",
                        static_cast<unsigned>(joystick_roll.as_usec().numeric_value()),
                        static_cast<unsigned>(joystick_pitch.as_usec().numeric_value()),
                        static_cast<unsigned>(joystick_yaw.as_usec().numeric_value()),
                        static_cast<unsigned>(joystick_thrust.as_usec().numeric_value())
       );

       if(cliSerial->available() > 0) {
         return (0);
       }
    }
}


//int8_t Plane::test_passthru(uint8_t argc, const Menu::arg *argv)
//{
//    print_hit_enter();
//    hal.scheduler->delay(1000);
//
//    while(1) {
//        hal.scheduler->delay(20);
//
//        // New radio frame? (we could use also if((millis()- timer) > 20)
//        if (hal.rcin->new_input()) {
//            cliSerial->print("CH:");
//            for(int16_t i = 0; i < 8; i++) {
//                cliSerial->print(hal.rcin->read(i));        // Print channel values
//                print_comma();
//                servo_write(i, hal.rcin->read(i)); // Copy input to Servos
//            }
//            cliSerial->println();
//        }
//        if (cliSerial->available() > 0) {
//            return (0);
//        }
//    }
//    return 0;
//}

int8_t Plane::test_radio(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);

    // read the radio to set trims
    // ---------------------------
   // trim_radio();
    if (!setup_joystick_trims()){
         cliSerial->printf("failed to set joystick trims\n");
    }
    while(1) {
        cliSerial->printf("TODO\n");
        hal.scheduler->delay(200);
//        read_radio();
//
//        channel_roll.calc_output_from_temp_output();
//        channel_pitch.calc_output_from_temp_output();
//        channel_thrust.calc_output_from_temp_output();
//        channel_yaw.calc_output_from_temp_output();
//
//        // write out the servo PWM values
//        // ------------------------------
//        set_servos();
//
//        cliSerial->printf("IN 1: %d\t2: %d\t3: %d\t4: %d\n",
//                        (int)channel_roll.get_control_in(),
//                        (int)channel_pitch.get_control_in(),
//                        (int)channel_thrust.get_control_in(),
//                        (int)channel_yaw.get_control_in());
//
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}
namespace {

   QUAN_QUANTITY_LITERAL(force, N)
}

int8_t Plane::test_failsafe(uint8_t argc, const Menu::arg *argv)
{
    uint8_t fail_test;
    print_hit_enter();
    for(int16_t i = 0; i < 50; i++) {
        hal.scheduler->delay(20);
        read_radio();
    }

    // read the radio to set trims
    // ---------------------------
  //  trim_radio();

    if (!setup_joystick_trims()){
       cliSerial->printf("failed to set joystick trims\n");
    }

    FlightMode const oldSwitchPosition = readControlSwitch();

    cliSerial->printf("Unplug battery, thrust in neutral, turn off radio.\n");

    while(joystick_thrust.as_force() > 0_N) {
        hal.scheduler->delay(20);
        read_radio();
    }

    while(1) {
        hal.scheduler->delay(20);
        read_radio();

        auto const thrust_force = joystick_thrust.as_force();
        if( thrust_force > 0_N) {
            cliSerial->printf("THROTTLE CHANGED %d \n", (int)thrust_force.numeric_value());
            fail_test++;
        }
        auto const new_switch_position = readControlSwitch();
        if(new_switch_position != oldSwitchPosition) {
            cliSerial->printf("CONTROL MODE CHANGED: ");
            print_flight_mode(cliSerial, static_cast<uint8_t>(new_switch_position));
            cliSerial->println();
            fail_test++;
        }

        if(in_rcin_failsafe()) {
            cliSerial->printf("THROTTLE FAILSAFE ACTIVATED: %d, ", (int)joystick_thrust.as_usec().numeric_value());
            print_flight_mode(cliSerial, static_cast<uint8_t>(readControlSwitch()));
            cliSerial->println();
            fail_test++;
        }

        if(fail_test > 0) {
            return (0);
        }
        if(cliSerial->available() > 0) {
            cliSerial->printf("LOS caused no change in APM.\n");
            return (0);
        }
    }
}

//int8_t Plane::test_relay(uint8_t argc, const Menu::arg *argv)
//{
//    print_hit_enter();
//    hal.scheduler->delay(1000);
//
//    while(1) {
//        cliSerial->printf("Relay on\n");
//        relay.on(0);
//        hal.scheduler->delay(3000);
//        if(cliSerial->available() > 0) {
//            return (0);
//        }
//
//        cliSerial->printf("Relay off\n");
//        relay.off(0);
//        hal.scheduler->delay(3000);
//        if(cliSerial->available() > 0) {
//            return (0);
//        }
//    }
//}

int8_t Plane::test_wp(uint8_t argc, const Menu::arg *argv)
{
    hal.scheduler->delay(1000);

    // save the alitude above home option
    if (g.RTL_altitude_cm < 0) {
        cliSerial->printf("Hold current altitude\n");
    }else{
        cliSerial->printf("Hold altitude of %dm\n", (int)g.RTL_altitude_cm/100);
    }

    cliSerial->printf("%d waypoints\n", (int)mission.num_commands());
    cliSerial->printf("Hit radius: %d\n", (int)g.waypoint_radius);
    cliSerial->printf("Loiter radius: %d\n\n", (int)g.loiter_radius);

    for(uint8_t i = 0; i <= mission.num_commands(); i++) {
        AP_Mission::Mission_Command temp_cmd;
        if (mission.read_cmd_from_storage(i,temp_cmd)) {
            test_wp_print(temp_cmd);
        }
    }

    return (0);
}

void Plane::test_wp_print(const AP_Mission::Mission_Command& cmd)
{
    cliSerial->printf("command #: %d id:%d options:%d p1:%d p2:%ld p3:%ld p4:%ld \n",
                    (int)cmd.index,
                    (int)cmd.id,
                    (int)cmd.content.location.options,
                    (int)cmd.p1,
                    (long)cmd.content.location.alt,
                    (long)cmd.content.location.lat,
                    (long)cmd.content.location.lng);
}

int8_t Plane::test_xbee(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);
    cliSerial->printf("Begin XBee X-CTU Range and RSSI Test:\n");

    while(1) {

        if (hal.uartC->available())
            hal.uartC->write(hal.uartC->read());

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}


int8_t Plane::test_modeswitch(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);

    cliSerial->printf("Control CH ");
    cliSerial->println(FLIGHT_MODE_CHANNEL, BASE_DEC);

    FlightMode oldSwitchPosition = readControlSwitch();
    while(1) {
        // quit if any key pressed
        if(cliSerial->available() > 0) {
            return (0);
        }
        hal.scheduler->delay(20);
        auto const newSwitchPosition = readControlSwitch();
        if (newSwitchPosition != oldSwitchPosition ) {
            cliSerial->printf("Position %d\n",  (int)newSwitchPosition);
            oldSwitchPosition = newSwitchPosition;
        }

    }
}

/*
 *  test the dataflash is working
 */
int8_t Plane::test_logging(uint8_t argc, const Menu::arg *argv)
{
    DataFlash.ShowDeviceInfo(cliSerial);
    return 0;
}

//-------------------------------------------------------------------------------------------
// tests in this section are for real sensors or sensors that have been simulated

int8_t Plane::test_gps(uint8_t argc, const Menu::arg *argv)
{
    print_hit_enter();
    hal.scheduler->delay(1000);

    uint32_t last_message_time_ms = 0;
    while(1) {
        hal.scheduler->delay(100);

        gps.update();

        if (gps.last_message_time_ms() != last_message_time_ms) {
            last_message_time_ms = gps.last_message_time_ms();
            const Location &loc = gps.location();
            cliSerial->printf("Lat: %ld, Lon %ld, Alt: %ldm, #sats: %d\n",
                                (long)loc.lat,
                                (long)loc.lng,
                                (long)loc.alt/100,
                                (int)gps.num_sats());
        } else {
            cliSerial->printf(".");
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

int8_t Plane::test_ins(uint8_t argc, const Menu::arg *argv)
{
    //cliSerial->printf("Calibrating.");
    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);

    ins.init(ins_sample_rate);
    ahrs.reset();

    print_hit_enter();
    hal.scheduler->delay(1000);

    uint8_t counter = 0;

    while(1) {
        hal.scheduler->delay(20);
        if (micros() - fast_loopTimer_us > 19000UL) {
            fast_loopTimer_us       = micros();

            // INS
            // ---
            ahrs.update();

            if(g.compass_enabled) {
                counter++;
                if(counter == 5) {
                    compass.update();
                    counter = 0;
                }
            }

            // We are using the INS
            // ---------------------
            Vector3f gyros  = ins.get_gyro();
            Vector3f accels = ins.get_accel();
            cliSerial->printf("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n",
                            (int)ahrs.roll_sensor / 100,
                            (int)ahrs.pitch_sensor / 100,
                            (uint16_t)ahrs.yaw_sensor / 100,
                            (double)gyros.x, (double)gyros.y, (double)gyros.z,
                            (double)accels.x, (double)accels.y, (double)accels.z);
        }
        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

namespace {

}

int8_t Plane::test_mag(uint8_t argc, const Menu::arg *argv)
{
    if (!g.compass_enabled) {
        cliSerial->printf("Compass: ");
        print_enabled(false);
        return (0);
    }

    if (!compass.init()) {
        cliSerial->println("Compass initialisation failed!");
        return 0;
    }
    bool disable_compass_learn_on_exit = false;

    if ( !compass.learn_offsets_enabled()){
       // enable learn temporarily
       compass.set_learn_offsets(true);
       disable_compass_learn_on_exit = true;
    }

    ahrs.init();
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_compass(&compass);

    // we need the AHRS initialised for this test
    ins.init(ins_sample_rate);
    ahrs.reset();

    uint16_t counter = 0;
    float heading = 0;

    print_hit_enter();
    // flush input buffer
    while(cliSerial->available() > 0){
       char const ch = static_cast<char>(cliSerial->read());
       (void)ch;
    }

   while(cliSerial->available() == 0) {
      hal.scheduler->delay(20);
      if (micros() - fast_loopTimer_us > 19000UL) {
         fast_loopTimer_us = micros();
         ahrs.update();
         if(counter % 5 == 0) {
            if (compass.update()) {
               // Calculate heading
               const Matrix3f &m = ahrs.get_dcm_matrix();
               heading = compass.calculate_heading(m);
               compass.learn_offsets();
            }
         }
         ++counter;
         if (counter > 20) {
            if (compass.healthy()) {
              const Vector3f &mag_ofs = compass.get_offsets();
              const Vector3f &mag = compass.get_field();
              cliSerial->printf("Heading: %ld, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n",
                  static_cast<long int>((wrap_360_cd(ToDeg(heading) * 100)) /100),
                     (double)mag.x, (double)mag.y, (double)mag.z,
                     (double)mag_ofs.x, (double)mag_ofs.y, (double)mag_ofs.z);
            } else {
               cliSerial->println("compass not healthy");
            }
            counter=0;
         }
      }
   }

    // restore old state
    if ( disable_compass_learn_on_exit == true){
       compass.set_learn_offsets(false);
       disable_compass_learn_on_exit = false;
    }
    // flush input buffer
    while(cliSerial->available() > 0){
       char const ch = static_cast<char>(cliSerial->read());
        (void)ch;
    }
    cliSerial->println("Save new offsets to eeprom y/n ?");

    while (! cliSerial->available() ) {
        asm volatile ("nop":::);
    }
//------
    {
       char const ch = static_cast<char>(cliSerial->read());
       if ( (ch == 'Y') || ( ch == 'y')){
          cliSerial->println("Saving new offsets");
          compass.save_offsets();
       }else{
          cliSerial->println("New offsets not saved");
       }
    }
    // flush input buffer
    while(cliSerial->available() > 0){
       char const ch = static_cast<char>(cliSerial->read());
       (void)ch;
    }
    return (0);
}

//-------------------------------------------------------------------------------------------
// real sensors that have not been simulated yet go here

int8_t Plane::test_airspeed(uint8_t argc, const Menu::arg *argv)
{
    if (!airspeed.enabled()) {
        cliSerial->printf("airspeed: ");
        print_enabled(false);
        return (0);
    }else{
        print_hit_enter();
        zero_airspeed(false);
        cliSerial->printf("airspeed: ");
        print_enabled(true);

        while(1) {
            hal.scheduler->delay(20);
            update_airspeed();
            cliSerial->printf("%.1f m/s\n", (double)airspeed.get_airspeed());

            if(cliSerial->available() > 0) {
                return (0);
            }
        }
    }
}

int8_t Plane::test_pressure(uint8_t argc, const Menu::arg *argv)
{
    cliSerial->printf("Uncalibrated relative airpressure\n");
    print_hit_enter();

    init_barometer();

    while(1) {
        hal.scheduler->delay(100);
        barometer.update();

        if (!barometer.healthy()) {
            cliSerial->println("not healthy");
        } else {
            cliSerial->printf("Alt: %0.2fm, Raw: %f Temperature: %.1f\n",
                                (double)barometer.get_altitude(),
                                (double)barometer.get_pressure(),
                                (double)barometer.get_temperature());
        }

        if(cliSerial->available() > 0) {
            return (0);
        }
    }
}

void Plane::print_enabled(bool b)
{
    if (b) {
        cliSerial->printf("en");
    } else {
        cliSerial->printf("dis");
    }
    cliSerial->printf("abled\n");
}

#endif // CLI_ENABLED
