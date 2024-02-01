
//
// Simple test for the AP_AHRS interface
//
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_AHRS/AP_AHRS_DCM.h>
#include <AP_OSD/AP_OSD_enqueue.h>
#include <AP_BattMonitor/AP_BattMonitor.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

   AP_InertialSensor             ins;
   Compass                       compass;
   AP_GPS                        gps;
   AP_Baro                       baro;
   AP_SerialManager              serial_manager;
   AP_BattMonitor                battery_monitor;
   AP_Vehicle::FixedWing         aparm;
   AP_Airspeed                   airspeed(aparm);
   AP_AHRS_DCM                   ahrs(ins, baro, gps);
}

void setup(void)
{
    AP_Param::set_object_value(&battery_monitor, battery_monitor.var_info, "_VOLT_PIN", 3);
    AP_Param::set_object_value(&battery_monitor, battery_monitor.var_info, "_CURR_PIN", 2);
    AP_Param::set_object_value(&battery_monitor, battery_monitor.var_info,"_VOLT_MULT", 4.092f);
    AP_Param::set_object_value(&battery_monitor, battery_monitor.var_info,"_AMP_OFFSET", 0.6f);
    AP_Param::set_object_value(&battery_monitor, battery_monitor.var_info,"_AMP_PERVOLT", 16.67f);

    battery_monitor.init();
    ins.init(AP_InertialSensor::RATE_100HZ);
    ahrs.init();
    serial_manager.init();
    AP_OSD::enqueue::initialise();
    baro.init();
    baro.calibrate();

    airspeed.init();
    airspeed.calibrate(false);
    
    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        // compass.set_offsets(0, {341,-295,525}); //orig board
        compass.set_offsets(0, {347.5,-284.5,649});    // ref2 board
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
    gps.init(NULL, serial_manager);
}

namespace {

  uint16_t print_counter =0;
  uint16_t counter10_Hz;
  float    heading = 0;
}

void loop(void)
{
   ins.wait_for_sample();
   
   if (++counter10_Hz == 5){
      counter10_Hz = 0;
      compass.read();

      heading = compass.calculate_heading(ahrs.get_dcm_matrix());

      baro.update();
      AP_OSD::enqueue::baro_alt(baro.get_altitude());

      gps.update();
      AP_OSD::gps_info_t gps_info;
      gps_info.ground_speed_m_per_s   = gps.ground_speed();
      gps_info.ground_course_cd       = gps.ground_course_cd();
      gps_info.num_sats               = gps.num_sats();
      gps_info.status                 = gps.status();
      AP_OSD::enqueue::gps_status(gps_info);

      AP_OSD::enqueue::gps_location(
         {gps.location().lat,gps.location().lng,gps.location().alt}
      );

      airspeed.read();
      AP_OSD::enqueue::airspeed(airspeed.get_airspeed());

      battery_monitor.read();
      AP_OSD::enqueue::battery(
         {battery_monitor.voltage(),
            battery_monitor.current_amps(),
               battery_monitor.current_total_mah()}
      );

   }

   ahrs.update();

   AP_OSD::enqueue::attitude({ToDeg(ahrs.pitch),ToDeg(ahrs.roll),ToDeg(ahrs.yaw)});

   if (++print_counter == 20) {
      print_counter = 0;
      Vector3f drift  = ahrs.get_gyro_drift();
      hal.console->printf(
         "r:%4.1f  p:%4.1f y:%4.1f "
         "drift=(%5.1f %5.1f %5.1f) hdg=%.1f \n",
            static_cast<double>(ToDeg(ahrs.roll)),
            static_cast<double>(ToDeg(ahrs.pitch)),
            static_cast<double>(ToDeg(ahrs.yaw)),
            static_cast<double>(ToDeg(drift.x)),
            static_cast<double>(ToDeg(drift.y)),
            static_cast<double>(ToDeg(drift.z)),
            static_cast<double>(ToDeg(heading))
      );
   }
}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_analogin = true;
      flags.init_uartA = true;
      flags.init_uartC = true;
      flags.init_i2c = true;
      flags.init_spi = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )
