

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Progmem/AP_Progmem.h>
#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS_DCM.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>

#include <AP_Scheduler/AP_Scheduler.h>

#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include "osd_message.hpp"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_InertialSensor ins;
Compass compass;
AP_GPS gps;
AP_Baro baro;
AP_SerialManager serial_manager;
AP_AHRS_DCM  ahrs(ins, baro, gps);

void setup(void)
{
#if 0
    ins.init(AP_InertialSensor::RATE_50HZ);
    ahrs.init();
    serial_manager.init();

    if( compass.init() ) {
        hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
        hal.console->printf("No compass detected\n");
    }
 

#endif

    ins.init(AP_InertialSensor::RATE_50HZ);
    ahrs.init();
    serial_manager.init();
    if (!compass.init()) {
        hal.console->printf("Compass initialisation failed!\n");
        while (1){;}
    }
 

    
    compass.set_offsets(0, {10.6004,-35.4656,-48.9482});

    gps.init(NULL, serial_manager);
    ahrs.set_fly_forward(true);
    ahrs.set_wind_estimation(true);
    ahrs.set_compass(&compass);
    
    // we need the AHRS initialised for this test

    ahrs.reset();

    Quan::osd_init();
}

namespace {

  //uint16_t counter =0;
  uint16_t print_counter =0;
  uint16_t compass_counter;
  float heading = 0;
  //uint32_t iter_time =0;
  float heading_deg = 0;
}

void loop(void)
{
   ins.wait_for_sample();
   ahrs.update();
  
   if ( ++compass_counter == 5){
      compass_counter = 0;
      if (compass.update()) {
        // Calculate heading
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        heading_deg = (wrap_360_cd_float(ToDeg(heading)*100.f))/100.f;
        //compass.learn_offsets();
        Quan::osd_send_heading(heading_deg);
      }
   }
   Quan::osd_send_attitude({ToDeg(ahrs.pitch),ToDeg(ahrs.roll),ToDeg(ahrs.yaw)});
   if ( ++print_counter == 20){
      print_counter = 0;

       if (compass.healthy()) {
           const Vector3f &mag_ofs = compass.get_offsets();
           const Vector3f &mag = compass.get_field();
           hal.console->printf("Heading: %d, XYZ: %.0f, %.0f, %.0f,\tXYZoff: %6.2f, %6.2f, %6.2f\n",
                               (int)(heading_deg + 0.5f),
                               (double)mag.x, (double)mag.y, (double)mag.z,
                               (double)mag_ofs.x, (double)mag_ofs.y, (double)mag_ofs.z);
       } else {
           hal.console->println("compass not healthy");
       }
   }

   // could add  
   
}

AP_HAL_MAIN();
