/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_Compass/AP_Compass.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <quantracker/osd/osd.hpp>
#include <quan/min.hpp>
#include <quan/max.hpp>
#include <AP_OSD/fonts.hpp>
#include <cstdio>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

namespace {

  Compass compass;

  AP_HAL::UARTDriver * uart = nullptr;
}

void setup() {
    
    // we output data on telemetry uart
    // which is connected to the RF modem.
    uart = hal.uartC;
    uart->begin(57600);

    uart->println("Compass library test");

    if (!compass.init()) {
        AP_HAL::panic("compass initialisation failed!");
    }
    uart->printf("init done - %u compasses detected\n", compass.get_count());

    // mod this to your own offsets

    Vector3f offsets{310.31,-302.91,519.73};
    compass.set_offsets(compass.get_primary(),offsets);
    compass.set_declination(ToRad(0.0f)); // set local difference between magnetic north and true north

    hal.scheduler->delay(1000);

}

namespace {

   float heading = 0.f;

   Vector3f raw_field;
   Vector3f mag;
   Vector3f field_min;
   Vector3f field_max;
   Vector3f field_offset;
   bool start = true;
}

void quan::uav::osd::on_draw() 
{ 

    pxp_type pos{-140,100};
    draw_text("Quan APM Compass Test",pos,Quan::FontID::MWOSD);

    char buf [100];

    sprintf(buf,"hdg = % 3.2f",static_cast<double>(ToDeg(heading)));
    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::OSD_Charset);

    sprintf(buf,"raw[% 6.2f,% 6.2f,% 6.2f]",
         static_cast<double>(raw_field.x),
         static_cast<double>(raw_field.y),
         static_cast<double>(raw_field.z));
    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::MWOSD);

    sprintf(buf,"mag[% 6.2f,% 6.2f,% 6.2f]",
         static_cast<double>(mag.x),
         static_cast<double>(mag.y),
         static_cast<double>(mag.z));

    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::MWOSD);

    sprintf(buf,"ofs[% 6.2f,% 6.2f,% 6.2f]",
      static_cast<double>(field_offset.x),
      static_cast<double>(field_offset.y), 
      static_cast<double>(field_offset.z));
    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::MWOSD);

    sprintf(buf,"max[% 6.2f,% 6.2f,% 6.2f]",
      static_cast<double>(field_max.x),
      static_cast<double>(field_max.y),
      static_cast<double>(field_max.z));
    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::MWOSD);

    sprintf(buf,"min[% 6.2f,% 6.2f,% 6.2f]",
      static_cast<double>(field_min.x),
      static_cast<double>(field_min.y), 
      static_cast<double>(field_min.z));
    pos.y -= 30;
    draw_text(buf,pos,Quan::FontID::MWOSD);

}

void loop()
{
   hal.scheduler->delay(100);

   compass.read();

   raw_field = compass.get_raw_field();

   uart->printf(
      "raw field [% 6.2f x, % 6.2f y, % 6.2f z]\n",
       static_cast<double>(raw_field.x) 
      ,static_cast<double>(raw_field.y)
      ,static_cast<double>(raw_field.z)
   );

   #if 1

   if (!compass.healthy()) {
       AP_HAL::panic("compass not healthy");
   }

   Matrix3f dcm_matrix;
   // use roll = 0, pitch = 0 for this example
   dcm_matrix.from_euler(0, 0, 0);
   heading = compass.calculate_heading(dcm_matrix);
   compass.learn_offsets();

   // capture field_min
   mag = compass.get_field();

   // we need to set something as min and max from the sensor, 
   // since offsets may be such that values never get entirely
   // positive or negative so starting at zero might always be 
   // an incorrect minimum or a maximum
   if (start){
      for (uint32_t i = 0; i < 3; ++i){
        field_min[i] = mag[i];
        field_max[i] = mag[i];
      }
      start = false;
   }else{
      for (uint32_t i = 0; i < 3; ++i){
        field_min[i] = quan::min(field_min[i],mag[i]);
        field_max[i] = quan::max(field_max[i],mag[i]);
      }
   }
   // calculate offsets
   field_offset = -(field_max + field_min)/2;
   // display all to user
   uart->printf("Heading: % 6.2f [% 3d,% 3d,% 3d]",
       static_cast<double>(ToDeg(heading)),
       (int)mag.x ,
       (int)mag.y ,
       (int)mag.z 
   );

   uart->printf(" offsets[% 6.2f, % 6.2f, % 6.2f\n",
                static_cast<double>(field_offset[0]) 
               ,static_cast<double>(field_offset[1])
               ,static_cast<double>(field_offset[2]));

   #endif
}


namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_uartC = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

