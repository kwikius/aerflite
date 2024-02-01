/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include <cstdio>
#include <cstring>
#include <cctype>

#include <quan/min.hpp>
#include <quan/max.hpp>
#include <quan/conversion/parse_number.hpp>
#include <quan/three_d/vect.hpp>

#include <AP_Compass/AP_Compass.h>
#include <Filter/LowPassFilter2p.h>
#include <AP_HAL_Quan/AP_HAL_Quan_Test_Main.h>
#include <AP_HAL_Quan/Storage.h>
#include <quantracker/osd/osd.hpp>
#include <AP_OSD/fonts.hpp>

#if defined QUAN_AERFLITE_BOARD
#include  <AP_HAL_Quan/i2c_task.hpp>
#else
#error Expected aerflite board to be defined
#endif

/**
 Note that we are using a special area ineeprom to read and write offsets in this test.
 Writing these offstes to eeprom  doesnt affect the Ardupilot Compass offsets,
 which will need to be written separately.
 Note also that in Ardupilot Proper rotations are applied to sensors before offsets are applied
 to the raw sensor output, therefore the offsets found here will need to be applied to
 the transformed x, y z offsets
**/
const AP_HAL::HAL& hal = AP_HAL::get_HAL();
using AP_HAL::millis;

namespace {

   Compass compass;

   AP_HAL::UARTDriver * uart = nullptr;

   LowPassFilter2p<Vector3f> mag_field;

   float cutoff_freq = 1.f;

   //quan::uav::osd::pxp_type ar[4];

   // wher the structure is stored
   uint32_t compass_params_eeprom_address = 0x10100;

   const char magic_value[5] = "quan";
   uint32_t magic_eeprom_address= 0x10000;

   struct compass_params_t{
      compass_params_t():offset{0.f,0.f,0.f},gain{1.f,1.f,1.f}{}
      Vector3f offset;
      Vector3f gain;
   };

   compass_params_t compass_params;

   // 0 on not inited,1 on success, -1 on ee read fail
   int read_magic()
   {
      const char magic[5]= {0,0,0,0,0};
      if (! Quan::storage_read((uint8_t*)magic,magic_eeprom_address,5)){
         uart->printf("read eeprom magic failed\n");
          return -1;
      }
      bool result = strncmp(magic,"quan",5)==0;
      if (result){
         uart->printf("magic found\n");
         return 1;
      }else{
         uart->printf("no magic found\n");
         return 0;
      }
   }

   void write_magic()
   {
      if (! Quan::storage_write(magic_eeprom_address,(uint8_t const*)"quan",5)){
         AP_HAL::panic("write eeprom magic failed\n");
      }
   }

   void read_compass_params(compass_params_t& c)
   {
      if (!Quan::storage_read((uint8_t*)&c,compass_params_eeprom_address,sizeof(compass_params_t))){
         AP_HAL::panic("read compass params failed\n");
      }
   }

   void write_compass_params(compass_params_t const & c)
   {
      if (!Quan::storage_write(compass_params_eeprom_address,(uint8_t const*)&c,sizeof(compass_params_t))){
         AP_HAL::panic("write compass params failed\n");
      }
   }

   void print_float (Vector3f & v)
   {
      uart->printf("[% 8.2f,% 8.2f,% 8.2f]",
        static_cast<double>(v.x)
        ,static_cast<double>(v.y)
        ,static_cast<double>(v.z)
      );
   }

   void view_params()
   {
      uart->printf("offsets = ");
      print_float(compass_params.offset);
      uart->printf("\ngains   = ");
      print_float(compass_params.gain);
      uart->printf("\n");
   }
}

Vector3f compassCalTest_update_compass()
{
   vTaskSuspendAll();
   compass.update();
   Vector3f field = mag_field.apply(compass.get_field());
   xTaskResumeAll();
   return field;
}

float compassCalTest_get_heading_deg()
{
   Matrix3f dcm_matrix;
   // use roll = 0, pitch = 0 for this example
   dcm_matrix.from_euler(0, 0, 0);
   vTaskSuspendAll();
   float heading = ToDeg(compass.calculate_heading(dcm_matrix));
   xTaskResumeAll();
   return heading;
}



void setup() {
    // we output data on telemetry uart
    // which is connected to the RF modem.
    uart = hal.uartA;
    uart->begin(115200);

    hal.scheduler->delay(1000);

    hal.console->println("Compass library test (console)\n");
    uart->println("Compass library test (uart)\n");

    int magic_exists = read_magic();
    if (magic_exists == -1){
      AP_HAL::panic("read magic failed\n");
    }
    if ( magic_exists == 0){
        uart->printf("initialising compass params to default\n");
        write_magic();
        write_compass_params(compass_params);
    }

    if (magic_exists ==1){
        uart->printf("reading params\n");
        read_compass_params(compass_params);
    }

    if (!compass.init()) {
        AP_HAL::panic("compass initialisation failed!");
    }

    compass.set_offsets(compass.get_primary(),compass_params.offset);
    compass.set_declination(ToRad(0.0f)); // set local difference between magnetic north and true north

    mag_field.set_cutoff_frequency(50,cutoff_freq);

    for ( uint8_t i = 0; i < 25; ++i){
      compassCalTest_update_compass();
      hal.scheduler->delay(20);
    }

    uart->printf("compass params\n");
    view_params();

    uart->printf("press space for menu\n");

}

namespace {

   /**
   @brief get from user if they want x, y or z channel
   @return  0 for x,1 for y,2 for z
   **/
   int parse_channel()
   {
      for(;;){
         uart->printf("enter x or y or z \r\n");
         while( !uart->available() ) {
            hal.scheduler->delay(20);
         }
         char user_input = uart->read();
         switch(user_input){
         case 'x':
            uart->printf("x entered\n");
            return 0;
         case 'y':
            uart->printf("y entered\n");
            return 1;
         case 'z':
            uart->printf("z entered\n");
            return 2;
         default:
            uart->printf("Error : invalid input\\nn");
            break;
         }
      }
   }

   /**
    @brief get from user a number , either int or float
    @return number as a float
   **/
   float parse_number()
   {
      for (;;){
         uart->printf("enter number\n");
         constexpr uint32_t buflen = 100;
         char buffer[buflen] = {'\0'};
         uint32_t idx = 0U;
         for (;;){
             while( !uart->available() ) {
                hal.scheduler->delay(20);
             }
             char ch = uart->read();
             if ( ch == '\r'){
               if ( idx > 0){
                  buffer[idx] = '\0';
                  uart->printf("\r\n");
                  break;
               }else{
                  uart->printf("no number, restarting parse\n\n");
                  idx = 0U;
               }
             }
             if ( idx < (buflen-1)){
               buffer[idx++] = ch;
               uart->printf("%c",ch);
             }else{
                uart->printf("number too long, restarting parse\n\n");
                idx = 0U;
             }
         }
         typedef quan::detail::number_parser parse_t;
         parse_t parse;
         double d_res;
         int64_t i_res;
         parse_t::num_type const num_type = parse(buffer,&d_res,&i_res,buflen);
         switch( num_type){
            case parse_t::num_type::FLOAT:
               uart->printf("%f entered\n",d_res);
               return d_res;
            case parse_t::num_type::INT:
               uart->printf("%d entered\n",static_cast<int>(i_res));
               return i_res;
            default:
               uart->printf("failed to recognise number, restarting parse\n\n");
               break;
         }
      }
   }

   /**
    * @brief get channel and offset for that channel from user and write to the dynamic compass offsets
    **/
   void do_offset()
   {
      int idx= parse_channel();
      float number = parse_number();
      compass_params.offset[idx] = number;
      compass.set_offsets(compass.get_primary(),compass_params.offset);
      uart->printf("offsets = ");
      print_float(compass_params.offset);

   }

   /**
    * @brief get channel and gain for that channel from user and write to the dynamic compass gains
    **/

   void do_gain()
   {
      int idx= parse_channel();
      float number = parse_number();
      compass_params.gain[idx] = number;
      quan::three_d::vect<float> g{0.f,0.f,0.f};
      g[idx] = number;
      Quan::set_gains(g);
      uart->printf("gains = ");
      print_float(compass_params.gain);
   }

   void do_filter_cutoff()
   {
      float new_cutoff_freq = parse_number();
      vTaskSuspendAll();
      cutoff_freq = new_cutoff_freq;
      mag_field.set_cutoff_frequency(50,new_cutoff_freq);
      xTaskResumeAll();
      uart->printf("cutoff set to %f\n",static_cast<double>(cutoff_freq));
   }

   void save_to_eeprom()
   {
      write_compass_params(compass_params);
      Quan::wait_for_eeprom_write_queue_flushed();
      uart->printf("params written to eeprom");
   }

   void print_menu()
   {
         while( uart->available() ) {
            uart->read();
         }
          uart->printf(
         "Menu:\r\n"
         "    V) view gains and offsets\r\n"
         "    O) set offset\r\n"
         "    G) set gain\r\n"
         "    F) set filter cutoff\r\n"
         "    S) save to eeprom\r\n");

         while( !uart->available() ) {
            hal.scheduler->delay(20);
         }

   }

   void display_options()
   {
      print_menu();
      char user_input = uart->read();

      switch (toupper(user_input)){
         case 'V':
         view_params();
         break;
         case 'O':
         do_offset();
         break;
         case 'G':
         do_gain();
         break;
         case 'F':
         do_filter_cutoff();
         break;
         case 'S':
         save_to_eeprom();
         break;
         default:
         uart->printf("unknown input\n");
         break;
      }

      while( uart->available() ) {
         uart->read();
      }
  }
}

void loop()
{
   hal.scheduler->delay(100);

   if (uart->available()){
      if (uart->read() == ' '){
         display_options();
      }
   }

}

namespace {
   uint32_t get_flags()
   {
      HAL_Quan::start_flags flags{0};
      flags.init_gpio = true;
      flags.init_scheduler = true;
      flags.init_uartA = true;
      flags.init_uartB = true;
      flags.init_i2c = true;
      return flags.value;
   }
}

AP_HAL_TEST_MAIN( get_flags() )

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

