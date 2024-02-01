
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include "FreeRTOS.h"
#include <task.h>
#include "i2c_task.hpp"
#include "lis3_mdl.hpp"

extern const AP_HAL::HAL& hal;

void Quan::set_gains(quan::three_d::vect<float> const & g)
{
   Quan::detail::compass_gain args;
   args.field = g;
   xQueueSendToBack(get_compass_gain_handle(),&args,20);
}

bool Quan::setup_compass()
{
    // TODO get gains and offsets
    // add whoami check section 7.1
    /*
        CTRL_REG1 section 7.2
        CTRL_REG2 section 7.3
        CTRL_REG3 section 7.4
        CTRL_REG4 section 7.5
         set full scale +-4 Gauss             CTRL_REG2  &= ~ ( bit(FS1) |  bit(FS0) )
         set high perf ( lowest noise )       CTRL_REG1  |=   ( bit(OM1) |  bit(OM0) )
                                              CTRL_REG4  |=   ( bit(OMZ1) | bit(OMZ0) )
         set single measurement mode
             set FODR to 1                    CTRL_REG1  |=    bit(FODR)
             set single measurement           CTRL_REG3   = (CTRL_REG3 & ~(bit(MD1)) | bit(MD0))
         temperature sensor on
    */

   bool result = lis3mdl_onboard::write( Quan::lis3mdl_onboard::reg::ctrlreg2, 0U ) &&
          lis3mdl_onboard::write( Quan::lis3mdl_onboard::reg::ctrlreg1, (0b1 << 7U) | (0b11 << 5U) | (0b1 << 1U))  &&
          lis3mdl_onboard::write( Quan::lis3mdl_onboard::reg::ctrlreg4, (0b11 << 2U) ) &&
          lis3mdl_onboard::write( Quan::lis3mdl_onboard::reg::ctrlreg3, (0b1 << 1U) ); // this puts it at idle

   if (! result){
      hal.console->write("lis3 mdl compass setup failed\n");
   }

   return result;
}

namespace {
   uint8_t __attribute__ ((section (".dma_memory"))) result_values[8] ;

}

// total 10 values takes 10us *10 * 10  == 1 ms
bool Quan::compass_start_read()
{
   // or the reg index with 0x80 for self increment of read registers
   return Quan::lis3mdl_onboard::read(Quan::lis3mdl_onboard::reg::out_X_reg_L | 0x80 ,result_values,8);
}

// result is ready 5 ms after return
/*
AN4602 2.3.1 Turn on time. In UHP mode data is ready after 6.65 ms after end of request conversion
  request conversion takes 300 us +
  so try waiting 7 ms after conversion request starts to read
*/
bool Quan::compass_request_conversion()
{
   return lis3mdl_onboard::write(Quan::lis3mdl_onboard::reg::ctrlreg3, 1U);
}

namespace {
   float compass_gains[3] = {1.0f,1.0f,1.0f};
}

bool Quan::compass_calculate()
{
   // redo the gains
   Quan::detail::compass_gain gain_args;
   if ( xQueueReceive(get_compass_gain_handle(),&gain_args,0) == pdTRUE){
      for (uint32_t i =0; i < 3; ++i){
         if (gain_args.field[i] > 0.5f){
             compass_gains[i] = gain_args.field[i];
         }
      }
   }
   quan::three_d::vect<int> vect;
   for ( uint32_t i = 0; i < 3; ++i){
      union {
         uint8_t ar[2];
         int16_t v;
      } u;
      u.ar[0] = result_values[ 2U * i];
      u.ar[1] = result_values[ 2U * i + 1];
      vect[i] = u.v;
   }

   typedef quan::magnetic_flux_density_<float>::milli_gauss mgauss;

   // AN4602 2.1 Full scale for milli gauss at +-4G full scale
   // data sheet table 3
   constexpr float scale_mult = static_cast<float>(1.0/6.842);
   // todo add gains
   quan::three_d::vect<mgauss> field {
       mgauss{vect.x * scale_mult }
      ,mgauss{vect.y * scale_mult }
      ,mgauss{vect.z * scale_mult }
   };

   Quan::detail::compass_args args;
   args.field = field;
   args.time_us = AP_HAL::micros();
   xQueueOverwrite(get_compass_queue_handle(),&args);

   return true;
}

#endif  // !defined QUAN_AERFLITE_BOARD
#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN


