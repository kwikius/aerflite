#ifndef LIS3_MDL_HPP_INCLUDED
#define LIS3_MDL_HPP_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include "i2c/i2c_driver/i2c_register_based_driver.hpp"

namespace Quan {

   struct lis3mdl_base {

   // the registers
      struct reg{
         static constexpr uint8_t offset_X_reg_L   = 0x05;
         static constexpr uint8_t offset_X_reg_H   = 0x06;
         static constexpr uint8_t offset_Y_reg_L   = 0x07;
         static constexpr uint8_t offset_Y_reg_H   = 0x08;
         static constexpr uint8_t offset_Z_reg_L   = 0x09;
         static constexpr uint8_t offset_Z_reg_H   = 0x0A;

         static constexpr uint8_t whoami           = 0x0F;

         static constexpr uint8_t ctrlreg1         = 0x20;
         static constexpr uint8_t ctrlreg2         = 0x21;
         static constexpr uint8_t ctrlreg3         = 0x22;
         static constexpr uint8_t ctrlreg4         = 0x23;
         static constexpr uint8_t ctrlreg5         = 0x24;
        
         static constexpr uint8_t status_reg       = 0x27;

         static constexpr uint8_t out_X_reg_L      = 0x28;
         static constexpr uint8_t out_X_reg_H      = 0x29;
         static constexpr uint8_t out_Y_reg_L      = 0x2A;
         static constexpr uint8_t out_Y_reg_H      = 0x2B;
         static constexpr uint8_t out_Z_reg_L      = 0x2C;
         static constexpr uint8_t out_Z_reg_H      = 0x2D;

         static constexpr uint8_t temp_out_L       = 0x2E;
         static constexpr uint8_t temp_out_H       = 0x2F;

         static constexpr uint8_t int_cfg          = 0x30;
      };

      // values
      struct val{
         static constexpr uint8_t whoami = 0b00111101;
      };

      // variants
      struct onboardID{
         static constexpr uint8_t  get_device_address() { return 0b00111000;}
         static constexpr const char * get_device_name() {return "LIS3MDL onboard";}
      };

   };

   // couple the lis3mdl registers with the driver
   template <typename ID>
   struct lis3mdl : lis3mdl_base, Quan::i2c_register_based_driver<ID> 
   {
   };

   typedef lis3mdl<lis3mdl_base::onboardID> lis3mdl_onboard;

} // Quan

#endif  // !defined QUAN_AERFLITE_BOARD
#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // LIS3_MDL_HPP_INCLUDED
