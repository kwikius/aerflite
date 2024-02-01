#ifndef BMP_280_HPP_INCLUDED
#define BMP_280_HPP_INCLUDED

#include "i2c/i2c_driver/i2c_register_based_driver.hpp"

namespace Quan{

   struct bmp280_base {

   // the registers
      struct reg{
         // temperature cal constant addresses
         static constexpr uint8_t dig_T1        = 0x88;
         static constexpr uint8_t dig_T2        = 0x8A;
         static constexpr uint8_t dig_T3        = 0x8C;
         // pressure cal constant addresses
         static constexpr uint8_t dig_P1        = 0x8E;
         static constexpr uint8_t dig_P2        = 0x90;
         static constexpr uint8_t dig_P3        = 0x92;
         static constexpr uint8_t dig_P4        = 0x94;
         static constexpr uint8_t dig_P5        = 0x96;
         static constexpr uint8_t dig_P6        = 0x98;
         static constexpr uint8_t dig_P7        = 0x9A;
         static constexpr uint8_t dig_P8        = 0x9C;
         static constexpr uint8_t dig_P9        = 0x9E;

         static constexpr uint8_t whoami        = 0xD0;
         static constexpr uint8_t reset         = 0xE0;
         static constexpr uint8_t status        = 0xF3;
         static constexpr uint8_t ctrl_meas     = 0xF4;
         static constexpr uint8_t config        = 0xF5;
         static constexpr uint8_t press_msb     = 0xF7;
         static constexpr uint8_t press_lsb     = 0xF8;
         static constexpr uint8_t press_xlsb    = 0xF9;
         static constexpr uint8_t temp_msb      = 0xFA;
         static constexpr uint8_t temp_lsb      = 0xFB;
         static constexpr uint8_t temp_xlsb     = 0xFC;
         
      };

      union config_bits{
           struct{
              bool    spi3w_en  : 1;
              bool              : 1;
              uint8_t filter    : 3;
              uint8_t t_sb      : 3;
           }; 
           uint8_t value;
           config_bits(uint8_t val = 0U) :value{val}{}
      } __attribute__ ((packed));

      union ctrl_meas_bits{
         struct{
             uint8_t mode       : 2;
             uint8_t osrs_p     : 3;
             uint8_t osrs_t     : 3;
         };
         uint8_t value;
         ctrl_meas_bits(uint8_t val = 0U) :value{val}{}
      } __attribute__ ((packed));

      union status_bits {
          struct {
               bool in_update   : 1;
               uint8_t          : 2;
               bool measuring   : 1;
               uint8_t          : 4;
          };
          uint8_t value;
          status_bits(uint8_t val = 0U) :value{val}{}
      } __attribute__ ((packed));
      

      // values
      struct val{
         static constexpr uint8_t whoami = 0x58;
      };

      // variants
      struct ID{
            // 111011x  
         static constexpr uint8_t  get_device_address() { return 0b11101100;}
         static constexpr const char * get_device_name() {return "BMP280";}
      };
   };

   // couple the bmp280 registers with the driver

   struct bmp280 : bmp280_base, Quan::i2c_register_based_driver<bmp280_base::ID> 
   {
       union calib_param_t{
          struct {
             volatile uint16_t dig_T1;
             volatile int16_t  dig_T2;
             volatile int16_t  dig_T3;
             volatile uint16_t dig_P1;
             volatile int16_t  dig_P2;
             volatile int16_t  dig_P3;
             volatile int16_t  dig_P4;
             volatile int16_t  dig_P5;
             volatile int16_t  dig_P6;
             volatile int16_t  dig_P7;
             volatile int16_t  dig_P8;
             volatile int16_t  dig_P9;
          };
          uint8_t  arr[24];
       } __attribute__ ((packed));

       static calib_param_t calib_param;
       
   };

}// Quan

#endif //BMP_280_HPP_INCLUDED
