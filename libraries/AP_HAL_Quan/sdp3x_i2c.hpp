#ifndef SDP3X_I2C_HPP_INCLUDED
#define SDP3X_I2C_HPP_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if defined QUAN_AERFLITE_BOARD

#include "i2c/i2c_driver/i2c_read_write_driver.hpp"
#include "i2c/i2c_driver/i2c_register_based_driver.hpp"

namespace Quan {

   struct sdp3x_i2c_base {

        // commands
        struct cmd{
           static constexpr uint16_t start_continuous_mass_flow = 0x3608;
           static constexpr uint16_t start_continuous_mass_flow_average = 0x3603;
           static constexpr uint16_t start_continuous_diff_pressure = 0x361E;
           static constexpr uint16_t start_continuous_diff_pressure_average = 0x3615;
           static constexpr uint16_t stop_continous = 0x3FF9;

           static constexpr uint16_t one_shot_mass_flow = 0x3624;
           static constexpr uint16_t one_shot_mass_flow_clk_stretch = 0x3626;
           static constexpr uint16_t one_shot_diff_pressure = 0x362F;
           static constexpr uint16_t one_shot_diff_pressure_clk_stretch = 0x372D;

           static constexpr uint16_t enter_sleep_mode = 0x3677;

           static constexpr uint16_t read_product_id [] = {0x367C, 0xE102};
           
        };

      // values
      struct val{
         static constexpr uint32_t product_id_SDP31 = 0x03010101;
         static constexpr uint32_t product_id_SDP32 = 0x03010201;
      };

      // variants
      struct ID0{
              // 0x21
         static constexpr uint8_t  get_device_address() { return 0b01000010;}
         static constexpr const char * get_device_name() {return "SDP3x-0";}
      };

   };

   // couple the sdp3x commands with the driver
   template <typename ID>
   struct sdp3x_i2c : sdp3x_i2c_base, Quan::i2c_read_write_driver<ID> 
   {
   };

   // TODO add other addresses
   typedef sdp3x_i2c<sdp3x_i2c_base::ID0> sdp3x_i2c_0 ;

} // Quan

#endif  // !defined QUAN_AERFLITE_BOARD
#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // SDP3X_I2C_HPP_INCLUDED
