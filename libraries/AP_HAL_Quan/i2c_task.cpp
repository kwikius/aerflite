
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
  originally written by Jose Julio, Pat Hickey and Jordi Muñoz
  Heavily modified by Andrew Tridgell

  heavily derived from Ardupilot AP_Baro_MS5611.cpp and AP_Compass_HMC5843.cpp
  by Andy Little ( www.github.com/kwikius)
*/

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#if ! defined QUAN_AERFLITE_BOARD

#include <stm32f4xx.h>
#include "i2c_task.hpp"
#include <quan/max.hpp>
#include <Filter/LowPassFilter2p.h>
#include <AP_HAL/I2CDriver.h>
/*
The MS56111 Baro and HMC5883 compass are on a single I2C bus

The AP_baro_driver and AP_Compass_Backend run in the APM_Task

The I2C_Task runs as a separate task in FreeRTOS

The I2Ctask uses FreeRTOS queues to communicate compass and Baro data to the APM_Task

The I2C_Task is run at 100 Hz
The task also filters the data
and puts the lastest value on the queues when there is free space

Needs usart and i2c and scheduler objects to be ready
  
*/

extern const AP_HAL::HAL& hal;

namespace Quan{
   AP_HAL::I2CDriver * get_quan_i2c_driver();
}

namespace {

   bool init_compass();
   bool request_compass_read();
   bool read_compass();

   bool init_baro();
   bool request_pressure_read();
   bool request_temperature_read();
   bool read_baro_pressure();
   bool read_baro_temperature();

   void wait_for_power_up();

   void panic(const char* text)
   {
      hal.console->write(text);
   }

   void hal_printf(const char* text)
   {
      hal.console->write(text);
   }

   // last waketime for the i2c compass baro task
   // used by FreeRTOS
   TickType_t last_wake_time = 0;

   TickType_t delay_wakeup_time = 0;
   void delay( uint32_t n)
   {
      vTaskDelayUntil(&delay_wakeup_time,n);
   }

   // queue for sending baro data to apm thread
   QueueHandle_t hBaroQueue = nullptr;

   // queue for sending compass data to apm thread
   QueueHandle_t hCompassQueue = nullptr;

   AP_HAL::I2CDriver * p_i2c = nullptr;

   // boolean count to sequentially read 1 * pressure then 1 * temperature
   // This is done to give a nominal 50 Hz update rate
   // for the baro pressure and temperature filters
   bool baro_read_pressure = true;

   void i2c_task(void* params)
   {
      // need to wait for 5V on the external sensor voltage regs
      wait_for_power_up();

      if ( (hBaroQueue == nullptr) || (hCompassQueue == nullptr) ){
         hal_printf("create FreeRTOS queues failed in I2c task\n");
      }
      p_i2c = Quan::get_quan_i2c_driver();

      p_i2c->begin();

      bool result = false;
      for ( uint8_t i = 0; i < 3; ++i){
         if(init_compass()){
            result = true;
            break;
         }
      }
      if (result){
      }else{
         panic("Compass init failed\n");
      }

      result = false;
      for ( uint8_t i = 0; i < 3; ++i){
         if ( init_baro() ){
            result = true;
            break;
         }
      }  
      if(result){
      }else{
           panic(" Baro init failed\n");
      }

      // i2c task running...
      for (;;){

         // every 1/100th sec
         vTaskDelayUntil(&last_wake_time, 10);
        // hal_printf("i2c >\n");
         
         auto * sem = p_i2c->get_semaphore();
         if ( sem && sem->take_nonblocking()){

          //  hal_printf("in sem\n");
            // TODO take care of failures?
            // do n tries to read compass
            // if no joy reinit?
            if ( ! read_compass() ){
               hal_printf("compass read fail\n");
            }else{
            }
         
            if (! request_compass_read()){
               hal_printf("compass req fail\n");
            }
            // on every 1 in 2 read the baro temperature
            // else read the baro pressure
            if (baro_read_pressure == false){
               if (!read_baro_temperature()){
                  hal_printf("baro temp read fail\n");
               }else{
               }
               if (!request_pressure_read()){
                  hal_printf("baro pressure req fail\n");
               }
               baro_read_pressure = true;
            }else{
               if (!read_baro_pressure()){
                  hal_printf("baro pressure read fail\n");
               }
               if(! request_temperature_read()){
                  hal_printf("baro temperature read fail\n");
               }
               baro_read_pressure = false;
            }
            sem->give();
         }else{
            hal_printf("i2ct no sem\n");
         }
      }
   }

   // i2c transfer return states
   constexpr uint8_t transfer_succeeded = 0U;
   constexpr uint8_t transfer_failed = 1U;
//------------baro ----------------------------------
   constexpr uint8_t baro_addr = 0x77;
   constexpr uint8_t baro_cal_base_reg = 0xA0;
   constexpr uint8_t baro_cmd_reset = 0x1E;
   constexpr uint8_t baro_cmd_read_pressure = 0x48;
   constexpr uint8_t baro_cmd_read_temperature = 0x58;
   // array of ms5611 cal values
   // baroCal[0] = setup params
   // baroCal[1:6] = actual cal values from baro ROM 
   // baroCal[7] is crc
   uint16_t baroCal[8];

   LowPassFilter2p<quan::pressure_<float>::Pa>    pressure_filter;
   LowPassFilter2p<quan::temperature_<float>::K>   temperature_filter;
   LowPassFilter2p<
      quan::three_d::vect<
         quan::magnetic_flux_density_<float>::milli_gauss
      > 
   > compass_filter;

   quan::pressure_<float>::Pa         filtered_pressure;
   quan::temperature_<float>::K       filtered_temperature;
  
   quan::three_d::vect<
      quan::magnetic_flux_density_<float>::milli_gauss
   > filtered_compass;
   
   
   uint8_t read_16bits(uint8_t addr, uint8_t reg, uint16_t & out)
   {
       uint8_t buf[2];
       if (p_i2c->readRegisters(addr, reg, 2, buf) == transfer_succeeded) {
           out = (((uint16_t)(buf[0]) << 8) | buf[1]);
           return transfer_succeeded;
       }else{
         return transfer_failed;
      }
   }

   uint8_t read_24bits(uint8_t addr, uint8_t reg, uint32_t & out)
   {
       uint8_t buf[3];
       if (p_i2c->readRegisters(addr, reg, 3, buf) == 0) {
          out = (((uint32_t)buf[0]) << 16) | (((uint32_t)buf[1]) << 8) | buf[2];
         return transfer_succeeded;
       }else{
         return transfer_failed;
       }
   }

   // store the raw data from the baro
   uint32_t latest_pressure_reading = 0;
   uint32_t latest_temperature_reading = 0;

   template <typename Pressure, typename Temperature>
   void ms5611_calc(int32_t D1, int32_t D2, Pressure & p, Temperature &t)
   {
      int32_t const dT = D2 - ( static_cast<int32_t>(baroCal[5]) << 8);
      int32_t TEMP = 2000 + ((dT * static_cast<int32_t>(baroCal[6])) >> 23);
      int64_t  OFF = ((static_cast<int64_t>(baroCal[2]) << 16)) 
         + ((static_cast<int64_t>(baroCal[4]) * dT ) >> 7);
      int64_t  SENS = (static_cast<int64_t>(baroCal[1]) << 15)
            + ( (static_cast<int64_t>(baroCal[3]) * dT) >> 8);

      int32_t T2 = 0;
      int64_t OFF2 = 0;
      int64_t SENS2 = 0;
      if ( TEMP < 2000 ){
         T2 = (static_cast<int64_t>(dT) * dT ) >> 31;
         OFF2 = (5 * static_cast<int64_t>(TEMP - 2000) * static_cast<int64_t>(TEMP - 2000)) >> 1;
         SENS2 = OFF2 >> 1;
      }
      TEMP = TEMP - T2;
      OFF = OFF- OFF2;
      SENS = SENS - SENS2;
      int32_t P = (((D1 * SENS)>> 21) - OFF) >> 15;
     
      p = pressure_filter.apply(quan::pressure::Pa{P});
      t = temperature_filter.apply(quan::temperature::K{(TEMP * 0.01f) + 273.15f});
   }

   bool calculate_baro_args()
   {
       ms5611_calc(
            latest_pressure_reading, 
            latest_temperature_reading,
            filtered_pressure,
            filtered_temperature
        );

      return true;
   }

   bool read_baro_pressure()
   {
      return read_24bits(baro_addr,0,latest_pressure_reading) == transfer_succeeded;
   }

   // on temperature read do the baro conversion calcs and try
   // moving existing temperature and pressure samples to the
   // quueue if possible
   bool read_baro_temperature()
   {
      if ( read_24bits(baro_addr,0,latest_temperature_reading) == transfer_succeeded){
         calculate_baro_args();
         if ( uxQueueSpacesAvailable(hBaroQueue) != 0){
            Quan::detail::baro_args args = {filtered_pressure,filtered_temperature};
            xQueueSendToBack(hBaroQueue,&args,0);
         }
         return true;
      }else{
         hal_printf("temp read failed\n");
         return false;
      }
   }

   // request the baro to prepare a pressure sample
   bool request_pressure_read()
   {
      uint8_t command = baro_cmd_read_pressure;
      return p_i2c->write(baro_addr,1,&command) == transfer_succeeded;
   }

   // request the baro to prepare a temperature sample
   bool request_temperature_read()
   {
      uint8_t command = baro_cmd_read_temperature;
      return p_i2c->write(baro_addr,1,&command) == transfer_succeeded;
   }

   bool do_baro_crc()
   {
    /* save the read crc */
      uint16_t const crc_read = baroCal[7];
    /* remove CRC byte */
      baroCal[7] = (0xFF00 & (baroCal[7]));
      uint16_t n_rem = 0x00;
      for (uint16_t cnt = 0; cnt < 16; ++cnt) {
      /* uneven bytes */
         if (cnt & 1) {
            n_rem ^= (uint8_t)((baroCal[cnt >> 1]) & 0x00FF);
         } else {
            n_rem ^= (uint8_t)(baroCal[cnt >> 1] >> 8);
         }
         for (uint8_t n_bit = 8U; n_bit > 0U; --n_bit) {
            if (n_rem & 0x8000) {
               n_rem = (n_rem << 1U) ^ 0x3000;
            } else {
               n_rem = (n_rem << 1U);
            }
         }
      }
    /* final 4 bit remainder is CRC value */
      n_rem = (0x000F & (n_rem >> 12));
      baroCal[7] = crc_read;
      /* return true if CRCs match */
      return (0x000F & crc_read) == (n_rem ^ 0x00);
   }

   // based on the AP_BARO_MS5611 constructor code
   bool init_baro()
   {
      
      AP_HAL::Semaphore * const sem = p_i2c->get_semaphore();
      if (!sem) {
         hal_printf("couldnt get semaphore");
         return false;
      }
      // TODO add timeout
      while ( !sem->take_nonblocking()){;}
      uint8_t command = baro_cmd_reset;
      if ( p_i2c->write(baro_addr,1,&command) != transfer_succeeded){
         hal_printf("coulndt write baro reset");
         sem->give();
         return false;
      }
      delay(4);
      for ( uint8_t i = 0; i < 8; ++i){
        if (  read_16bits(baro_addr,baro_cal_base_reg + 2*i, baroCal[i]) != transfer_succeeded){
            hal_printf("read baro cal failed\n");
            sem->give();
            return false;
        }
      }
      if (! do_baro_crc()){
          hal_printf("read baro crc failed\n");
          sem->give();
          return false;
      }
      // 14 degrees C is average world temperature
       // 50 Hz sample and  1 Hz cutoff
      temperature_filter.set_cutoff_frequency(50,0.5);
      temperature_filter.reset(quan::temperature_<float>::K{273.15 + 14}, 1000);
      // standard atmospheric pressure in Pa
      // 50 Hz sample and 10 Hz cutoff
      pressure_filter.set_cutoff_frequency(50,5);
      pressure_filter.reset(quan::pressure_<float>::Pa{101325},1000);
      // start the ball rolling
      bool result = request_pressure_read();
      if ( result){
      }else{
          hal_printf("request first baro sample failed\n");
      }
      sem->give();
      return result;
   }
//------------mag------------------------------------

   // index into the array below from HMC5883 datasheet
   constexpr uint8_t mag_runtime_gain_index = 0b001;     // range of +- 1.3 Gauss
   constexpr uint8_t mag_calibration_gain_index = 0b11;  // range of  +- 2.5 Gauss
   typedef quan::magnetic_flux_density_<float>::milli_gauss milli_gauss;
   // scaling to get from the mag value to milligauss
   // mag_scaling was derived in the calibration at startup
   // value_in_milligauss = compass_gain_per_lsb[mag_runtime_gain_index] * hmc5883_raw_value * mag_scaling
   // for x, y, z
   constexpr milli_gauss compass_gain_per_lsb[] = {
      milli_gauss{0.73f}
      ,milli_gauss{0.92f}
      ,milli_gauss{1.22f}
      ,milli_gauss{1.52f}
      ,milli_gauss{2.27f}
      ,milli_gauss{2.56f}
      ,milli_gauss{3.03f}
      ,milli_gauss{4.35f}
   };

   constexpr uint8_t mag_addr = 0x1E;
   constexpr uint8_t mag_configA = 0x0;
   constexpr uint8_t mag_configB = 0x1;
   constexpr uint8_t mag_modereg =  0x02;
   constexpr uint8_t mag_msb_x = 0x03;
   constexpr uint8_t mag_single_measurement = 0x01;
   constexpr uint8_t mag_positive_bias_config = 0x11;
   constexpr uint8_t mag_negative_bias_config = 0x12;
   
   quan::three_d::vect<float> mag_scaling{0.f,0.f,0.f};

   // wait for the switcher to start putting out 5V
   // wait a nominal 0.2 sec after mcu power up 
   // mcu powers up when switcher is outputting ~ 3.3V
   void wait_for_power_up();
   bool do_compass_calibration();
   bool do_mag_runtime_config();

   // add init of gains etc
   // This is based squarely on the AP_Compass_HMC5843 source
   bool init_compass()
   {
      AP_HAL::Semaphore * const sem = p_i2c->get_semaphore();
      if (!sem) {
         hal_printf("couldnt get semaphore");
         return false;
      }

      // compass filter sample at 100 Hz, cutoff 12 Hz
      compass_filter.set_cutoff_frequency(100,12);
      typedef quan::magnetic_flux_density_<float>::milli_gauss mgauss;
      compass_filter.reset(quan::three_d::vect<mgauss>{mgauss{1.0},mgauss{0.0},mgauss{0.0}},1000);
      
      // TODO add timeout
      while ( !sem->take_nonblocking()){hal_printf("loop taking semaphore\n");}

      if (! do_compass_calibration() ){
         sem->give();
         return false;
      }
      // start the read loop
      bool const result = do_mag_runtime_config() && request_compass_read();
      sem->give();
      return result;
   }

   bool do_mag_runtime_config()
   {
      constexpr uint8_t sample_average8 = (0b11 << 5);
      return (p_i2c->writeRegister(mag_addr,mag_configA,sample_average8) == transfer_succeeded)
       &&  (p_i2c->writeRegister(mag_addr,mag_configB,(mag_runtime_gain_index << 5)) == transfer_succeeded) ;
   }

   bool request_compass_read()
   {
      return (p_i2c->writeRegister(mag_addr,mag_modereg, mag_single_measurement) == transfer_succeeded);
   }


   int16_t mag_convert_to_int16(uint8_t * d)
   {
      union{
         uint8_t in[2] ;
         int16_t out;
      }u;
      u.in[0] = d[1];
      u.in[1] = d[0];
      return u.out;
   }

   void copy_new_values(uint8_t (&raw_magnetometer_values)[6], quan::three_d::vect<int16_t> & result_out )
   {
      result_out.x = -mag_convert_to_int16(raw_magnetometer_values);
      result_out.y = mag_convert_to_int16(raw_magnetometer_values + 4);
      result_out.z = -mag_convert_to_int16(raw_magnetometer_values + 2);
   }

   bool read_compass_raw(quan::three_d::vect<int16_t> & result_out, int32_t & time_out)
   {
      uint8_t raw_mag_values[6] = {0,0,0,0,0,0};
      if ( p_i2c->readRegisters(mag_addr,mag_msb_x,6,raw_mag_values) == transfer_succeeded){
         time_out = AP_HAL::millis();
         copy_new_values(raw_mag_values,result_out);
         return true;
      }else{
         return false;
      }
   }

   // when the new value is ready
   // read the compass and if possible, put the filtered result on the queue to the
   // stub AP_CompassBackend
   bool read_compass()
   {
      int32_t time_ms = 0;
      quan::three_d::vect<int16_t> result;

      if ( read_compass_raw(result, time_ms)){
         // Quan::detail::compass_args args;
         constexpr milli_gauss milli_gauss_per_lsb = compass_gain_per_lsb[mag_runtime_gain_index];
         filtered_compass = compass_filter.apply({
            result.x * milli_gauss_per_lsb * mag_scaling.x
            ,result.y * milli_gauss_per_lsb * mag_scaling.y
            ,result.z * milli_gauss_per_lsb * mag_scaling.z
         });
         if ( uxQueueSpacesAvailable(hCompassQueue) != 0 ){
            Quan::detail::compass_args args;
            args.field = filtered_compass;
            args.time_us = AP_HAL::micros();
            xQueueSendToBack(hCompassQueue,&args,0);
         }
         return true;
      }else{
         return false;
      }
   }

   TaskHandle_t task_handle;
   void * dummy_params;

} // namespace

namespace Quan {
   void create_i2c_task()
   {
      // As long as the baro data is read at less than 50 Hz
      // and the compass at less than 100 Hz
      // 1 in queue should be ok
      // The update rates are listed in ArduPlane.cpp
      // as 10 Hz for update_compass
      // and 10 Hz for update_alt ( baro)
      // (The higher rate accumulate functions are not used or required)
      // Same rates in ArduCopter
      constexpr uint32_t num_in_queue = 1;
      hBaroQueue = xQueueCreate(num_in_queue,sizeof(Quan::detail::baro_args));
      hCompassQueue = xQueueCreate(num_in_queue,sizeof(Quan::detail::compass_args));

      xTaskCreate(
         i2c_task,"I2C_task",
         1000,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
         & task_handle
      ) ;
   }

   QueueHandle_t get_compass_queue_handle()
   {
      if ( hCompassQueue == nullptr){
         panic("Requesting null compass queue handle\n");
      }
      return hCompassQueue;
   }

   QueueHandle_t get_baro_queue_handle()
   {
      if ( hBaroQueue == nullptr){
         panic("Requesting null baro queue handle\n");
      }
      return hBaroQueue;
   }
}

namespace {

   bool do_compass_calibration()
   {
      // gives a range of +-2.5 Ga
      constexpr uint16_t expected_x = 766;
      constexpr uint16_t expected_yz = 713;

      int numAttempts = 0;
      int good_count = 0;
      bool success = false;

      while (success == 0 && numAttempts < 25 && good_count < 5){

         ++numAttempts;

         // force positiveBias (compass should return 715 for all channels)
         if (p_i2c->writeRegister(mag_addr,mag_configA, mag_positive_bias_config) != transfer_succeeded){
            continue;   // compass not responding on the bus
         }
         delay(50);

         // set gains
         if ( !((p_i2c->writeRegister(mag_addr,mag_configB, (mag_calibration_gain_index << 5)) == transfer_succeeded) &&
             (p_i2c->writeRegister(mag_addr,mag_modereg, mag_single_measurement) == transfer_succeeded))
         ){
            continue;
         }

         delay(50);
         quan::three_d::vect<int16_t> mag_result;
         int32_t time_ms =0;
         if (!read_compass_raw(mag_result,time_ms)){
            continue;      // we didn't read valid raw_magnetometer_values
         }
         delay(10);

         quan::three_d::vect<float> cal;

         cal.x = fabsf(expected_x  / static_cast<float>(mag_result.x) );
         cal.y = fabsf(expected_yz / static_cast<float>(mag_result.y) );
         cal.z = fabsf(expected_yz / static_cast<float>(mag_result.z) );

         // throw away the first two samples as the compass may
         // still be changing its state from the application of the
         // strap excitation. After that, accept values in a
         // reasonable range

         if (numAttempts <= 2) {
            continue;
         }

         auto cal_valid = [] (float val) { return (val > 0.7f) && (val < 1.35f);};

         if ( cal_valid(cal.x) && cal_valid(cal.y) && cal_valid(cal.z)) {
            ++good_count;
            mag_scaling += cal;
         }

      } // while

      if (good_count >= 5) {
         mag_scaling /= good_count;
         success = true;
      } else {
         /* best guess */
        mag_scaling = {1.0f,1.0f,1.0f};
      }
      return success;
   }

   void wait_for_power_up()
   {
      uint32_t const now_ms = AP_HAL::millis();
      if( now_ms < 200){
         delay(200 - now_ms);
      }
   }
}

#endif // #if !defined QUAN_AERFLITE_BOARD
#endif // AP_HAL_QUAN
