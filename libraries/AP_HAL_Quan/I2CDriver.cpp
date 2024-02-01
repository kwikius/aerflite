

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#if !defined QUAN_AERFLITE_BOARD

#include <AP_HAL_Empty/I2CDriver.h>
#include <AP_HAL_Empty/Semaphores.h>
#include "I2CDriver.h"
#include <quan/stm32/freertos/freertos_i2c_task.hpp>
#include <quan/malloc_free.hpp>
#include <quan/meta/integer_max.hpp>
#include <AP_HAL_Quan/Semaphores.h>

#include <string>

using namespace Quan;


/*
it appears that APM address in is to be shifted left 1
 then if its a write address ored with 1
 (See the AP_HAL_AVR::AVRI2CDriver source)
 Note may need to increase the GPIO speed for fast modes
*/

namespace {

   typedef quan::mcu::pin<quan::stm32::gpioa,8> i2c3_scl;
   typedef quan::mcu::pin<quan::stm32::gpioc,9> i2c3_sda;
   //typedef quan::stm32::i2c3  i2c_task;
   typedef quan::stm32::freertos::freertos_i2c_task<
      quan::stm32::i2c3,i2c3_scl,i2c3_sda
    > i2c3_task;
} 
// TODO add for each I2C port
extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_EV_IRQHandler()
{     
   static_assert(std::is_same<i2c3_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");
   i2c3_task::handle_irq();
}

extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler()
{
   static_assert(std::is_same<i2c3_task::i2c_type, quan::stm32::i2c3>::value,"incorrect port irq");

   uint32_t const sr1 = i2c3_task::i2c_type::get()->sr1.get();
   //i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler;
    //TODO ideally should allow multiple errors
   if ( sr1 & (1 << 8) ){
      i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler_BERR;
   }else{
      if (sr1 & (1 << 9) ){
         i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler_ARLO;
      }else{
         if (sr1 & (1 << 10) ){
            i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler_AF;
         }else{
            if (sr1 & (1 << 11) ){
               i2c3_task::i2c_errno = i2c3_task::errno_t::i2c_err_handler_OVR;
            }else{
               if (sr1 & (1 << 12) ){
                  i2c3_task::i2c_errno =  i2c3_task::errno_t::i2c_err_handler_PECERR; 
               }else{
                  if (sr1 & (1 << 14) ){
                     i2c3_task::i2c_errno =  i2c3_task::errno_t::i2c_err_handler_TIMEOUT; 
                  }else{
                     if (sr1 & (1 << 15) ){
                        i2c3_task::i2c_errno =  i2c3_task::errno_t::i2c_err_handler_SMB_ALERT;
                     }else{
                        i2c3_task::i2c_errno =  i2c3_task::errno_t::unknown_i2c_err_handler; 
                     }
                  }
               }
            }
         }
      }
   }
  
   i2c3_task::i2c_type::get()->sr1.set(0); 
   i2c3_task::i2c_type::get()->cr1.bb_setbit<15>();
  // i2c3_task::i2c_type::get()->cr1.bb_clearbit<15>(); 
}

extern const AP_HAL::HAL& hal;

namespace {

   const char* do_i2c_errno_str(quan::stm32::freertos::freertos_i2c_task_base::errno_t e)
   {
      typedef quan::stm32::freertos::freertos_i2c_task_base::errno_t errno_t;
      switch(e){
         case errno_t::no_error:
            return "no error has been detected";
         case errno_t::invalid_address:
            return "invalid address";
         case errno_t::cant_start_new_transfer_when_i2c_busy:
            return "cant start new transfer when i2c busy";
         case errno_t::zero_data:
            return "zero data";
         case errno_t::data_pointer_is_null:
            return "data pointer is null";
         case errno_t::invalid_num_bytes_in_rx_multibyte_btf:
            return "invalid numbytes in rx multibyte buffer";
         case errno_t::unexpected_single_total_bytes_in_rx_btf:
            return "unexpected single total bytes in rx btf";
         case errno_t::unexpected_not_last_byte_in_rxne:
            return "unexpected not last byte in rxne";
         case errno_t::unexpected_flags_in_irq:
            return "unexpected flags in irq";
         case errno_t::unknown_i2c_err_handler:
            return "unknown i2c errhandler";
         case errno_t::unknown_exti_irq:
            return "unknown exti irq";
         case errno_t::address_timed_out:
            return "timed out after sending address";
         case errno_t:: i2c_err_handler_BERR:
            return "bus error";
         case errno_t::i2c_err_handler_AF:
            return "acknowledge failure";
         case errno_t::i2c_err_handler_ARLO:
            return "arbitration lost";
         case errno_t::i2c_err_handler_OVR:
            return "under/overrun error";
         case errno_t::i2c_err_handler_TIMEOUT:
            return "time out error";
         default:
            return "unlisted i2c error";
      }
   }

   void do_i2c_errno(quan::stm32::freertos::freertos_i2c_task_base::errno_t e)
   {
      hal.console->printf("i2c port transaction failed with \"%s\"\n",do_i2c_errno_str(e));
   }

   template <typename I2CPort>
   struct i2c_driver_t final : public AP_HAL::I2CDriver {
   
      typedef I2CPort i2c_port;
      /*
      AVR port looks to return 1 on fail and 0 on success
      */
      static constexpr uint8_t transfer_succeeded = 0U;
      static constexpr uint8_t transfer_failed = 1U;

      void begin() 
      {
         NVIC_SetPriority(I2C3_EV_IRQn,14);
         i2c_port::init(false,false); 
         m_mutex.init();
         m_lockup_count = 0;
      }

      // TODO 
      void end()
      {
        // disable the i2c port
      }

      // avr port waits indefinitely if ms == 0
      void setTimeout(uint16_t ms)
      { 
         // 0 time will wait for approx 600 hours
         int32_t const wait_ms = (ms > 0)? ms : quan::meta::integer_max<int32_t>::value;
         i2c_port::set_max_addr_wait_time(quan::time_<int32_t>::ms{wait_ms});
      }

      // TODO 
      void setHighSpeed(bool active) 
      {

      }

      // blocking (yields) till transfer complete or timout
      uint8_t write(uint8_t addr, uint8_t len, uint8_t* data)
      {
         if( i2c_port::transfer_request( addr << 1, data, len) == true){
            return transfer_succeeded;
         }else{
            handle_errors();
            return transfer_failed;
         }
      } 

      // blocking (yields) till transfer complete or timout
      uint8_t writeRegister(uint8_t addr, uint8_t reg, uint8_t val)
      { 
         uint8_t data[2] = {reg,val};
         return write( addr,2,data );
      }

      // blocking (yields) till transfer complete or timout
      uint8_t writeRegisters(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* data_in)
      {  
         // need to copy data to local buffer to make a continuous block
         // and single transaction
         uint8_t * data_out = (uint8_t*)quan::malloc(len + 1);
         if (! data_out){
            hal.console->printf("malloc failed in i2c writeRegisters\n");
            return transfer_failed;
         }
         data_out[0] = reg; memcpy(data_out +1, data_in,len);
         uint8_t const result = write(addr,len+1,data_out);
         quan::free(data_out);
         return result;
      }

      // blocking (yields) till transfer complete or timout
      uint8_t read(uint8_t addr, uint8_t len, uint8_t* data)
      {
         if( i2c_port::transfer_request( (addr << 1) | 1U, data, len) == true){
            return transfer_succeeded;
         }else{
            handle_errors();
            return transfer_failed;
         }
      }

      // blocking (yields) till transfer complete or timout
      uint8_t readRegisters(uint8_t addr, uint8_t reg,uint8_t len, uint8_t* data_in)
      {
         uint8_t  reg_out = reg;
         if ( ( write(addr,1,&reg_out) == transfer_succeeded ) && ( read(addr,len,data_in) == transfer_succeeded ) ){
            return transfer_succeeded;
         }else{
            // has reported in read or write
            return transfer_failed;
         }
      }

      // blocking (yields) till transfer complete or timout
      uint8_t readRegister(uint8_t addr, uint8_t reg, uint8_t* data_in)
      {
         return readRegisters(addr,reg,1,data_in);
      }

      uint8_t lockup_count() {return m_lockup_count;}
      void ignore_errors(bool b) { ; }
      AP_HAL::Semaphore* get_semaphore() {return & m_mutex;}
   private:
      static uint8_t m_lockup_count;
      static Quan::QuanSemaphore m_mutex;
      void handle_errors()
      {
         do_i2c_errno(i2c_port::i2c_errno);
         i2c_port::reset();
         ++ m_lockup_count;
      }
   };

   template <typename I2CPort>
   Quan::QuanSemaphore i2c_driver_t<I2CPort>::m_mutex{};

   template <typename I2CPort>
   uint8_t i2c_driver_t<I2CPort>::m_lockup_count = 0;

 /*
   todo
   for multiple i2c ports lookup the port based on the address?
   addresses must have been registered with a particular port
*/
   i2c_driver_t<i2c3_task> i2c3; 

}// namespace

namespace Quan{
AP_HAL::I2CDriver * get_quan_i2c_driver()
{
   return &i2c3;
}

}
namespace Quan{
    AP_HAL::I2CDriver * get_i2c_driver()
    { return new Empty::EmptyI2CDriver{new Empty::EmptySemaphore} ;}
}

#endif  // !defined QUAN_AERFLITE_BOARD
#endif  // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
