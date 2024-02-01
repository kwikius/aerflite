
#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>

#include "RCInput.h"

/*
TODO  need to detect overflows
More than one overflow between pulses
clearly means the pulse is invalid
so should detect that.
Possibly also detetct no input and sync lost
Though I guess the main code is doing that
*/

namespace {

#if defined QUAN_AERFLITE_BOARD
   typedef quan::stm32::tim14 rc_in_timer;
   // TIM14_CH1
   typedef quan::mcu::pin<quan::stm32::gpioa,7> rc_input_pin;
#else
   // TIM1_CH4
   typedef quan::stm32::tim1 rc_in_timer;
   typedef quan::mcu::pin<quan::stm32::gpioa,11> rc_input_pin;
#endif
   
   constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<rc_in_timer>();
   constexpr uint16_t min_sync_pulse = 3000U ;
   constexpr uint16_t min_pulsewidth = 900U ;
   constexpr uint16_t max_pulsewidth = 2100U ;
   constexpr uint8_t min_num_channels = 5U;
   constexpr uint8_t max_num_channels = 11U;

   void rc_input_timer_setup()
   {
      quan::stm32::module_enable<rc_input_pin::port_type>();
      quan::stm32::module_enable<rc_in_timer>();

      // capture on falling edge
      quan::stm32::apply<
         rc_input_pin,
#if defined QUAN_AERFLITE_BOARD
         quan::stm32::gpio::mode::af9,  
#else
         quan::stm32::gpio::mode::af1,  
#endif
         quan::stm32::gpio::pupd::pull_up
      >();

#if defined QUAN_AERFLITE_BOARD
     // configure TIM14_CH1 as input
      {
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s   = 0b01;// CC1 channel is configured as input, IC1 is mapped on TI1
         ccmr1.ic1f   = 0b00 ;// no filter 
         ccmr1.ic1psc = 0b00; // no prescaler
         rc_in_timer::get()->ccmr1.set(ccmr1.value);
      }
      {
         quan::stm32::tim::ccer_t ccer = 0;
         ccer.cc1p = true; // capture falling edges
         ccer.cc1np = false; // capture falling edges
         ccer.cc1ne = false; // applies to output only
         ccer.cc1e = true;  // enable capture
         rc_in_timer::get()->ccer.set(ccer.value);
      }

      {
         quan::stm32::tim::dier_t dier= 0;
         dier.cc1ie = true;
         rc_in_timer::get()->dier.set(dier.value);
      }
#else
     // configure TIM1_CH4 as input
      {
         quan::stm32::tim::ccmr2_t ccmr2 = 0;
         ccmr2.cc3s = 0b10;// CC3 channel is configured as input, IC3 is mapped on TI4
         ccmr2.ic3f = 0b00 ;// no filter 
         ccmr2.ic3psc =0b00; // no prescaler
         rc_in_timer::get()->ccmr2.set(ccmr2.value);
      }
      {
         quan::stm32::tim::ccer_t ccer = 0;
         ccer.cc3p = true; // capture falling edges
         ccer.cc3np = false; // capture falling edges
         ccer.cc3ne = false; // applies to output only
         ccer.cc3e = true;  // enable capture
         rc_in_timer::get()->ccer.set(ccer.value);
      }
      {
         quan::stm32::tim::dier_t dier= 0;
         dier.cc3ie = true;
         rc_in_timer::get()->dier.set(dier.value);
      }
#endif
      rc_in_timer::get()->psc = (timer_freq / 1000000 )-1;
      rc_in_timer::get()->arr = 0xFFFF;
      rc_in_timer::get()->cnt = 0x0;
      rc_in_timer::get()->sr = 0;

#if defined QUAN_AERFLITE_BOARD
      NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn,15); // low priority
      NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
#else
      NVIC_SetPriority(TIM1_CC_IRQn,15); // low priority
      NVIC_EnableIRQ(TIM1_CC_IRQn);
#endif
   }

   void start_timer()
   {
      rc_in_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }

   uint16_t last_edge = 0U;
   volatile uint8_t  m_num_channels = 0;
   volatile uint8_t  channel_idx = max_num_channels + 3;
   volatile bool     m_new_input = false;

   bool have_sync()
   {
      return channel_idx <= max_num_channels;
   }

   volatile uint16_t m_input_rc_channels [max_num_channels] ;
   int16_t  m_overrides[max_num_channels];

   void on_edge()
   {
#if defined QUAN_AERFLITE_BOARD
      uint16_t const edge = rc_in_timer::get()->ccr1;
#else
      uint16_t const edge = rc_in_timer::get()->ccr3;
#endif
      uint16_t const pulse = edge - last_edge;
      last_edge = edge;
      if (pulse < min_sync_pulse ) {
         if (channel_idx < max_num_channels){
            m_input_rc_channels[channel_idx++]
            = quan::constrain(
               pulse
               ,min_pulsewidth 
               ,max_pulsewidth 
            );
         } 
      }else{ // sync pulse
         if(channel_idx <= max_num_channels){
            m_num_channels = channel_idx ;
            m_new_input = true;
            channel_idx = 0;
         }else{ // waiting for enough sync pulses
            -- channel_idx;
         }
      }
   }

} // namespace

#if defined QUAN_AERFLITE_BOARD
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void TIM8_TRG_COM_TIM14_IRQHandler()
#else
extern "C" void TIM1_CC_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
extern "C" void TIM1_CC_IRQHandler()
#endif
{
   rc_in_timer::get()->sr.set(0); // clear flags
   on_edge();
}

namespace {
   struct rc_inputs_t final : public AP_HAL::RCInput{
      void init(void* )
      {
         // TODO set thrust to min pulsewidth
         for ( auto & pulse : m_input_rc_channels)
         { pulse = (min_pulsewidth + max_pulsewidth)/2;}
     
         rc_input_timer_setup();
         start_timer();
      }

    /**
        * Return true if there has been new input since the last read()
        * call. This call also clears the new_input flag, so once it
        * returns true it won't return true again until another frame is
        * received.
        */

      bool new_input() 
      {
         if ( have_sync()){
            bool const result = m_new_input;
            m_new_input = false;
            return result;
         }else{
            return false;
         }
      }

    /**
        * Return the number of input channels in last read()
        */
      uint8_t num_channels() 
      {
         if ( have_sync()){
            return m_num_channels;
         }else{
            return 0U;
         }
      }

    /**
        * read(uint8_t):
        * Read a single channel at a time
        */

      uint16_t read(uint8_t ch)
      {
         if ( have_sync() && (ch < max_num_channels) ){
            if ( m_overrides[ch] > 0){
               return m_overrides[ch];
            }else{
               return m_input_rc_channels[ch];
            }
         }else{
            return (ch == 2)?900:1500; /* thrust (ch[2]) should be low, for safety */
         }
      }

     /**
        * read(uint16_t*,uint8_t):
        * Read an array of channels, return the valid count
        */

      uint8_t read(uint16_t* periods, uint8_t len_in)
      {
         if ( have_sync()){
            uint8_t const len = quan::min(len_in,max_num_channels);
            for ( uint8_t i = 0; i < len ; ++i){
               periods[i] = this->read(i);
            }
            return len;
         }else{
            return 0U;
         }
      }

      bool set_overrides(int16_t *overrides, uint8_t len_in) 
      {
         uint8_t const len = quan::min(len_in,max_num_channels);
         bool result = false;
         for ( uint8_t i = 0; i < len ; ++i){
            result |= this->set_override(i, overrides[i]);
         }
         return result;
      }

    /**
        * Overrides: these are really grody and don't belong here but we need
        * them at the moment to make the port work.
        * case v of:
        *  v == -1 -> no change to this channel
        *  v == 0  -> do not override this channel
        *  v > 0   -> set v as override.
        */
      bool set_override(uint8_t channel, int16_t override) 
      {
         if ((channel < max_num_channels ) && ( override >= 0) ) {
            m_overrides[channel] = override;
            return override != 0;
         }else{
            return false;
         }
      }

      void clear_overrides()
      {
         for (auto & v : m_overrides) {
            v = 0;
         }
      }
   } rc_inputs;
}

namespace Quan{

   AP_HAL::RCInput *  get_rc_inputs() 
   {
      return & rc_inputs;
   }

}
