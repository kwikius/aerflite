

#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <quan/stm32/tim.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/constrain.hpp>
#include <quan/min.hpp>

#include "RCOutput.h"
/*
 Problem with the simple option is that all outputs go high
at same time
*/

extern const AP_HAL::HAL& hal;

namespace {

   typedef quan::stm32::tim4 rcout_1to4_timer;

#if defined QUAN_AERFLITE_BOARD
   typedef quan::stm32::tim1 rcout_5to7_timer;
#endif
   // use PWM on all channels
   typedef quan::mcu::pin<quan::stm32::gpiob,6> rcout_ch1;   
   typedef quan::mcu::pin<quan::stm32::gpiob,7> rcout_ch2;
   typedef quan::mcu::pin<quan::stm32::gpiob,8> rcout_ch3;
   typedef quan::mcu::pin<quan::stm32::gpiob,9> rcout_ch4;

#if defined QUAN_AERFLITE_BOARD
   typedef quan::mcu::pin<quan::stm32::gpiob,0> rcout_ch5;
   typedef quan::mcu::pin<quan::stm32::gpiob,1> rcout_ch6;
   typedef quan::mcu::pin<quan::stm32::gpioa,11> rcout_ch7;
#endif

   constexpr uint32_t min_freq_hz = 50U;
   constexpr uint32_t max_freq_hz = 400U;

   constexpr uint16_t max_pulsewidth = 2100U;
   constexpr uint16_t min_pulsewidth = 400U;

#if defined QUAN_AERFLITE_BOARD
   constexpr uint8_t  rcout_num_channels = 7;
#else
   constexpr uint8_t  rcout_num_channels = 4;
#endif

   void rcout_setup_1to4_pins()
   {
       quan::stm32::module_enable<quan::stm32::gpiob>();
       quan::stm32::apply<
         rcout_ch1
         ,quan::stm32::gpio::mode::af2
         ,quan::stm32::gpio::pupd::pull_down
         ,quan::stm32::gpio::ospeed::slow
      >();

      quan::stm32::apply<
         rcout_ch2
         ,quan::stm32::gpio::mode::af2
         ,quan::stm32::gpio::pupd::pull_down
         ,quan::stm32::gpio::ospeed::slow
      >();

      quan::stm32::apply<
         rcout_ch3
         ,quan::stm32::gpio::mode::af2
         ,quan::stm32::gpio::pupd::pull_down
         ,quan::stm32::gpio::ospeed::slow
      >();

      quan::stm32::apply<
         rcout_ch4
         ,quan::stm32::gpio::mode::af2
         ,quan::stm32::gpio::pupd::pull_down
         ,quan::stm32::gpio::ospeed::slow
      >();
   }

   void rcout_1to4_timer_setup()
   {
      
      quan::stm32::module_enable<rcout_1to4_timer>();
      rcout_setup_1to4_pins();
       
      // set all channels to pwm mode
      // period = 20 ms == 20,000 us
      // set 1 usec tick
      {
         // set all channels to PWM mode 1
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc1s = 0b00;  // output
         ccmr1.oc1m = 0b110; // PWM mode 1
         ccmr1.cc2s = 0b00;  // output
         ccmr1.oc2m = 0b110; // PWM mode 1
         rcout_1to4_timer::get()->ccmr1.set(ccmr1.value);
      }
      {
         quan::stm32::tim::ccmr2_t ccmr2 = 0;
         ccmr2.cc3s = 0b00;  // output
         ccmr2.oc3m = 0b110; // PWM mode 1
         ccmr2.cc4s = 0b00;  // output
         ccmr2.oc4m = 0b110; // PWM mode 1
         rcout_1to4_timer::get()->ccmr2.set(ccmr2.value);
      }
      {
         // default disabled
         quan::stm32::tim::ccer_t ccer = 0;
//         ccer.cc1p =  false;
//         ccer.cc1np = false;
//         ccer.cc1e =  false; // disable ch1 default
//         ccer.cc2p =  false;
//         ccer.cc2np = false;
//         ccer.cc2e =  false; // disable ch2
//         ccer.cc3p =  false;
//         ccer.cc3np = false;
//         ccer.cc3e =  false; // disable ch3
//         ccer.cc4p =  false;
//         ccer.cc4np = false;
//         ccer.cc4e =  false; // disable ch4
         rcout_1to4_timer::get()->ccer.set(ccer.value);
      }
      // set all ccr regs to center except thrust set low
      rcout_1to4_timer::get()->ccr1 = 1500;
      rcout_1to4_timer::get()->ccr2 = 1500;
      rcout_1to4_timer::get()->ccr3 = 900;  // thrust
      rcout_1to4_timer::get()->ccr4 = 1500;
      // set the ocpe (preload) bits in ccmr1 , ccmr2 
      rcout_1to4_timer::get()->ccmr1 |= ((1 << 3) | (1 << 11));
      rcout_1to4_timer::get()->ccmr2 |= ((1 << 3) | (1 << 11));

      constexpr uint32_t timer_1to4_freq = quan::stm32::get_raw_timer_frequency<rcout_1to4_timer>();

      rcout_1to4_timer::get()->psc = (timer_1to4_freq / 1000000 )-1;
      rcout_1to4_timer::get()->arr = 20000 -1;
      rcout_1to4_timer::get()->cnt = 0x0;
      {
         quan::stm32::tim::cr1_t cr1 = 0;
         cr1.arpe = true ;// auto preload
         rcout_1to4_timer::get()->cr1.set(cr1.value);
      }
      rcout_1to4_timer::get()->sr = 0;
   }

#if defined QUAN_AERFLITE_BOARD

   void rcout_setup_rc_out_5to7_pins()
   {
      quan::stm32::module_enable<quan::stm32::gpioa>();

      // set up as mode TIM1_CH2N
      quan::stm32::apply<
         rcout_ch5
         ,quan::stm32::gpio::mode::af1
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::slow
      >();

      // set up as mode TIM1_CH3N
      quan::stm32::apply<
         rcout_ch6
         ,quan::stm32::gpio::mode::af1
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::slow
      >();

       // set up as TIM1 CH4
       quan::stm32::apply<
         rcout_ch7
         ,quan::stm32::gpio::mode::af1
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::slow
      >();
   }

   void rcout_5to7_timer_setup()
   {
      rcout_setup_rc_out_5to7_pins();

      quan::stm32::module_enable<rcout_5to7_timer>();
      constexpr uint32_t timer_5to7_freq = quan::stm32::get_raw_timer_frequency<rcout_5to7_timer>();
      // set the granularity to 1 us, period to 50 Hz
      rcout_5to7_timer::get()->psc = (timer_5to7_freq / 1000000 )-1;
      rcout_5to7_timer::get()->rcr = 0;
      rcout_5to7_timer::get()->arr = 20000 -1;  // 50 Hz
      rcout_5to7_timer::get()->cnt = 0x0;
    
      {
         quan::stm32::tim::cr1_t cr1 = 0;  
         cr1.urs = true;   // Only counter overflow/underflow generates an update interrupt or DMA request if enabled
         cr1.arpe = true ; // recommended for these TIM1/8 with PWM 
         rcout_5to7_timer::get()->cr1.set(cr1.value);
      }

      {
         quan::stm32::tim::cr2_t cr2 = 0; 
         cr2.ois2n = false;  // output idle state state when moe == 0
         cr2.ois3n = false;  // output idle state state when moe == 0
         cr2.ois4 = false;
         cr2.ccpc = false;  
         cr2.ccds = false;
         rcout_5to7_timer::get()->cr2.set(cr2.value);
      }

       {
        // set pwm mode on ch2 pwm mode in OCxM in TIM1_CCMRx
         quan::stm32::tim::ccmr1_t ccmr1 = 0;
         ccmr1.cc2s = 0b00; // channel 2 output
         ccmr1.oc2m = 0b110;  // PWM mode 1
         ccmr1.oc2pe = true;  // recommended setting
         rcout_5to7_timer::get()->ccmr1.set(ccmr1.value);
      }

      {
        // set pwm mode on ch3 in OCxM in TIM1_CCMRx
        // set pwm mode on ch4 in OCXM 
         quan::stm32::tim::ccmr2_t ccmr2 = 0;
         ccmr2.cc3s = 0b00; // channel 3 output
         ccmr2.oc3m = 0b110;  // PWM mode 1
         ccmr2.oc3pe = true;  // recommended setting
         ccmr2.cc4s = 0b00; // channel 4 output
         ccmr2.oc4m = 0b110; // pwm mode 1
         ccmr2.oc4pe = true;
         rcout_5to7_timer::get()->ccmr2.set(ccmr2.value);
      }

      rcout_5to7_timer::get()->ccr2 = 1500;
      rcout_5to7_timer::get()->ccr3 = 1500;
      rcout_5to7_timer::get()->ccr4 = 1500;

      {
        // set up TIM1_CH2N,TIM1_CH3N, TIM1_CH4 polarity high pulse
         quan::stm32::tim::ccer_t ccer = 0;
//         ccer.cc2np = false ; // TIM1_CH2N is positive pulse
//         ccer.cc2e = false  ; // enable TIM1_CH2N 
//         ccer.cc2ne = false ; // true to enable TIM1_CH2N, start disabled
//         ccer.cc3np = false ; // TIM1_CH3N is positive pulse
//         ccer.cc3e = false  ; // enable TIM1_CH3N
//         ccer.cc3ne = false ; // true to enable TIM1_CH3N, start disabled
//         ccer.cc4e  = false ; // Ch 4 start disabled
//         ccer.cc4p = false ; // ch4 positive pulse
         rcout_5to7_timer::get()->ccer.set(ccer.value);
      }
 
      {
         quan::stm32::tim::bdtr_t bdtr = 0;
         bdtr.moe = true;  // main output enable
         bdtr.aoe = true;  // automatic output enable
         bdtr.ossi = true; // offs stet select, enable the outputs in idle
         bdtr.ossr = true; // enable the outputs in run mode
         bdtr.bke = false;   // disable braek inputs
         bdtr.bkp = false;   // break polarity is dont care
         bdtr.dtg = 0;
         rcout_5to7_timer::get()->bdtr.set(bdtr.value);
      }
      rcout_5to7_timer::get()->sr = 0;
   }

#endif

   void rcout_timer_setup()
   {
      rcout_1to4_timer_setup();
#if defined QUAN_AERFLITE_BOARD
      rcout_5to7_timer_setup();
#endif
   }

   void start_1to4_timer()
   {
      rcout_1to4_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }

#if defined QUAN_AERFLITE_BOARD
   void start_5_6_timer()
   {
      rcout_5to7_timer::get()->cr1.bb_setbit<0>(); // (CEN)
   }
#endif

   void rcout_start_timers()
   {
      start_1to4_timer();
#if defined QUAN_AERFLITE_BOARD
      start_5_6_timer();
#endif
   }

   struct rc_outputs_t final : public AP_HAL::RCOutput {
      void     init(void*)
      {
         rcout_timer_setup();
         rcout_start_timers();
      }
      // no op unless the mask is for all  of 0 to 3 channels
      // and or all of 4 and 5 channels for aerflite
      void set_freq(uint32_t chmask, uint16_t freq_hz)
      {
         if ((freq_hz >= min_freq_hz) && ( freq_hz <= max_freq_hz)){

            if ( (chmask & 0x0000000F) == 0x0000000F ){
               rcout_1to4_timer::get()->arr = (1000000U / freq_hz) -1 ;
            }
#if defined QUAN_AERFLITE_BOARD
            if ( (chmask & 0x00000070) == 0x00000070 ){
               rcout_5to7_timer::get()->arr = (1000000U / freq_hz) -1 ;
            }
#endif
         }
      }

      uint16_t get_freq(uint8_t ch)
      {
         if (ch < 4){
            return 1000000U / ( rcout_1to4_timer::get()->arr + 1);
         }
#if defined QUAN_AERFLITE_BOARD
         if ( ch < 7) {
           return 1000000U / ( rcout_5to7_timer::get()->arr + 1);
         }
#endif
         return 0;
      }

      void enable_ch(uint8_t ch)
      {
       //  hal.console->printf("Enabling servo[%u]\n",static_cast<unsigned>(ch));
         if ( ch < 4){
            rcout_1to4_timer::get()->ccer |= (1 << (ch * 4));
            return;
         }
#if defined QUAN_AERFLITE_BOARD
         if ( ch < 6){
            rcout_5to7_timer::get()->ccer |= (1U << ((ch - 3U)*4U + 2U));
            return;
         }else{
            if (ch == 6){  // ccer.cc4e
               rcout_5to7_timer::get()->ccer.bb_setbit<12>();
            }
         }
#endif
      }

      void disable_ch(uint8_t ch)
      {
       //  hal.console->printf("Disabling servo[%u]\n",static_cast<unsigned>(ch));
         if ( ch < 4){
            rcout_1to4_timer::get()->ccer &= ~(1 << (ch * 4));
            return;
         }
#if defined QUAN_AERFLITE_BOARD
         if ( ch < 6){
            rcout_5to7_timer::get()->ccer &= ~(1U << ((ch - 3U)*4U + 2U));
            return;
         }else{
            if (ch == 6){  // ccer.cc4e
               rcout_5to7_timer::get()->ccer.bb_clearbit<12>();
            }
         }
#endif
      }
     
      void write(uint8_t ch, uint16_t pulsewidth_in_us)
      {
         if ( ch < 4){
            volatile uint32_t * ccrs = &rcout_1to4_timer::get()->ccr1;
            ccrs[ch] = quan::constrain(pulsewidth_in_us,min_pulsewidth, max_pulsewidth);
            return;
         }
#if defined QUAN_AERFLITE_BOARD
         if ( ch < 7){
            volatile uint32_t * ccrs = &rcout_5to7_timer::get()->ccr2;
            ccrs[ch - 4] = quan::constrain(pulsewidth_in_us,min_pulsewidth, max_pulsewidth);
         }
#endif
      }

      void write(uint8_t ch, uint16_t* period_us, uint8_t len_in)
      {
         uint8_t const end = quan::min(ch + len_in,rcout_num_channels);
         for (uint8_t i = ch,period_idx = 0; i < end ; ++i, ++period_idx){
            this->write(i, period_us[period_idx]);
         }
      }
      
      uint16_t read(uint8_t ch)
      {
        if ( ch < 4){
           volatile uint32_t const * ccrs = &rcout_1to4_timer::get()->ccr1;
           return ccrs[ch] ;
        }
#if defined QUAN_AERFLITE_BOARD
        if ( ch < 7){
            volatile uint32_t * ccrs = &rcout_5to7_timer::get()->ccr2;
            return ccrs[ch - 4];
        }
#endif
        return 0;
      }

      void read(uint16_t* period_us, uint8_t len)
      {
          for (uint8_t i = 0U; i < len; ++i) {
            if ( i < rcout_num_channels){
               period_us[i] = this->read(i);
            }else{
               period_us[i] = 1500U;
            }
          }
      }

   } rc_outputs;

} // namespace 

namespace Quan{

   AP_HAL::RCOutput * get_rc_outputs() 
   {
      return & rc_outputs;
   }
}



