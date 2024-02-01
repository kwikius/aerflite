
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include <semphr.h>
#include <task.h>
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include "AnalogIn.h"
#include <Filter/Filter.h>
#include <Filter/Butter.h>
#include <atomic>
#include <quan/min.hpp>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/push_pop_fp.hpp>
#include <quan/constrain.hpp>

extern const AP_HAL::HAL& hal;

//using namespace Quan;
/*
APM analog inputs available


g.rssi_pin
airspeed
voltage
current
ANALOG_INPUT_NONE

 pin number 254 == vcc whether 3.3 V or 5v?
 pin number 255 == ANALOG_INPUT_NONE

function pins 
board_voltage
servo_rail_voltage

PC0   ADC123_IN10 ( there but not currently used in aerflite)
PC1   ADC123_IN11
PC2   ADC123_IN12 (aerflite)
PC3   ADC123_IN13
PC4   ADC12_IN14
PC5   ADC12_IN15 (aerflite)

*/

namespace {
   
   // n.b could use one of the rc out timers if timers get short
   // would be restricted to their update rate e.g 50 Hz
   typedef quan::stm32::tim8 adc_timer;

#if defined QUAN_AERFLITE_BOARD
   constexpr uint32_t num_adc_channels = 5;

   //  typedef quan::mcu::pin<quan::stm32::gpioc,0> analog_video_sample - ignore
   typedef quan::mcu::pin<quan::stm32::gpioc,1> analog_pin1;  // analog in
   typedef quan::mcu::pin<quan::stm32::gpioc,2> analog_pin2;  // rssi in
   typedef quan::mcu::pin<quan::stm32::gpioc,3> analog_pin3;  // battery current
   typedef quan::mcu::pin<quan::stm32::gpioc,4> analog_pin4;  // battery voltage
   typedef quan::mcu::pin<quan::stm32::gpioc,5> analog_pin5;  // airspeed

#else
   constexpr uint32_t num_adc_channels = 4;

   typedef quan::mcu::pin<quan::stm32::gpioc,0> analog_pin1;  // battery voltage
   typedef quan::mcu::pin<quan::stm32::gpioc,1> analog_pin2;  // battery current
   typedef quan::mcu::pin<quan::stm32::gpioc,3> analog_pin3;  // airspeed
   typedef quan::mcu::pin<quan::stm32::gpioc,4> analog_pin4;  // rssi

#endif
   // raw A2D values updated by DMA
   // N.B. last index is a dummy to trap out of range
   volatile uint16_t adc_results [num_adc_channels + 1 ] __attribute__((section(".telem_buffer"))) = {0};

   template <typename Pin>
   void setup_adc_pin()
   {
      quan::stm32::module_enable<typename Pin::port_type>();
      quan::stm32::apply<
         Pin
         ,quan::stm32::gpio::mode::analog
         ,quan::stm32::gpio::pupd::none
      >();
   };

   // setup adc timer to overflow at 100 Hz
   // setup TRGO --> ADC on overflow
   void setup_adc_timer()
   {
      setup_adc_pin<analog_pin1>();
      setup_adc_pin<analog_pin2>();
      setup_adc_pin<analog_pin3>();
      setup_adc_pin<analog_pin4>();
#if defined QUAN_AERFLITE_BOARD
      setup_adc_pin<analog_pin5>();
#endif

      quan::stm32::module_enable<adc_timer>();

      constexpr uint32_t timer_freq = quan::stm32::get_raw_timer_frequency<adc_timer>();
      static_assert((timer_freq % static_cast<uint32_t>(1000000U))==0U,"unexpected raw timer frequency");
      constexpr uint32_t psc = (timer_freq / static_cast<uint32_t>(1000000U)) - 1U;
      // presc to 1 MHz
      adc_timer::get()->cr1 = 0;
      adc_timer::get()->psc = psc;
      adc_timer::get()->arr = 10000U-1U;  // 100Hz overflow 
      adc_timer::get()->cnt = 0;
      adc_timer::get()->sr = 0;

      // trgo of adc_timer is used to fire adc conversion sequence
      {
         quan::stm32::tim::cr2_t cr2 = adc_timer::get()->cr2.get();
         cr2.mms = 0b010; // TRGO is timer update
         adc_timer::get()->cr2.set(cr2.value);
      }

      // setup adc
      // enable  and reset the adc1 module
      quan::stm32::rcc::get()->apb2enr |= (1 << 8); //( ADC1)
      quan::stm32::rcc::get()->apb2rstr |= ( 1 << 8 ); // (ADC1)
      quan::stm32::rcc::get()->apb2rstr &= ~( 1 << 8 );// (ADC1)

      // set adc clk to 21 MHz
      // PCLK == sysclk/2 == 84 MHz
      // so set /4 --> 21 MHz
      // but maybe *2 so set /8
      // TODO verify which it is!
      ADC->CCR |= (0b11 << 16);

      ADC1->CR2  |= (0b01 << 28); // (EXTEN) external trigger rising edge
      ADC1->CR2  |= (0b1110 << 24) ;// (EXTSEL) TIM8 TRGO

      ADC1->CR1 &= ~(0b11 << 24); // 12 bit conversion
      ADC1->CR1 |= (1 << 8);     // (SCAN)

      constexpr uint32_t sati_bits = 0b011; // reg sampling time bits
// if quantracker
       // sequence  ADC123_IN10,ADC123_IN11,ADC123_IN1,ADC12_IN14
       // == ch10, ch11, ch13, ch14  
// if aerflite
      // ch11, ch12, ch13, ch14,ch15

      ADC1->CR2  |= (1 << 9); //  (DDS) dma request continue sfter all done in sequence
#if defined QUAN_AERFLITE_BOARD
       // sampling time
       // SMPR1 for ch10 [0:2)]to 18 [24:26]
      // setup channels 11 to 15
      ADC1->SMPR1 =  ( sati_bits << (1*3) ) | ( sati_bits << (2*3)) | ( sati_bits << (3*3)) | ( sati_bits << (4*3)) | ( sati_bits << 5*3);
      // conversion sequence just setup in order of ports
      ADC1->SQR3 = (11 << (0 *5)) | ( 12 << (1*5)) | ( 13 << (2*5)) | ( 14 << (3*5)) | (15 << (4*5));
#else
      ADC1->SMPR1 = ( sati_bits << 0) | ( sati_bits << 3) | ( sati_bits << 9) | ( sati_bits << 12);
      ADC1->SQR3 = (10 << 0) | ( 11 << 5) | ( 13 << 10) | ( 14 << 15);
#endif
     // setup number of channels to convert
      ADC1->SQR1 |= ((num_adc_channels-1) << 20);

      // ADC1 DMA ------------------------------------------------------------------------
      // USE DMA2.Stream4.Channel0  
      // Enable and reset DMA2 module
      // add a check that it isnt already enable  
      quan::stm32::rcc::get()->ahb1enr |= (1 << 22);
      for ( uint8_t i = 0; i < 20; ++i){
         asm volatile ("nop" : : :);
      }

      DMA_Stream_TypeDef * dma_stream = DMA2_Stream4;
      constexpr uint32_t  dma_channel = 0;
      constexpr uint32_t  dma_priority = 0b00; // low
      dma_stream->CR = (dma_stream->CR & ~(0b111 << 25)) | ( dma_channel << 25); //(CHSEL) select channel
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 16)) | (dma_priority << 16U); // (PL) priority
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) | (01 << 13); // (MSIZE) 16 bit memory transfer
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) | (01 << 11); // (PSIZE) 16 bit transfer
      dma_stream->CR |= (1 << 10);// (MINC)
      dma_stream->CR &= ~(0b11 << 6) ; // (DIR ) peripheral to memory
      dma_stream->CR |= ( 1 << 4) ; // (TCIE)

      dma_stream->PAR = (uint32_t)&ADC1->DR;  // periph addr
      dma_stream->M0AR = (uint32_t)adc_results; 
      dma_stream->NDTR = num_adc_channels;

      NVIC_SetPriority(DMA2_Stream4_IRQn,15);  // low prio
      NVIC_EnableIRQ(DMA2_Stream4_IRQn);

      DMA2->HIFCR = (0b111101 << 0) ; // clear flags for Dma2 Stream 4
      ADC1->CR2 |= (1 << 8);    //  (DMA) enable adc DMA
      DMA2_Stream4->CR |= (1 << 0); // (EN)  enable DMA
      ADC1->CR2 |= (1 << 0); // (ADON)
   }

   void start_adc_timer()
   {
      adc_timer::get()->cr1.bb_setbit<0>(); //(CEN)
   }

   SemaphoreHandle_t adc_semaphore;

   BaseType_t higherPriorityTaskWoken = 0;
 
}

// TODO add ADC Overrun interrupt

extern "C" void DMA2_Stream4_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

extern "C" void DMA2_Stream4_IRQHandler() 
{   
   DMA2_Stream4->CR &= ~(1 << 0); // (EN)
   while(DMA2_Stream4->CR & (1 << 0)){;}
   DMA2->HIFCR = (0b111101 << 0) ; // clear flags for Dma2 Stream 4
   DMA2_Stream4->M0AR = (uint32_t)adc_results;
   DMA2_Stream4->NDTR = num_adc_channels;
   DMA2_Stream4->CR |= (1 << 0); // (EN)

   xSemaphoreGiveFromISR(adc_semaphore,&higherPriorityTaskWoken );

   portEND_SWITCHING_ISR(higherPriorityTaskWoken);
}

namespace {

   // various filters

   typedef Filter<float> filter;

   struct dummy_filter_t final : public Filter<float>{
      float apply(float sample){ return 0.f;}
      void reset(){;}
   };

   struct simple_filter_t final : public Filter<float>{
       float apply(float sample){ return sample;}
       void reset(){;}
   };

   struct average_filter_t final : public Filter<float>{
      average_filter_t(float init_value, float gain) 
      : m_last_value{init_value}, m_gain{quan::constrain(gain, 0.f, 1.f)}{}
      float apply(float sample)
      { 
         return m_last_value = m_last_value * (1.f - m_gain) + sample * m_gain;
      }
      void reset(){;}
      private:
        float m_last_value;
        float const m_gain;
   };

   average_filter_t   batt_voltage_adc{0.f, 0.2f};
   average_filter_t   batt_current_adc{0.f, 0.2f};
   average_filter_t   airspeed_adc{2.5f , 0.2f};
   average_filter_t   rssi_adc{0.f, 0.5f};
#if defined QUAN_AERFLITE_BOARD
   average_filter_t   analog_in_adc{2.5f , 0.2f};
#endif
   dummy_filter_t     dummy_filter;
   

#if defined QUAN_AERFLITE_BOARD
  filter* filters[6] = { 
      &analog_in_adc,
      &rssi_adc,
      &batt_current_adc,
      &batt_voltage_adc,
      &airspeed_adc,
      &dummy_filter
   };
#else
   filter* filters[5] = { 
      &batt_voltage_adc,
      &batt_current_adc,
      &airspeed_adc,
      &rssi_adc,
      &dummy_filter
   };
#endif

   float raw_adc_voltages[num_adc_channels +1 ] = {0.f};
   float filtered_adc_values[num_adc_channels + 1] = {0.f};

   // n.b though the actual voltage is 0 to 3.3V
   // Ardupilot expects everything in a 0 to 5 v range
   // so we scale it as if 5V here
   void process_adc()
   {
      for (uint8_t i = 0; i < num_adc_channels ; ++i)
      {
         float const voltage = (adc_results[i] * 5.f) / 4096;
         raw_adc_voltages[i] = voltage;
         filtered_adc_values[i]= filters[i]->apply(voltage);
      }
   };

   void adc_task( void * params){

      adc_semaphore = xSemaphoreCreateBinary();
      for (;;){
         xSemaphoreTake(adc_semaphore, portMAX_DELAY);
         process_adc();
      }
   }

   TaskHandle_t adc_task_handle;
   void * dummy_params;

   void create_adc_task()
   {
      xTaskCreate(
         adc_task,"adc_task",
         500,
         &dummy_params,
         tskIDLE_PRIORITY + 2, // want slightly higher than apm task priority
         & adc_task_handle
      ) ;
   }   

   template <uint8_t Pin>
   struct analog_source  final : public AP_HAL::AnalogSource{

      float voltage_latest(){return raw_adc_voltages[Pin];}
   
      float voltage_average() {return filtered_adc_values[Pin];}
      // scaled 0 to 5 V
      float voltage_average_ratiometric(){ return voltage_average() ;}
      // should be safe else
      // caller would have to know about impl
      float read_latest() {return voltage_latest();}
      float read_average() {return voltage_average();}
      // not available
      void set_pin(uint8_t p){}
      void set_stop_pin(uint8_t p){}
      void set_settle_time(uint16_t settle_time_ms){}
   };

#if defined QUAN_AERFLITE_BOARD

   analog_source<0> undedicated_analog_in;
   analog_source<1> rc_rssi;
   analog_source<2> battery_current;
   analog_source<3> battery_voltage;
   analog_source<4> airspeed;
   analog_source<5> dummy;
   AP_HAL::AnalogSource* analog_sources[num_adc_channels +1] =
   {
      & undedicated_analog_in
     ,& rc_rssi
     ,& battery_current
     ,& battery_voltage
     ,& airspeed
     ,& dummy
   };

#else

   analog_source<0> battery_voltage;
   analog_source<1> battery_current;
   analog_source<2> airspeed;
   analog_source<3> rssi;
   analog_source<4> dummy;
   AP_HAL::AnalogSource* analog_sources[num_adc_channels +1] =
   {
     & battery_voltage
     ,& battery_current
     ,& airspeed
     ,& rssi
     ,& dummy
   };

#endif

   struct analog_in_t final : public AP_HAL::AnalogIn{
      analog_in_t(){}
      void init(void* )
      {
         setup_adc_timer();
         create_adc_task();
         start_adc_timer();
      }

      AP_HAL::AnalogSource* channel(int16_t n) 
      {
         // set out of range chennels to channels[4] (dummy channel)
         return analog_sources[(((n >= 0 ) && ( n < static_cast<int16_t>(num_adc_channels) ))?n:num_adc_channels)];
      }

      // should be reading the actual board voltage
      // we can monitor the VBat voltage and
      // if it drops below a level
      // drop this one maybe
      float board_voltage(void)
      {
          return 3.3f;
      }

   } analog_in;

}// namespace 

namespace Quan{

  AP_HAL::AnalogIn* get_analog_in()
  {
    return & analog_in;
  }  

}

#endif //  CONFIG_HAL_BOARD == HAL_BOARD_QUAN
