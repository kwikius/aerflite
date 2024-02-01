#include <AP_HAL_Quan/bmi_160.hpp>
#include <stm32f4xx.h>
#include <quan/stm32/tim.hpp>
#include <quan/stm32/tim/temp_reg.hpp>
#include <quan/stm32/get_raw_timer_frequency.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/three_d/vect.hpp>
#include <quan/stm32/millis.hpp>
#include <quan/stm32/rcc.hpp>
#include <quan/stm32/f4/exti/set_exti.hpp>
#include <quan/stm32/f4/syscfg/module_enable_disable.hpp>
#include <quan/stm32/push_pop_fp.hpp>

//###############################################
// therefore scheduler must start before BMI160
#include <AP_HAL/HAL.h>  // for scheduler delay
#include <AP_HAL/Scheduler.h>
//################################################

/*
see ArduPilot/libraries/AP_InertialSensor_BMI160.cpp
*/

extern const AP_HAL::HAL& hal;

Quan::bmi160::dma_rx_buffer_t Quan::bmi160::dma_rx_buffer 
__attribute__((section(".telem_buffer"))) 
__attribute__ ((aligned (1024)));

uint8_t const Quan::bmi160::dma_tx_buffer[13] = {
   Quan::bmi160::reg::gyro_data_lsb | 0x80,0,0,0,
   0,0,0,0,
   0,0,0,0,
   0};

bool  Quan::bmi160::m_initialised = false;

namespace {

   void spi_setup_rcc()
   {
      quan::stm32::rcc::get()->apb2enr.bb_setbit<12>(); 
      quan::stm32::rcc::get()->apb2rstr.bb_setbit<12>();
      quan::stm32::rcc::get()->apb2rstr.bb_clearbit<12>();
   }

   void spi_setup_pins()
   {
      quan::stm32::module_enable<Quan::spi::mosi_pin::port_type>();
      quan::stm32::module_enable<Quan::spi::miso_pin::port_type>();
      quan::stm32::module_enable<Quan::spi::sck_pin::port_type>();

      quan::stm32::apply<
         Quan::spi::mosi_pin
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::medium_fast
      >();

      quan::stm32::apply<
         Quan::spi::miso_pin
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
      >();

      quan::stm32::apply<
         Quan::spi::sck_pin
         ,quan::stm32::gpio::mode::af5  
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::medium_fast
      >();
   }

   void spi_setup_regs()
   {
      Quan::spi::device::get()->cr1 = 
         (    1 << 0 )        // (CPHA)
         |  ( 1 << 1 )      // (CPOL)
         |  ( 1 << 2 )      // (MSTR)
         |  ( 1 << 8 )      // (SSI)
         |  ( 1 << 9 )      // (SSM)
      ;
      // could try 10.5 MHz but is just out of spec for bmi160
      Quan::spi::set_clock_frequency_Hz<21000000,4>();// 5.25 MHz
   }

   void toggle_bmi16_not_CS()
   {
      // toggle ncs low --> high to make put bmi_160 in SPI mode
      for ( uint32_t i = 0U; i < 1000U; ++i){
         asm volatile("nop":::);
      }
      quan::stm32::clear<Quan::bmi160::not_CS>();
      for ( uint32_t i = 0U; i < 1000U; ++i){
         asm volatile("nop":::);
      }
      quan::stm32::set<Quan::bmi160::not_CS>();
   }

   void setup_bmi160_pins()
   {
      quan::stm32::module_enable<Quan::bmi160::not_CS::port_type>();
      quan::stm32::apply<
         Quan::bmi160::not_CS
         ,quan::stm32::gpio::mode::output
         ,quan::stm32::gpio::pupd::none
         ,quan::stm32::gpio::ospeed::medium_fast
         ,quan::stm32::gpio::ostate::high
      >();

      quan::stm32::module_enable<Quan::bmi160::not_DR::port_type>();
      quan::stm32::apply<
          Quan::bmi160::not_DR
         ,quan::stm32::gpio::mode::input
         ,quan::stm32::gpio::pupd::pull_up
      >();

     toggle_bmi16_not_CS();
  
   }

   void spi_setup_irqs()
   {
      NVIC_SetPriority(SPI1_IRQn,13); 
      NVIC_EnableIRQ(SPI1_IRQn);
   }
 } // ~namespace 

void Quan::spi::setup()
{
   spi_setup_rcc();
   spi_setup_pins();
   spi_setup_regs();
   spi_setup_irqs();
   enable();
}

//------------ bmi 160 ---- 
namespace {

   bool bmi_160_accel_setup()
   {
      Quan::bmi160::reg_write(Quan::bmi160::reg::cmd,Quan::bmi160::cmd::acc_pmu_mode_normal);
      hal.scheduler->delay(5);
      {
         Quan::bmi160::acc_conf_bits acc_conf;

         acc_conf.us  = false;  //in normal power mode undersampling should be disabled

      /*
           datasheet says ODR <= 1600
           ODR = 100/ (2^(8-odr_reg_value)
           therefore
           odr_reg_value = 8 - log2(100/ODR)
      */
         constexpr uint8_t odr_1600Hz = 12;
//         constexpr uint8_t odr_800Hz  = 11;
//         constexpr uint8_t odr_400Hz  = 10;
//         constexpr uint8_t odr_200Hz  =  9;
//         constexpr uint8_t odr_100Hz  =  8;
//         constexpr uint8_t odr_50Hz   =  7;
//         constexpr uint8_t odr_25Hz   =  6;

         acc_conf.odr = odr_1600Hz;   // n.b  divided by oversampling below
        
      /*
          when .us = false 
          the bwp reg value represents filtering
      */
   //      constexpr uint8_t bwp_OSR1 = 0b010; //  1x oversampling
   //      constexpr uint8_t bwp_OSR2 = 0b001; //  2x oversampling
         constexpr uint8_t bwp_OSR4 = 0b000; //  4x oversampling

         acc_conf.bwp = bwp_OSR4;   // 2x oversampling actual odr == 800 Hz
         
         Quan::bmi160::reg_write(Quan::bmi160::reg::acc_conf,acc_conf.value);
        
      }
   //--------------------
      {
         Quan::bmi160::acc_range_bits acc_range;

//         constexpr uint8_t range_2g  = 0b0011;
//         constexpr uint8_t range_4g  = 0b0101;
         constexpr uint8_t range_8g  = 0b1000;
    //     constexpr uint8_t range_16g = 0b1100;

         acc_range.range = range_8g;

         Quan::bmi160::reg_write(Quan::bmi160::reg::acc_range,acc_range.value);
         Quan::bmi160::set_accel_constant(8);
      }
      // read the array to clear dr
      uint8_t arr[20];
      Quan::bmi160::read(Quan::bmi160::reg::data_0,arr,20);

      return Quan::bmi160::check_no_errors();
   }


   bool bmi_160_gyro_setup()
   {
      // should this be last in fun?
      // can we setup flags then enable?
      Quan::bmi160::reg_write(Quan::bmi160::reg::cmd,Quan::bmi160::cmd::gyr_pmu_mode_normal);
      hal.scheduler->delay(81);

      {
         Quan::bmi160::gyr_conf_bits gyr_conf;

    //     constexpr uint8_t odr_3200Hz = 13;
           constexpr uint8_t odr_1600Hz = 12;
   //      constexpr uint8_t odr_800Hz  = 11;
   //      constexpr uint8_t odr_400Hz  = 10;
   //      constexpr uint8_t odr_200Hz  =  9;
   //      constexpr uint8_t odr_100Hz  =  8;
   //      constexpr uint8_t odr_50Hz   =  7;
   //      constexpr uint8_t odr_25Hz   =  6;

         gyr_conf.odr = odr_1600Hz;   // n.b  divided by oversampling below

   //      constexpr uint8_t bwp_OSR1 = 0b010; //  1x oversampling
   //      constexpr uint8_t bwp_OSR2 = 0b001; //  2x oversampling
         constexpr uint8_t bwp_OSR4 = 0b000; //  4x oversampling

         gyr_conf.bwp = bwp_OSR4;   // 4x oversampling actual odr == 800 Hz

         Quan::bmi160::reg_write(Quan::bmi160::reg::gyr_conf,gyr_conf.value);
      }
      {
         Quan::bmi160::gyr_range_bits gyr_range;

         constexpr uint8_t range_2000_deg_s  = 0b000;
   //      constexpr uint8_t range_1000_deg_s  = 0b001;
   //      constexpr uint8_t range_500_deg_s   = 0b010;
   //      constexpr uint8_t range_250_deg_s   = 0b011;
   //      constexpr uint8_t range_125_deg_s   = 0b100;

         gyr_range.range = range_2000_deg_s;
  
         Quan::bmi160::reg_write(Quan::bmi160::reg::gyr_range,gyr_range.value);
         Quan::bmi160::set_gyro_constant(2000);
      }
      // read the whole array
      uint8_t arr[20];
      Quan::bmi160::read(Quan::bmi160::reg::data_0,arr,20);

      return Quan::bmi160::check_no_errors();
   }

   void setup_exti()
   {
      quan::stm32::module_enable<quan::stm32::syscfg>(); 
      quan::stm32::set_exti_syscfg<Quan::bmi160::not_DR>();
      quan::stm32::set_exti_falling_edge<Quan::bmi160::not_DR>();
      quan::stm32::nvic_enable_exti_irq<Quan::bmi160::not_DR>();
      NVIC_SetPriority(
         quan::stm32::detail::get_exti_irq_num<Quan::bmi160::not_DR::pin_value>::value
         ,13
      );
   }

   void setup_dma()
   {
      // DMA2
      quan::stm32::rcc::get()->ahb1enr |= (1 << 22);
      for ( uint8_t i = 0; i < 20; ++i){
         asm volatile ("nop" : : :);
      }

      // RX
      DMA_Stream_TypeDef * dma_stream = DMA2_Stream0;
      constexpr uint32_t  dma_channel = 3;
      constexpr uint32_t  dma_priority = 0b01; // medium
      dma_stream->CR = (dma_stream->CR & ~(0b111 << 25U)) | ( dma_channel << 25U); //(CHSEL) select channel
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 16U)) | (dma_priority << 16U); // (PL) priority
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) ; // (MSIZE) 8 bit memory transfer
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) ; // (PSIZE) 8 bit transfer
      dma_stream->CR |= (1 << 10);// (MINC)
      dma_stream->CR &= ~(0b11 << 6) ; // (DIR ) peripheral to memory
      dma_stream->CR |= ( 1 << 4) ; // (TCIE)
    //  dma_stream->CR = ( dma_stream->CR &  ~(0b11 << 23)) | (0b01 << 23); // ( MBURST)
      dma_stream->CR &= ~(0b11 << 23);
      dma_stream->CR &=  ~(0b11 << 21); // ( PBURST)
      dma_stream->PAR = (uint32_t)&SPI1->DR;  // periph addr
      dma_stream->M0AR = (uint32_t) Quan::bmi160::dma_rx_buffer.arr+1; 
      dma_stream->NDTR = Quan::bmi160::dma_buffer_size;
      NVIC_SetPriority(DMA2_Stream0_IRQn,13); 
      NVIC_EnableIRQ(DMA2_Stream0_IRQn);
      DMA2->LIFCR = ( 0b111101 << 0) ; // Stream 0 clear flags

      // TX
      dma_stream = DMA2_Stream5;
      constexpr uint32_t  dma_channel1 = 3;
      dma_stream->CR = (dma_stream->CR & ~(0b111 << 25)) | ( dma_channel1 << 25); //(CHSEL) select channel
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 16)) | (dma_priority << 16U); // (PL) priority
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 13)) ; // (MSIZE) 8 bit memory transfer
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 11)) ; // (PSIZE) 8 bit transfer
      dma_stream->CR |= (1 << 10);// (MINC)
      dma_stream->CR = (dma_stream->CR & ~(0b11 << 6)) | (0b01 << 6) ; // (DIR )  memory to peripheral
     // dma_stream->CR |= ( 1 << 4) ; // (TCIE)
      dma_stream->PAR = (uint32_t)&SPI1->DR;  // periph addr
      dma_stream->M0AR = (uint32_t)Quan::bmi160::dma_tx_buffer; 
      dma_stream->NDTR = Quan::bmi160::dma_buffer_size;
      
      DMA2->HIFCR = ( 0b111101 << 6) ; // Stream 5 clear flags
      DMA2->LIFCR = ( 0b111101 << 0) ; // Stream 0 clear flags
   }

   bool bmi_160_interrupt_setup()
   { 
      {
         Quan::bmi160::int_map_1_bits int_map_1;
         int_map_1.drdy = true;
         Quan::bmi160::reg_write(Quan::bmi160::reg::int_map_1,int_map_1.value);
      }
      {
         Quan::bmi160::int_out_ctrl_bits int_out_ctrl;
         int_out_ctrl.int1_edge_ctrl = false; // level triggered
         int_out_ctrl.int1_output_en = true;  // enable output
         int_out_ctrl.int1_lvl       = false; // active low
         int_out_ctrl.int1_od        = false; // push-pull 

         Quan::bmi160::reg_write(Quan::bmi160::reg::int_out_ctrl,int_out_ctrl.value);
      }
      return Quan::bmi160::check_no_errors();
   }

  
}// namespace

bool Quan::bmi160::setup()
{
   /*
      check that board has been live for more than 0.5 secs to make sure VCC is good all over the board and periphs
   */
   uint32_t now = AP_HAL::millis();
   if ( now < 500){
      hal.scheduler->delay(500 - now);
   }

   setup_bmi160_pins();
   soft_reset();
   setup_exti();
   setup_dma();

   m_initialised =
      whoami_test()              &&
      bmi_160_accel_setup()      &&
      bmi_160_gyro_setup()       &&
      bmi_160_interrupt_setup()  &&
      check_no_errors();
   set_output_date_rate_Hz(1600);
   return m_initialised;
}

void Quan::bmi160::soft_reset()
{
   reg_write(Quan::bmi160::reg::cmd,Quan::bmi160::cmd::soft_reset);
   hal.scheduler->delay(200U);
   reg_read(0x7F);
}

bool Quan::bmi160::whoami_test()
{
   return reg_read(reg::chip_id) == val::chip_id;
}



