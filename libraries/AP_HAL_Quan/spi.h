#ifndef ARDUPILOT_LIBRARIES_AP_HAL_QUAN_SPI_H_INCLUDED
#define ARDUPILOT_LIBRARIES_AP_HAL_QUAN_SPI_H_INCLUDED

#include <cstdint>
#include <quan/meta/log2.hpp>
#include <quan/meta/rational.hpp>
#include <quan/stm32/spi.hpp>
#include <quan/stm32/gpio.hpp>
#include <quan/stm32/get_module_bus_frequency.hpp>
#include <quan/angle.hpp>
#include <quan/acceleration.hpp>
#include <quan/reciprocal_time.hpp>

namespace Quan {

   struct spi{

      static void setup();

      typedef quan::stm32::spi1 device;

      typedef quan::mcu::pin<quan::stm32::gpiob,5>   mosi_pin;
      typedef quan::mcu::pin<quan::stm32::gpiob,4>   miso_pin;
      typedef quan::mcu::pin<quan::stm32::gpiob,3>   sck_pin;

      // must be a rational representing the freq in Hz
      // e.g set_clock_frequency_Hz<21000000,2>() == 10.5 MHz
      template <uint32_t N, uint32_t D>
      static bool set_clock_frequency_Hz()
      {
         if ( !busy() ){
            constexpr uint32_t bus_freq = quan::stm32::get_module_bus_frequency<device>(); // APB1 42 MHz, APB2 84 MHz
            typedef typename quan::meta::binary_op<
               quan::meta::rational<bus_freq,1>, 
               quan::meta::divides, 
               quan::meta::rational<N,D> 
            > ::type divider;
            static_assert(divider::denominator == 1,"invalid spi clock setting");
            static_assert(divider::numerator > 1,"requested clock speed too high");
            static_assert(divider::numerator < 257,"requested clock speed too low");
            constexpr uint32_t new_brr = (quan::meta::log2<divider::numerator>::value-1 ) << 3;
            constexpr uint32_t clear_mask = ~(0b111 << 3);
            device::get()->cr1 = (device::get()->cr1 & clear_mask) | new_brr;
            return true;
         }else{
            return false;
         }
      }

      static void enable()
      {
         device::get()->cr1.bb_setbit<6>(); //( SPE)
      }

      static void disable()
      {
         device::get()->cr1.bb_clearbit<6>(); //( SPE)
      }

      static void enable_rx_dma()
      { 
         device::get()->cr2.bb_setbit<0>() ;//( RXDMAEN)
      }

      static void disable_rx_dma()
      { 
         device::get()->cr2.bb_clearbit<0>() ; // ( RXDMAEN)
      }

      static void enable_dma()
      { 
         device::get()->cr2 |= (( 1 << 1 ) | ( 1 << 0)) ;// (TXDMAEN ) | ( RXDMAEN)
      }

      static void disable_dma()
      { 
         device::get()->cr2 &= ~(( 1 << 1 ) | ( 1 << 0)) ;// (TXDMAEN ) | ( RXDMAEN)
      }

      static void enable_txeie()
      {
         device::get()->cr2.bb_setbit<7>() ; // ( TXEIE)
      }

      static void disable_txeie()
      {
         device::get()->cr2.bb_clearbit<7>() ; // ( TXEIE)
      }

      static void enable_rxneie()
      {
         device::get()->cr2.bb_setbit<6>() ; // ( RXNEIE)
      }

      static void disable_rxneie()
      {
         device::get()->cr2.bb_clearbit<6>() ; // ( RXNEIE)
      }

      static uint8_t transfer(uint8_t data)
      {
         while (!txe()){;}
         ll_write(data);
         while ( rxe() ){;}
         return ll_read();
      }

      static bool txe(){ return device::get()->sr.bb_getbit<1>();}
      static bool rxne(){ return device::get()->sr.bb_getbit<0>();}
      static bool rxe() { return device::get()->sr.bb_getbit<0>() == false;}
      static bool busy(){ return device::get()->sr.bb_getbit<7>();}

      static uint8_t ll_read() { return device::get()->dr;}
      static void ll_write( uint8_t val){ device::get()->dr = val;} 
public:
      template <typename Pin>
      static void cs_assert()
      {
         // 20 ns setup time
         // 6ns isntruction --> ~3 nop
         quan::stm32::clear<Pin>();
         (void)ll_read();
         asm volatile ("nop":::);
         asm volatile ("nop":::);
      }

      template <typename Pin>
      static void cs_release()
      {
         while (busy()) { asm volatile ("nop":::);}
         // 40 ns hold 
         // 6ns instruction --> ~6 nop
         asm volatile ("nop":::);
         asm volatile ("nop":::);
         asm volatile ("nop":::);
         asm volatile ("nop":::);
         asm volatile ("nop":::);
         asm volatile ("nop":::);
         quan::stm32::set<Pin>();
      } 
private:
      static void transfer (const uint8_t *tx, uint8_t* rx, uint16_t len)
      {
         for ( uint16_t i = 0; i < len; ++i){
            rx[i] = transfer(tx[i]);
         }
      }

    public:

      template <typename Pin>
      static bool transaction(const uint8_t *tx, uint8_t *rx, uint16_t len) 
      {
         cs_assert<Pin>();
         transfer(tx,rx,len);
         cs_release<Pin>();
         return true;
      } 

      template <typename Pin>
      static void read(uint8_t reg, uint8_t* data, uint16_t len)
      {
         cs_assert<Pin>();
         transfer(reg | 0x80);
         transfer(data,data,len);
         cs_release<Pin>();
      }

      template <typename Pin>
      static void reg_write( uint8_t r, uint8_t v)
      {
         uint8_t arr[2] = {r, v};
         transaction<Pin>(arr,arr,2U);
      }

      template <typename Pin>
      static uint8_t reg_read( uint8_t r)
      {
         uint8_t arr[2] = {static_cast<uint8_t>(r | 0x80),0U};
         transaction<Pin>(arr,arr,2U);
         return arr[1];
      }
   };

} // Quan

#endif // ARDUPILOT_LIBRARIES_AP_HAL_QUAN_SPI_H_INCLUDED
