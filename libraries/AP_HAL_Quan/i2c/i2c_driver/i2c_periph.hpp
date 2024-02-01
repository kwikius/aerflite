#ifndef QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
#define QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED

#include <stm32f4xx.h>
#include <quan/stm32/i2c/typedefs.hpp>

// dont use tx dma with ArduPilot with OSD since this DMA channel is used by the OSD
//#define QUAN_I2C_TX_DMA
//#define QUAN_I2C_RX_DMA
//#define QUAN_I2C_DEBUG

// required for friend declarations
// dma and event handlers dependent on I2C bus number
// Here i2c3
//extern "C" void DMA1_Stream4_IRQHandler() __attribute__ ( (interrupt ("IRQ")));
//extern "C" void DMA1_Stream2_IRQHandler() __attribute__ ( (interrupt ("IRQ")));

template <int Dma, int Stream> void DMA_IRQ_Handler();

// template <int N> void I2C_EV_IRQ_Handler();
extern "C" void I2C3_EV_IRQHandler() __attribute__ ((interrupt ("IRQ")));
extern "C" void I2C3_ER_IRQHandler() __attribute__ ((interrupt ("IRQ")));

extern "C" void TIM5_IRQHandler() __attribute__ ((interrupt ("IRQ")));

/*
for stm32f405/7
//I2C1
   I2C1_RX on DMA[1].Stream[0].Ch[1] or DMA[1].Stream[5].Ch[1]
   I2C1_TX on DMA[1].Stream[6].Ch[1] or DMA[1].Stream[7].Ch[1]
//I2C2
   I2C2_RX on DMA[1].Stream[2].Ch[7] or DMA[1].Stream[3].Ch[7]
   I2C2_TX on DMA[1].Stream[7].Ch[7]
//I2C3
   I2C3_RX on DMA[1].Stream[2].Ch[3]
   I2C3_TX on DMA[1].Stream[4].Ch[3]
*/
/*
   --- plug in arch per bus address ---
   * Acquire the bus and install plugin
   * do work
   * remove plugin and release bus
    Plugin part is the irq and dma functions

    add fun to release bus
    to use bus check busy first
    record last write time to eeprom
    and use that to see if it is ok to read ( write takes 5 ms)

    specific to quan::stm32f4:: i2c_bus stm32f4
*/
/*
 parts of i2c reqd

 i2c_bus
 i2c_pins  various pins are valid per i2c_bus
 // perhaps validate that they are ok for use with this bus
 i2c_dma_streams
  various tx and rx streams are available per bus
 i2c_irq_handlers
 These can be aliased to provide 

 DMA_Stream_TypeDef* tx_dma_stream
 DMA_Stream_TypeDef* rx_dma_stream
 typedef ? scl_pi_type
 typedef  ? sda_pin_type
 Dma

*/
namespace Quan{

   struct i2c_periph{

      enum class errno_t{
         no_error, 
         invalid_address,
         cant_start_new_transfer_when_i2c_busy,
         zero_data,
         data_pointer_is_null,
         invalid_num_bytes_in_rx_multibyte_btf,
         unexpected_single_total_bytes_in_rx_btf,
         unexpected_not_last_byte_in_rxne,
         unexpected_flags_in_irq,
         unknown_i2c_err_handler,
         address_timed_out,
         unknown_exti_irq,
         i2c_err_handler_BERR,
         i2c_err_handler_AF,
         i2c_err_handler_ARLO,
         i2c_err_handler_OVR,
         i2c_err_handler_TIMEOUT,
         i2c_err_handler_PECERR,
         i2c_err_handler_SMB_ALERT
      };

   private:
       typedef quan::stm32::i2c3  i2c_type;
   public:
      static void init();

      // n.b this can affect the i2c_periph state so
      // seems somewhat problematic to use this
      // always call bus_released first
   // from ref man 23.6.7 NOTE
   /*
   Reading I2C_SR2 after reading I2C_SR1 clears the ADDR flag, even if the ADDR flag was
   set after reading I2C_SR1. Consequently, I2C_SR2 must be read only when ADDR is found
   set in I2C_SR1 or when the STOPF bit is cleared
   */
     // static bool get_thread_mode() { return m_thread_mode;}
    //  static void set_thread_mode(bool b) { m_thread_mode = b;}
      static bool is_busy()
      {
          constexpr uint8_t i2c_sr2_busy_bit = 1;
          return i2c_type::get()->sr2.bb_getbit<i2c_sr2_busy_bit>();
      }

      // n.b the bus may still be busy after it has been released by an irq handler
      // may be waiting for stop bits etc
      // so check if bus is free use bus_free();
      // could start a timer here
      // 4 us must elapse between scl high ( stop) and the next scl low
      // after transaction 1 clock pulse for ack
      // Do not allow device that use clock stretching since this violates determinacy
      // assume that there are 2 clock cycles from btf to stop condition
      // 100 kHz -> 20 usec + 4 usec
      static bool bus_released() { return m_bus_taken_token == false;}

      // true if bus now available
      static bool bus_free(){
         if (!bus_released()){
            return false;
         }
         return !is_busy();
      }

      static bool get_bus()
      {
          if ( !bus_free() ){ return false;}
          m_bus_taken_token = true;
        //  led::on();
          return true;
      }
      static void release_bus();
      static void default_event_handler();
      static void default_error_handler();
#if defined QUAN_I2C_TX_DMA
      static void default_dma_tx_handler();
#endif
#if defined QUAN_I2C_RX_DMA
      static void default_dma_rx_handler();
#endif
      static void set_default_handlers()
      {
         pfn_event_handler = default_event_handler;
         pfn_error_handler = default_error_handler;
#if defined QUAN_I2C_TX_DMA
         pfn_dma_tx_handler  = default_dma_tx_handler;
#endif
#if defined QUAN_I2C_RX_DMA
         pfn_dma_rx_handler = default_dma_rx_handler;
#endif
      }

      static void set_event_handler( void(*pfn_event)()){pfn_event_handler = pfn_event;}
      static void set_error_handler( void(*pfn_event)()){ pfn_error_handler = pfn_event;}
#if defined QUAN_I2C_TX_DMA
      static void set_dma_tx_handler( void(*pfn_event)()){pfn_dma_tx_handler = pfn_event;}
#endif
#if defined QUAN_I2C_RX_DMA
      static void set_dma_rx_handler( void(*pfn_event)()){pfn_dma_rx_handler = pfn_event;}
#endif
      static void request_start_condition(){constexpr uint8_t cr1_start_bit = 8; i2c_type::get()->cr1.bb_setbit<cr1_start_bit>();}
      static void request_stop_condition(){constexpr uint8_t cr1_stop_bit =9;i2c_type::get()->cr1.bb_setbit<cr1_stop_bit>();}
      static void enable_dma_bit(bool b){constexpr uint8_t cr2_dmaen = 11; i2c_type::get()->cr2.bb_putbit<cr2_dmaen>(b);}
      static void enable_ack_bit(bool b){ uint8_t constexpr  i2c_cr1_ack_bit = 10;i2c_type::get()->cr1.bb_putbit<i2c_cr1_ack_bit>(b);}
      static void enable_dma_last_bit(bool b){constexpr uint8_t cr2_last = 12; i2c_type::get()->cr2.bb_putbit<cr2_last>(b);}
      static void enable_pos_bit(bool b){constexpr uint8_t cr1_pos_bit = 11; i2c_type::get()->cr1.bb_putbit<cr1_pos_bit>(b);}
      static void enable_error_interrupts(bool b){constexpr uint8_t cr2_error_bit = 8;i2c_type::get()->cr2.bb_putbit<cr2_error_bit>(b);}
      static void enable_event_interrupts(bool b){ constexpr uint8_t cr2_itevten_bit = 9;i2c_type::get()->cr2.bb_putbit<cr2_itevten_bit>(b);}
      static void enable_buffer_interrupts(bool b){ constexpr uint8_t cr2_itbufen_bit = 10;i2c_type::get()->cr2.bb_putbit<cr2_itbufen_bit>(b);}

#if defined QUAN_I2C_TX_DMA
      static void enable_dma_tx_stream(bool b)
      {
         if (b){
           DMA1_Stream4->CR |= (1U << 0U); // (EN)
         } else {
           DMA1_Stream4->CR &= ~(1U << 0U); // (EN)
           while (DMA1_Stream4->CR & (1U << 0U)) { asm volatile("nop":::);}
         }
      }
#endif
#if defined QUAN_I2C_RX_DMA
      static void enable_dma_rx_stream(bool b)
      {
        if (b){
          DMA1_Stream2->CR |= (1U << 0U); // (EN)
        } else {
          DMA1_Stream2->CR &= ~(1U << 0U); // (EN)
          while (DMA1_Stream2->CR & (1U << 0U)) { asm volatile("nop":::);}
         }
      }
#endif
#if defined QUAN_I2C_TX_DMA
      static void set_dma_tx_buffer(uint8_t const* data, uint16_t numbytes)
      {
          DMA1_Stream4->M0AR = (uint32_t)data; // buffer address
          DMA1_Stream4->NDTR = numbytes;       // num data
      }
#endif
#if defined QUAN_I2C_RX_DMA
      static void set_dma_rx_buffer(uint8_t * data, uint16_t numbytes)
      {
          DMA1_Stream2->M0AR = (uint32_t)data; // buffer address
          DMA1_Stream2->NDTR = numbytes;       // num data
      }
#endif

      static void peripheral_enable(bool b)
      {
        uint8_t constexpr i2c_cr1_pe_bit = 0;
        i2c_type::get()->cr1.bb_putbit<i2c_cr1_pe_bit>(b);
      }
   // todo add clear interrupt flags
#if defined QUAN_I2C_TX_DMA
      static void clear_dma_tx_stream_flags(){ DMA1->HIFCR = (0b111101 << 0U) ;} // clear flags for Dma1 Stream 4
      static void clear_dma_tx_stream_tcif(){DMA1->HIFCR = (1 << 5) ;  } // DMA1.Stream4 (TCIF)
#endif
#if defined QUAN_I2C_RX_DMA
      static void clear_dma_rx_stream_flags(){DMA1->LIFCR = (0b111101 << 16U) ;} // clear flags for Dma1 Stream 2
      static void clear_dma_rx_stream_tcif()
      {
         DMA1->LIFCR = (1 << 21) ; // DMA1.Stream2 (TCIF)
      }
#endif

      static uint16_t get_sr1(){return i2c_type::get()->sr1.get();}
      static uint16_t get_sr2(){return i2c_type::get()->sr2.get(); }

      static bool get_sr2_msl() {constexpr uint8_t sr2_msl = 0; return i2c_type::get()->sr2.bb_getbit<sr2_msl>();}
      static bool get_sr1_btf()  {constexpr uint8_t sr1_btf = 2;return i2c_type::get()->sr1.bb_getbit<sr1_btf>();}
      static bool get_sr1_txe()  {constexpr uint8_t sr1_txe = 7;return  i2c_type::get()->sr1.bb_getbit<sr1_txe>(); }
      static bool get_sr2_tra() {constexpr uint8_t sr2_tra = 2;return i2c_type::get()->sr2.bb_getbit<sr2_tra>();}
      static bool get_sr1_addr() {constexpr uint8_t sr1_addr = 1;return i2c_type::get()->sr1.bb_getbit<sr1_addr>();}
      static bool get_sr1_sb() {constexpr uint8_t sr1_sb = 0;return i2c_type::get()->sr1.bb_getbit<sr1_sb>();}
      static bool get_sr1_rxne(){constexpr uint8_t sr1_rxne = 6;return i2c_type::get()->sr1.bb_getbit<sr1_rxne>();}
      static bool get_sr1_stopf(){constexpr uint8_t sr1_stopf_bit =4;return i2c_type::get()->sr1.bb_getbit<sr1_stopf_bit>();}

      static void send_data(uint8_t data){i2c_type::get()->dr = data;}
      static uint8_t receive_data(){return static_cast<uint8_t>(i2c_type::get()->dr);}
      static const char* get_error_string();
      static bool has_errored() { return m_errored;}
      static const char* get_last_error_c_str();
      static errno_t get_last_error(){ return m_last_error;}
      static bool clear_i2c_bus();
   private:
#if defined QUAN_I2C_TX_DMA
      friend void ::DMA_IRQ_Handler<1,4>() ;
#endif
#if defined QUAN_I2C_RX_DMA
      friend void ::DMA_IRQ_Handler<1,2>() ;
#endif
      friend void ::I2C3_EV_IRQHandler() ;
      friend void ::I2C3_ER_IRQHandler();
      friend void ::TIM5_IRQHandler();
#if defined QUAN_I2C_TX_DMA
      static void setup_tx_dma();
#endif
      static void setup_rx_dma();

      static volatile bool m_bus_taken_token;
      static volatile bool m_errored;
      static errno_t       m_last_error;
    //  static volatile bool m_thread_mode;
      // add a is_running_threaded
      static void (* volatile pfn_event_handler)();
      static void (* volatile pfn_error_handler)();
#if defined QUAN_I2C_TX_DMA
      static void (* volatile pfn_dma_tx_handler)();
#endif
#if defined QUAN_I2C_RX_DMA
      static void (* volatile pfn_dma_rx_handler)();
#endif
      i2c_periph() = delete;
      i2c_periph(i2c_periph const & ) = delete;
      i2c_periph& operator = (i2c_periph&) = delete;
      
   };

   bool wait_for_bus_free_ms(uint32_t t_ms);



} // Quan
#endif // QUAN_STM32_EEPROM_TEST_I2C_HPP_INCLUDED
