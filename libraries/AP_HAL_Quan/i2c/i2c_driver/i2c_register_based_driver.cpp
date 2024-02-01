
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#include "i2c_register_based_driver.hpp"

#if defined QUAN_I2C_DEBUG
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_debug.hpp>
#endif

Quan::i2c_register_based_driver_base::data_ptr_type Quan::i2c_register_based_driver_base::m_data{}; // return data
uint32_t Quan::i2c_register_based_driver_base::m_data_length = 0U;
uint8_t  Quan::i2c_register_based_driver_base::m_register_index = 0U; //
#if !defined QUAN_I2C_RX_DMA
uint32_t Quan::i2c_register_based_driver_base::m_data_idx = 0U;
uint32_t Quan::i2c_register_based_driver_base::m_bytes_left = 0U;
#endif
using AP_HAL::panic;
extern const AP_HAL::HAL& hal;

bool Quan::i2c_register_based_driver_base::ll_read(uint8_t register_index, uint8_t * data, uint32_t len)
{
   m_data.read_ptr = data;
   m_register_index = register_index;
   m_data_length = len;
   m_bytes_left = len;
   m_data_idx = 0;

   Quan::i2c_periph::set_error_handler(on_read_error);
   Quan::i2c_periph::set_event_handler(on_read_start_sent); // first handler

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_ack_bit(true);
   Quan::i2c_periph::enable_pos_bit(false);

#if defined QUAN_I2C_RX_DMA
   if (m_data_length > 1 ){
      Quan::i2c_periph::enable_dma_bit(true);
      Quan::i2c_periph::enable_dma_last_bit(true);
      Quan::i2c_periph::set_dma_rx_buffer(data,m_data_length);
      Quan::i2c_periph::clear_dma_rx_stream_flags();
      Quan::i2c_periph::set_dma_rx_handler(on_read_dma_transfer_complete);
      Quan::i2c_periph::enable_dma_rx_stream(true);
   }
#endif

   Quan::i2c_periph::request_start_condition();

   uint32_t const max_wait_ms = 1U + len / 10U + ((len % 10)?1:0);
   if (ulTaskNotifyTake(pdTRUE,max_wait_ms)!= 0){
      return true;
   }else{
      hal.console->printf("i2c_register_based_driver : read notify failed\n");
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error : trying reset\n");
         Quan::i2c_periph::init();
      }
      return false;
   }
   
}

// read handlers
// EV5 master mode select  BUSY, MSL and SB flag set
void Quan::i2c_register_based_driver_base::on_read_start_sent()
{     
   Quan::i2c_periph::enable_event_interrupts(false);
 // sb (bit 0)  set
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_start_sent",flags);
#endif
   if (flags & 1U){ // sb
      Quan::i2c_periph::set_event_handler(on_read_device_address_sent);
      Quan::i2c_periph::send_data(get_device_address()); //
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

// EV6 master transmitter mode selected BUSY, MSL, ADDR, TXE and TRA flags 
void Quan::i2c_register_based_driver_base::on_read_device_address_sent()
{  
   Quan::i2c_periph::enable_event_interrupts(false);
    // txe (bit 7) , addr (bit 1)
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_device_address_sent",flags);
#endif
   if ( flags & 2U){ // addr
      Quan::i2c_periph::get_sr2();  // clear addr bit
      Quan::i2c_periph::send_data(m_register_index);
      Quan::i2c_periph::set_event_handler(on_read_reg_index_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_read_reg_index_sent()
{     
   Quan::i2c_periph::enable_event_interrupts(false);
   // txe (bit 7) btf ( bit2)
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_reg_index_sent",flags);
#endif
   if (flags & 4U){ // btf
      Quan::i2c_periph::receive_data(); //clear the txe and btf flags
      Quan::i2c_periph::request_start_condition();
      Quan::i2c_periph::set_event_handler(on_read_repeated_start_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_read_repeated_start_sent()
{    
   Quan::i2c_periph::enable_event_interrupts(false);
  // sb (bit 0)
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_repeated_start_sent",flags);
#endif
   if ( flags & 1U){ // sb
      Quan::i2c_periph::send_data(get_device_address() | 1);  //add
      Quan::i2c_periph::set_event_handler(on_read_device_read_address_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_read_device_read_address_sent()
{   
   Quan::i2c_periph::enable_event_interrupts(false);
   // addr
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_device_read_address_sent",flags);
#endif
   if ( flags & 2U){ // addr
      Quan::i2c_periph::get_sr2();
      if ( m_data_length == 1){// dma doesnt work for single byte read
         Quan::i2c_periph::enable_ack_bit(false);
         Quan::i2c_periph::request_stop_condition();
         Quan::i2c_periph::enable_buffer_interrupts(true); // enable rxne
         Quan::i2c_periph::set_event_handler(on_read_single_byte_handler);
      }
   #if !defined QUAN_I2C_RX_DMA
      else{
         if (m_data_length == 2){
            Quan::i2c_periph::enable_pos_bit(true);
            Quan::i2c_periph::enable_ack_bit(false);
         }
         Quan::i2c_periph::set_event_handler(on_read_multi_byte_handler);
      }
      Quan::i2c_periph::enable_event_interrupts(true);
   #endif
   }
}
#if !defined QUAN_I2C_RX_DMA
void Quan::i2c_register_based_driver_base::on_read_multi_byte_handler()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   // btf
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_multi_byte_handler",flags);
#endif
   if ( flags & 4U){
      if (m_data_length == 2){
         Quan::i2c_periph::request_stop_condition();
         m_data.read_ptr[0] = Quan::i2c_periph::receive_data();
         Quan::i2c_periph::set_default_handlers();
         m_data.read_ptr[1] = Quan::i2c_periph::receive_data();
         Quan::i2c_periph::release_bus();
      }else{
        if ( m_bytes_left > 3){
            m_data.read_ptr[m_data_idx] = Quan::i2c_periph::receive_data();
            ++m_data_idx;
            --m_bytes_left;
        }else{
         // 3 bytes  
            Quan::i2c_periph::enable_ack_bit(false);
            m_data.read_ptr[m_data_idx] = Quan::i2c_periph::receive_data();
            ++m_data_idx;
            --m_bytes_left;
            Quan::i2c_periph::request_stop_condition();
            m_data.read_ptr[m_data_idx] = Quan::i2c_periph::receive_data();
            ++m_data_idx;
            --m_bytes_left;
            Quan::i2c_periph::set_event_handler(on_read_single_byte_handler);
            Quan::i2c_periph::enable_buffer_interrupts(true); // enable rxne
        }
        Quan::i2c_periph::enable_event_interrupts(true);
      }
   }
}
#endif

void Quan::i2c_register_based_driver_base::on_read_single_byte_handler()
{  
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_read_single_byte_handler",flags);
#endif
   if ( flags & 64U){
      m_data.read_ptr[m_data_idx] = Quan::i2c_periph::receive_data();
      Quan::i2c_periph::set_default_handlers();
      Quan::i2c_periph::release_bus();
   }
}
#if defined QUAN_I2C_RX_DMA
// dma handler
void Quan::i2c_register_based_driver_base::on_read_dma_transfer_complete()
{ 
   Quan::i2c_periph::enable_dma_rx_stream(false);
   Quan::i2c_periph::enable_dma_bit(false);
   Quan::i2c_periph::enable_dma_last_bit(false);
   Quan::i2c_periph::clear_dma_rx_stream_tcif();
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   Quan::i2c_periph::release_bus();
}
#endif

void Quan::i2c_register_based_driver_base::on_read_error()
{
   Quan::i2c_periph::default_error_handler();
}

void Quan::i2c_register_based_driver_base::on_write_error()
{
   Quan::i2c_periph::default_error_handler();
}

bool Quan::i2c_register_based_driver_base::ll_write(uint8_t register_index, uint8_t value)
{
   m_data.value_to_write = value;
   m_register_index = register_index;
   m_data_length = 1U;

   Quan::i2c_periph::set_error_handler(on_write_error);
   Quan::i2c_periph::set_event_handler(on_write_start_sent);

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);

   Quan::i2c_periph::request_start_condition();

   constexpr uint32_t max_wait_ms = 2;
   if (ulTaskNotifyTake(pdTRUE,max_wait_ms)!= 0){
      return true;
   }else{
      hal.console->printf("i2c_register_based_driver : write notify failed\n");
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error : trying reset\n");
         Quan::i2c_periph::init();
      }
      return false;
   }
}

// write handlers
// only one reg is written at a time for setting up
 // sb
void Quan::i2c_register_based_driver_base::on_write_start_sent()
{  
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_write_start_sent",flags);
#endif
   if (flags & 1U){ // sb
      Quan::i2c_periph::send_data(get_device_address());
      Quan::i2c_periph::set_event_handler(on_write_device_address_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_write_device_address_sent()
{   
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_write_device_address_sent",flags);
#endif
   if (flags & 2U){ // addr
      Quan::i2c_periph::get_sr2();
      Quan::i2c_periph::send_data(m_register_index);
      Quan::i2c_periph::set_event_handler(on_write_reg_index_sent); // on_write_reg_index_sent
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_write_reg_index_sent()
{  
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_write_reg_index_sent",flags);
#endif
   if ( flags & 4U){ //  btf
      Quan::i2c_periph::send_data(m_data.value_to_write);
      Quan::i2c_periph::set_event_handler(on_write_value_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_register_based_driver_base::on_write_value_sent()
{   
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("on_write_value_sent",flags);
#endif
   if (flags & 4U){ // btf
      Quan::i2c_periph::request_stop_condition();
      Quan::i2c_periph::set_default_handlers();
      Quan::i2c_periph::release_bus();
   }
}
