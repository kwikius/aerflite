
#include <AP_HAL_Quan/AP_HAL_Quan.h>
#include <AP_HAL_Quan/eeprom.hpp>
#include "i2c_eeprom_driver.hpp"

#if defined QUAN_I2C_DEBUG
#include <AP_HAL_Quan/i2c/i2c_driver/i2c_debug.hpp>
#endif

extern const AP_HAL::HAL& hal;

using AP_HAL::millis;

TickType_t                    Quan::i2c_eeprom_driver_base::m_write_cycle_time_ms;
volatile bool                 Quan::i2c_eeprom_driver_base::m_write_in_progress = false;
volatile TickType_t           Quan::i2c_eeprom_driver_base::m_write_start_time_ms = 0U;
uint32_t                      Quan::i2c_eeprom_driver_base::m_memory_size_bytes ;
uint32_t                      Quan::i2c_eeprom_driver_base::m_page_size_bytes;

#if !defined QUAN_I2C_TX_DMA
uint32_t   Quan::i2c_eeprom_driver_base::m_data_length = 0;
Quan::i2c_eeprom_driver_base::data_ptr_type Quan::i2c_eeprom_driver_base::m_data;
#endif

#if !defined QUAN_I2C_RX_DMA
uint32_t Quan::i2c_eeprom_driver_base::m_data_idx = 0U;
uint32_t Quan::i2c_eeprom_driver_base::m_bytes_left = 0U;
#endif

uint8_t         Quan::i2c_eeprom_driver_base::m_data_address[2] ={0U,0U};
uint8_t         Quan::i2c_eeprom_driver_base::m_data_address_hi = 0U;

// non isr no state change
TickType_t Quan::i2c_eeprom_driver_base::get_write_time_left_ms()
{
   if ( m_write_in_progress){
      TickType_t const eewrite_runtime = xTaskGetTickCountFromISR() - m_write_start_time_ms;
      if ( eewrite_runtime < m_write_cycle_time_ms){
         return m_write_cycle_time_ms - eewrite_runtime;
      }else{
         return 0U;
      }
   }else{ // no write in progress
      return 0U;
   }
}

// non isr no state change
bool Quan::i2c_eeprom_driver_base::wait_for_write_complete()
{
   if ( m_write_in_progress){
      for ( uint32_t time_left = get_write_time_left_ms(); time_left > 0U; time_left = get_write_time_left_ms()){
         vTaskDelay(time_left);
      }
      // the write in progress callback will probably not have fired yet
      while ( m_write_in_progress){
        vTaskDelay(1);
      }
   }
   return true;
}

// call from isr
void Quan::i2c_eeprom_driver_base::set_new_write_start_time_from_isr(BaseType_t & hpthw)
{
   m_write_start_time_ms = xTaskGetTickCountFromISR() ;
   m_write_in_progress = true;
   auto handle = get_eeprom_timer_handle();
   xTimerChangePeriodFromISR(handle, m_write_cycle_time_ms , & hpthw);
   xTimerStartFromISR(handle,&hpthw);
}

bool Quan::i2c_eeprom_driver_base::install_device(
      const char* name, 
      uint8_t address,
      uint32_t memory_size,
      uint32_t page_size,
      TickType_t write_cycle_time_ms
){

   if (!wait_for_write_complete()){
      return false;
   }

   if (! Quan::wait_for_bus_free_ms(5U)){
      return false;
   }

   if ( ! i2c_driver::install_device(name,address)){
      return false;
   }
   set_memory_size_bytes(memory_size);
   set_page_size_bytes(page_size);
   set_write_cycle_time_ms(write_cycle_time_ms);
   return true;  
}

template <typename ID>
bool Quan::i2c_eeprom_driver<ID>::read(uint32_t start_address_in,uint8_t* data_out, uint32_t len)
{
   if ( len == 0){
      AP_HAL::panic("eeprom write zero length");
      return false;
   }
   if ( data_out == nullptr){
      AP_HAL::panic("eeprom read data ptr is null");
      return false;
   }
   // one past the last address to write
   uint32_t const end_address = start_address_in + len;

   if ( end_address >= ID::get_memory_size_bytes()){
      AP_HAL::panic("eeprom read address out of range");
      return false;
   }
   uint32_t const start_page = start_address_in / ID::get_page_size_bytes();
   uint32_t const end_page =  (end_address -1) / ID::get_page_size_bytes();
   if ( start_page == end_page){
      return read_page(start_address_in,data_out, len);
   }else{
      uint32_t  start_address = start_address_in;
      uint8_t * data = data_out;
      uint8_t * const data_end = data_out + len;
      uint32_t  cur_page = start_page;
      uint32_t  bytes_to_read  = ID::get_page_size_bytes() - (start_address_in % ID::get_page_size_bytes());

      while (data < data_end){
         if (!read_page(start_address,data,bytes_to_read)){
            return false;
         }
         start_address += bytes_to_read; // dest eeprom address
         data += bytes_to_read; //advance
          if ( ++cur_page == end_page){
            bytes_to_read = data_end - data;
         }else{
            bytes_to_read = ID::get_page_size_bytes();
         }
      }
      return true;
   }
}

bool Quan::i2c_eeprom_driver_base::ll_read(uint32_t start_address_in, uint8_t * data, uint32_t len)
{
   m_data_address[0] = static_cast<uint8_t>((start_address_in & 0xFF00) >> 8U);
   m_data_address[1] = static_cast<uint8_t>(start_address_in & 0xFF);
   m_data_address_hi = static_cast<uint8_t>((start_address_in >> 15U) & 0b10);

#if !defined QUAN_I2C_TX_DMA
   m_data.read_ptr = data;
   m_data_length   = len;
   m_bytes_left    = len;
   m_data_idx      = 0;
#endif   
   
   Quan::i2c_periph::set_error_handler(on_read_error);
   Quan::i2c_periph::set_event_handler(on_read_start_sent);

   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::enable_ack_bit(true);
   Quan::i2c_periph::enable_pos_bit(false);
#if defined QUAN_I2C_RX_DMA
   if (len > 1 ){
      Quan::i2c_periph::enable_dma_bit(true);
      Quan::i2c_periph::enable_dma_last_bit(true);
      Quan::i2c_periph::set_dma_rx_buffer(data,len);
      Quan::i2c_periph::clear_dma_rx_stream_flags();
      Quan::i2c_periph::set_dma_rx_handler(on_read_dma_transfer_complete);
      Quan::i2c_periph::enable_dma_rx_stream(true);
   }
#endif

#if defined QUAN_I2C_DEBUG
    if (Quan::want_flags_index_reset()){ Quan::reset_i2c_sr1_flags_index();}
#endif
   Quan::i2c_periph::request_start_condition();

   uint32_t const max_wait_ms = 2U + len / 10U + ((len % 10)?1:0);
   if (ulTaskNotifyTake(pdTRUE,max_wait_ms)!= 0){
      return true;
   }else{
      hal.console->printf("i2c_eeprom : read notify failed\n");
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error %s : trying reset\n",i2c_periph::get_last_error_c_str());
         Quan::i2c_periph::init();
      }
      return false;
   }
}

void Quan::i2c_eeprom_driver_base::on_read_start_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_start_sent",flags);
#endif
   if (flags & 1U){ // sb
      Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi);
      Quan::i2c_periph::set_event_handler(on_read_device_address_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_read_device_address_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_device_address_sent",flags);
#endif
   if (flags & 2U){ // addr txe
      Quan::i2c_periph::get_sr2();
      Quan::i2c_periph::send_data(m_data_address[0]);
      Quan::i2c_periph::set_event_handler(on_read_data_address_hi_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_read_data_address_hi_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_data_address_hi_sent",flags);
#endif
   if (flags & 4U){ //  txe btf
      Quan::i2c_periph::send_data(m_data_address[1]);
      Quan::i2c_periph::set_event_handler(on_read_data_address_lo_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_read_data_address_lo_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_data_address_lo_sent",flags);
#endif
   if (flags & 4U){ //  txe btf
      Quan::i2c_periph::receive_data(); //clear the txe and btf flags
      Quan::i2c_periph::request_start_condition();
      Quan::i2c_periph::set_event_handler(on_read_repeated_start_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_read_repeated_start_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_repeated_start_sent",flags);
#endif
   if (flags & 1U){ // sb
      Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi | 1U); // send eeprom read address
      Quan::i2c_periph::set_event_handler(on_read_device_read_address_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_read_device_read_address_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();

#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_device_read_address_sent",flags);
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
   #else  //rx dma
      Quan::i2c_periph::set_event_handler(Quan::i2c_periph::default_event_handler);
   #endif
   }
}
#if !defined QUAN_I2C_RX_DMA
void Quan::i2c_eeprom_driver_base::on_read_multi_byte_handler()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_multi_byte_handler",flags);
#endif
   if ( flags & 4U){ // btf
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

void Quan::i2c_eeprom_driver_base::on_read_single_byte_handler()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   Quan::i2c_periph::enable_buffer_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_read_single_byte_handler",flags);
#endif
   if (flags & 64U){
      m_data.read_ptr[m_data_idx] = Quan::i2c_periph::receive_data();
      Quan::i2c_periph::set_default_handlers();
      Quan::i2c_periph::release_bus();
   }
}

#if defined QUAN_I2C_RX_DMA
void Quan::i2c_eeprom_driver_base::on_read_dma_transfer_complete()
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

void Quan::i2c_eeprom_driver_base::on_read_error()
{
   Quan::i2c_periph::default_error_handler();
}

//--------------------------

/*
write any length of data
 This will block until the last write has started
*/

template <typename ID>
bool Quan::i2c_eeprom_driver<ID>::write(uint32_t start_address_in, uint8_t const * data_in, uint32_t len)
{

   if ( len == 0){
      AP_HAL::panic("eeprom_orig write zero length");
      return false;
   }
   if ( data_in == nullptr){
      AP_HAL::panic("eeprom_orig write data is null");
      return false;
   }
   // end address is one past the last address to write
   uint32_t const end_address = start_address_in + len;

   if ( end_address >= ID::get_memory_size_bytes()){
      hal.console->printf("memory size = %lu\n",ID::get_memory_size_bytes() );
      AP_HAL::panic("eeprom_write address out of range");
      return false;
   }
   uint32_t constexpr page_size = ID::get_page_size_bytes();
   uint32_t const start_page = start_address_in / page_size;
   uint32_t const end_page =  (end_address -1) / page_size;
  
   if ( start_page == end_page){
      return write_page(start_address_in,data_in, len);
   }else{

      uint32_t             start_address  = start_address_in;
      uint8_t const*       data           = data_in;
      uint8_t const* const data_end       = data_in + len;
      uint32_t             cur_page       = start_page;
      uint32_t             bytes_to_write = page_size - (start_address_in % page_size);

      while (data < data_end){

         if (! write_page(start_address,data, bytes_to_write)){
           AP_HAL::panic("eewr failed2\n");
           return false;
         }
         start_address += bytes_to_write; // dest eeprom_orig address
         data += bytes_to_write; //advance
         if ( ++cur_page == end_page){
            bytes_to_write = data_end - data;
         }else{
            bytes_to_write = page_size;
         }
      }
   }
   return true;
};

template struct Quan::i2c_eeprom_driver<Quan::eeprom_info::eeprom_m24m01>;

bool Quan::i2c_eeprom_driver_base::ll_write(uint32_t data_address_in, uint8_t const * data_in, uint32_t len)
{
   if ( len == 0){
      AP_HAL::panic("eeprom_orig write zero length");
      return false;
   }
   if ( data_in == nullptr){
      AP_HAL::panic("eeprom_orig write data is null");
      return false;
   }
   
   m_data_address[0] = static_cast<uint8_t>((data_address_in & 0xFF00) >> 8U);
   m_data_address[1] = static_cast<uint8_t>(data_address_in & 0xFF);
// TODO check that address in range
   m_data_address_hi = static_cast<uint8_t>((data_address_in >> 15U) & 0b10);

#if !defined QUAN_I2C_TX_DMA
   m_data.write_ptr    = data_in;
   m_data_length       = len;
   m_data_idx          = 0;
#endif   
   
   Quan::i2c_periph::set_error_handler(on_write_error);
   Quan::i2c_periph::set_event_handler(on_write_start_sent);
#if defined QUAN_I2C_TX_DMA
   Quan::i2c_periph::set_dma_tx_handler(on_dma_transfer_complete);
   Quan::i2c_periph::set_dma_tx_buffer(data,len);
   Quan::i2c_periph::enable_dma_tx_stream(false);
   Quan::i2c_periph::clear_dma_tx_stream_flags();
   //Quan::i2c_periph::enable_dma_bit(true);
#endif
   
   Quan::i2c_periph::enable_error_interrupts(true);
   Quan::i2c_periph::enable_event_interrupts(true);
   Quan::i2c_periph::enable_buffer_interrupts(false);

#if defined QUAN_I2C_DEBUG
    if (Quan::want_flags_index_reset()){ Quan::reset_i2c_sr1_flags_index();}
#endif

   Quan::i2c_periph::request_start_condition();

   uint32_t const max_wait_ms = 100U + len / 10U + ((len % 10)?1:0);
   if (ulTaskNotifyTake(pdTRUE,max_wait_ms)!= 0){
      return true;
   }else{
      hal.console->printf("i2c_eeprom : write notify failed\n");
      if(Quan::i2c_periph::has_errored()){
         hal.console->printf("i2c error : trying reset\n");
         Quan::i2c_periph::init();
      }
      return false;
   }
}

void Quan::i2c_eeprom_driver_base::on_write_start_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_write_start_sent",flags);
#endif
   if ( flags & 0x01){ // sb
      Quan::i2c_periph::send_data(get_device_address() | m_data_address_hi);
      Quan::i2c_periph::set_event_handler(on_write_device_address_sent);
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_write_device_address_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_write_device_address_sent",flags);
#endif
   if ( flags & 0x02){ // addr txe
      Quan::i2c_periph::get_sr2();
      Quan::i2c_periph::send_data(m_data_address[0]);
      Quan::i2c_periph::set_event_handler(on_write_data_address_hi_sent); 
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_write_data_address_hi_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_write_data_address_hi_sent",flags);
#endif
   if (flags & 0x04) {// btf txe
      Quan::i2c_periph::send_data(m_data_address[1]);
   #if defined QUAN_I2C_TX_DMA
      Quan::i2c_periph::set_event_handler(on_write_data_address_lo_sent);
   #else
      Quan::i2c_periph::set_event_handler(on_writing_data);
   #endif
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}

#if defined QUAN_I2C_TX_DMA
// txe on 2nd data address
// disable i2c events
// do final dma setup
// and point dma handler to end of dma handler
// start sending the data using dma
void Quan::i2c_eeprom_driver_base::on_write_data_address_lo_sent()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_write_data_address_lo_sent",flags);
#endif
   if (flags & 0x04) {// btf
     Quan::i2c_periph::enable_dma_tx_stream(true);
   }
}

void Quan::i2c_eeprom_driver_base::on_write_dma_transfer_complete()
{
   Quan::i2c_periph::enable_dma_tx_stream(false);
   Quan::i2c_periph::enable_dma_bit(false);
   Quan::i2c_periph::clear_dma_tx_stream_tcif();
   Quan::i2c_periph::set_event_handler(on_write_last_byte_transfer_complete);
   Quan::i2c_periph::enable_event_interrupts(true);  // btf txe
}

#else  // non dma version
void Quan::i2c_eeprom_driver_base::on_writing_data()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if defined QUAN_I2C_DEBUG
   capture_i2c_sr1_flags("eeprom on_writing_data",flags);
#endif
   if (flags & 0x84){ // btf txe
      Quan::i2c_periph::send_data(m_data.write_ptr[m_data_idx]);
//      if ( m_data_idx == 0){ 
//         Quan::i2c_periph::enable_buffer_interrupts(true); // txe
//      }

      if ( ++m_data_idx == m_data_length){
        // hal.gpio->write(1,1);
         Quan::i2c_periph::set_event_handler(on_write_last_byte_transfer_complete);
//         Quan::i2c_periph::request_stop_condition();
//         Quan::i2c_periph::set_default_handlers();
//         set_new_write_end_time();
//         Quan::i2c_periph::release_bus();
//        // Quan::i2c_periph::enable_buffer_interrupts(false);
      }
      Quan::i2c_periph::enable_event_interrupts(true);
   }
}
#endif

// txe on last
void Quan::i2c_eeprom_driver_base::on_write_last_byte_transfer_complete()
{
   Quan::i2c_periph::enable_event_interrupts(false);
   uint32_t const flags = Quan::i2c_periph::get_sr1();
#if !defined QUAN_I2C_DEBUG
   (void)flags;
#else
   capture_i2c_sr1_flags("on_write_last_byte_transfer_complete",flags);
#endif
   
  // Quan::i2c_periph::enable_buffer_interrupts(false);
   Quan::i2c_periph::request_stop_condition();
   Quan::i2c_periph::set_default_handlers();
   BaseType_t hpthw = pdFALSE;
   set_new_write_start_time_from_isr(hpthw);
   Quan::i2c_periph::release_bus();
   portEND_SWITCHING_ISR(hpthw);
}

void Quan::i2c_eeprom_driver_base::on_write_error()
{
   Quan::i2c_periph::default_error_handler();
}
