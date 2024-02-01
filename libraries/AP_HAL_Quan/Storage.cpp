
#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "Storage.h"

#if  defined QUAN_AERFLITE_BOARD
#include "eeprom.hpp"
#include <quan/min.hpp>

#endif

extern const AP_HAL::HAL& hal;

using namespace Quan;

QuanStorage::QuanStorage()
{}

void QuanStorage::init(void*)
{}

void QuanStorage::read_block(void* dst, uint16_t eeprom_address, size_t n) 
{
   if ( !Quan::storage_read(dst,eeprom_address,n) ){
      hal.console->write("eeprom read failed\n");
   }
}

void QuanStorage::write_block(uint16_t eeprom_address, const void* src, size_t n)
{
   if ( !Quan::storage_write(eeprom_address,src,n) ){
      hal.console->write("eeprom write failed\n");
   }  
}

#if defined QUAN_AERFLITE_BOARD

namespace Quan{

   bool eeprom_write_queue_flushed(){ return uxQueueMessagesWaiting(get_eeprom_write_handle()) == 0U;}

   void wait_for_eeprom_write_queue_flushed()
   { 
      for( ;;){
         UBaseType_t messages_waiting = uxQueueMessagesWaiting(get_eeprom_write_handle());
         if ( messages_waiting == 0U){
            return;
         }else{
            vTaskDelay(Quan::eeprom::get_write_cycle_time_ms() * messages_waiting);
         }
      }
   }

   // This function blocks while the data is read
   // since it is i2c eeprom, it may take a while!
   bool storage_read(void * buffer,uint32_t storage_address,size_t n)
   {
      if ( (storage_address + n) > eeprom_info::eeprom_m24m01::get_memory_size_bytes() ){
         AP_HAL::panic("size out of range in eeprom read");
         return false;
      }

      eeprom_read_msg msg{buffer,storage_address,n};
      
      if  (xQueueSendToBack(get_eeprom_read_handle(),&msg,500) == pdFALSE ){
         hal.console->write("eeprom read send to queue failed\n");
         return false;
      }

      if (xSemaphoreTake(get_read_complete_semaphore(), 500) == pdTRUE){
  
         return true;
      }else{
         hal.console->write("eeprom get read complete sem failed\n");
         return false;
      }
   }
}// Quan

namespace {

   // pass the data by copy to the write queue
   // Normally this function doesnt need to block
   // If it does block then it probably needs a larger queue
   bool add_to_write_queue(void const * mcu_data, uint32_t eeprom_address,  uint8_t len)
   {
      eeprom_write_msg msg{mcu_data,eeprom_address,len};
      if (xQueueSendToBack(get_eeprom_write_handle(),&msg,200)){
         return true;
      }else{
         AP_HAL::panic("write to eeprom : failed to find a place on the queue in 200 ms");
         return false;
      }
   }

   // split the data again to fit into the constant size alloc of the write queue structure
   // then add each segment to the queue
   bool split_into_write_queue_elements( void const * mcu_data,uint32_t eeprom_address_in, uint32_t len )
   {
      uint32_t constexpr buf_size = eeprom_write_msg::buffer_length;
      static_assert(buf_size <= 255,"unexpected large buf size");
      uint32_t eeprom_address     = eeprom_address_in;
      uint32_t bytes_left         = len;
      uint8_t const * data        = static_cast<uint8_t const *>(mcu_data);
      uint8_t bytes_to_write      = static_cast<uint8_t>(quan::min(len,buf_size));

      while (bytes_left > 0U ){
         if (add_to_write_queue(data,eeprom_address,bytes_to_write)){ 
            eeprom_address += bytes_to_write;
            data += bytes_to_write;
            bytes_left -= bytes_to_write;
            bytes_to_write = static_cast<uint8_t>(quan::min(bytes_left,buf_size));
         }else{
            return false;
         }
      }
      return true;
   }

   // split the data up so that it doesnt cross any eeprom page boundaries
   bool split_into_write_pages( void const * mcu_data, uint32_t eeprom_address_in, uint32_t len )
   {
      uint32_t constexpr page_size =  eeprom_info::eeprom_m24m01::get_memory_size_bytes();
      uint32_t const end_address = eeprom_address_in + (len -1); // one past last to write
      uint32_t const start_page  = eeprom_address_in / page_size;
      uint32_t const end_page    = end_address / page_size;

      if ( start_page == end_page ){
         return split_into_write_queue_elements( mcu_data,eeprom_address_in, len);
      }else{
         uint32_t          eeprom_address = eeprom_address_in;
         uint8_t const*    data           = static_cast<uint8_t const *>(mcu_data);
         uint8_t const* const data_end    = data + len;
         uint32_t          cur_page       = start_page;
         uint32_t          bytes_to_write = page_size - (eeprom_address_in % page_size);

         while (data < data_end){
            if (! split_into_write_queue_elements(data,eeprom_address,bytes_to_write) ){
               return false;
            }
            eeprom_address += bytes_to_write; // dest eeprom_orig address
            data += bytes_to_write; //advance
            if ( ++cur_page == end_page){
               bytes_to_write = data_end - data;
            }else{
               bytes_to_write = page_size;
            }
         }
         return true;
      }
   }
}

namespace Quan{
   // called from the APM Storage object in apm task
   // wont block unless its a very large write
   bool storage_write(uint32_t storage_address, void const * buffer,size_t n)
   {
      if ( (storage_address + n) > eeprom_info::eeprom_m24m01::get_memory_size_bytes()){
         return false;
      }
      return split_into_write_pages(buffer,storage_address,n);
   }

} // Quan

#endif // defined QUAN_AERFLITE_BOARD
#endif   // CONFIG_HAL_BOARD == HAL_BOARD_QUAN

