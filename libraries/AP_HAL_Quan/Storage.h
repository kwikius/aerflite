
#ifndef __AP_HAL_QUAN_STORAGE_H__
#define __AP_HAL_QUAN_STORAGE_H__

#include <AP_HAL_Quan/AP_HAL_Quan.h>

namespace Quan{

    bool storage_read(void * buffer,uint32_t eeprom_address,size_t n);
    bool storage_write(uint32_t eeprom_address, void const * buffer,size_t n);

#if defined QUAN_AERFLITE_BOARD
    bool eeprom_write_queue_flushed();
    void wait_for_eeprom_write_queue_flushed();
#endif

}

class Quan::QuanStorage : public AP_HAL::Storage {
public:
    QuanStorage();
    void init(void *);
    void read_block(void *dst, uint16_t src, size_t n);
    void write_block(uint16_t dst, const void* src, size_t n);
};

#endif // __AP_HAL_QUAN_STORAGE_H__
