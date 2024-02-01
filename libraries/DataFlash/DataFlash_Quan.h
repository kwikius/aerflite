#ifndef DATAFLASH_QUAN_H_INCLUDED
#define DATAFLASH_QUAN_H_INCLUDED

#include <DataFlash/DataFlash_Backend.h>

class DataFlash_Quan : public DataFlashBackend{

    DataFlash_Quan(DataFlash_Class &front, const char *log_directory);
       // initialisation
    void Init(const struct LogStructure *structure, uint8_t num_types);
    bool CardInserted(void);

    // erase handling
    void EraseAll();

    // possibly time-consuming preparation handling:
    bool NeedPrep();
    void Prep();

    /* Write a block of data at current offset */
    bool WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical);
    uint16_t bufferspace_available();

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint16_t & start_page, uint16_t & end_page);
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc);
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data);
    uint16_t get_num_logs() override;
    uint16_t start_new_log(void);
    void LogReadProcess(const uint16_t log_num,
                        uint16_t start_page, uint16_t end_page, 
                        print_mode_fn print_mode,
                        AP_HAL::BetterStream *port);
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);
    void ListAvailableLogs(AP_HAL::BetterStream *port);
   
};

#endif // DATAFLASH_QUAN_H_INCLUDED
