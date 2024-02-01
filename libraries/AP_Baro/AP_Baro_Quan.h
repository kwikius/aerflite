#ifndef AP_BARO_QUAN_H_INCLUDED
#define AP_BARO_QUAN_H_INCLUDED

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Baro.h"
#include "AP_baro_driver.h"
#include <AP_HAL_Quan/i2c_task.hpp>

class AP_Baro_Quan final: public AP_baro_driver
{
public:
    AP_Baro_Quan();
    void update()const override;

    AP_baro_driver* connect(AP_Baro & baro);
private:
    AP_Baro * m_baro;
    void copy_to_frontend(quan::pressure_<float>::Pa const &, quan::temperature_<float>::K const &)const;
    QueueHandle_t                 m_hQueue;
    uint8_t                       m_instance;
  
   AP_Baro_Quan(AP_Baro_Quan const & ) = delete;
   AP_Baro_Quan& operator = (AP_Baro_Quan const & ) = delete;
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#endif // AP_BARO_QUAN_H_INCLUDED
