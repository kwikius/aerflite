
#ifndef __AP_HAL_QUAN_MAIN_H__
#define __AP_HAL_QUAN_MAIN_H__

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan.h>

#define AP_HAL_MAIN() \
\
void osd_setup();\
void create_draw_task();\
void create_telemetry_transmitter_task();\
void create_apm_task(uint32_t flags);\
extern "C" void vTaskStartScheduler();\
\
extern "C" {\
   int main (void) \
   {\
      osd_setup(); \
   \
      create_draw_task(); \
      create_telemetry_transmitter_task();\
      create_apm_task(HAL_Quan::start_main); \
   \
      vTaskStartScheduler(); \
   }\
}

#endif // HAL_BOARD_QUAN

#endif // __AP_HAL_QUAN_MAIN_H__
