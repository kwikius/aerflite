
#ifndef __AP_HAL_QUAN_TEST_MAIN_H__
#define __AP_HAL_QUAN_TEST_MAIN_H__

#include <AP_HAL_Quan/HAL_Quan_Class.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#define AP_HAL_TEST_MAIN(flags) \
\
void osd_setup();\
void create_draw_task();\
void create_telemetry_transmitter_task();\
void create_apm_task(uint32_t );\
extern "C" void vTaskStartScheduler();\
\
extern "C" {\
   int main (void) \
   {\
      osd_setup(); \
   \
      create_draw_task(); \
      create_telemetry_transmitter_task();\
      create_apm_task(flags); \
   \
      vTaskStartScheduler(); \
   }\
}

#endif // HAL_BOARD_QUAN

#endif // __AP_HAL_QUAN_TEST_MAIN_H__
