#ifndef __AP_HAL_MAIN_H__
#define __AP_HAL_MAIN_H__

#include "HAL.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE
#define CONFIG_MAIN_WITHOUT_ARGC_ARGV 1
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_HAL_Quan/AP_HAL_Quan_Main.h>
#else

#ifndef AP_MAIN
#define AP_MAIN main
#endif

#if CONFIG_MAIN_WITHOUT_ARGC_ARGV

#define AP_HAL_MAIN() extern "C" { \
    int AP_MAIN(void); \
    int AP_MAIN(void) { \
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(0, NULL, &callbacks); \
        return 0; \
    } \
    }

#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { \
    int AP_MAIN(void); \
    int AP_MAIN(void) { \
        hal.run(0, NULL, CALLBACKS); \
        return 0; \
    } \
    }

#else

#define AP_HAL_MAIN() extern "C" { \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        AP_HAL::HAL::FunCallbacks callbacks(setup, loop); \
        hal.run(argc, argv, &callbacks); \
        return 0; \
    } \
    }

#define AP_HAL_MAIN_CALLBACKS(CALLBACKS) extern "C" { \
    int AP_MAIN(int argc, char* const argv[]); \
    int AP_MAIN(int argc, char* const argv[]) { \
        hal.run(argc, argv, CALLBACKS); \
        return 0; \
    } \
    }
#endif // CONFIG_MAIN_WITHOUT_ARGC_ARGV  
#endif // CONFIG_HAL_BOARD != HAL_BOARD_QUAN

#endif // __AP_HAL_MAIN_H__
