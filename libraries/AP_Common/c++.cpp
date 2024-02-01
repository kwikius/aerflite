// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// C++ runtime support not provided by Arduino
//
// Note: use new/delete with caution.  The heap is small and
// easily fragmented.

#include <AP_HAL/AP_HAL.h>
#include <stdlib.h>
#include <stdio.h>

/*
  globally override new and delete to ensure that we always start with
  zero memory. This ensures consistent behaviour.
 */

#if CONFIG_HAL_BOARD != HAL_BOARD_QUAN
void * operator new(size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete(void *p)
{
    if (p) free(p);
}

void * operator new[](size_t size)
{
    if (size < 1) {
        size = 1;
    }
    return(calloc(size, 1));
}

void operator delete[](void * ptr)
{
    if (ptr) free(ptr);
}
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
extern "C" int _kill(int pid) { return 0;}
extern "C" int _getpid(void) {return 0;}
extern "C" ssize_t _write(int fd,const void* buf, size_t size) { return -1;}
extern "C" int _close(int fd) { return 0;}
extern "C" int _fstat(int fd, void* buf) { return 0;}
extern "C" int _lseek (int fd,int count) { return 0;}
extern "C" int _read(int fd, char* buf, int count){return 0;}
extern "C" int _isatty(int fd){return 0;}

#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

__extension__ typedef int __guard __attribute__((mode (__DI__)));

int __cxa_guard_acquire(__guard *g)
{
    return !*(char *)(g);
};

void __cxa_guard_release (__guard *g){
    *(char *)g = 1;
};

void __cxa_guard_abort (__guard *) {
};


#endif // CONFIG_HAL_BOARD

