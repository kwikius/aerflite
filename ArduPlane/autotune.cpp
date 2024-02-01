

#include "Plane.h"

/*
  called when entering autotune
 */
void Plane::autotune_start(void)
{
    rollController.autotune_start();
    pitchController.autotune_start();
}

/*
  called when exiting autotune
 */
void Plane::autotune_restore(void)
{
    rollController.autotune_restore();
    pitchController.autotune_restore();
}

/*
  enable/disable autotune for AUTO modes
 */
void Plane::autotune_enable(bool enable)
{
    if (enable) {
        autotune_start();
    } else {
        autotune_restore();
    }
}