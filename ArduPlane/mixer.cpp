#include "Plane.h"

extern const AP_HAL::HAL& hal;

namespace {
   void output_action(uint8_t channel);
}

#if (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
   #include "mixers/sitl_mixer.cpp"
#elif (defined(QUAN_MIXER_DISCO))
   #include "mixers/disco_mixer.cpp"
#elif (defined(QUAN_MIXER_FALCON))
   #include "mixers/falcon_mixer.cpp"
#elif (defined(QUAN_MIXER_TRANQUILITY))
   #include "mixers/tranquility_mixer.cpp"
#else
  #pragma message "no mixer defined - using default"
  #include "mixers/default_mixer.cpp"
#endif

namespace {
   void output_action(uint8_t channel)
   {
      float const v1 = (output[channel] + 3.f) * 500.f;
      uint16_t const out = quan::constrain(
         static_cast<uint16_t>(v1),
         static_cast<uint16_t>(1000U),
         static_cast<uint16_t>(2000U)
      );
      hal.rcout->write(channel,out);
   }
}

bool Plane::create_mixer()
{
   return true;
}

void Plane::mix()
{
    mixer_eval();
}
