#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL


#include "RCInput.h"

using namespace HALSITL;

extern const AP_HAL::HAL& hal;

void SITLRCInput::init(void* machtnichts)
{
    clear_overrides();
    m_joystick = new quan::joystick{"/dev/input/js0"};
    
}

bool SITLRCInput::new_input()
{
//    if (_sitlState->new_rc_input) {
//        _sitlState->new_rc_input = false;
//        return true;
//    }
//    return false;
     return true;
}

uint16_t SITLRCInput::read(uint8_t ch) {
    if (ch >= 8) {
        return 0;
    }
   // return _override[ch]? _override[ch] : _sitlState->pwm_input[ch];
   if (m_joystick != nullptr){
      return static_cast<uint16_t>(m_joystick->get_channel(ch) * (500.f/32767.f) + 1500.f);
   }else{
      if (ch == 2){
         return 1000U;
      }else{
         return 1500U;
      }
   }
   
}

uint8_t SITLRCInput::read(uint16_t* periods, uint8_t len) {
//    for (uint8_t i=0; i<len; i++) {
//        periods[i] = _override[i]? _override[i] : _sitlState->pwm_input[i];
//    }
//    return 8;
     return 0;
}

bool SITLRCInput::set_overrides(int16_t *overrides, uint8_t len) {
//    bool res = false;
//    for (uint8_t i = 0; i < len; i++) {
//        res |= set_override(i, overrides[i]);
//    }
//    return res;
     return false;
}

bool SITLRCInput::set_override(uint8_t channel, int16_t override) {
//    if (override < 0) return false; /* -1: no change. */
//    if (channel < 8) {
//        _override[channel] = override;
//        if (override != 0) {
//            return true;
//        }
//    }
    return false;
}

void SITLRCInput::clear_overrides()
{
    for (uint8_t i = 0; i < 8; i++) {
        _override[i] = 0;
    }
}
#endif
