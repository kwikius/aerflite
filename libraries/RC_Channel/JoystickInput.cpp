#include "JoystickInput.h"
#include "FlightControl.h"
#include <quan/constrain.hpp>
#include <AP_HAL/HAL.h>

constexpr JoystickInput_base::usec JoystickInput_base::m_min;
constexpr JoystickInput_base::usec JoystickInput_base::m_max;

extern const AP_HAL::HAL& hal ;

void FltCtrlOutput_base::set_float(float const & in)
{
   m_value = quan::constrain(in,-1.f,1.f); 
}

void FltCtrlInput<FlightAxis::Thrust>::set_force(force_type const & in) 
{
   m_value = in;
}

void FltCtrlOutput<FlightAxis::Thrust>::set_ap(FltCtrlInput<FlightAxis::Thrust> const & in)
{
   this->set_float((in.get().numeric_value()/ 50.f)-1.f); 
}

void JoystickInput_base::update() 
{
  usec const rcin{hal.rcin->read(m_rcin_idx)};
  if ( input_sense_reversed()){
     m_value = (get_min() + get_max()) - rcin ;
  }else{
     m_value = rcin;
  }
}

float JoystickInput_angle::as_float()const 
{ 
   return quan::constrain(
      ((this->as_usec() - this->get_trim()) * 2.f) / get_range()
      ,-1.f, 1.f
   );
}

JoystickInput_angle::cdeg JoystickInput_angle::as_angle() const
{
   return (( as_usec() - get_trim() ) * cdeg{4500} * 2 ) 
      / get_range();
}

// N.B doesnt allow for negative throttle
JoystickInput<FlightAxis::Thrust>::force_type 
JoystickInput<FlightAxis::Thrust>::as_force() const
{
   return (( as_usec() - get_min() ) * force_type{100} ) 
   / get_range();
}

bool JoystickInput_angle::set_trim( usec const & in)
{
   if ( (in < get_max() ) && (in > get_min()) &&  ( (abs(in - get_default_trim()) * 8) < get_range()) ){
      m_trim = in;
      return true;
   }else{
      return false;
   }
}
