#ifndef AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED
#define AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED

#include "JoystickInput_base.h"
#include "FlightAxes.h"

#include <quan/force.hpp>


template <typename FlightAxisT>
struct JoystickInput : JoystickInput_angle{
   
   explicit JoystickInput(uint8_t ch_in) : JoystickInput_angle{ch_in}{}
};

template <>
struct JoystickInput<FlightAxis::Thrust> : JoystickInput_base{

   explicit JoystickInput(uint8_t ch_in) : JoystickInput_base{ch_in,get_min()}{}
   // use force to catch errors rather than as a real rep of thrust here
   // one day it may be possible to work this out
   // properly
   typedef quan::force_<int32_t>::N force_type;
   force_type as_force() const;
};

#endif // AERFLITE_JOYSTICK_INPUT_HPP_INCLUDED
