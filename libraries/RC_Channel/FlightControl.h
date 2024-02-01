#ifndef AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
#define AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED

#include "JoystickInput.h"
#include <quan/constrain.hpp>
#include <quan/time.hpp>


template <typename FlightAxisT>
struct FltCtrlInput{
   typedef quan::angle_<int32_t>::cdeg cdeg;
   void set_js(JoystickInput<FlightAxisT> const & in){ this->set_angle(in.as_angle());}
   void set_angle( cdeg const & in) {m_value = in;}
   cdeg get() const {return m_value;}
   FltCtrlInput() : m_value{cdeg{0}}{}
private:
   cdeg m_value; // value between -4500 and 4500 cdeg
   FltCtrlInput(FltCtrlInput const & ) = delete;
   FltCtrlInput & operator =(FltCtrlInput const & ) = delete; 
};

template <>
struct FltCtrlInput<FlightAxis::Thrust>{
   typedef quan::force_<int32_t>::N force_type;
   void set_js(JoystickInput<FlightAxis::Thrust> const & in){ this->set_force(in.as_force());}
   void set_force(force_type const & in) ;
   void constrain(force_type const & min_in,force_type const &  max_in) { m_value = quan::constrain(m_value,min_in,max_in);}
   force_type get() const {return m_value;}
   FltCtrlInput() : m_value{force_type{0}}{}
private:
   force_type m_value; // value between 0 and 100 N
   FltCtrlInput(FltCtrlInput const & ) = delete;
   FltCtrlInput & operator =(FltCtrlInput const & ) = delete; 
};

struct FltCtrlOutput_base {
 //  void print() const;
  // typedef quan::time_<int32_t>::us usec;
   void set_float(float const & in);
  // void set(float const & in) { m_value = quan::constrain(in,-1.f,1.f);}
   void constrain(float const & min_in, float const & max_in) { m_value = quan::constrain(m_value,min_in,max_in);}
   float get() const  {return m_value;}
protected:
   explicit FltCtrlOutput_base( float const & value) : m_value{quan::constrain(value,-1.f,1.f)}{}
private:
   float m_value;
   FltCtrlOutput_base(FltCtrlOutput_base const & ) = delete;
   FltCtrlOutput_base & operator =(FltCtrlOutput_base const & ) = delete; 
};

template <typename FlightAxisT>
struct FltCtrlOutput : FltCtrlOutput_base{
   FltCtrlOutput() :  FltCtrlOutput_base{0.f}{}
   using FltCtrlOutput_base::set_float;
   using FltCtrlOutput_base::get;
   void set_js(JoystickInput<FlightAxisT> const & in)
   {
      this->set_float(in.as_float());
   }

   void set_ap(FltCtrlInput<FlightAxisT> const & in)
   {
      this->set_float(in.get().numeric_value()/4500.f); 
   }
  // float as_float() const {return this->get();}
private:
   FltCtrlOutput(FltCtrlOutput const & ) = delete;
   FltCtrlOutput & operator =(FltCtrlOutput const & ) = delete; 
};

template <>
struct FltCtrlOutput<FlightAxis::Thrust> : FltCtrlOutput_base{

   FltCtrlOutput() :  FltCtrlOutput_base{-1.f}{}
   void set_js(JoystickInput<FlightAxis::Thrust> const & in)
   {
      this->set_float( (in.as_force().numeric_value()/ 50.f)-1.f);
   }

   void set_ap(FltCtrlInput<FlightAxis::Thrust> const & in);

   void enable(){}
   void disable(){}

private:
   FltCtrlOutput(FltCtrlOutput const & ) = delete;
   FltCtrlOutput & operator =(FltCtrlOutput const & ) = delete; 
};

#endif // AERFLITE_ARDUPILOT_FLIGHT_CONTROL_H_INCLUDED
