
#ifndef AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED
#define AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED

/// @file	JoystickInput.h
/// @brief	JoystickInput manager, with EEPROM-backed storage of constants.

#include <AP_Param/AP_Param.h>
#include <quan/time.hpp>

#include <quan/angle.hpp>

/// @class	JoystickInput
/// @brief	rcin input buffer and switch abstraction to map an rc channel
  //  to map a specific joystick input
  // e.g 
  // JoyStickInput joystick_yaw{0};  // map to ch0 

extern const AP_HAL::HAL& hal;

struct JoystickInput_base {
    typedef quan::time_<int32_t>::us usec;
    static constexpr uint8_t max_channels = 14;
    // note that the init_value is not reversed
    // even if the raw input sense is reversed
    // so the init_value doesnt need to change
    // irespective of which sense user wants for input
   protected:
    explicit JoystickInput_base(uint8_t ch_in, usec const & init_value) :
        m_value{init_value} 
        ,m_rcin_idx{ch_in} 
    {
       //potentially reversed from eeprom
       // AP_Param::setup_object_defaults(this, var_info);
    }
public:
    // startup
    void        load_eeprom(void);
    void        save_eeprom(void);
    void        save_trim(void);

    /* 
      update from rcin
      reverse the sense of the raw rcin value if input_sense_reversed() is true
    */
    void        update() ;
   /* set the value to centre */

   /* set the value to min */
    void        set_min() { this->m_value = get_min();}

    /*
      return a normalised input in range -1 to 1,
      centered around the channel trim. 
     */

    usec        as_usec()const {return m_value;}

protected:
    void set_value(usec in) { m_value = in;}

private:   
   static constexpr usec m_min{1000};
   static constexpr usec m_max{2000};
public:
    static constexpr usec get_min() {return m_min;}
    static constexpr usec get_max() {return m_max;}
    // statistical range 
    static constexpr usec get_range(){return get_max() - get_min();}
   // static constexpr usec get_defualt_trim() {return m_default_trim;}
    uint8_t get_rcin_index() const { return m_rcin_idx;}
    void set_reversed (bool b) { m_is_reversed = b;}
#if !defined QUAN_PUBLIC_PRIVATE_MEMBERS
private:
#endif
   bool       input_sense_reversed(void) const { return m_is_reversed;}
   usec           m_value;
   uint8_t const  m_rcin_idx;
   bool           m_is_reversed;
   JoystickInput_base(JoystickInput_base const & ) = delete;
   JoystickInput_base & operator =(JoystickInput_base const & ) = delete; 
};

struct JoystickInput_angle : JoystickInput_base {
   typedef quan::angle_<int32_t>::cdeg cdeg;
   static   constexpr usec get_default_trim() { return (get_max() + get_min()) / 2;}
   explicit JoystickInput_angle(uint8_t ch_in) 
   : JoystickInput_base{ch_in,get_default_trim()}, m_trim{get_default_trim()}{}
   bool set_trim( usec const & in);
   usec     get_trim() const  {return m_trim;}
   float    as_float()const ;
   void     set_centre() { set_value(get_trim());}
   cdeg     as_angle() const ;
   private:
      usec m_trim;
   JoystickInput_angle(JoystickInput_angle const & ) = delete;
   JoystickInput_angle & operator =(JoystickInput_angle const & ) = delete; 
   
};

#endif  //AERFLITE_JOYSTICK_INPUT_BASE_HPP_INCLUDED
