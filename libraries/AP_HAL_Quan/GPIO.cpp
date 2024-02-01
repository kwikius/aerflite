

#include <AP_HAL/AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "GPIO.h"

#include <quan/stm32/gpio.hpp>
#include <resources.hpp>

using namespace Quan;

QuanGPIO::QuanGPIO()
{}

/*
 In fact the only gpios are LEDs
*/


namespace {

   template <typename Pin> struct gpio_pin final : public AP_HAL::DigitalSource{
      typedef typename Pin::port_type port_type;
      static_assert(quan::is_model_of<quan::stm32::gpio::Pin,Pin>::value,"not a port pin");

      void mode(uint8_t mode_in)
      {
         switch(mode_in){
            case HAL_GPIO_INPUT:
               quan::stm32::apply<
                  Pin 
                  ,quan::stm32::gpio::mode::input
                  ,quan::stm32::gpio::pupd::none
               >();
               break;
            case HAL_GPIO_OUTPUT:
               quan::stm32::apply<
                  Pin 
                  ,quan::stm32::gpio::mode::output
                  ,quan::stm32::gpio::pupd::none
                  ,quan::stm32::gpio::ospeed::slow
                  ,quan::stm32::gpio::ostate::low
               >();
               break;
            default:
               break;
         }
      }

      uint8_t read()
      {
         return quan::stm32::get<Pin>() == true? 1: 0;
      }

      void write(uint8_t value)
      {
         quan::stm32::put<Pin>(value);
      }

      void  toggle()
      {
         quan::stm32::complement<Pin>();
      }      
  };

   gpio_pin<heartbeat_led_pin>                      pin1;
   gpio_pin<quan::mcu::pin<quan::stm32::gpioa,14> > pin2;
   gpio_pin<quan::mcu::pin<quan::stm32::gpioa,13> > pin3;
   
   AP_HAL::DigitalSource * const pins_array[] =
   {
     &pin1
     ,&pin2
     ,&pin3
   };

   constexpr uint8_t num_pins() { return sizeof(pins_array) / sizeof(AP_HAL::DigitalSource*); }

   bool map_pin_to_array( uint8_t val_in , uint8_t & val_out)
   {
      if (( val_in < 1) || ( val_in > num_pins())){
         return false;
      }
      val_out = val_in-1;
      return true;
   }
   
} // namespace


void Quan::QuanGPIO::init()
 {
      quan::stm32::module_enable<decltype(pin1)::port_type>();
      quan::stm32::module_enable<decltype(pin2)::port_type>();
      quan::stm32::module_enable<decltype(pin3)::port_type>();
 }

void Quan::QuanGPIO::pinMode(uint8_t pin, uint8_t pin_mode)
{
   uint8_t pin_idx = 0;
   if ( map_pin_to_array(pin,pin_idx)){
      pins_array[pin_idx]->mode(pin_mode);
   }
}

int8_t Quan::QuanGPIO::analogPinToDigitalPin(uint8_t pin)
{
	return -1;
}


uint8_t Quan::QuanGPIO::read(uint8_t pin) {
   uint8_t pin_idx = 0;
   if ( map_pin_to_array(pin,pin_idx)){
      return pins_array[pin_idx]->read();
   }else{
      return 0;
   }
}

void Quan::QuanGPIO::write(uint8_t pin, uint8_t value)
{
   uint8_t pin_idx = 0;
   if ( map_pin_to_array(pin,pin_idx)){
      pins_array[pin_idx]->write(value);
   }
}

void Quan::QuanGPIO::toggle(uint8_t pin)
{
   uint8_t pin_idx = 0;
   if ( map_pin_to_array(pin,pin_idx)){
      pins_array[pin_idx]->toggle();
   }
}

/* Alternative interface: */
// could point invalid idx at dummy channel to avoid nullptr?
AP_HAL::DigitalSource* 
Quan::QuanGPIO::channel(uint16_t n) 
{
   uint8_t pin_idx = 0;
   if ( map_pin_to_array(n,pin_idx)){
      return pins_array[pin_idx];
   }else{
      return nullptr;
   }
}

/* Interrupt interface: */
bool Quan::QuanGPIO::attach_interrupt(uint8_t interrupt_num, AP_HAL::Proc p,
        uint8_t mode) 
{
    return true;
}

bool Quan::QuanGPIO::usb_connected(void)
{
    return false;
}

#endif
