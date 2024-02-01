

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN

#include "AP_Compass_Quan.h"
#include "compass_product_id.hpp"

namespace {
   AP_Compass_Quan compass_driver;
}

template <> AP_Compass_Backend * connect_compass_driver<Quan::tag_board>(Compass& compass)
{
    return compass_driver.connect(compass);
}

extern const AP_HAL::HAL& hal;

AP_Compass_Quan::AP_Compass_Quan()
:m_hQueue{NULL},m_instance{static_cast<uint8_t>(-1)}
{}

void  AP_Compass_Quan::update(void)
{
   Quan::detail::compass_args args;
   // receive should be available
   if ( xQueueReceive(m_hQueue, &args,0) == pdTRUE) {

      Vector3f raw_field{
//         args.field.x.numeric_value()
//         ,args.field.y.numeric_value()
//         ,args.field.z.numeric_value()
         args.field.y.numeric_value(),
         args.field.x.numeric_value(),
         -args.field.z.numeric_value()
      };

      rotate_field(raw_field, m_instance);
      publish_raw_field(raw_field, args.time_us, m_instance);
      correct_field(raw_field, m_instance);
      publish_unfiltered_field(raw_field, args.time_us, m_instance);
      publish_filtered_field(raw_field, m_instance);
   }
}

AP_Compass_Quan * AP_Compass_Quan::connect(Compass &compass)
{
    this->set_compass(compass);
    this->m_hQueue = Quan::get_compass_queue_handle();
    this->m_instance = compass.register_compass();
    this->set_dev_id(m_instance,AP_COMPASS_TYPE_QUAN);
    this->set_external(m_instance, true);
    return this;
}

#endif  //#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
