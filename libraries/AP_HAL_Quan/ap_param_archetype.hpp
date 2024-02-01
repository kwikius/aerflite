#ifndef APM_AP_HAL_QUAN_AP_PARAM_ARCHETYPE_HPP_INCLUDED
#define APM_AP_HAL_QUAN_AP_PARAM_ARCHETYPE_HPP_INCLUDED

#include <AP_Param/AP_Param.h>
#include <quan/is_model_of.hpp>
#include <type_traits>

namespace quan{

   struct AP_Param_Archetype; // struct tag representing the Archetype

   namespace impl{
      // make any AP_Param a model of the archetype
      template <typename T> 
      struct is_model_of_impl<
         AP_Param_Archetype,T,typename quan::where_<std::is_base_of<AP_Param,T> >::type 
      > : quan::meta::true_{};
   }

}

#endif // APM_AP_HAL_QUAN_AP_PARAM_ARCHETYPE_HPP_INCLUDED
