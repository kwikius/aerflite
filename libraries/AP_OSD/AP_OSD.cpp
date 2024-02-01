

#include <AP_OSD/AP_OSD.h>

extern const AP_HAL::HAL& hal;

AP_OSD::OSD_params::OSD_params()
 : artifical_horizon_pitch_adjustment{quan::angle_<float>::deg{-7.f}}
 ,viewing_distance_px{250}
 ,battery_pos{120,-140,-80}
 ,gps_pos{-170,-140,-80}
 ,control_mode_pos{40,-140,-80}
 ,airspeed_pos{-170,-100,-80}
 ,homeinfo_pos{-170,100,70}
 ,alt_scale_pos{100,0,0}
{}






