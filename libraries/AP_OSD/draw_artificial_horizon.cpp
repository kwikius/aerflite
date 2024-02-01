
#include <quan/uav/osd/api.hpp>
#include <quan/two_d/rotation.hpp>
#include <quan/uav/osd/get_aircraft_attitude.hpp>
#include <quan/meta/integer_min.hpp>
#include <quan/meta/integer_max.hpp>
#include <quan/constrain.hpp>
#include <AP_OSD/AP_OSD_dequeue.h>

using namespace quan::uav::osd;

/*
 potentially this is tricky since a 3d point may be anywhere in space
 and not displayable, a line may be partially displayabale etc, so should apply more comprehensive calcs
 Here I just constrain so that if pitch is more than some amount, its not displayed at all.
  Even then the size of the horizon bar and the viewing distance could cause a barf
*/

namespace {
   /*
      this "rotation" class just precalcs all the trig values
      call the function operator on the vector to get the transformed vector
   */
   struct rotation {
      // construct with required pitch and roll
      // z is along the fuselag
      // x is horizontal
      // y vertical 
      // e.g to make display natural
      rotation (quan::angle_<float>::deg const & pitch, quan::angle_<float>::deg const & roll, float z)
         : cos_roll {cos (roll) }
      , sin_roll {sin (roll) }
      , cos_pitch {cos (pitch) }
      , sin_pitch {sin (pitch) }
      , viewing_distance{z}
      {
         sin_roll_x_cos_pitch = sin_roll * cos_pitch;
         sin_roll_x_sin_pitch = sin_roll * sin_pitch;
         cos_roll_x_cos_pitch = cos_roll * cos_pitch;
         cos_roll_x_sin_pitch = cos_roll * sin_pitch;
         view_dist_x_sin_roll_x_sin_pitch = z * sin_roll_x_sin_pitch;
         view_dist_x_cos_roll_x_sin_pitch = z * cos_roll_x_sin_pitch;
         view_dist_x_cos_pitch = z * cos_pitch;
      }
      // could do as a three_d op?
      quan::two_d::vect<float> operator() (quan::two_d::vect<float> const & in) const
      {
         // Z represents the distance of the screen from the viewers eye.
         // prob calc from viewing angle
         // where screen is z pixel units wide then can work from there
         // really should be mm
         // should be adjustable within constraints
         // z should be large compared with display dimensions
         // then easier to solve angle issues
         // visual angle
       //  float const z = viewing_distance_px;
         quan::two_d::vect<float> result {
            in.x * cos_roll - in.y  * sin_roll_x_cos_pitch + view_dist_x_sin_roll_x_sin_pitch
            , in.x * sin_roll + in.y * cos_roll_x_cos_pitch - view_dist_x_cos_roll_x_sin_pitch
         };

         float z1 = in.y * sin_pitch + view_dist_x_cos_pitch;
         
         if (std::fabs(z1) < 0.5f) {
               int const sign_z1 = (z1 < 0.f) ? -1.f : 1.f;
               z1 = 0.5f * sign_z1;
            }
//         // problem if z1 is
//         if ( z1 < 0.5){
//            z1 = 0.5;
//         }
         float const ratio = viewing_distance / z1;
         result *= ratio;
         result.x = quan::constrain<float> (
                       result.x
                       , quan::meta::integer_min<int32_t>::value
                       , quan::meta::integer_max<int32_t>::value
                    );
         result.y = quan::constrain<float> (
                       result.y
                       , quan::meta::integer_min<int32_t>::value
                       , quan::meta::integer_max<int32_t>::value
                    );
         return result;
      }
   private:
      float cos_roll;
      float sin_roll;
      float cos_pitch;
      float sin_pitch;
      float sin_roll_x_cos_pitch;
      float sin_roll_x_sin_pitch ;
      float cos_roll_x_cos_pitch ;
      float cos_roll_x_sin_pitch ;
      float view_dist_x_sin_roll_x_sin_pitch;
      float view_dist_x_cos_roll_x_sin_pitch;
      float view_dist_x_cos_pitch;
      float viewing_distance;
   };
}

/*
ideally use a better calc to clip stuff to the diplay
*/
void AP_OSD::draw_artificial_horizon(
      AP_OSD::dequeue::osd_info_t const & info,
      AP_OSD::OSD_params const & osd)
{
   
   auto pitch = info.attitude.pitch + osd.artifical_horizon_pitch_adjustment;
   constexpr int32_t centre_rad = 5;
   if ( abs(pitch) < angle_type{30}) {
      rotation rotate {pitch,info.attitude.roll, static_cast<float>(osd.viewing_distance_px)};
      constexpr int32_t width = 127;
      constexpr int32_t outer_h_bar_len = 16;
      constexpr int32_t outer_stop_height = 8;
      
      pxp_type left_end { -width / 2, 0};
      pxp_type right_end {width / 2, 0};
      
      for (int32_t i = -1; i < 2; ++i) {
         const colour_type c = (i) ? colour_type::black : colour_type::white;
         draw_line (
            rotate (left_end + pxp_type {1, i}),
            rotate (left_end + pxp_type {outer_h_bar_len , i}), c
         );
         draw_line (
            rotate (right_end + pxp_type {0, i}),
            rotate (right_end + pxp_type { -outer_h_bar_len, i}), c
         );
         draw_line (
            rotate (pxp_type {left_end.x - i, outer_stop_height / 2}),
            rotate (pxp_type {left_end.x - i, -outer_stop_height / 2}), c
         );
         draw_line (
            rotate (pxp_type {right_end.x + i, outer_stop_height / 2}),
            rotate (pxp_type {right_end.x + i, -outer_stop_height / 2}), c
         );
         
         draw_circle (centre_rad + i, rotate ( {0, 0}), c);
         draw_line (rotate (pxp_type { -20, i}), rotate (pxp_type { -centre_rad - 1 , i}), c);
         draw_line (rotate (pxp_type {20, i}), rotate (pxp_type {centre_rad + 1, i}), c);
      }
   }

   for (int32_t i = -1; i < 2; ++i) {
      const colour_type c = (i) ? colour_type::black : colour_type::white;
      draw_circle(centre_rad + i, {0, 0}, c);
      draw_line({-20,i},{-centre_rad - 1 ,i},c);
      draw_line({centre_rad + 1 ,i},{20,i},c);
      draw_line( {i,centre_rad + 1 },{i,10},c);
   }
}
