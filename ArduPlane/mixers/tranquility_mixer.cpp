
namespace {

   // servo output channels
   uint8_t constexpr stbd_aileron_channel = 0U;  // OutCh1
   uint8_t constexpr stbd_v_tail_channel = 1U;   //OutCh2
   uint8_t constexpr throttle_channel = 2U;      //OutCh3
   // power_supply on ch                         // PSU on CH 4
   uint8_t constexpr port_v_tail_channel = 4U;    //OutCh5
   uint8_t constexpr port_aileron_channel = 5U;   // OutCh6

   float constexpr stbd_aileron_dir = 1.f;
   float constexpr port_aileron_dir = -stbd_aileron_dir;
   float constexpr port_v_tail_dir = 1;
   float constexpr stbd_v_tail_dir = - port_v_tail_dir;

   // up ail gain should be more than down
   float constexpr up_ail_gain = 0.75f/2.f;
   float constexpr down_ail_gain = 0.5f/2.f;

   // flaps are offset at neutral to compensate for greater flpas down angle
   // relative to flaps up angle
   float constexpr flap_offset = 0.315;

   float constexpr up_flap_gain = 0.1f;
   float constexpr down_flap_gain = 0.3f;

   float constexpr rudder_roll_gain = -0.2f;
   float constexpr elev_flap_gain = 0.1f;

   float constexpr elev_aileron_gain = 0.0f;

   float constexpr v_tail_pitch_gain = 0.6f;
   float constexpr v_tail_yaw_gain = 0.4f;

   uint8_t constexpr num_outputs = 6;
   float output[num_outputs] = {0.f,0.f,0.f,0.f,0.f,0.f};

   void mixer_eval()
   {
     bool const manual_mode = plane.get_control_mode() == FlightMode::MANUAL;

     float const roll = -plane.get_roll_demand();
     bool const positive_roll = roll > 0.f;
     float const port_aileron = roll * (positive_roll? up_ail_gain: down_ail_gain);
     float const stbd_aileron = -roll * (positive_roll? down_ail_gain: up_ail_gain);

     float const thrust_in = plane.get_thrust_demand();

     output[throttle_channel] = thrust_in;

     float const flap0 = (manual_mode == true)?-plane.get_flap_demand():0.f;
     bool const  up_flap = flap0 < 0.f;
     float const flap = flap0 * (up_flap?up_flap_gain:down_flap_gain);

     output[port_aileron_channel] = (port_aileron  + flap_offset - flap ) * port_aileron_dir;
     output[stbd_aileron_channel] = (stbd_aileron  + flap_offset - flap ) * stbd_aileron_dir;

     float const elev_to_flap = flap * elev_flap_gain;
     float const roll_to_rudder = roll * rudder_roll_gain;
   //  float const elev_to_crow = elev_to_crow_gain * crow;
     float const pitch = plane.get_pitch_demand();
     float const yaw = plane.get_yaw_demand();

     float const ail_to_elev = abs(roll) * elev_aileron_gain;

     output[port_v_tail_channel] = (
          pitch * v_tail_pitch_gain
          + yaw * v_tail_yaw_gain
          + roll_to_rudder
          + elev_to_flap
          + ail_to_elev
     ) * port_v_tail_dir;
    // output[stbd_v_tail] = -pitch * 0.5f + yaw * 0.5f + roll_to_rudder - elev_to_flap - elev_to_crow;
     output[stbd_v_tail_channel] = (
         pitch * v_tail_pitch_gain
         - yaw * v_tail_yaw_gain
         - roll_to_rudder
         + elev_to_flap
         + ail_to_elev) * stbd_v_tail_dir;

     for ( uint8_t i = 0; i < num_outputs; ++i){
         output_action(i);
     }
   }
}
