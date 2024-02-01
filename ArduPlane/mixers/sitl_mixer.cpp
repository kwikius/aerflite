
// sitl mixer
namespace {

   uint8_t constexpr num_outputs = 4;
   float output[num_outputs] = {0.f,0.f,0.f};

   void mixer_eval()
   {
       output[0] = plane.get_roll_demand();
       output[1] = -plane.get_pitch_demand();
       output[2] = plane.get_thrust_demand();
       output[3] = plane.get_yaw_demand();

       for ( uint8_t i = 0; i < num_outputs; ++i){
         output_action(i);
       }
   }
}
