
// simple Flying wing mixer
namespace {

   float constexpr roll_gain = -0.5f;
   float constexpr pitch_gain = -0.5f;

   uint8_t constexpr num_outputs = 3;
   float output[num_outputs] = {0.f,0.f,0.f};

   void mixer_eval()
   {
      float const roll = plane.get_roll_demand() * roll_gain;
      float const pitch = plane.get_pitch_demand() * pitch_gain;

      output[0] = roll + pitch;
      output[1] = roll - pitch;
      output[2] = plane.get_thrust_demand();

      for ( uint8_t i = 0; i < num_outputs; ++i){
         output_action(i);
      }
   }
}



