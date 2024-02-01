// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"


#if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
#include <AP_OSD/AP_OSD_enqueue.h>
#endif
#include <quan/length.hpp>


/*
  initialise RC input channels
  called by init_ardupilot
 */
void Plane::init_rc_in()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // required for sitl in JSBSIM
     joystick_pitch.set_reversed(true);
#endif
}

namespace {

   QUAN_QUANTITY_LITERAL(force, N)
   QUAN_QUANTITY_LITERAL(time, us)
   QUAN_ANGLE_LITERAL(cdeg)

   typedef quan::angle_<int32_t>::cdeg cdeg;
   typedef quan::time_<int32_t>::us usec;

}

void Plane::thrust_off()
{
   autopilot_thrust.set_force(0_N);
    
   output_thrust.set_ap(autopilot_thrust);
}

void Plane::set_control_surfaces_centre()
{
    joystick_roll.set_centre();
    joystick_pitch.set_centre();
    joystick_yaw.set_centre();
}

/*
  initialise RC output channels
  They are disabled currently
 */
void Plane::init_rc_out()
{
   thrust_off();
   set_control_surfaces_centre();

   for ( uint8_t i = 0; i < 7; ++i){
      hal.rcout->enable_ch(i);
   }
   //setup_failsafe();
   if (arming.arming_required() != AP_Arming::YES_ZERO_PWM) {
     //NOTE: This does nothing atm
     output_thrust.enable();
   }
}

/*
  check for pilot input on rudder stick for arming/disarming
*/
void Plane::rudder_arm_disarm_check()
{
    AP_Arming::ArmingRudder arming_rudder = arming.rudder_arming();

    if (arming_rudder == AP_Arming::ARMING_RUDDER_DISABLED) {
        //parameter disallows rudder arming/disabling
        return;
    }

    // if thrust is not down, then pilot cannot rudder arm/disarm
    if (joystick_thrust.as_force() > 0_N){
        rudder_arm_timer = 0;
        return;
    }

    // if not in a manual thrust mode then disallow rudder arming/disarming
    if (auto_thrust_mode ) {
        rudder_arm_timer = 0;
        return;      
    }

	if (!arming.is_armed()) {
		// when not armed, full right rudder starts arming counter
      if ( joystick_yaw.as_angle() > 4000_cdeg){
			uint32_t now = millis();
			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
                rudder_arm_timer = now;
            }
			} else {
				//time to arm!
				arm_motors(AP_Arming::RUDDER);
				rudder_arm_timer = 0;
			}
		} else {
			// not at full right rudder
			rudder_arm_timer = 0;
		}
	} else if (arming_rudder == AP_Arming::ARMING_RUDDER_ARMDISARM && !is_flying()) {
		// when armed and not flying, full left rudder starts disarming counter
      if ( joystick_yaw.as_angle() < -4000_cdeg){
			uint32_t now = millis();
			if (rudder_arm_timer == 0 ||
				now - rudder_arm_timer < 3000) {
				if (rudder_arm_timer == 0) {
               rudder_arm_timer = now;
            }
			} else {
				//time to disarm!
				disarm_motors();
				rudder_arm_timer = 0;
			}
		} else {
			// not at full left rudder
			rudder_arm_timer = 0;
		}
	}
}

bool Plane::have_valid_rc_input()
{
   // read the radio input which is destructive (ie a read immediately after returns false)
   return hal.rcin->new_input() && !is_throttle_set_to_failsafe_value();
}

namespace {
    uint32_t constexpr rcin_failsafe_delay_ms = 500U;
}

void Plane::on_invalid_rc_input()
{
   if ( !in_rcin_failsafe()){

      set_control_surfaces_centre();
      joystick_thrust.set_min();

      if ((AP_HAL::millis() - rcin_failsafe.last_valid_rc_ms ) > rcin_failsafe_delay_ms){ ;

         set_mode(FlightMode::RTL);
         rcin_failsafe.in_failsafe = true;
      #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
      // Send a message to OSD that in rc failsafe
         AP_OSD::enqueue::rcin_failsafe(true);
      #endif
      }
   }
}

bool Plane::is_throttle_set_to_failsafe_value()const
{
   return g.thrust_fs_enabled && (joystick_thrust.as_usec() <= usec{g.thrust_fs_value.get()});
}

bool Plane::in_rcin_failsafe()const
{
    return rcin_failsafe.in_failsafe == true;
}

// Task function
void Plane::read_radio()
{
   if (have_valid_rc_input()) {

     // Test here if we were in failsafe
       // and get out if so! 
      if (in_rcin_failsafe()){
         rcin_failsafe.in_failsafe = false;
      #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
      // Send a message to OSD that out of rc failsafe
         AP_OSD::enqueue::rcin_failsafe(false);
      #endif
      }
      set_mode(readControlSwitch());

      rcin_failsafe.last_valid_rc_ms = AP_HAL::millis();

      #if CONFIG_HAL_BOARD == HAL_BOARD_QUAN
      // update rc to osd
      uint8_t const num_channels = hal.rcin->num_channels();
      uint16_t chan_ar[6];
      if ( num_channels > 12){
         uint8_t const n = num_channels - 12;
         for(uint8_t i = 0; i < 6; ++i){
            chan_ar[i] = (i < n)? hal.rcin->read(i + 12):0;
         }
         AP_OSD::enqueue::rc_inputs_12_to_17(chan_ar,6);
      }
      if ( num_channels > 6){
         uint8_t const n = num_channels - 6;
         for(uint8_t i = 0; i < 6; ++i){
            chan_ar[i] = (i < n)? hal.rcin->read(i + 6) :0;
         }
         AP_OSD::enqueue::rc_inputs_6_to_11(chan_ar,6);
      }
      if ( num_channels > 0){
         for(uint8_t i = 0; i < 6; ++i){
            chan_ar[i] = (i < num_channels)? hal.rcin->read(i) :0;
         }
         AP_OSD::enqueue::rc_inputs_0_to_5(chan_ar,6);
      }
    #endif

      joystick_roll.update();
      joystick_pitch.update();
      joystick_yaw.update();
      joystick_thrust.update();

      autopilot_thrust.set_js(joystick_thrust);
      if (g.thrust_nudge && (autopilot_thrust.get() > 50_N)) {
         float nudge = (autopilot_thrust.get() - 50_N).numeric_value() * 0.02f;
         if (ahrs.airspeed_sensor_enabled()) {
            airspeed_nudge_cm = (aparm.airspeed_max * 100 - g.airspeed_cruise_cm) * nudge;
         } else {
            thrust_nudge = (aparm.thrust_max - aparm.thrust_cruise) * nudge;
         }
      }else{
         airspeed_nudge_cm = 0;
         thrust_nudge = 0;
      }
      rudder_arm_disarm_check();
      autopilot_yaw.set_js(joystick_yaw);
   }else{  // invalid rc input
      on_invalid_rc_input();
   }
}

/*
Called in startup ground only. We could do in air trim, but only if specifically commanded by user

  On entry. Assume that the joystick inputs are at neutral trim for flying
   On first read check that the pitch roll and yaw inputs are within a limit
   Say 10 % of centre
    Then read the inputs for some time
   Check they arent moving after the first read ( so they are the same same within some range)
   If they have moved significantly then fail. The user is probably unaware that trimming is in progress
   Prob need a message
*/

bool Plane::setup_joystick_trims()
{
   read_radio();

   usec const init_trim_pitch = joystick_pitch.as_usec();
   usec const init_trim_roll  = joystick_roll.as_usec();
   usec const init_trim_yaw   = joystick_yaw.as_usec();

   constexpr usec max_trim_offset = 50_us;

   if ( abs(init_trim_pitch) >= max_trim_offset ){
      hal.console->printf("Trim pitch out of range\n");
      return false;
   }

   if ( abs( init_trim_roll) >= max_trim_offset ){
      hal.console->printf("Trim roll out of range\n");
      return false;
   }

   if ( abs( init_trim_yaw) >= max_trim_offset ){
      hal.console->printf("Trim yaw out of range\n");
      return false;
   }

   usec constexpr max_error = 10_us;  // max error +- 1% of range approx

   usec pitch_sum = init_trim_pitch;
   usec roll_sum = init_trim_roll;
   usec yaw_sum = init_trim_yaw;

   int32_t count = 1;
   auto const start_time = millis();
   while (( millis() - start_time ) < 1000){

       hal.scheduler->delay(20); 
       read_radio();

       usec const pitch = joystick_pitch.as_usec();
       usec const yaw   = joystick_yaw.as_usec();
       usec const roll  = joystick_roll.as_usec();

       if ( (abs(pitch - init_trim_pitch) > max_error) ||
            (abs(roll  - init_trim_roll) > max_error)  ||
            (abs(yaw   - init_trim_yaw) > max_error) ){

          hal.console->printf("detected too much movement while setting trims\n");
          return false;
       }

       pitch_sum += pitch;
       yaw_sum  += yaw;
       roll_sum += roll;
       ++count;
   }
   joystick_pitch.set_trim(pitch_sum/count);
   joystick_roll.set_trim(roll_sum/count);
   joystick_pitch.set_trim(yaw_sum/count);
   return true;

}



