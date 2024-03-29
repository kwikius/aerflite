/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  parent class for aircraft simulators
*/

#include "SIM_Aircraft.h"
#include <quan/atan2.hpp>

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef __CYGWIN__
#include <windows.h>
#include <time.h>
#include <Mmsystem.h>
#endif

namespace {

    using rad_per_s = quan::reciprocal_time_<
      quan::angle_<double>::rad
   >::per_s ;

  // template <typename T>
   using deg_per_s = quan::reciprocal_time_<
      quan::angle_<double>::deg 
   >::per_s ;

   using rev_per_min = typename quan::reciprocal_time_<
      typename quan::angle_<double>::rev
   >::per_min ;

}

namespace SITL {

/*
  parent class for all simulator types
 */

Aircraft::Aircraft(const char *home_str, const char *frame_str) :
    ground_level(0),
    frame_height(0),
    dcm(),
    gyro(),
    velocity_ef(),
    mass(0),
    accel_body(0, 0, -GRAVITY_MSS),
    time_now_us(0),
    gyro_noise(radians(0.1f)),
    accel_noise(0.3),
    rate_hz(1200),
    autotest_dir(NULL),
    frame(frame_str),
#ifdef __CYGWIN__
    min_sleep_time(20000)
#else
    min_sleep_time(5000)
#endif
{
    float yaw_degrees;

    parse_home(home_str, home, yaw_degrees);
    location = home;
    ground_level = home.alt*0.01;

    dcm.from_euler(0, 0, radians(yaw_degrees));

    set_speedup(1);

    last_wall_time_us = get_wall_time_us();
    frame_counter = 0;
}


/*
  parse a home string into a location and yaw
 */
bool Aircraft::parse_home(const char *home_str, Location &loc, float &yaw_degrees)
{
    char *saveptr=NULL;
    char *s = strdup(home_str);
    if (!s) {
        return false;
    }
    char *lat_s = strtok_r(s, ",", &saveptr);
    if (!lat_s) {
        return false;
    }
    char *lon_s = strtok_r(NULL, ",", &saveptr);
    if (!lon_s) {
        return false;
    }
    char *alt_s = strtok_r(NULL, ",", &saveptr);
    if (!alt_s) {
        return false;
    }
    char *yaw_s = strtok_r(NULL, ",", &saveptr);
    if (!yaw_s) {
        return false;
    }

    memset(&loc, 0, sizeof(loc));
    loc.lat = atof(lat_s) * 1.0e7;
    loc.lng = atof(lon_s) * 1.0e7;
    loc.alt = atof(alt_s) * 1.0e2;

    yaw_degrees = atof(yaw_s);
    free(s);

    return true;
}
    
/*
   return true if we are on the ground
*/
bool Aircraft::on_ground(const Vector3f &pos) const
{
    return (-pos.z) + home.alt*0.01f <= ground_level + frame_height;
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    float bearing = degrees(atan2f(position.y, position.x));
    float distance = sqrtf(sq(position.x) + sq(position.y));

    location = home;
    location_update(location, bearing, distance);

    location.alt  = home.alt - position.z*100.0f;

    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    sync_frame_time();
}

/*
   rotate to the given yaw
*/
void Aircraft::set_yaw_degrees(float yaw_degrees)
{
    float roll, pitch, yaw;
    dcm.to_euler(&roll, &pitch, &yaw);

    yaw = radians(yaw_degrees);
    dcm.from_euler(roll, pitch, yaw);
}

/* advance time by deltat in seconds */
void Aircraft::time_advance(float deltat)
{
    time_now_us += deltat * 1.0e6f;
}

/* setup the frame step time */
void Aircraft::setup_frame_time(float new_rate, float new_speedup)
{
    rate_hz = new_rate;
    target_speedup = new_speedup;
    frame_time_us = 1.0e6f/rate_hz;

    scaled_frame_time_us = frame_time_us/target_speedup;
    last_wall_time_us = get_wall_time_us();
    achieved_rate_hz = rate_hz;
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    if (rate_hz != new_rate) {
        rate_hz = new_rate;
        frame_time_us = 1.0e6f/rate_hz;
        scaled_frame_time_us = frame_time_us/target_speedup;
    }
}

/*
   try to synchronise simulation time with wall clock time, taking
   into account desired speedup
   This tries to take account of possible granularity of
   get_wall_time_us() so it works reasonably well on windows
*/
void Aircraft::sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    if (frame_counter >= 40 &&
        now > last_wall_time_us) {
        float rate = frame_counter * 1.0e6f/(now - last_wall_time_us);
        achieved_rate_hz = (0.99f*achieved_rate_hz) + (0.01f*rate);
        if (achieved_rate_hz < rate_hz * target_speedup) {
            scaled_frame_time_us *= 0.999f;
        } else {
            scaled_frame_time_us /= 0.999f;
        }
#if 0
        ::printf("achieved_rate_hz=%.3f rate=%.2f rate_hz=%.3f sft=%.1f\n",
                 (double)achieved_rate_hz,
                 (double)rate,
                 (double)rate_hz,
                 (double)scaled_frame_time_us);
#endif
        uint32_t sleep_time = scaled_frame_time_us*frame_counter;
        if (sleep_time > min_sleep_time) {
            usleep(sleep_time);
        }
        last_wall_time_us = now;
        frame_counter = 0;
    }
}

/* add noise based on throttle level (from 0..1) */
void Aircraft::add_noise(float throttle)
{
    gyro += Vector3f(rand_normal(0, 1),
                     rand_normal(0, 1),
                     rand_normal(0, 1)) * gyro_noise * throttle;
    accel_body += Vector3f(rand_normal(0, 1),
                           rand_normal(0, 1),
                           rand_normal(0, 1)) * accel_noise * throttle;
}

/*
  normal distribution random numbers
  See
  http://en.literateprograms.org/index.php?title=Special:DownloadCode/Box-Muller_transform_%28C%29&oldid=7011
*/
double Aircraft::rand_normal(double mean, double stddev)
{
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached)
    {
        double x, y, r;
        do
        {
            x = 2.0*rand()/RAND_MAX - 1;
            y = 2.0*rand()/RAND_MAX - 1;

            r = x*x + y*y;
        }
        while (r == 0.0 || r > 1.0);
        {
            double d = sqrt(-2.0*log(r)/r);
            double n1 = x*d;
            n2 = y*d;
            double result = n1*stddev + mean;
            n2_cached = 1;
            return result;
        }
    }
    else
    {
        n2_cached = 0;
        return n2*stddev + mean;
    }
}

/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm(struct sitl_fdm &fdm) const
{
    fdm.timestamp_us = time_now_us;
//###########################
    fdm.latitude  = quan::angle_<double>::deg10e7{location.lat};
    fdm.longitude = quan::angle_<double>::deg10e7{location.lng};
    fdm.altitude_asl  = quan::length_<double>::cm{location.alt};
//#######################
    fdm.heading   = quan::atan2(velocity_ef.y, velocity_ef.x);
    fdm.speedN    = quan::velocity::m_per_s{velocity_ef.x};
    fdm.speedE    = quan::velocity::m_per_s{velocity_ef.y};
    fdm.speedD    = quan::velocity::m_per_s{velocity_ef.z};
    fdm.xAccel    = quan::acceleration::m_per_s2{accel_body.x};
    fdm.yAccel    = quan::acceleration::m_per_s2{accel_body.y};
    fdm.zAccel    = quan::acceleration::m_per_s2{accel_body.z};
    fdm.rollRate  = deg_per_s{quan::angle::rad{gyro.x}};
    fdm.pitchRate = deg_per_s{quan::angle::rad{gyro.y}};
    fdm.yawRate   = deg_per_s{quan::angle::rad{gyro.z}};
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = quan::angle_<float>::rad{r};
    fdm.pitchDeg = quan::angle_<float>::rad{p};
    fdm.yawDeg   = quan::angle_<float>::rad{y};
    fdm.airspeed = quan::velocity_<double>::m_per_s{airspeed};
    fdm.battery_voltage = quan::voltage_<double>::V{battery_voltage};
    fdm.battery_current = quan::current_<double>::A{battery_current};
    fdm.rpm1 = rev_per_min{quan::angle::rev{rpm1}};
    fdm.rpm2 = rev_per_min{quan::angle::rev{rpm2}};
}

uint64_t Aircraft::get_wall_time_us() const
{
#ifdef __CYGWIN__
    static DWORD tPrev;
    static uint64_t last_ret_us;
    if (tPrev == 0) {
        tPrev = timeGetTime();
        return 0;
    }
    DWORD now = timeGetTime();
    last_ret_us += (uint64_t)((now - tPrev)*1000UL);
    tPrev = now;
    return last_ret_us;
#else
    struct timeval tp;
    gettimeofday(&tp,NULL);
    return tp.tv_sec*1.0e6 + tp.tv_usec;
#endif
}

/*
  set simulation speedup
 */
void Aircraft::set_speedup(float speedup)
{
    setup_frame_time(rate_hz, speedup);
}

} // namespace SITL
