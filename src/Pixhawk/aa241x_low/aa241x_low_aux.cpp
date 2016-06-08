/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file aa241x_low_aux.cpp
 *
 * Auxiliary file to the AA241x low priority control loop.
 * This file contains the definition of all shared global variables for
 * this module and the functions that are able to be called by students
 * from the aa241x_low.cpp file.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <drivers/drv_hrt.h>

#include "aa241x_low_aux.h"

// set these variables for help in debugging (these will be sent to the ground station)
// TODO: add local position setpoint info

namespace aa241x_low {

// Tait-Bryan Euler angles in radian
float roll = 0.0f;		/**< range from -pi .. pi */
float pitch = 0.0f;		/**< range from -pi/2 .. pi/2 */
float yaw = 0.0f;		/**< range from -pi .. pi */

// angular rates, Tait-Bryan NED [rad/s]
float roll_rate = 0.0f;
float pitch_rate = 0.0f;
float yaw_rate = 0.0f;

// body velocities [m/s]
float speed_body_u = 0.0f;
float speed_body_v = 0.0f;
float speed_body_w = 0.0f;

// body accelerations [m/s^2]
float accel_body_x = 0.0f;
float accel_body_y = 0.0f;
float accel_body_z = 0.0f;

// velocities in the NED frame [m/s]
float vel_N = 0.0f;
float vel_E = 0.0f;
float vel_D = 0.0f;

// local position in NED frame [m] from center of lake lag
float position_N = 0.0f;
float position_E = 0.0f;
float position_D_gps = 0.0f; 		// note: this is down, so altitude above ground is a negative value
float position_D_baro = 0.0f; 		// note: this is down, so altitude above ground is a negative value

// ground course and speed
float ground_speed = 0.0f;		// speed relative to ground in [m/s]
float ground_course = 0.0f; 	// this is course over ground (direction of velocity relative to North in [rad])

// airspeed [m/s]
//float air_speed = 0.0f;		// speed relative to air in [m/s] (measured by pitot tube)

// status check
bool gps_ok = false; 				// boolean as to whether or not the gps data coming in is valid
bool local_pos_ne_valid = false;	// boolean as to whether or not the horizontal local position values are valid
bool local_pos_d_valid = false;		// boolean as to whether or not the vertical local position value is valid


// battery info
float battery_voltage = 0.0f;			// current voltage across the battery 	[V]
float battery_current = 0.0f;			// current current being consumed		[A]

// RC info
// output for each of the controls ranging from -1..1 except for throttle which ranges from 0..1
float roll_servo_out = 0.0f;
float pitch_servo_out = 0.0f;
float yaw_servo_out = 0.0f;
float throttle_servo_out = 0.0f;

// manual control inputs
// input for each of the controls from the remote control, ranging from -1..1 except for throttle, which ranges from 0..1
float man_roll_in = 0.0f;
float man_pitch_in = 0.0f;
float man_yaw_in = 0.0f;
float man_throttle_in = 0.0f;

// additional inputs from remote control that you can use for custom mode switching, etc
// these will only have values if you included them in the radio calibration
float man_flaps_in = 0.0f;
float man_aux1_in = 0.0f;
float man_aux2_in = 0.0f;

// trim conditions (from remote control)
float roll_trim = 0.0f;
float pitch_trim = 0.0f;
float yaw_trim = 0.0f;

// time information
uint64_t timestamp = 0; 				// timestamp of microseconds since boot (set at the beginning of loop)
uint64_t utc_timestamp = 0; 			// GPS UTC timestamp in microseconds
uint64_t previous_loop_timestamp = 0;	// timestamp of start of previous loop
// float mission_time;						// the mission time in minutes
// mission stuff

bool in_mission;
hrt_abstime start_time;
float current_time;
float final_time;
bool mission_failed;
bool in_turn;
int32_t turn_num;
float turn_degrees;
int32_t num_violations;
bool in_violation;
bool out_of_bounds;

// data from threads
struct aa241x_low_data_s low_data = {};
struct aa241x_high_data_s high_data = {};


// user config parameters structure
struct aal_params aal_parameters = {};		// struct containing all of the user editable parameters (via ground station)

struct mis_params mission_parameters = {};


/* functions */

// TODO: DEFINE FUNCTIONS HERE

/* EXAMPLE
void take_picture()
{
	aa241x_picture_request_s pic_request;
	pic_request.time_us = hrt_absolute_time();
	pic_request.pos_N = position_N;
	pic_request.pos_E = position_E;
	pic_request.pos_D = position_D_gps;

	// publish the picture request 
	if (_picture_request_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_picture_request), _picture_request_pub, &pic_request);
	} else {
		_picture_request_pub = orb_advertise(ORB_ID(aa241x_picture_request), &pic_request);
	}
}
*/

}