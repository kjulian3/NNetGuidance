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

/*
 * @file aa241x_fw_aux.h
 *
 * Auxiliary header file containing all the variables accessible for
 * the control law in the aa241x_fw_control.cpp file.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */
#pragma once

#ifndef AA241X_FW_AUX_H_
#define AA241X_FW_AUX_H_

#include <stdint.h>
#include <stdlib.h>
#include <vector>
#include <uORB/uORB.h>
#include <uORB/topics/aa241x_low_data.h>
#include <uORB/topics/aa241x_high_data.h>

#include "aa241x_high_params.h"

// wrap everything into a namespace for allowing for common variable names
namespace aa241x_high {

// set these variables for help in debugging (these will be sent to the ground station)
extern float roll_desired;
extern float pitch_desired;
extern float yaw_desired;
extern float throttle_desired;


// Tait-Bryan Euler angles in radian
extern float roll;		// -pi .. pi
extern float pitch;		// -pi/2 .. pi/2
extern float yaw;		// -pi .. pi

// angular rates, Tait-Bryan NED [rad/s]
extern float roll_rate;
extern float pitch_rate;
extern float yaw_rate;

// body velocities [m/s]
extern float speed_body_u;
extern float speed_body_v;
extern float speed_body_w;

// body accelerations [m/s^2]
extern float accel_body_x;
extern float accel_body_y;
extern float accel_body_z;

// velocities in the NED frame [m/s]
extern float vel_N;
extern float vel_E;
extern float vel_D;

// local position in NED frame [m] from center of lake lag
extern float position_N;
extern float position_E;
extern float position_D_gps; 		// note: this is down, so altitude above ground is a negative value
extern float position_D_baro; 		// note: this is down, so altitude above ground is a negative value

// ground course and speed
extern float ground_speed;		// speed relative to ground in [m/s]
extern float ground_course; 	// this is course over ground (direction of velocity relative to North in [rad] from -PI .. PI)

// airspeed [m/s]
//extern float air_speed;		// speed relative to air in [m/s] (measured by pitot tube)

// status check
extern bool gps_ok; 			// boolean as to whether or not the gps data coming in is valid
extern bool local_pos_ne_valid;	// boolean as to whether or not the horizontal local position values are valid
extern bool local_pos_d_valid;	// boolean as to whether or not the vertical local position value is valid

// battery info
extern float battery_voltage;			// current voltage across the battery 	[V]
extern float battery_current;			// current current being consumed		[A]

// RC info
// output for each of the controls ranging from -1..1 except for throttle which ranges from 0..1
extern float roll_servo_out;
extern float pitch_servo_out;
extern float yaw_servo_out;
extern float throttle_servo_out;
extern float flaps_servo_out;
extern float aux1_servo_out;

// manual control inputs
// input for each of the controls from the remote control, ranging from -1..1 except for throttle, which ranges from 0..1
extern float man_roll_in;
extern float man_pitch_in;
extern float man_yaw_in;
extern float man_throttle_in;

// additional inputs from remote control that you can use for custom mode switching, etc
// these will only have values if you included them in the radio calibration
extern float man_flaps_in;
extern float man_aux1_in;
extern float man_aux2_in;

// trim conditions (from remote control)
extern float roll_trim;
extern float pitch_trim;
extern float yaw_trim;

// time information
extern uint64_t timestamp; 					// unix timestamp in microseconds of the beginning of the current loop
extern uint64_t utc_timestamp; 				// GPS UTC timestamp in microseconds
extern uint64_t previous_loop_timestamp; 	// timestamp of start of previous loop
// extern float mission_time;					// the mission time in minutes

// mission stuff
extern bool in_mission;
extern hrt_abstime start_time;
extern float current_time;
extern float final_time;
extern bool mission_failed;
extern bool in_turn;
extern int32_t turn_num;
extern float turn_degrees;
extern int32_t num_violations;
extern bool in_violation;
extern bool out_of_bounds;


// communication data
extern aa241x_low_data_s low_data;
extern aa241x_high_data_s high_data;

// TODO: DECLARE ADDITION VARIABLES TO EXPOSE TO STUDENTS HERE

// user config parameters structure
extern struct aah_params aah_parameters;		// struct containing parameters for high priority loop

// mission definition parameters
struct mis_params {
    float min_alt;
    float max_alt;
    float start_pos_N;
    float start_pos_E;
    float keepout_radius;
    float tilt;
    float ctr_lat;
    float ctr_lon;
    float ctr_alt;
    float leg_length;
    float gate_width;
    int team_num;

};
extern struct mis_params mission_parameters;			/**< local copies of mission parameters */


/* functions */

// DECLARE CUSTOM FUNCTIONS TO EXPOSE TO STUDENTS HERE
// (see example below)

/**
 * Function to call in order to take a picture from the current location of the UAV.
 *
 * When a picture result has been received, the new_pic boolean will be set to true,
 * and the new data will be able to be found in the pic_result struct.
 *
 * Note: new_pic boolean will only be set to true when a successful picture has been taken
 * so if you spam the take picture function, do not expect new_pic to trigger after every time
 * you call take_picture(), there is a rate limit for picture taking.
 */
//void	take_picture();


}

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control();




#endif /* AA241X_FW_AUX_H_ */
