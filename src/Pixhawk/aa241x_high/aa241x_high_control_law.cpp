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
 * @file aa241x_fw_control.cpp
 *
 * Secondary file to the fixedwing controller containing the
 * control law for the AA241x class.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 *  @author YOUR NAME			<YOU@EMAIL.COM>
 */

#include <uORB/uORB.h>
#include <math.h>

// include header file
#include "aa241x_high_control_law.h"
#include "aa241x_high_aux.h"

// needed for variable names
using namespace aa241x_high;

// define trim conditions
#define aileron_trimmed 0.0f;
#define rudder_trimmed 0.0f;
#define throttle_trimmed 0.77f;


//Prototype functions
static void setDesiredAltitude(float start_E, float end_E, float start_N, float end_N);

//Module level variables
static float altitudeSetpoint = aah_parameters.altitude_desired; //Initialize it as the altitude setting in qgroundcontrol

// desired attitude rates
// Note the use of x.0f, this is important to specify that these are single and not double float values!
float rollrate_desired = 0.0f;
float pitchrate_desired = 0.0f;

// initialize control outputs
float RollCorrection;
float PitchCorrection;
float ThrottleCorrection;
float elevator_trimmed;

void set_elevator_trimmed(float speed, float bank);

/**
 * Main function in which your code should be written.
 *
 * This is the only function that is executed at a set interval,
 * feel free to add all the function you'd like, but make sure all
 * the code you'd like executed on a loop is in this function.
 */
void flight_control() {

	static float way_E[15] = {-125.0f, 138.1557f, 111.8443f, 138.1557f, 111.8443f, 138.1557f,137.12436f, -80.91666f, -69.08334f, -80.91666f, -69.08334f, -80.91666f,-89.0f, -88.0f,-56.0f};
	static float way_N[15] = {158.0f, 51.78828f, 42.21172f, 51.78828f, 42.21172f, 51.78828f, 40.0f, -105.68831f, -80.31169f, -105.68831f, -80.31169f, -105.68831f, -93.0f, 185.0f,170.0f};

	
	if (hrt_absolute_time() - previous_loop_timestamp > 500000.0f) { 
		pitch_desired = 0.05f; //pitch
		roll_desired = 0.0f;
		throttle_desired = throttle_trimmed;
		aah_parameters.speed_desired_u = speed_body_u;
		high_data.time_start = hrt_absolute_time();
	}


	high_data.min_alt = mission_parameters.min_alt;
	high_data.max_alt = mission_parameters.max_alt;
	/* Longitudinal control */

	set_elevator_trimmed(ground_speed,roll);

	float ind = low_data.ind;
	if ((abs(ind-12) < 0.1) || (abs(ind-13) < 0.1)) {
		setDesiredAltitude(way_E[12],way_E[13],way_N[12],way_N[13]);
	} else {
		altitudeSetpoint = aah_parameters.altitude_desired;
	}

    // multiply gps altitude by -1: height scale flipped
	float altitude_setpoint = altitudeSetpoint;
    if ((-1.0f*position_D_gps) <= (altitude_setpoint - aah_parameters.altitude_bound)){

        pitch_desired = aah_parameters.proportional_altitude_gain * ((altitude_setpoint- aah_parameters.altitude_bound) - (-1.0f*position_D_gps)) + aah_parameters.pitch_trimmed;

	if (pitch_desired>(30.0f/180.0f*3.1415926f)){
		pitch_desired = 30.0f/180.0f*3.1415926f;
	}
	if (pitch_desired<(-20.0f/180.0f*3.1415926f)){
		pitch_desired = -20.0f/180.0f*3.1415926f;
	}
        PitchCorrection = aah_parameters.proportional_pitch_gain * (pitch_desired - pitch) + elevator_trimmed;

    }
    else if ((-1.0f*position_D_gps) >= (altitude_setpoint + aah_parameters.altitude_bound)) {

        pitch_desired = aah_parameters.proportional_altitude_gain * ((altitude_setpoint + aah_parameters.altitude_bound) - (-1.0f*position_D_gps)) + aah_parameters.pitch_trimmed;

	if (pitch_desired>(30.0f/180.0f*3.1415926f)){
		pitch_desired = 30.0f/180.0f*3.1415926f;
	}
	if (pitch_desired<(-20.0f/180.0f*3.1415926f)){
		pitch_desired = -20.0f/180.0f*3.1415926f;
	}
	
        PitchCorrection = aah_parameters.proportional_pitch_gain * (pitch_desired - pitch) + elevator_trimmed;

    }
    else {

        pitch_desired = aah_parameters.pitch_trimmed;
        PitchCorrection = aah_parameters.proportional_pitch_gain * (pitch_desired - pitch) +
                aah_parameters.derivative_pitch_gain * (pitchrate_desired - pitch_rate) + elevator_trimmed;

    }
	high_data.alt_des = altitude_setpoint;

    /* Lateral control */
    // maintain roll for banked turn
    
    roll_desired = aah_parameters.roll_des/180.0f*PI;
	if (low_data.phi_c < 90.0f/180.0f*3.1415926f){
		if (low_data.phi_c > -90.0f/180.0f*3.1415926f){
			roll_desired = low_data.phi_c;
		}
	}
    RollCorrection = aah_parameters.proportional_roll_gain * (roll_desired - roll) +
            aah_parameters.derivative_roll_gain * (rollrate_desired - roll_rate) + aileron_trimmed;



    // Do bounds checking to keep the correction within the -1..1 limits of the servo output
    if (RollCorrection > 1.0f) {
        RollCorrection = 1.0f;
    } else if (RollCorrection < -1.0f ) {
        RollCorrection = -1.0f;
	}
    if (PitchCorrection > 1.0f) {
        PitchCorrection = 1.0f;
    } else if (PitchCorrection < -1.0f ) {
        PitchCorrection = -1.0f;
    }
    if (ThrottleCorrection > 1.0f) {
        ThrottleCorrection = 1.0f;
    } else if (ThrottleCorrection < 0.0f ) {
        ThrottleCorrection = 0.0f;
    }

    // Set output of servos: manual or mission mode
    if (aah_parameters.roll_mode > 0.5f)
    {
        roll_servo_out = man_roll_in;
    } else {
        roll_servo_out = RollCorrection;
    }

    if (aah_parameters.pitch_mode > 0.5f)
    {
        pitch_servo_out = -man_pitch_in;
    } else {
        pitch_servo_out = -PitchCorrection;
    }

	if (hrt_absolute_time() - high_data.time_start > 5000000.0f)
	{
		throttle_servo_out = 1.0f;
	} else {
		throttle_servo_out = man_throttle_in;
	}
    //if (aah_parameters.throttle_mode > 0.5f)
    //{
    //    throttle_servo_out = man_throttle_in;
    //} else {
    //    throttle_servo_out = man_throttle_in;
    //}
}

void set_elevator_trimmed(float speed, float bank){
	elevator_trimmed = 0.1f + 0.45f*abs(bank)/(60.0f/180.0f*3.1415926f);
}

//Returns the projected distance on the last stretch, index is length of array - 2
static void setDesiredAltitude(float start_E, float end_E, float start_N, float end_N) {
    //Convert the last stretch to local coordinates
    float localEnd_E = end_E - start_E;
    float localEnd_N = end_N - start_N;
    float lastStretchLength = sqrt(localEnd_E*localEnd_E+localEnd_N*localEnd_N);
    //Get projected distance
    float distanceAlongLastStretch = ((position_E-start_E)*localEnd_E+(position_N-start_N)*localEnd_N)/lastStretchLength;
    //Calculate the altitude setpoint using a linear interpolation
    altitudeSetpoint = aah_parameters.altitude_desired+(-6)*distanceAlongLastStretch/lastStretchLength;
}

