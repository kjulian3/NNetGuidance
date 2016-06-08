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
 * @file aa241x_fw_control_params.c
 *
 * Definition of custom parameters for fixedwing controllers
 * being written for AA241x.
 *
 *  @author Adrien Perkins		<adrienp@stanford.edu>
 */

#include "aa241x_high_params.h"

/*
 *  controller parameters, use max. 15 characters for param name!
 *
 */

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_EXAMPLE and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 10.0.
 *
 * @unit meter 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
	PARAM_DEFINE_FLOAT(AAH_EXAMPLE, 10.0f);

/**
 * This is an example parameter.  The name of the parameter in QGroundControl
 * will be AAH_PROPROLLGAIN and will be in the AAH dropdown.  Make sure to always
 * start your parameters with AAH to have them all in one place.
 *
 * The default value of this float parameter will be 1.0.
 *
 * @unit none 						(the unit attribute (not required, just helps for sanity))
 * @group AA241x High Params		(always include this)
 */
	PARAM_DEFINE_FLOAT(AAH_ROLL_PGAIN, 1.3f);
	PARAM_DEFINE_FLOAT(AAH_ROLL_DGAIN, 0.1f);
	PARAM_DEFINE_FLOAT(AAH_ROLL_DES, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_PITCH_PGAIN, 1.0f);
	PARAM_DEFINE_FLOAT(AAH_PITCH_DGAIN, 0.1f);
	PARAM_DEFINE_FLOAT(AAH_PITCH_DES, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_PITCH_TRIM, 0.05f);
	PARAM_DEFINE_FLOAT(AAH_YAW_PGAIN, 0.3f);
	PARAM_DEFINE_FLOAT(AAH_YAW_DGAIN, 1.0f);

	PARAM_DEFINE_FLOAT(AAH_ALT_PGAIN, 0.1f);
	PARAM_DEFINE_FLOAT(AAH_ALT_DGAIN, 1.0f);
	PARAM_DEFINE_FLOAT(AAH_ALT_BOUND, 0.3f);
	PARAM_DEFINE_FLOAT(AAH_ALT_DES, 53.0f);
	PARAM_DEFINE_FLOAT(AAH_LINE_PGAIN, 0.1f);
	PARAM_DEFINE_FLOAT(AAH_Y_TEST, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_THRT_PGAIN, 0.05f);
	PARAM_DEFINE_FLOAT(AAH_SPEEDU_DES, 14.0f);

	PARAM_DEFINE_FLOAT(AAH_ROLL_MODE, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_YAW_MODE, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_PITCH_MODE, 0.0f);
	PARAM_DEFINE_FLOAT(AAH_THRT_MODE, 0.0f); // throttle is dangerous, start with manual

	PARAM_DEFINE_FLOAT(AAH_MODE, 1.0f);

// TODO: define custom parameters here


int aah_parameters_init(struct aah_param_handles *h)
{

	/* for each of your custom parameters, make sure to define a corresponding
	 * variable in the aa_param_handles struct and the aa_params struct these
	 * structs can be found in the aa241x_fw_control_params.h file
	 *
	 * NOTE: the string passed to param_find is the same as the name provided
	 * in the above PARAM_DEFINE_FLOAT
	 */
	h->example_high_param		= param_find("AAH_EXAMPLE");
	h->proportional_roll_gain 	= param_find("AAH_ROLL_PGAIN");
	h->derivative_roll_gain		= param_find("AAH_ROLL_DGAIN");
	h->roll_des			= param_find("AAH_ROLL_DES");
	h->proportional_pitch_gain	= param_find("AAH_PITCH_PGAIN");
	h->derivative_pitch_gain	= param_find("AAH_PITCH_DGAIN");	
	h->pitch_des			= param_find("AAH_PITCH_DES");
	h->pitch_trimmed                = param_find("AAH_PITCH_TRIM");
	h->proportional_yaw_gain	= param_find("AAH_YAW_PGAIN");
	h->derivative_yaw_gain		= param_find("AAH_YAW_DGAIN");

	h->proportional_altitude_gain	= param_find("AAH_ALT_PGAIN");
	h->derivative_altitude_gain	= param_find("AAH_ALT_DGAIN");
	h->altitude_bound	        = param_find("AAH_ALT_BOUND");
	h->altitude_desired		= param_find("AAH_ALT_DES");
	h->proportional_line_gain	= param_find("AAH_LINE_PGAIN");
	h->y_test			= param_find("AAH_Y_TEST");
	h->proportional_throttle_gain	= param_find("AAH_THRT_PGAIN");
	h->speed_desired_u		= param_find("AAH_SPEEDU_DES");

	h->roll_mode			= param_find("AAH_ROLL_MODE");
	h->yaw_mode			= param_find("AAH_YAW_MODE");
	h->pitch_mode			= param_find("AAH_PITCH_MODE");
	h->throttle_mode		= param_find("AAH_THRT_MODE");

	h->mission_mode			= param_find("AAH_MODE");

	// TODO: add the above line for each of your custom parameters........

	return OK;
}

int aah_parameters_update(const struct aah_param_handles *h, struct aah_params *p)
{

	// for each of your custom parameters, make sure to add this line with
	// the corresponding variable name
	param_get(h->example_high_param, &(p->example_high_param));
	param_get(h->proportional_roll_gain, &(p->proportional_roll_gain));
	param_get(h->derivative_roll_gain, &(p->derivative_roll_gain));
	param_get(h->roll_des, &(p->roll_des));
	param_get(h->proportional_pitch_gain, &(p->proportional_pitch_gain));
	param_get(h->derivative_pitch_gain, &(p->derivative_pitch_gain));
	param_get(h->pitch_des, &(p->pitch_des));
	param_get(h->pitch_trimmed, &(p->pitch_trimmed));
	param_get(h->proportional_yaw_gain, &(p->proportional_yaw_gain));
	param_get(h->derivative_yaw_gain, &(p->derivative_yaw_gain));

	param_get(h->proportional_altitude_gain, &(p->proportional_altitude_gain));
	param_get(h->derivative_altitude_gain, &(p->derivative_altitude_gain));
	param_get(h->altitude_bound, &(p->altitude_bound));
	param_get(h->altitude_desired, &(p->altitude_desired));
	param_get(h->proportional_line_gain, &(p->proportional_line_gain));
	param_get(h->y_test, &(p->y_test));
	param_get(h->proportional_throttle_gain, &(p->proportional_throttle_gain));
	param_get(h->speed_desired_u, &(p->speed_desired_u));

	param_get(h->roll_mode, &(p->roll_mode));
	param_get(h->yaw_mode, &(p->yaw_mode));
	param_get(h->pitch_mode, &(p->pitch_mode));
	param_get(h->throttle_mode, &(p->throttle_mode));
	
	param_get(h->mission_mode, &(p->mission_mode));

	// TODO: add the above line for each of your custom parameters.....

	return OK;
}
