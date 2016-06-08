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
 * @file aa241x_fw_control_main.cpp
 *
 * Wrapper for low priority (slow) control law.
 *
 * @author Adrien Perkins	<adrienp@stanford.edu>
 *
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include "nnet.cpp"

#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/aa241x_mission_status.h>
#include <uORB/topics/aa241x_low_data.h>
#include <uORB/topics/aa241x_high_data.h>
#include <uORB/topics/aa241x_local_data.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/battery_status.h>

#include <systemlib/err.h>
#include <systemlib/systemlib.h>
#include <systemlib/perf_counter.h>
#include <systemlib/param/param.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

#include "aa241x_low_params.h"
#include "aa241x_low_aux.h"

using namespace aa241x_low;

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int aa241x_low_main(int argc, char *argv[]);

class LowPriorityLoop
{
public:
	/**
	 * Constructor
	 */
	LowPriorityLoop();

	/**
	 * Destructor, also kills the main task.
	 */
	~LowPriorityLoop();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool	_task_should_exit;		/**< if true, attitude control task should exit */
	bool	_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

	int		_loop_counter;			/**< count of iterations of main loop */

	// handles to the subscriptions needed
	int		_att_sub;				/**< vehicle attitude subscription */
	int		_accel_sub;				/**< accelerometer subscription */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_airspeed_sub;			/**< airspeed subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 	_params_sub;			/**< notification of parameter updates */
	int 	_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_local_pos_sub;			/**< local position subscription */
	int		_vehicle_status_sub;	/**< vehicle status subscription */
	int		_sensor_combined_sub;	/**< sensor data subscription */
	int		_battery_status_sub;	/**< battery status subscription */

	int		_mission_status_sub;	/**< aa241x mission status subscription */
	int		_high_data_sub;			/**< high priority loop data subscription */

	// TODO: ADD ADDITION SUBSCRIBERS HERE

	// the data that will be published from this controller
	orb_advert_t	_low_data_pub;			/**< data fields to be shared with high priority module */


	// structures of data that comes in from the uORB subscriptions
	struct vehicle_attitude_s			_att;				/**< vehicle attitude */
	struct accel_report					_accel;				/**< body frame accelerations */
	struct manual_control_setpoint_s	_manual;			/**< r/c channel data */
	struct airspeed_s					_airspeed;			/**< airspeed */
	struct vehicle_control_mode_s		_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s	_global_pos;		/**< global position */
	struct vehicle_local_position_s		_local_pos;			/**< local position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct sensor_combined_s			_sensor_combined;	/**< raw / minimal filtered sensor data (for some accelerations) */
	struct battery_status_s				_battery_status;	/**< battery status */

	struct aa241x_mission_status_s		_mis_status;		/**< current mission status */
	// low data struct is in attached aux header file

	// TODO: ADD CUSTOM STRUCTS USED HERE (e.g. if having additional topics)

	// some flags
	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;					/**< if set to true, print debug output */

	map_projection_reference_s		_lake_lag_proj_ref;		/**< projection reference given by the center of the lake */

	float		_initial_baro_offset;	/**< the initial baro alt difference from adjusted gps in [m] */
	bool		_initial_offset_valid;	/**< boolean to flag whether or not the above offset is valid */

	// general RC parameters
	struct {
		float trim_roll;
		float trim_pitch;
		float trim_yaw;
	}		_parameters;			/**< local copies of interesting parameters */

	// handles for general RC parameters
	struct {
		/* rc parameters */
		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;

		/* mission parameters */
		param_t min_alt;
		param_t max_alt;
		param_t start_pos_N;
		param_t start_pos_E;
		param_t keepout_radius;
		param_t tilt;
		param_t ctr_lat;
		param_t ctr_lon;
		param_t ctr_alt;
		param_t leg_length;
		param_t gate_width;
		param_t team_num;
	}		_parameter_handles;		/**< handles for interesting parameters */


	// handles for custom parameters
	// NOTE: the struct for the parameters themselves can be found in the aa241x_fw_aux file
	struct aal_param_handles		_aal_parameter_handles;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void	control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void	vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void	vehicle_manual_poll();


	/**
	 * Check for airspeed updates.
	 */
	void	vehicle_airspeed_poll();

	/**
	 * Check for accel updates.
	 */
	void	vehicle_accel_poll();

	/**
	 * Check for global position updates.
	 */
	void	global_pos_poll();

	/**
	 * Check for local position updates.
	 */
	void	local_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void	vehicle_status_poll();

	/**
	 * Check for combined sensor data updates.
	 */
	void	sensor_combined_poll();

	/**
	 * Check for battery status updates.
	 */
	void	battery_status_poll();

	// TODO: DEFINE ADDITIONAL POLL FUNCTIONS HERE

	/**
	 * Check for a mission status update.
	 */
	void	mission_status_poll();

	/**
	 * Check for an update of the low priority loop data.
	 */
	void	high_data_poll();

	/**
	 * Publish the data fields from this module (high priority thread).
	 */
	void	publish_low_data();

	/**
	 * Set all the aux variables needed for control law.
	 */
	void 	set_aux_values();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void	task_main();

};

/* define namespace to hold the controller */
namespace low_control
{

// oddly, ERROR is not defined for c++
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

LowPriorityLoop	*g_control = nullptr;
}


LowPriorityLoop::LowPriorityLoop() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

/* subscriptions */
	_att_sub(-1),
	_accel_sub(-1),
	_airspeed_sub(-1),
	_vcontrol_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_global_pos_sub(-1),
	_local_pos_sub(-1),
	_vehicle_status_sub(-1),
	_sensor_combined_sub(-1),
	_battery_status_sub(-1),
	_mission_status_sub(-1),
	_high_data_sub(-1),

/* publications */
	_low_data_pub(nullptr),

/* states */
	_setpoint_valid(false),
	_debug(false),
	_initial_baro_offset(NAN),
	_initial_offset_valid(false)
{
	/* safely initialize structs */
	_att = {};
	_accel = {};
	_manual = {};
	_airspeed = {};
	_vcontrol_mode = {};
	_global_pos = {};
	_local_pos = {};
	_vehicle_status = {};
	_sensor_combined = {};
	_battery_status = {};

	_mis_status = {};

	_lake_lag_proj_ref = {};

	// initialize the global remote parameters
	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.min_alt = param_find("AA_ALT_MIN");
	_parameter_handles.max_alt = param_find("AA_ALT_MAX");
	_parameter_handles.start_pos_N = param_find("AAMIS_SPOS_N");
	_parameter_handles.start_pos_E = param_find("AAMIS_SPOS_E");
	_parameter_handles.keepout_radius = param_find("AAMIS_RAD_KPT");
	_parameter_handles.tilt = param_find("AAMIS_TILT");
	_parameter_handles.leg_length = param_find("AAMIS_LEG_LEN");
	_parameter_handles.gate_width = param_find("AAMIS_GTE_WID");
	_parameter_handles.ctr_lat = param_find("AAMIS_CTR_LAT");
	_parameter_handles.ctr_lon = param_find("AAMIS_CTR_LON");
	_parameter_handles.ctr_alt = param_find("AAMIS_CTR_ALT");
	_parameter_handles.team_num = param_find("AA_TEAM");

	// initialize the aa241x control parameters
	aal_parameters_init(&_aal_parameter_handles);


	// fetch initial remote parameters
	parameters_update();

}

LowPriorityLoop::~LowPriorityLoop()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	low_control::g_control = nullptr;
}

int
LowPriorityLoop::parameters_update()
{

	// update the remote control parameters
	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));

	// update the mission parameters
	param_get(_parameter_handles.min_alt, &(mission_parameters.min_alt));
	param_get(_parameter_handles.max_alt, &(mission_parameters.max_alt));
	param_get(_parameter_handles.start_pos_N, &(mission_parameters.start_pos_N));
	param_get(_parameter_handles.start_pos_E, &(mission_parameters.start_pos_E));
	param_get(_parameter_handles.keepout_radius, &(mission_parameters.keepout_radius));
	param_get(_parameter_handles.tilt, &(mission_parameters.tilt));
	param_get(_parameter_handles.gate_width, &(mission_parameters.gate_width));
	param_get(_parameter_handles.leg_length, &(mission_parameters.leg_length));
	param_get(_parameter_handles.ctr_lat, &(mission_parameters.ctr_lat));
	param_get(_parameter_handles.ctr_lon, &(mission_parameters.ctr_lon));
	param_get(_parameter_handles.ctr_alt, &(mission_parameters.ctr_alt));
	param_get(_parameter_handles.team_num, &(mission_parameters.team_num));

	// update the aa241x control parameters
	aal_parameters_update(&_aal_parameter_handles, &aal_parameters);

	return OK;
}

void
LowPriorityLoop::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
LowPriorityLoop::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
LowPriorityLoop::vehicle_airspeed_poll()
{
	/* check if there is a new position */
	bool airspeed_updated;
	orb_check(_airspeed_sub, &airspeed_updated);

	if (airspeed_updated) {
		orb_copy(ORB_ID(airspeed), _airspeed_sub, &_airspeed);
	}
}

void
LowPriorityLoop::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
LowPriorityLoop::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
LowPriorityLoop::local_pos_poll()
{
	/* check if there is a new local position */
	bool local_pos_updated;
	orb_check(_local_pos_sub, &local_pos_updated);

	if (local_pos_updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
	}
}

void
LowPriorityLoop::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);
	}
}

void
LowPriorityLoop::sensor_combined_poll()
{
	/* check if there is new sensor combined data */
	bool sensor_combined_updated;
	orb_check(_sensor_combined_sub, &sensor_combined_updated);

	if (sensor_combined_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}


void
LowPriorityLoop::battery_status_poll()
{
	/* check if there is a new battery status */
	bool battery_status_updated;
	orb_check(_battery_status_sub, &battery_status_updated);

	if (battery_status_updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}


void
LowPriorityLoop::mission_status_poll()
{
	/* check if there is a new mission status */
	bool mission_status_updated;
	orb_check(_mission_status_sub, &mission_status_updated);

	if (mission_status_updated) {
		orb_copy(ORB_ID(aa241x_mission_status), _mission_status_sub, &_mis_status);
	}
}

void
LowPriorityLoop::high_data_poll()
{
	/* check if there is a new high priority loop data */
	bool high_data_updated;
	orb_check(_high_data_sub, &high_data_updated);

	if (high_data_updated) {
		orb_copy(ORB_ID(aa241x_high_data), _high_data_sub, &high_data);
	}
}

void
LowPriorityLoop::publish_low_data()
{
	/* publish the low priority loop data */
	if (_low_data_pub != nullptr) {
		orb_publish(ORB_ID(aa241x_low_data), _low_data_pub, &low_data);
	} else {
		_low_data_pub = orb_advertise(ORB_ID(aa241x_low_data), &low_data);
	}
}


void
LowPriorityLoop::set_aux_values()
{

	// set the euler angles and rates
	roll = _att.roll;
	pitch = _att.pitch;
	yaw = _att.yaw;

	// set the angular rates
	roll_rate = _att.rollspeed;
	pitch_rate = _att.pitchspeed;
	yaw_rate = _att.yawspeed;


	// calculate the body velocities
	speed_body_u = 0.0f;		// set them to 0 just in case unable to do calculation
	speed_body_v = 0.0f;
	speed_body_w = 0.0f;
	if(_att.R_valid) 	{
		speed_body_u = PX4_R(_att.R, 0, 0) * _global_pos.vel_n + PX4_R(_att.R, 1, 0) * _global_pos.vel_e + PX4_R(_att.R, 2, 0) * _global_pos.vel_d;
		speed_body_v = PX4_R(_att.R, 0, 1) * _global_pos.vel_n + PX4_R(_att.R, 1, 1) * _global_pos.vel_e + PX4_R(_att.R, 2, 1) * _global_pos.vel_d;
		speed_body_w = PX4_R(_att.R, 0, 2) * _global_pos.vel_n + PX4_R(_att.R, 1, 2) * _global_pos.vel_e + PX4_R(_att.R, 2, 2) * _global_pos.vel_d;
	} else	{
		if (_debug && _loop_counter % 10 == 0) {
			warnx("Did not get a valid R\n");
		}
	}

	// body accelerations [m/s^2]
	accel_body_x = _sensor_combined.accelerometer_m_s2[0];
	accel_body_y = _sensor_combined.accelerometer_m_s2[1];
	accel_body_z = _sensor_combined.accelerometer_m_s2[2];

	// velocities in the NED frame [m/s]
	// TODO: maybe use local position...
	vel_N = _global_pos.vel_n;
	vel_E = _global_pos.vel_e;
	vel_D = _global_pos.vel_d;

	// local position in NED frame [m] from center of lake lag
	position_N = 0.0f;
	position_E = 0.0f;
	position_D_baro = 0.0f;
	position_D_gps = -_global_pos.alt + mission_parameters.ctr_alt;
	map_projection_project(&_lake_lag_proj_ref, _global_pos.lat, _global_pos.lon, &position_N, &position_E);
	if (_local_pos.z_valid) {
		position_D_baro = _local_pos.z;

		if (_initial_offset_valid) {
			position_D_baro += _initial_baro_offset;
		}
	}
	local_pos_ne_valid = _local_pos.xy_valid;
	local_pos_d_valid = _local_pos.z_valid;

	if (gps_ok && !_initial_offset_valid && local_pos_d_valid) {
		_initial_baro_offset = position_D_baro - position_D_gps;
		_initial_offset_valid = true;
	}

	// ground course and speed
	// TODO: maybe use local position....
	ground_speed = sqrtf(_global_pos.vel_n * _global_pos.vel_n + _global_pos.vel_e * _global_pos.vel_e);		// speed relative to ground in [m/s]
	ground_course = _global_pos.yaw; 	// this is course over ground (direction of velocity relative to North in [rad])

	// airspeed [m/s]
//	air_speed = _airspeed.true_airspeed_m_s;	// speed relative to air in [m/s] (measured by pitot tube)

	// status check
	gps_ok = _vehicle_status.gps_failure; 		// boolean as to whether or not the gps data coming in is valid

	// battery info
	battery_voltage = _battery_status.voltage_filtered_v;
	battery_current = _battery_status.current_a;

	// manual control inputs
	// input for each of the controls from the remote control
	man_roll_in = _manual.y;
	man_pitch_in = _manual.x;
	man_yaw_in = _manual.r;
	man_throttle_in = _manual.z;

	man_flaps_in = _manual.flaps;
	man_aux1_in = _manual.aux1;
	man_aux2_in = _manual.aux2;

	// trim conditions (from remote control)
	roll_trim = _parameters.trim_roll;
	pitch_trim = _parameters.trim_pitch;
	yaw_trim = _parameters.trim_yaw;

	// time information
	timestamp = hrt_absolute_time();
	utc_timestamp = _global_pos.time_utc_usec;

	// mission time
	// mission_time = _mis_status.mission_time;

	// mission stuff

	in_mission = _mis_status.in_mission;
	start_time = _mis_status.start_time;
	current_time = _mis_status.current_time;
	final_time = _mis_status.final_time;
	mission_failed = _mis_status.mission_failed;
	in_turn = _mis_status.in_turn;
	turn_num = _mis_status.turn_num;
	turn_degrees = _mis_status.turn_degrees;
	num_violations = _mis_status.num_violations;
	in_violation = _mis_status.in_violation;
	out_of_bounds = _mis_status.out_of_bounds;

}

void
LowPriorityLoop::task_main_trampoline(int argc, char *argv[])
{
	low_control::g_control->task_main();
}

void
LowPriorityLoop::task_main()
{

	/* inform about start */
	warnx("Initializing..");
	fflush(stdout);

	/*
	 * do subscriptions
	 */
	_att_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));

	_mission_status_sub = orb_subscribe(ORB_ID(aa241x_mission_status));
	_high_data_sub = orb_subscribe(ORB_ID(aa241x_high_data));

	/* rate limit vehicle status updates to 50Hz */
	orb_set_interval(_vcontrol_mode_sub, 20);
	/* rate limit attitude control to 50 Hz */
	orb_set_interval(_att_sub, 20);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_airspeed_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	global_pos_poll();
	local_pos_poll();
	vehicle_status_poll();
	sensor_combined_poll();
	battery_status_poll();

	// 241x stuff
	mission_status_poll();
	high_data_poll();

	/* initialize projection reference */
	map_projection_init(&_lake_lag_proj_ref, (double) mission_parameters.ctr_lat, (double) mission_parameters.ctr_lon);


	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2] = {};

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _att_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	//bool wasAuto = false;
	void* nnet = load_network(PX4_ROOTFSDIR"/fs/microsd/Race72.nnet");
	while (!_task_should_exit) {

		_loop_counter = 0;

		/* wait for up to 200ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 200);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0)
			continue;

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run loop if attitude changed */
		if (fds[1].revents & POLLIN) {


			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();

			/* guard against too large deltaT's */
			if (deltaT > 1.0f)
				deltaT = 0.01f;

			/* load local copies */
			orb_copy(ORB_ID(vehicle_attitude), _att_sub, &_att);


			// update vehicle information structs as needed
			vehicle_airspeed_poll();
			vehicle_accel_poll();
			vehicle_control_mode_poll();
			vehicle_manual_poll();
			global_pos_poll();
			local_pos_poll();
			vehicle_status_poll();
			sensor_combined_poll();
			battery_status_poll();

			// update aa241x data structs as needed
			mission_status_poll();
			high_data_poll();

			// DEBUG
			/*
			// set all the variables needed for the control law
			set_aux_values();

			// run the custom control law
			low_loop();

			// publish the shared data
			publish_low_data();

			// update previous loop timestamp
			previous_loop_timestamp = timestamp;
			*/
			// DEBUG


			if (_vcontrol_mode.flag_control_auto_enabled) {
				//if (wasAuto){
				//	wasAuto = false;
				//	nnet = load_network(PX4_ROOTFSDIR"/fs/microsd/Race75.nnet");
				//}

				// set all the variables needed for the control law
				set_aux_values();

				// run the custom control law
				low_loop(nnet);

				// publish the shared data
				publish_low_data();

				// update previous loop timestamp
				previous_loop_timestamp = timestamp;

			}
			//else if (!wasAuto){
			//	wasAuto = true;
			//	destroy_network(nnet);
			//}
		}

		_loop_counter++;
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_exit(0);
}

int
LowPriorityLoop::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("aa241x_low",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_DEFAULT - 10,
				       1800,
				       (main_t)&LowPriorityLoop::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int aa241x_low_main(int argc, char *argv[])
{
	if (argc < 1)
		errx(1, "usage: aa241x_low {start|stop|status}");

	if (!strcmp(argv[1], "start")) {

		if (low_control::g_control != nullptr)
			errx(1, "already running");

		low_control::g_control = new LowPriorityLoop;

		if (low_control::g_control == nullptr)
			errx(1, "alloc failed");

		if (OK != low_control::g_control->start()) {
			delete low_control::g_control;
			low_control::g_control = nullptr;
			err(1, "start failed");
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (low_control::g_control == nullptr || !low_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}
		printf("\n");

		exit(0);
	}

	if (!strcmp(argv[1], "stop")) {
		if (low_control::g_control == nullptr)
			errx(1, "not running");

		delete low_control::g_control;
		low_control::g_control = nullptr;
		exit(0);
	}

	if (!strcmp(argv[1], "status")) {
		if (low_control::g_control) {
			errx(0, "running");

		} else {
			errx(1, "not running");
		}
	}

	warnx("unrecognized command");
	return 1;
}
