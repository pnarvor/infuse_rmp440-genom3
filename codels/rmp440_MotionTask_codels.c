/*
 * Copyright (c) 2009-2017 CNRS/LAAS
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/**
 ** rmp440MotionTaskCodels.c
 **
 ** Codels called by execution task rmp440MotionTask
 **
 ** Author: Matthieu Herrb
 ** Date: April 2009 for rmp400, updated for rmp440, May 2013, 
 **       updated for genom3, January 2017
 **
 **/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <rmp440/rmp440.h>
#include <fe/ftdi-emergency.h>
#include <gyroLib/gyro.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"


/* --- Task MotionTask -------------------------------------------------- */


/** Codel initOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether, rmp440_odo.
 * Throws rmp440_emergency_stop.
 */
genom_event
initOdoAndAsserv(or_genpos_cart_state *robot, or_genpos_cart_ref *ref,
                 rmp440_kinematics_str *kinematics,
                 rmp440_max_accel *max_accel, rmp440_gyro *gyro,
                 rmp440_gyro_mode *gyro_mode,
                 rmp440_gyro_asserv *gyro_asserv, genom_context self)
{
	/* Kinematics */
	kinematics->leftWheelRadius = RMP_X2_TIRE_DIAMETER/2.0;
	kinematics->rightWheelRadius = RMP_X2_TIRE_DIAMETER/2.0;
	kinematics->axisWidth = RMP_DEFAULT_WHEEL_TRACK_WIDTH;

	/* configuration */
	robot->xRob = 0.;
	robot->yRob = 0.;
	robot->theta = 0.;
	robot->xRef = 0;
	robot->yRef = 0;
	robot->v = 0;
	robot->w = 0;

	/* Ref Data */
	ref->dataType = or_genpos_pos_and_speed_data;
	ref->backFlag = or_genpos_forward_motion;
	ref->x = robot->xRef;
	ref->y = robot->yRef;
	ref->theta = robot->theta;
	ref->v = 0.;
	ref->w = 0.;

	ref->vmax = RMP_DEFAULT_MAXIMUM_VELOCITY;
	ref->wmax = RMP_DEFAULT_MAXIMUM_YAW_RATE;
	ref->linAccelMax = RMP_DEFAULT_MAXIMUM_ACCEL;
	ref->angAccelMax = RMP_DEFAULT_MAXIMUM_YAW_ACCEL;

#if 0
	/* Control parameters */
	cmdId = &rmp440DataStrId->cmd;
	cmdId->distPoint = RMP440_DIST_TO_CONTROLLED_POINT;
	cmdId->KpLongit = RMP440_KP_LONGIT;
	cmdId->KiLongit = RMP440_KI_LONGIT;
	cmdId->KpTransv = RMP440_KP_TRANSV;
	cmdId->KiTransv = RMP440_KI_TRANSV;

	cmdId->KpAng = RMP440_KP_ANG;
	cmdId->KiAng = RMP440_KI_ANG;

	cmdId->longitMaxGap = RMP440_LONGIT_MAX_GAP;
	cmdId->transvMaxGap = RMP440_TRANSV_MAX_GAP;
	cmdId->angMaxGap    = RMP440_ANG_MAX_GAP;

	cmdId->longitErrorMax = RMP440_LONGIT_ERROR_MAX;
	cmdId->transvErrorMax = RMP440_TRANSV_ERROR_MAX;
	cmdId->angErrorMax = RMP440_ANG_ERROR_MAX;

	cmdId->longitMaxErrSum = RMP440_LONGIT_MAX_ERR_SUM;
	cmdId->transvMaxErrSum = RMP440_TRANSV_MAX_ERR_SUM;
	cmdId->angMaxErrSum = RMP440_ANG_MAX_ERR_SUM;
#endif

	/* gyro */
	gyro->params.mode = rmp440_gyro_on;
	gyro->gyroOn = false;
	gyro->gyroToRobotOffset = 0.0;
	gyro->gyroTheta = 0.0;

	/* gyro asserv */
	gyro_asserv->enabled = 0;
	gyro_asserv->first = 1;
	gyro_asserv->straight = 0;


	/* max accel */
	max_accel->prev_vel_command = 0.;
	max_accel->prev_vel_command_t = -1.;
#ifdef notyet
	/* Initialize Pom */
	initPomPosters();
#endif

	return rmp440_odo;
}


/** Codel odo of task MotionTask.
 *
 * Triggered by rmp440_odo.
 * Yields to rmp440_ether, rmp440_asserv.
 * Throws rmp440_emergency_stop.
 */
genom_event
odo(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}


/** Codel asserv of task MotionTask.
 *
 * Triggered by rmp440_asserv.
 * Yields to rmp440_ether, rmp440_end.
 * Throws rmp440_emergency_stop.
 */
genom_event
asserv(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}


/** Codel endOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_end.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop.
 */
genom_event
endOdoAndAsserv(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}


/* --- Activity Init ---------------------------------------------------- */

/** Codel rmp440InitStart of activity Init.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_main.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitStart(const char *device, rmp440_io **rmp, FE_STR **fe,
                rmp440_feedback **rs_data, genom_context self)
{
	/* error if already connected */
	if (rmp != NULL)
		return rmp440_already_initialized(self);

	/* connect emergency stop */
	*fe = fe_init(NULL);
	if (fe == NULL)
		return rmp440_malloc_error(self);
	*rs_data = rmp440FeedbackInit(NULL);
	if (*rs_data == NULL)
		return rmp440_malloc_error(self);

	if (device[0] == '/') /* /dev/ttyACM0 */
		*rmp = rmp440Init(RMP440_INTERFACE_USB, device);
	else if (device[0] >= '0' && device[0] <= '9')
		/* ip address:port */
		*rmp = rmp440Init(RMP440_INTERFACE_ETHERNET, device);
	else if (strncmp(device, "fake:", 5) == 0) 
		/* simulation with config file name */
		*rmp = rmp440Init(RMP440_INTERFACE_FAKE, device+5);

	if (*rmp == NULL) {
		return rmp440_rmplib_error(self);
	}
	rmp440SetOperationalMode(*rmp, RMP_TRACTOR_REQUEST);
	return rmp440_main;
}

/** Codel rmp440InitMain of activity Init.
 *
 * Triggered by rmp440_main.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitMain(const rmp440_io *rmp, rmp440_feedback **rs_data,
               rmp440_mode *rs_mode, rmp440_dynamic_str *dynamics,
               rmp440_kinematics_str *kinematics, genom_context self)
{
	rmp440_feedback *data = *rs_data;

	rmp440ReceiveAndDecode(rmp, data);

	/* Check motors status */
	if (data->operational_state != 4) {
		rmp440CmdNone(rmp);
		return rmp440_exec;
	}
	printf("Motors ON\n");
	*rs_mode = rmp440_mode_idle;
	// sdi_f->statusgen.rmp_model = RMP_MODEL_440;
	/* init kinematics from robot NVRAM */
	kinematics->leftWheelRadius = data->fram_tire_diameter/2.0;
	kinematics->rightWheelRadius = data->fram_tire_diameter/2.0;
	kinematics->axisWidth = data->fram_wheel_track_width;
	/* dynamics */
	dynamics->vMax = data->vel_limit;
	dynamics->wMax = data->yaw_rate_limit;
	dynamics->linAccelMax = data->fram_accel_limit;
	dynamics->angAccelMax = data->fram_yaw_accel_limit;
	/* check and warn if some values are not coherent */
	if (data->fram_vel_limit != data->vel_limit)
		printf("WARNING fram_vel_limit != vel_limit %f %f\n",
		    data->fram_vel_limit, data->vel_limit);
	if (data->fram_yaw_rate_limit != data->yaw_rate_limit)
		printf("WARNING fram_yaw_rate_limit != yaw_rate_limit %f %f\n",
		    data->fram_yaw_rate_limit, data->yaw_rate_limit);
	if (data->fram_accel_limit != data->fram_decel_limit)
		printf("WARNING fram_accel_limit != fram_decel_limit %f %f\n",
		    data->fram_accel_limit, data->fram_decel_limit);

	return rmp440_ether;
}


/* --- Activity JoystickOn ---------------------------------------------- */

/** Codel rmp440JoystickOnStart of activity JoystickOn.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether, rmp440_main, rmp440_inter.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnStart(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}

/** Codel rmp440JoystickOnMain of activity JoystickOn.
 *
 * Triggered by rmp440_main.
 * Yields to rmp440_ether, rmp440_inter.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnMain(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}

/** Codel rmp440JoystickOnInter of activity JoystickOn.
 *
 * Triggered by rmp440_inter.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnInter(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_ether;
}


/* --- Activity Gyro ---------------------------------------------------- */

/** Codel rmp440GyroStart of activity Gyro.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_exec, rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_gyro_error.
 */
genom_event
rmp440GyroStart(rmp440_gyro_mode mode, genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_exec;
}

/** Codel rmp440GyroExec of activity Gyro.
 *
 * Triggered by rmp440_exec.
 * Yields to rmp440_exec, rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_gyro_error.
 */
genom_event
rmp440GyroExec(genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return rmp440_exec;
}
