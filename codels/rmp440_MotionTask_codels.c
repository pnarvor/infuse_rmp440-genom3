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

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rmp440/rmp440.h>
#include <fe/ftdi-emergency.h>
#include <gyroLib/gyro.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "orMathLib.h"
#include "odoProba.h"
#include "codels.h"

/* --- Task MotionTask -------------------------------------------------- */

/*----------------------------------------------------------------------*/

/** Codel initOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether, rmp440_odo.
 * Throws rmp440_emergency_stop.
 */
genom_event
initOdoAndAsserv(rmp440_ids *ids,
                 const rmp440_StatusGeneric *StatusGeneric,
                 genom_context self)
{
	rmp_status_str *statusgen = StatusGeneric->data(self);
	rmp440_kinematics_str *kinematics = &ids->kinematics;
	or_genpos_cart_state *robot = &ids->robot;
	or_genpos_cart_ref *ref = &ids->ref;
	rmp440_gyro *gyro = &ids->gyro;
	rmp440_gyro_asserv *gyro_asserv = &ids->gyro_asserv;
	rmp440_max_accel *max_accel = &ids->max_accel;

	statusgen->rmp_model = rmp_model_440;

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

#ifdef notyet
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

	return rmp440_odo;
}


/** Codel odoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_odo.
 * Yields to rmp440_ether, rmp440_pause_odo, rmp440_end.
 * Throws rmp440_emergency_stop.
 */
genom_event
odoAndAsserv(const rmp440_io *rmp,
             const rmp440_kinematics_str *kinematics,
             const rmp440_var_params *var_params,
             const rmp440_Joystick *Joystick, GYRO_DATA **gyroId,
             FE_STR **fe, or_genpos_cart_state *robot,
             or_genpos_cart_ref *ref, rmp440_max_accel *max_accel,
             or_genpos_track_mode *track_mode,
             rmp440_feedback **rs_data, rmp440_mode *rs_mode,
             rmp440_gyro *gyro, rmp440_gyro_asserv *gyro_asserv,
             const rmp440_Odo *Odo, const rmp440_Pose *Pose,
             const rmp440_Status *Status,
             const rmp440_StatusGeneric *StatusGeneric,
             genom_context self)
{
	rmp440_feedback *data = *rs_data;
	double direction;
	uint32_t date;
	rmp440_status_str *status = Status->data(self);
	rmp_status_str *statusgen = StatusGeneric->data(self);
	or_pose_estimator_state *pose = Pose->data(self);
	double vRef, wRef;
	double vCommand, wCommand;
	genom_event report = genom_ok;
	or_genpos_cart_config_var *var = &Odo->data(self)->var;

#if DEBUG>=1
	static int count = 0;
#endif
	if (rmp == NULL)
		return rmp440_pause_odo; /* not initialized yet */

	if (rmp440ReceiveAndDecode(rmp, data) < 0) {
		printf("no data??\n");
		/* Probably motors OFF - no communication on the RMP440 */
		status->rs_mode = statusgen->rs_mode = rmp440_mode_motors_off;
		rmp440CmdNone(rmp);
		return rmp440_pause_odo;
	}

	rmp440DataUpdate(data, *fe, status, statusgen);

	/* Read config */
	rmp440VelocityGet(data, robot);
	if (ref->backFlag == or_genpos_forward_motion)
		direction = 1;
	else if (ref->backFlag == or_genpos_backward_motion)
		direction = -1;
	else
		direction = 0;

	robot->xRef = robot->xRob;
	robot->yRef = robot->yRob;

	odoProba(robot, var,
	    kinematics->axisWidth, var_params->coeffLinAng,
	    rmp440_sec_period);


	/* Gyro */
	if (gyroId != NULL /* && gyro->currentMode != RMP440_GYRO_OFF */) {
		if (gyroRead(*gyroId, &(gyro->gyroTheta),
			    &(gyro->gyroOmega), &date) != 0) {
			gyro->currentMode = rmp440_gyro_off;
			gyroEnd(*gyroId);
			*gyroId = NULL;
		}
		else {
		  gyro->gyroTheta = - gyro->gyroTheta;
		  gyro->gyroOmega = - gyro->gyroOmega;
		}
	}
	if (gyro->currentMode == rmp440_gyro_off ||
	    (gyro->currentMode == rmp440_gyro_on_if_motion
		&& fabs(robot->w) < 0.017
		&& fabs(robot->v) < 0.01)) {
		/* -- odo is reference */

		/* reset gyro offset to match odo */
		gyro->gyroToRobotOffset = angleLimit(robot->theta
		    - gyro->gyroTheta);
		/* switch back to odo */
		if (gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> odo (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = false;
		}
	} else {
		/* -- gyro is reference */
		robot->theta = angleLimit(gyro->gyroTheta +
		    gyro->gyroToRobotOffset);
		/* switch back to gyro */
		if (!gyro->gyroOn) {
#if DEBUG>=1
			printf ("-> gyro (%5.2f d) v %5.2f w%5.3f\n",
			    RAD_TO_DEG(robot->theta), robot->v, robot->w);
#endif
			gyro->gyroOn = true;
			gyro_asserv->first = 1;
		}
	}

	/*debug */
#if DEBUG>=1
	if (count==10) {
		printf ("MODE %s. gyro %5.3f + offset %5.3f = %5.3f =? %5.3f\n",
		    gyro->gyroOn ? "GYRO" : "ODO",
		    RAD_TO_DEG(gyro->gyroTheta),
		    RAD_TO_DEG(gyro->gyroToRobotOffset),
		    RAD_TO_DEG(angleLimit(gyro->gyroTheta +
			    gyro->gyroToRobotOffset)),
		    RAD_TO_DEG(robot->theta));
		count=0;
	}
	count++;
#endif
#ifdef notyet
	if (odometryMode == RMP440_ODO_3D)
		rmp440Odo3d(EXEC_TASK_PERIOD(RMP440_MOTIONTASK_NUM));
#endif
	/* fill pose */
	pose->ts.sec = data->timestamp.tv_sec;
	pose->ts.nsec = data->timestamp.tv_nsec;
	pose->intrinsic = true;
	pose->pos._present = true;
	pose->pos._value.x = robot->xRob;
	pose->pos._value.y = robot->yRob;
	pose->pos._value.z = 0.0; 	/* XXXX */
	pose->vel._present = true;
	pose->vel._value.vx = robot->v;
	pose->vel._value.vy = 0;
	pose->vel._value.vz = 0;
	pose->vel._value.wx = 0;	/* XXX */
	pose->vel._value.wy = 0;	/* XXX */
	pose->vel._value.wz = robot->w;

	/*
	 * Asserv
	 */

	switch (*rs_mode) {
	case rmp440_mode_motors_off:
		/* No motion possible */
		return rmp440_pause_odo;

	case rmp440_mode_emergency:
	case rmp440_mode_idle:
		vRef = 0.0;
		wRef = 0.0;
		ref->x = robot->xRef;
		ref->y = robot->yRef;
		ref->theta = robot->theta;
		break;


	case rmp440_mode_manual:
		Joystick->read(self);
		ref->x = robot->xRef;
		ref->y = robot->yRef;
		ref->theta = robot->theta;
		getJoystickSpeeds(Joystick->data(self), data, &vRef, &wRef,
		    &ref->linAccelMax, &ref->angAccelMax);
		ref->v = vRef;
		ref->w = wRef;
		break;

	case rmp440_mode_track:
		report = track(ref, robot, *track_mode, &vRef, &wRef, self);
		break;

	default:
		return rmp440_end;
	}
	if (report != genom_ok) {
		/* In case an error occured,
		   stop the robot and the tracking */
		*rs_mode = statusgen->rs_mode = rmp440_mode_idle;
		*track_mode = or_genpos_no_tracking;

		vRef = 0;
		wRef = 0;
		return report;
	}

	// SDI_F->vReference = vRef;
	// SDI_F->wReference = wRef;

	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	double t = tv.tv_sec + tv.tv_nsec*1e-9;
	if (*rs_mode == rmp440_mode_track)
		bound_accels(max_accel, t, &vRef, &wRef);

	if (gyro->gyroOn)
		control_yaw(gyro_asserv, t, vRef, wRef,
		    gyro->gyroOmega, gyro->gyroTheta, &wRef);

	rmp440VelocitySet(rmp, data, vRef, wRef, &vCommand, &wCommand);

	/* publish */
	memcpy(&Odo->data(self)->robot, robot, sizeof(or_genpos_cart_state));
	Pose->write(self);
	Odo->write(self);
	Status->write(self);
	StatusGeneric->write(self);

	return rmp440_pause_odo;
}


/** Codel endOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_end.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop.
 */
genom_event
endOdoAndAsserv(rmp440_io **rmp, rmp440_feedback **rs_data,
                genom_context self)
{
	rmp440_feedback *data = *rs_data;

	if (*rmp == NULL)
		return rmp440_ether;
	if (data->operational_state == 4)
		rmp440SetOperationalMode(*rmp, RMP_STANDBY_REQUEST);
	free(data);
	rmp440End(*rmp);
	*rmp = NULL;
	*rs_data = NULL;
	return rmp440_ether;
}


/* --- Activity Init ---------------------------------------------------- */

/** Codel rmp440InitStart of activity Init.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_init_main.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitStart(const char device[32], rmp440_io **rmp, FE_STR **fe,
                rmp440_feedback **rs_data, genom_context self)
{
	/* error if already connected */
	if (*rmp != NULL || *rs_data != NULL)
		return rmp440_already_initialized(self);

	/* connect emergency stop */
	*fe = fe_init(NULL);
	if (fe == NULL)
		return rmp440_malloc_error(self);
	*rs_data = rmp440FeedbackInit(NULL);
	if (*rs_data == NULL) {
		fe_end(*fe);
		*fe = NULL;
		return rmp440_malloc_error(self);
	}

	if (device[0] == '/') /* /dev/ttyACM0 */
		*rmp = rmp440Init(RMP440_INTERFACE_USB, device);
	else if (device[0] >= '0' && device[0] <= '9')
		/* ip address:port */
		*rmp = rmp440Init(RMP440_INTERFACE_ETHERNET, device);
	else if (strncmp(device, "fake:", 5) == 0)
		/* simulation with config file name */
		*rmp = rmp440Init(RMP440_INTERFACE_FAKE, device+5);

	if (*rmp == NULL) {
		free(*rs_data);
		*rs_data = NULL;
		fe_end(*fe);
		*fe = NULL;
		return rmp440_rmplib_error(self);
	}
	rmp440SetOperationalMode(*rmp, RMP_TRACTOR_REQUEST);
	printf("-- sent tractor request\n");
	return rmp440_init_main;
}

/** Codel rmp440InitMain of activity Init.
 *
 * Triggered by rmp440_init_main.
 * Yields to rmp440_pause_init_main, rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_already_initialized,
 *        rmp440_malloc_error, rmp440_rmplib_error.
 */
genom_event
rmp440InitMain(rmp440_io **rmp, FE_STR **fe, rmp440_feedback **rs_data,
               rmp440_mode *rs_mode, rmp440_dynamic_str *dynamics,
               rmp440_kinematics_str *kinematics, genom_context self)
{
	rmp440_feedback *data = *rs_data;

	printf("-- init_main: receiveAndDecode\n");
	if (rmp440ReceiveAndDecode(*rmp, data) < 0) {
		printf("-- receiveAndDecode return -1 errno: %d\n", errno);
		rmp440End(*rmp);
		*rmp = NULL;
		free(*rs_data);
		*rs_data = NULL;
		fe_end(*fe);
		*fe = NULL;
		return rmp440_rmplib_error(self);
	}

	/* Check motors status */
	if (data->operational_state != 4) {
		printf("-- init_main: operational_state != 4 %d\n",
		    data->operational_state);
		rmp440SetOperationalMode(*rmp, RMP_TRACTOR_REQUEST);
		return rmp440_pause_init_main;
	}
	printf("Motors ON\n");
	*rs_mode = rmp440_mode_idle;
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

	rmp440CmdNone(*rmp);
	return rmp440_ether;
}


/* --- Activity JoystickOn ---------------------------------------------- */

/** Codel rmp440JoystickOnStart of activity JoystickOn.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_js_main.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnStart(const rmp440_Joystick *Joystick,
                      rmp440_mode *rs_mode, genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	*rs_mode = rmp440_mode_manual;

	return rmp440_js_main;
}

/** Codel rmp440JoystickOnMain of activity JoystickOn.
 *
 * Triggered by rmp440_js_main.
 * Yields to rmp440_pause_js_main, rmp440_inter.
 * Throws rmp440_emergency_stop, rmp440_bad_ref, rmp440_rmplib_error,
 *        rmp440_joystick_error, rmp440_motors_off,
 *        rmp440_power_cord_connected.
 */
genom_event
rmp440JoystickOnMain(const rmp440_Joystick *Joystick,
                     rmp440_mode rs_mode, genom_context self)
{
	struct or_joystick_state *joy;

	if (Joystick->read(self)) {
		printf("%s: read joystick failed\n", __func__);
		return rmp440_joystick_error(self);
	}
	joy = Joystick->data(self);

	if (joy == NULL) {
		printf("%s: joystick data failed\n", __func__);
		return rmp440_joystick_error(self);
	}

	/* Check if mode changed */
	switch (rs_mode) {
	case rmp440_mode_power_coord:
		return rmp440_power_cord_connected(self);
	case rmp440_mode_emergency:
		return  rmp440_emergency_stop(self);
	case rmp440_mode_motors_off:
		return rmp440_motors_off(self);
	default:
		break;
	}
	/* Check for user abort */
	if (joystickQuit(joy)) {
		printf("Stop joystick\n");
		return rmp440_inter;
	}
	return rmp440_pause_js_main;
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
rmp440JoystickOnInter(rmp440_mode *rs_mode, or_genpos_cart_ref *ref,
                      genom_context self)
{
	//ref->linAccelMax = rmp_default_maximum_accel;
	//ref->angAccelMax = rmp_default_maximum_yaw_rate;
	printf("%s\n", __func__);
	*rs_mode = rmp440_mode_idle; /* XXXXXX */
	return rmp440_ether;
}


/* --- Activity Gyro ---------------------------------------------------- */

/** Codel rmp440GyroExec of activity Gyro.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether.
 * Throws rmp440_emergency_stop, rmp440_gyro_error.
 */
genom_event
rmp440GyroExec(const rmp440_gyro_params *params,
               const or_genpos_cart_state *robot, rmp440_gyro *gyro,
               GYRO_DATA **gyroId, genom_context self)
{

	if (*gyroId == NULL) {
		*gyroId = gyroInit(params->type, params->port,
			    params->latitude, params->woffset);
		if (*gyroId == NULL) {
			gyro->params.mode = rmp440_gyro_off;
			gyro->currentMode = rmp440_gyro_off;
			return rmp440_gyro_error(self);
		}
	}

	/* read gyro once */
	if (gyroReadAngle(*gyroId, &gyro->gyroTheta) != 0) {
		gyro->params.mode = rmp440_gyro_off;
		gyro->currentMode = rmp440_gyro_off;
		gyroEnd(*gyroId);
		*gyroId = NULL;
		return rmp440_gyro_error(self);
	} else
		gyro->gyroTheta = - gyro->gyroTheta;
	/* reset gyro offset to match odo */
	gyro->gyroToRobotOffset = robot->theta - gyro->gyroTheta;

	gyro->currentMode = gyro->params.mode;

	return rmp440_ether;
}
