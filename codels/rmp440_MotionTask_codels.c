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

/* --- Task MotionTask -------------------------------------------------- */

/*----------------------------------------------------------------------*/
/*
 * rmp440DataUpdate - copy data from feedback structure to the IDS
 */
static void
rmp440DataUpdate(rmp440_feedback *data, FE_STR *fe,
    rmp440_status_str *status, rmp_status_str *statusgen)
{
	static float prevFrame = 0;
	static int i, noData = 0;
	struct timeval tv;

	gettimeofday(&tv, NULL); // FIXME should be measured more thoroughly in rmp440-libs and with blocking reads
	double receive_date = tv.tv_sec + tv.tv_usec*1e-6;

	if (data->frame_count == prevFrame)
		noData++;
	else
		prevFrame = data->frame_count;

	/* If not data for several periods - set motors off */
	if (noData >  10) {
		printf("No data ?\n");
		status->rs_mode = statusgen->rs_mode = rmp440_mode_motors_off;
	}
	if (noData)
		return;

	if (data->operational_state != 4)
		status->rs_mode = statusgen->rs_mode = rmp440_mode_motors_off;

	// status->rs_data = *((rmp440_feedback *)data); XXX

	statusgen->receive_date = receive_date;
	statusgen->propulsion_battery_level = data->min_propulsion_batt_soc;
	statusgen->aux_battery_level = data->aux_batt_soc;
	statusgen->pitch = DEG_TO_RAD(data->pse_pitch);
	statusgen->roll = DEG_TO_RAD(data->pse_roll);
	statusgen->yaw_rate = DEG_TO_RAD(data->pse_yaw_rate);
	statusgen->v = data->linear_vel;
	statusgen->w = data->differential_wheel_vel;
	statusgen->v_target = data->vel_target;
	statusgen->w_target = data->yaw_rate_target;
	statusgen->right_front_vel = data->right_front_vel;
	statusgen->left_front_vel = data->left_front_vel;
	statusgen->right_rear_vel = data->right_rear_vel;
	statusgen->left_rear_vel = data->left_rear_vel;
	statusgen->right_front_pos = data->right_front_pos;
	statusgen->left_front_pos = data->left_front_pos;
	statusgen->right_rear_pos = data->right_rear_pos;
	statusgen->left_rear_pos = data->left_rear_pos;
	statusgen->right_front_torque = data->right_front_current;
	statusgen->left_front_torque = data->left_front_current;
	statusgen->right_rear_torque = data->right_rear_current;
	statusgen->left_rear_torque = data->left_rear_current;

	/* Check emergency stop */
	fe_get_status(fe);
	/* ignore pause if motors are off */
	if ((fe_pins(fe) & FE_PAUSE) != 0 &&
	    status->rs_mode != rmp440_mode_emergency &&
	    status->rs_mode != rmp440_mode_motors_off) {
		printf("Emergency pause!\n");
		if (status->rs_mode != rmp440_mode_motors_off)
			status->rs_mode = statusgen->rs_mode
			    = rmp440_mode_emergency;
	}
	return;
}

/*----------------------------------------------------------------------*/
/*
 * Compute speed
 *
 * sets robot.v and robot.w in the SDI from motion in the motor structs.
 */


static void
rmp440VelocityGet(const rmp440_feedback *data, or_genpos_cart_state *robot)
{
	/* Angular speed of the robot */
	/* + correction of observed differential_wheel_vel bias */
	robot->w = - data->differential_wheel_vel  + 0.00129327;
	if (fabs(robot->w) < 0.0015)
		robot->w = 0.0;

	/* linear speed */
	robot->v = data->linear_vel;
}

/*----------------------------------------------------------------------*/
/*
 * Send given linear and angular speeds to the hardware
 *
 */
static void
rmp440VelocitySet(const rmp440_io *rmp, const rmp440_feedback *data,
    double v, double w, double *vCommand, double *wCommand)
{

	/* Last safety check, in case uninitialized values are used */
	if (fabs(v) > data->fram_vel_limit) {
		fprintf(stderr, "WARNING: v > VMAX: %lf\n", v);
		v = v > 0.0 ? data->fram_vel_limit : -data->fram_vel_limit;
	}
	/* Save the set velocities */
	*vCommand = v;

	/* Normalize */
	v = v/data->fram_vel_limit;

	if (fabs(w) > data->fram_yaw_rate_limit) {
		fprintf(stderr, "WARNING: w > WMAX: %lf\n", w);
		w = w > 0.0 ? data->fram_yaw_rate_limit
		    : -data->fram_yaw_rate_limit;
	}

	/* Save the set velocities */
	*wCommand = w;

	/* Normalize and invert sign */
	w = -w/data->fram_yaw_rate_limit;

	/* Send commands to motors if ON */
	if (data->operational_state == 4)
		rmp440Motion(rmp, v, w);
	else
		rmp440CmdNone(rmp);
}

/*----------------------------------------------------------------------*/

/**
 ** Actual tracking code
 **/
static genom_event
track(const or_genpos_cart_ref *ref, const or_genpos_cart_state *robot,
    or_genpos_track_mode track_mode,
    double *vRef, double *wRef, genom_context self)
{
	static double prevV, prevW;
	static bool prevDrifted = false;
	bool drifted = false;

	switch (track_mode) {
#ifdef notyet
	case or_genpos_track_pos:
		if (ref->dataType != or_genpos_pos_data) {
			fprintf(stderr, "wrong data type for TRACK_POS\n");
			return rmp440_bad_ref(self);
		}
		drifted = contLawCartPositionControl(&rmp440DataStrId->cmd,
		    ref, robot, EXEC_TASK_PERIOD(RMP440_MOTIONTASK_NUM),
		    prevV, prevW, vRef, wRef);
		break;

	case or_genpos_track_config:
		if (ref->dataType != or_genpos_pos_and_speed_data) {
			fprintf(stderr, "wrong data type for TRACK_CONFIG\n");
			return rmp440_bad_ref(self);
		}
		drifted = contLawCartConfigControl(&rmp440DataStrId->cmd,
		    ref, robot, EXEC_TASK_PERIOD(RMP440_MOTIONTASK_NUM),
		    prevV, prevW, vRef, wRef);
		break;
#endif
	case or_genpos_track_speed:
		if (ref->dataType != or_genpos_speed_data) {
			fprintf(stderr, "wrong data type for TRACK_SPEED %d\n",
			    ref->dataType);
			return rmp440_bad_ref(self);
		}
		*vRef = (ref->backFlag == or_genpos_forward_motion ? ref->v :
		    -ref->v);
		*wRef = ref->w;
		break;
	default:
		return rmp440_bad_ref(self);
	}
	if (drifted && !prevDrifted) {
		printf("odoAndAsserv: important drift!\n");
		printf("ref: %.2lf %.2lf %.2lf\n",
		       ref->x, ref->y, ref->theta);
		printf("pos: %.2lf %.2lf %.2lf\n",
		       robot->xRef, robot->yRef, robot->theta);
	}
	if (!drifted && prevDrifted)
		printf("odoAndAsserv: drift cancelled\n");
	prevDrifted = drifted;

	prevV = *vRef;
	prevW = *wRef;
	return genom_ok;
}

/*----------------------------------------------------------------------*/

/*
 * Filter velocities to keep accelerations within max bounds
 *
 * don't use real time but rather module's period, so that if some periods
 * are lost it doesn't accelerate too much afterwards
 *
 * When moving along an arc (keep_radius = true) adjust angular speed
 * to respect the circle's radius
 */
static void
bound_accels(rmp440_max_accel *acc, double t,
    double *vel_reference, double *ang_reference)
{
	const double epsilon = 1e-3;
	int keep_radius = (fabs(*ang_reference) > epsilon &&
	    fabs(*vel_reference) > epsilon);
	double radius;

	if (keep_radius)
		radius = *vel_reference / *ang_reference;

	// linear velocity
	double sign_accel = *vel_reference > acc->prev_vel_command ?
	    +1.0 : -1.0;
	if (acc->prev_vel_command * *vel_reference < 0)
		*vel_reference = 0.0;

	double max_accel = fabs(acc->prev_vel_command) < fabs(*vel_reference) ?
	    RMP_DEFAULT_MAXIMUM_ACCEL : RMP_DEFAULT_MAXIMUM_DECEL;
	double max_vel = acc->prev_vel_command +
	    sign_accel * max_accel * rmp440_sec_period;
	*vel_reference = sign_accel *
	    fmin(sign_accel * *vel_reference, sign_accel * max_vel);
	acc->prev_vel_command = *vel_reference;

	// angular velocity to preserve radius
	if (keep_radius) *ang_reference = *vel_reference / radius;
}

/*----------------------------------------------------------------------*/

/*
 * Control robot rotations
 *
 * The internal control laws are so inefficient that the robot
 * is not able to follow an arc of circle of a given radius
 *
 * So implement a control using an external (gyroscop) measure of the
 * yaw rate to adjust the speeds to make that better
 *
 * inputs:
 *   gyro - data from IDS
 *   t - current time
 *   vel_reference  - desired linear velocity
 *   yawr_reference - desired angular velocity
 *   yawr_measure   - mesured angular velocity
 *   yaw_measure    - mesured angular position
 * outputs
 *   yawr_command - corrected angular velocity to be sent to the robot
 *
 * also access the gyroAsserv member of the IDS directly.
 */

static void
control_yaw(rmp440_gyro_asserv *gyro,
    double t, double vel_reference, double yawr_reference,
	    double yawr_measure, double yaw_measure, double *yawr_command)
{
	*yawr_command = gyro->prev_command;

	if (gyro->first) {
		*yawr_command = yawr_reference;
		gyro->first = 0;
		gyro->enabled = 1;
		gyro->jump_t = 0.;
	} else {
		double dt = t - gyro->prev_t;

		if (yawr_reference == 0.0) {
			if (vel_reference != 0.0) {
				// straight -> directly asserv on angle
				if (gyro->straight) {
					if (yawr_measure == gyro->prev_measure)
						return;

					// PI controller on angle commanded in speed
					// I is needed to remove static error
					// D is not needed because only used
					//   to go straight (small errors)
					double error_s = gyro->straight_angle-yaw_measure;
					if (error_s > M_PI) error_s -= 2*M_PI;
					if (error_s < -M_PI) error_s += 2*M_PI;
					gyro->integral_s += error_s * (t-gyro->prev_t);
					*yawr_command =
						rmp440_kp_gyro_theta*dt * error_s +
						rmp440_ki_gyro_theta*dt * gyro->integral_s;
				} else {
					*yawr_command = 0.0;
					gyro->straight = 1;
					gyro->straight_angle = yaw_measure;
					gyro->integral_s = 0.0;
				}
			} else {
				// do nothing, command (0,0) is correctly respected
				*yawr_command = 0.0;
				gyro->straight = 0;
			}
		} else {
			const double ref_ratio = 1.5;
			const double noise_th = 0.1;

			// if changing sign or increasing from around 0
			// then directly apply a priori command
			if ((yawr_reference*gyro->prev_reference < 0) ||
			    ((fabs(gyro->prev_reference) < noise_th)
				&& (fabs(yawr_reference) >= noise_th*ref_ratio))) {
				*yawr_command = yawr_reference;
				gyro->jump_t = t;
			}
			// if changing more than 50%
			// then directly apply extrapolated command
			if ((fabs(gyro->prev_reference) >= noise_th) &&
			    (yawr_reference*gyro->prev_reference > 0) &&
			    ((yawr_reference/gyro->prev_reference > ref_ratio)
				|| (gyro->prev_reference/yawr_reference > ref_ratio))) {
				*yawr_command = gyro->prev_command * yawr_reference
				    / gyro->prev_reference;
				gyro->jump_t = t;
			}
			// otherwise if delay from last jump has elapsed
			// P controller on speed commanded in speed
			if (t-gyro->jump_t >= rmp440_delay_gyro_omega) {
				if (yawr_measure == gyro->prev_measure) return;

				*yawr_command = gyro->prev_command +
				    rmp440_kp_gyro_omega*dt
				    * (yawr_reference-yawr_measure);
			} else if (t != gyro->jump_t) {

				// if we are in the lock period
				// and we didn't just reenter it,
				// then we have to follow the reference changes
				if (fabs(gyro->prev_reference) < noise_th
				    || fabs(yawr_reference - gyro->prev_reference) < 1e-10)
					*yawr_command = gyro->prev_command;
				else
					*yawr_command = gyro->prev_command
					    * yawr_reference / gyro->prev_reference;
			}
			gyro->straight = 0;
		}
	}

	//*yawr_command = yawr_reference; // TEST remove asserv

	gyro->prev_reference = yawr_reference;
	gyro->prev_measure = yawr_measure;
	gyro->prev_command = *yawr_command;
	gyro->prev_t = t;
}

/*----------------------------------------------------------------------*/

/** Codel initOdoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_start.
 * Yields to rmp440_ether, rmp440_odo.
 * Throws rmp440_emergency_stop.
 */
genom_event
initOdoAndAsserv(rmp440_ids *ids, genom_context self)
{
	rmp440_kinematics_str *kinematics = &ids->kinematics;
	or_genpos_cart_state *robot = &ids->robot;
	or_genpos_cart_ref *ref = &ids->ref;
	rmp440_gyro *gyro = &ids->gyro;
	rmp440_gyro_asserv *gyro_asserv = &ids->gyro_asserv;
	rmp440_max_accel *max_accel = &ids->max_accel;

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
#ifdef notyet
	/* Initialize Pom */
	initPomPosters();
#endif

	return rmp440_odo;
}


/** Codel odoAndAsserv of task MotionTask.
 *
 * Triggered by rmp440_odo.
 * Yields to rmp440_ether, rmp440_pause_odo, rmp440_end.
 * Throws rmp440_emergency_stop.
 */
genom_event
odoAndAsserv(const rmp440_io *rmp, or_genpos_track_mode track_mode,
             const rmp440_kinematics_str *kinematics,
             const rmp440_var_params *var_params, GYRO_DATA **gyroId,
             FE_STR **fe, or_genpos_cart_state *robot,
             or_genpos_cart_ref *ref, rmp440_max_accel *max_accel,
             or_genpos_cart_config_var *var, rmp440_feedback **rs_data,
             rmp440_mode *rs_mode, rmp440_gyro *gyro,
             rmp440_gyro_asserv *gyro_asserv,
             const rmp440_Status *Status,
             const rmp440_StatusGeneric *StatusGeneric,
             genom_context self)
{
	rmp440_feedback *data = *rs_data;
	double direction;
	uint32_t date;
	rmp440_status_str *status = Status->data(self);
	rmp_status_str *statusgen = StatusGeneric->data(self);
	double vRef, wRef;
	double vCommand, wCommand;
	genom_event report = genom_ok;

#if DEBUG>=1
	static int count = 0;
#endif

	if (rmp == NULL)
		return rmp440_pause_odo; /* not initialized yet */

	rmp440ReceiveAndDecode(rmp, data);
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

#ifdef notyet
	case rmp440_mode_manual:
		if (joystickPoll(joy) < 0) {
			*report = S_rmp440_JOYSTICK_ERROR;
			break;
		}
		ref->x = robot->xRef;
		ref->y = robot->yRef;
		ref->theta = robot->theta;
		getJoystickSpeeds(&vRef, &wRef, &ref->linAccelMax, &ref->angAccelMax);
		ref->v = vRef;
		ref->w = wRef;
		break;
#endif

	case rmp440_mode_track:
		report = track(ref, robot, track_mode, &vRef, &wRef, self);
		break;

	default:
		return rmp440_end;
	}
	if (report != genom_ok) {
		/* In case an error occured,
		   stop the robot and the tracking */
		*rs_mode = statusgen->rs_mode = rmp440_mode_idle;
#ifdef notyet
		trackStr->trackMode = GENPOS_NO_TRACKING;
#endif
		vRef = 0;
		wRef = 0;
		return report;
	}

	// SDI_F->vReference = vRef;
	// SDI_F->wReference = wRef;

	struct timespec tv;
	clock_gettime(CLOCK_REALTIME, &tv);
	double t = tv.tv_sec + tv.tv_nsec*1e-9;
	//if (SDI_F->status.rs_mode == RMP440_TRACK)
	bound_accels(max_accel, t, &vRef, &wRef);

	if (gyro->gyroOn)
		control_yaw(gyro_asserv, t, vRef, wRef,
		    gyro->gyroOmega, gyro->gyroTheta, &wRef);
	//(SDI_F->status.rs_data[0].yaw_rate + SDI_F->status.rs_data[1].yaw_rate)/2.0; // do not use internal gyros, they are wrong

	rmp440VelocitySet(rmp, data, vRef, wRef, &vCommand, &wCommand);

	/* publish */
	Status->write(self);
	StatusGeneric->write(self);
#ifdef notyet
	updatePomPosters();
#endif


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
 * Yields to rmp440_main.
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
		return rmp440_main;
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
