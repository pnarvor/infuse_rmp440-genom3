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

#include <sys/time.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <rmp440/rmp440.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "orMathLib.h"
#include "codels.h"

/*----------------------------------------------------------------------*/

/**
 ** Actual tracking code
 **/
genom_event
track(const or_genpos_cart_ref *ref, const or_genpos_cart_state *robot,
    or_genpos_track_mode track_mode,
    double *vRef, double *wRef, genom_context self)
{

	switch (track_mode) {
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
		return rmp440_bad_track_mode(self);
	}
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
void
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

void
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

