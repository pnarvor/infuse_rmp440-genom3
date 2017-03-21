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
#include <fe/ftdi-emergency.h>

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "orMathLib.h"
#include "codels.h"

/*----------------------------------------------------------------------*/
/*
 * rmp440DataUpdate - copy data from feedback structure to the IDS
 */
void
rmp440DataUpdate(rmp440_feedback *data, FE_STR *fe,
    rmp440_status_str *status, rmp_status_str *statusgen)
{
	static float prevFrame = 0;
	static int noData = 0;

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

	status->rs_data.timestamp.sec = data->timestamp.tv_sec;
	status->rs_data.timestamp.nsec = data->timestamp.tv_sec;
	status->rs_data.fault_status[0] = data->fault_status[0];
	status->rs_data.fault_status[1] = data->fault_status[1];
	status->rs_data.fault_status[2] = data->fault_status[2];
	status->rs_data.fault_status[3] = data->fault_status[3];
	status->rs_data.mcu_fault_status[0] = data->mcu_fault_status[0];
	status->rs_data.mcu_fault_status[1] = data->mcu_fault_status[1];
	status->rs_data.mcu_fault_status[2] = data->mcu_fault_status[2];
	status->rs_data.mcu_fault_status[3] = data->mcu_fault_status[3];
	status->rs_data.frame_count = data->frame_count;
	status->rs_data.operational_state = data->operational_state;
	status->rs_data.dynamic_response = data->dynamic_response;
	status->rs_data.min_propulsion_batt_soc =
	    data->min_propulsion_batt_soc;
	status->rs_data.aux_batt_soc = data->aux_batt_soc;
	status->rs_data.inertial_x_acc = data->inertial_x_acc;
	status->rs_data.inertial_y_acc = data->inertial_y_acc;
	status->rs_data.inertial_x_rate = data->inertial_x_rate;
	status->rs_data.inertial_y_rate = data->inertial_y_rate;
	status->rs_data.inertial_z_rate = data->inertial_z_rate;
	status->rs_data.pse_pitch = data->pse_pitch;
	status->rs_data.pse_pitch_rate = data->pse_pitch_rate;
	status->rs_data.pse_roll = data->pse_roll;
	status->rs_data.pse_roll_rate = data->pse_roll_rate;
	status->rs_data.pse_yaw_rate = data->pse_yaw_rate;
	status->rs_data.pse_data_is_valid = data->pse_data_is_valid;
	status->rs_data.yaw_rate_limit = data->yaw_rate_limit;
	status->rs_data.vel_limit = data->vel_limit;
	status->rs_data.linear_accel = data->linear_accel;
	status->rs_data.linear_vel = data->linear_vel;
	status->rs_data.differential_wheel_vel =
	    data->differential_wheel_vel;
	status->rs_data.right_front_vel = data->right_front_vel;
	status->rs_data.left_front_vel = data->left_front_vel;
	status->rs_data.right_rear_vel = data->right_rear_vel;
	status->rs_data.left_rear_vel = data->left_rear_vel;
	status->rs_data.right_front_pos = data->right_front_pos;
	status->rs_data.left_front_pos = data->left_front_pos;
	status->rs_data.right_rear_pos = data->right_rear_pos;
	status->rs_data.left_rear_pos = data->left_rear_pos;
	status->rs_data.linear_pos = data->linear_pos;
	status->rs_data.right_front_current = data->right_front_current;
	status->rs_data.left_front_current = data->left_front_current;
	status->rs_data.right_rear_current = data->right_rear_current;
	status->rs_data.left_rear_current = data->left_rear_current;
	status->rs_data.max_motor_current = data->max_motor_current;
	status->rs_data.right_front_current_limit =
	    data->right_front_current_limit;
	status->rs_data.left_front_current_limit =
	    data->left_front_current_limit;
	status->rs_data.right_rear_current_limit =
	    data->right_rear_current_limit;
	status->rs_data.left_rear_current_limit =
	    data->left_rear_current_limit;
	status->rs_data.min_motor_current_limit =
	    data->min_motor_current_limit;
	status->rs_data.front_base_batt_1_soc = data->front_base_batt_1_soc;
	status->rs_data.front_base_batt_2_soc = data->front_base_batt_2_soc;
	status->rs_data.rear_base_batt_1_soc = data->rear_base_batt_1_soc;
	status->rs_data.rear_base_batt_2_soc = data->rear_base_batt_2_soc;
	status->rs_data.front_base_batt_1_temp =
	    data->front_base_batt_1_temp;
	status->rs_data.front_base_batt_2_temp =
	    data->front_base_batt_2_temp;
	status->rs_data.rear_base_batt_1_temp = data->rear_base_batt_1_temp;
	status->rs_data.rear_base_batt_2_temp = data->rear_base_batt_2_temp;
	status->rs_data.vel_target = data->vel_target;
	status->rs_data.yaw_rate_target = data->yaw_rate_target;
	status->rs_data.angle_target = data->angle_target;
	status->rs_data.aux_batt_voltage = data->aux_batt_voltage;
	status->rs_data.aux_batt_current = data->aux_batt_current;
	status->rs_data.aux_batt_temp = data->aux_batt_temp;
	status->rs_data.abb_system_status = data->abb_system_status;
	status->rs_data.abb_batt_status = data->abb_batt_status;
	status->rs_data.aux_batt_faults = data->aux_batt_faults;
	status->rs_data.ccu_7p2_battery_voltage =
	    data->ccu_7p2_battery_voltage;
	status->rs_data.sp_sw_build_id = data->sp_sw_build_id;
	status->rs_data.uip_sw_build_id = data->uip_sw_build_id;
	status->rs_data.mcu_inst_power[4] = data->mcu_inst_power[4];
	status->rs_data.mcu_total_energy[4] = data->mcu_total_energy[4];
	status->rs_data.fram_vel_limit = data->fram_vel_limit;
	status->rs_data.fram_accel_limit = data->fram_accel_limit;
	status->rs_data.fram_decel_limit = data->fram_decel_limit;
	status->rs_data.fram_dtz_decel_limit = data->fram_dtz_decel_limit;
	status->rs_data.fram_coastdown_decel = data->fram_coastdown_decel;
	status->rs_data.fram_yaw_rate_limit = data->fram_yaw_rate_limit;
	status->rs_data.fram_yaw_accel_limit = data->fram_yaw_accel_limit;
	status->rs_data.fram_tire_diameter = data->fram_tire_diameter;
	status->rs_data.fram_wheel_base_length =
	    data->fram_wheel_base_length;
	status->rs_data.fram_wheel_track_width =
	    data->fram_wheel_track_width;
	status->rs_data.fram_transmission_ratio =
	    data->fram_transmission_ratio;
	status->rs_data.fram_config_bitmap = data->fram_config_bitmap;
	status->rs_data.fram_eth_ip_address = data->fram_eth_ip_address;
	status->rs_data.fram_eth_port_number = data->fram_eth_port_number;
	status->rs_data.fram_eth_subnet_mask = data->fram_eth_subnet_mask;
	status->rs_data.fram_eth_gateway = data->fram_eth_gateway;
	status->rs_data.user_feedback_bitmap[0] =
	    data->user_feedback_bitmap[0];
	status->rs_data.user_feedback_bitmap[1] =
	    data->user_feedback_bitmap[1];
	status->rs_data.user_feedback_bitmap[2] =
	    data->user_feedback_bitmap[2];
	status->rs_data.user_feedback_bitmap[3] =
	    data->user_feedback_bitmap[3];


	statusgen->receive_date = data->timestamp.tv_sec
	  + data->timestamp.tv_nsec*1e-9;
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


void
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
void
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
	if (data->operational_state == 4) {
#if DEBUG > 1
		printf("rmp440Motion(rmp, %lf, %lf)\n", v, w);
		rmp440CmdNone(rmp);
#else
		rmp440Motion(rmp, v, w);
#endif
	} else
		rmp440CmdNone(rmp);
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

