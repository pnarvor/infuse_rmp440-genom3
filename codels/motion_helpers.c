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

	// status->rs_data = *((rmp440_feedback *)data);
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
#endif
		//rmp440Motion(rmp, v, w);
		rmp440CmdNone(rmp);
	} else
		rmp440CmdNone(rmp);
}

