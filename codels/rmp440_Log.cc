/*
 * Copyright (c) 2009-2017 CNRS/LAAS
 *
 * Permission to use, copy, modify, and distribute this software for any
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

#include <sys/types.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

extern "C" {
#include <rmp440/rmp440.h>
#include <fe/ftdi-emergency.h>
}

#include "acrmp440.h"

#include "rmp440_c_types.h"

#include "rmp440_Log.h"
#include "codels.h"

#ifdef notyet
int
rmp440LogPose(struct rmp440_log_str *log, const or_pose_estimator_state *pose)
{

	if (fprintf(log->out, "%lf\t"
		"%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t%2.6g\t"
		"%2.6g\t%2.6g\t%2.6g\t%2.6g\t%2.6g\t%2.6g\n",
		pose->ts.sec + pose->ts.nsec*1.0E-9,
		pose->vel._value.vx, pose->vel._value.wz,
		pose->pos._value.x, pose->pos._value.y, pose->pos._value.z,
		pose->pos._value.qw, pose->pos._value.qx, 
		pose->pos._value.qy, pose->pos._value.qz,
		pose->vel._value.vx, pose->vel._value.vy, pose->vel._value.vz,
		pose->vel._value.wx, pose->vel._value.wy, pose->vel._value.wz) 
	    < 0) {
		printf ("%s: cannot log in %s\n", __func__, log->fileName);
		return -1;
	}

	return 0;
}
#endif



int
rmp440LogFeedback(const struct rmp440_log_str *log,
    const rmp440_gyro *gyro,
    const rmp440_gyro_asserv *gyro_asserv,
    const rmp440_cmd_str *cmd,
    const rmp440_feedback *data)
{

	if (fprintf(log->out, "%lf"
		"\t%.6g\t%.6g"
		"\t%.6g\t%.6g"
		"\t%.6g\t%.6g"
		"\t%d\t%.6g"
		/* start of feedback */
		"\t0x%08x\t0x%08x\t0x%08x\t%08x"
		"\t0x%08x\t0x%08x\t0x%08x\t%08x"
		"\t%.6g\t%d\t%d\t%.6g\t%.6g"
		"\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\t%d\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\t0x%08x\t0x%08x\t0x%08x\t%.6g\t0x%08x\t0x%08x"
		"\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g\t%.6g"
		"\n",
		/* timestamps */
		data->timestamp.tv_sec + data->timestamp.tv_nsec*1.0E-9,
		/* the KVH gyro */
		gyro->gyroTheta,
		gyro->gyroOmega,
		cmd->vReference, cmd->vCommand,
		cmd->wReference, cmd->wCommand,
		gyro_asserv->straight, gyro_asserv->straight_angle,
		/* rmp440 feedback */
		data->fault_status[0], data->fault_status[1],
		data->fault_status[2], data->fault_status[3],
		data->mcu_fault_status[0], data->mcu_fault_status[1],
		data->mcu_fault_status[2], data->mcu_fault_status[3],
		data->frame_count,
		data->operational_state,
		data->dynamic_response,
		data->min_propulsion_batt_soc,
		data->aux_batt_soc,
		data->inertial_x_acc,
		data->inertial_y_acc,
		data->inertial_x_rate,
		data->inertial_y_rate,
		data->inertial_z_rate,
		data->pse_pitch,
		data->pse_pitch_rate,
		data->pse_roll,
		data->pse_roll_rate,
		data->pse_yaw_rate,
		data->pse_data_is_valid,
		data->yaw_rate_limit,
		data->vel_limit,
		data->linear_accel,
		data->linear_vel,
		data->differential_wheel_vel,
		data->right_front_vel,
		data->left_front_vel,
		data->right_rear_vel,
		data->left_rear_vel,
		data->right_front_pos,
		data->left_front_pos,
		data->right_rear_pos,
		data->left_rear_pos,
		data->linear_pos,
		data->right_front_current,
		data->left_front_current,
		data->right_rear_current,
		data->left_rear_current,
		data->max_motor_current,
		data->right_front_current_limit,
		data->left_front_current_limit,
		data->right_rear_current_limit,
		data->left_rear_current_limit,
		data->min_motor_current_limit,
		data->front_base_batt_1_soc,
		data->front_base_batt_2_soc,
		data->rear_base_batt_1_soc,
		data->rear_base_batt_2_soc,
		data->front_base_batt_1_temp,
		data->front_base_batt_2_temp,
		data->rear_base_batt_1_temp,
		data->rear_base_batt_2_temp,
		data->vel_target,
		data->yaw_rate_target,
		data->angle_target,	/* only used on omni platforms */
		data->aux_batt_voltage,
		data->aux_batt_current,
		data->aux_batt_temp,
		data->abb_system_status,
		data->abb_batt_status,
		data->aux_batt_faults,
		data->ccu_7p2_battery_voltage,
		data->sp_sw_build_id,
		data->uip_sw_build_id,
		data->mcu_inst_power[0], data->mcu_inst_power[1],
		data->mcu_inst_power[2], data->mcu_inst_power[3],
		data->mcu_total_energy[0], data->mcu_total_energy[1],
		data->mcu_total_energy[2], data->mcu_total_energy[3]) < 0)
	{
		printf ("%s: cannot log in %s\n", __func__, log->fileName);
		return -1;
	}
	return 0;
}
