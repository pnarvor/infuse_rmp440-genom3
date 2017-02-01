/*
 * Copyright (c) 2017 CNRS/LAAS
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
#ifndef _CODELS_H
#define _CODELS_H

typedef struct rmp440_log_str {
	FILE *out;
} rmp440_log_str;

#define rmp440_log_header \
	"date\tdate1\tdate2" \
	"\tgyroAngle\tgyroRate" \
	"\tsent_velocity_reference\tsent_velocity_command" \
	"\tsent_turn_reference\tsent_turn_command" \
	"\tstraight\tstraight_angle" \
	/* begin of rmp440 feedback */ \
	"\tfault_status0\tfault_status1\tfaultstatus2\tfault_status3" \
	"\tmcu_fault0\tmcu_fault1\tmcu_fault2\tmcu_fault3" \
	"\tframe_count\toperational_state\tdynamic_response" \
	"\tmin_propulstion_bat_soc\taux_batt_soc" \
	"\tinertial_x_acc\tinertial_y_acc\tinertial_x_rate" \
	"\tinertial_y_rate\tinertial_z_rate" \
	"\tpse_pitch\tpse_pitch_rate\tpse_roll\tpse_roll_rate" \
	"\tpse_yaw_rate\tpse_data_is_valid" \
	"\tyaw_rate_limit\tvel_limit\tlinear_accel\tlinear_vel" \
	"\tdifferential_wheel_vel\tright_front_vel\tleft_front_vel" \
	"\tright_rear_vel\tleft_rear_vel" \
	"\tright_front_pos\tleft_front_pos" \
	"\tright_rear_pos\tleft_rear_pos\tlinear_pos" \
	"\tright_front_current\tleft_front_current" \
	"\tright_rear_current\tleft_rear_current\tmax_motor_current" \
	"\tright_front_current_limit\tleft_front_current_limit" \
	"\tright_rear_current_limit\tleft_rear_current_limit" \
	"\tmin_motor_current_limit" \
	"\tfront_base_batt_1_soc\tfront_base_batt_2_soc" \
	"\trear_base_batt_1_soc\trear_base_batt_2_soc" \
	"\tfront_base_batt_1_temp\tfront_base_batt_2_temp" \
	"\trear_base_batt_1_temp\trear_base_batt_2_temp" \
	"\tvel_target\tyaw_rate_target\tangle_target" \
	"\taux_batt_voltage\taux_batt_current\taux_batt_temp" \
	"\tabb_system_status\tabb_batt_status\taux_batt_faults" \
	"\tccu_7p2_battery_voltage\tsp_sw_build_id\tuip_sw_build_id" \
	"\tmcu_inst_power0\tmcu_inst_power1\tmcu_inst_power2" \
	"\tmcu_inst_power3" \
	"\tmcu_total_energy0\tmcu_total_energy1" \
	"\tmcu_total_energy2\tmcu_total_energy3\n"


#endif /* _CODELS_H */
