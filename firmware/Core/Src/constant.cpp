/*
 * constant.c
 *
 *  Created on: Jul 28, 2025
 *      Author: Ti Manh
 */

#include "constants.h"

const MotorConfig default_motor = {
	.velocity_kp = 19.0,
	.velocity_ki = 0.1,
	.velocity_kd = 0.9,

	.position_kp = 2.0,
	.position_ki = 0.07,
	.position_kd = 0.07,

	.max_velocity = 100.0,
	.smoothing_alpha = 0.2,

	.reverse_encoder = false,

	.pid_arr = 99,
	.pos_scale = 2,
	.vel_scale = 1,

	.encoder_ppr = 256,
	.ramp_speed = 990,
};

const DeviceConfig default_config = {
    .version = 250807,
    .crc = 0,
    .motor1 = default_motor,
    .motor2 = default_motor,
    .can_id = 1,
};
DeviceConfig current_config;



