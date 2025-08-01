/*
 * constants.h
 *
 *  Created on: Apr 25, 2025
 *      Author: Ti Manh
 */
#ifndef INC_CONSTANTS_H
#define INC_CONSTANTS_H

#include "stm32g4xx_hal.h"

#define MAX_DUTY 990

#define MIN_DUTY 0

#define FLASH_CONFIG_ADDRESS 0x0801F800

#define SERIAL_SPEED 115200

typedef struct {
    double velocity_kp;
    double velocity_ki;
    double velocity_kd;

    double position_kp;
    double position_ki;
    double position_kd;

    double max_velocity;
    double smoothing_alpha;

    bool reverse_encoder;

    uint16_t pid_arr;
    uint16_t pos_scale;

    uint16_t encoder_ppr;
    uint16_t ramp_speed;

} MotorConfig;

typedef struct {
    uint32_t version;
    uint32_t crc;
    MotorConfig motor1;
    MotorConfig motor2;
    uint8_t can_id;
} DeviceConfig;

extern const DeviceConfig default_config;
extern const MotorConfig default_motor;
extern DeviceConfig current_config;

#endif /* INC_CONSTANTS_H_ */
