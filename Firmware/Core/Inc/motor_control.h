/*
 * motor_control.h
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */

#ifndef INC_MOTOR_CONTROL_H_
#define INC_MOTOR_CONTROL_H_

#include "stm32g4xx_hal.h"
#include "constants.h"


class MOTOR {
public:
	bool init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right);
	void initEncoder(int16_t ppr,uint8_t port);
	void setPwm(int16_t pwm);

	int64_t getPosition();
	int32_t getRPM();

	void positionMode();
	void velocityMode();

private:

	void getData();

	TIM_TypeDef *L_TIM;
	uint8_t L_CHN;
	TIM_TypeDef *R_TIM;
	uint8_t R_CHN;

	bool hasEncoder = false;

	int16_t pulsePerRev;
	uint8_t encoderPort;

	int64_t encoderPosition;
	int32_t encoderSpeed;
};

#endif /* INC_MOTOR_CONTROL_H_ */
