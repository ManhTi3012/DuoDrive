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
#include "PID.h"

class Motor {
public:
	bool Init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right);
	void InitEncoder(double ppr,TIM_TypeDef *timer_encoder);

    void SetRampTime(uint16_t time_s);

    double GetPPR();
	double GetPosition();
	double GetRPM();
	int16_t GetPwm();


	void PositionMode(double target);
	void VelocityMode(double target);
	void OpenLoopMode(int16_t pwm);
	void SetPPR(double ppr);
	void ResetPos(float new_position = 0.0d);

    struct {
        PID position;  // Outer loop (position → velocity)
        PID velocity;  // Inner loop (velocity → PWM)
    } pid;

	void Update(double dt);

private:
	void SetPwm(int16_t pwm);

	void RampPwm(double dt);

    double current_pwm_ = 0;
    int16_t target_pwm_ = 0;
    double ramp_speed = 1980;
    double ramp_time_s_ = 0.5f;

	void UpdateData(double dt);

	enum ControlMode {
	    POSITION,
	    VELOCITY,
		OPEN_LOOP
	} current_mode_ = OPEN_LOOP;

    double target_velocity_ = 0;
    double target_position_ = 0;

	TIM_TypeDef *L_TIM;
	uint8_t L_CHN;
	TIM_TypeDef *R_TIM;
	uint8_t R_CHN;

	bool has_encoder_ = false;
	bool reverse_encoder_ = true;

	double pulse_per_rev_;
	TIM_TypeDef *encoder_port_;

	double encoder_position_;
	double encoder_speed_;

	uint16_t last_encoder_value_ = 0;
};

#endif /* INC_MOTOR_CONTROL_H_ */
