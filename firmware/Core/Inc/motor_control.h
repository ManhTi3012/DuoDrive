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
	bool Init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right, TIM_TypeDef* pid_timer);
	void InitEncoder(TIM_TypeDef *timer_encoder);
	void LoadConfig(MotorConfig config);
	MotorConfig GetConfig();

	void PositionMode(double target);
	void VelocityMode(double target);
	void OpenLoopMode(int16_t pwm);

	void ResetPos();
	void ResetPos(int64_t new_position);

	void SetMaxVel(double max_vel);
	void SetAlpha(double alpha);

	void ReverseEncoder(bool reverse);

	void SetARR(uint16_t arr);
	void SetVScale(uint16_t vscale);
	void SetPScale(uint16_t pscale);

	void SetPPR(uint16_t ppr);
	void SetRampSpeed(uint16_t ramp_speed);

	void SetGainP(double set_Kp, double set_Ki, double set_Kd);
	void SetGainV(double set_Kp, double set_Ki, double set_Kd);

	double GetMaxVel();
	double GetAlpha();

	bool IsEncoderReversed();

	uint16_t GetARR();
	uint16_t GetVScale();
	uint16_t GetPScale();

	uint16_t GetPPR();
	uint16_t GetRampSpeed();

	void GetGainP(double &p, double &i, double &d);
	void GetGainV(double &p, double &i, double &d);

	int64_t GetPosition();
	int16_t GetPwm();
	double GetRPM();

	void UpdateData(double dt);
	void PositionLoop(double dt);
	void VelocityLoop(double dt);


private:

	TIM_TypeDef *L_TIM;
	uint8_t L_CHN;
	TIM_TypeDef *R_TIM;
	uint8_t R_CHN;
	TIM_TypeDef* pid_timer_;
	void SetPwm(int16_t pwm);
	void RampPwm(double dt);

	void Update(double dt); //Unused

    struct {
        PID position;  // Outer loop (position → velocity)
        PID velocity;  // Inner loop (velocity → PWM)
    } pid;

	enum ControlMode {
	    POSITION,
	    VELOCITY,
		OPEN_LOOP
	} current_mode_ = OPEN_LOOP;

	MotorConfig config_;

	int16_t target_pwm_ = 0;
    double current_pwm_ = 0;

    double target_velocity_ = 0;

    double target_position_ = 0;

	bool has_encoder_ = false;
	double encoder_speed_ = 0.0;

	TIM_TypeDef *encoder_port_;
	volatile int64_t encoder_position_;
	volatile uint16_t last_encoder_value_ = 0;

	int16_t delta_history[VELOCITY_HISTORY_SIZE] = {};
	int history_index = 0;

};

#endif /* INC_MOTOR_CONTROL_H_ */
