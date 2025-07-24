/*
 * motor_control.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */

#include "motor_control.h"

bool Motor::Init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right){
	if(channel_right > 4 || channel_left > 4){
		return false;
	}
	L_TIM = timer_left;
	L_CHN = channel_left;
	R_TIM = timer_right;
	R_CHN = channel_right;

    pid.position.setGains(POS_P, POS_I, POS_D);  // Kp, Ki, Kd
    pid.position.setOutputLimits(-MAX_VELOCITY, MAX_VELOCITY);

    // Velocity PID (outputs PWM)
    pid.velocity.setGains(VEL_P, VEL_I, VEL_D);
    pid.velocity.setOutputLimits(-MAX_DUTY, MAX_DUTY);

	return true;
}

void Motor::InitEncoder(double ppr,TIM_TypeDef *timer_encoder){
	has_encoder_ = true;
	pulse_per_rev_ = ppr;
	encoder_port_ = timer_encoder;
}

double Motor::GetPosition(){
	if(has_encoder_){
		return encoder_position_;
	}
	else{
		return 0;
	}
}

double Motor::GetRPM(){
	if(has_encoder_){
		return encoder_speed_;
	}
	else{
		return 0;
	}
}

int Motor::GetPPR(){
	if(has_encoder_){
		return pulse_per_rev_;
	}
	else{
		return 0;
	}
}

int16_t Motor::GetPwm(){
	return target_pwm_;
}

void Motor::ResetPos(){
	encoder_port_->CNT = 0;
	last_encoder_value_ = 0;
	encoder_position_ = 0;
}

void Motor::ResetPos(double new_position){
	encoder_port_->CNT = 0;
	last_encoder_value_ = 0;
	encoder_position_ = new_position;
}

void Motor::PositionMode(double target){
	if(!has_encoder_){return;}
	else{
		current_mode_ = POSITION;
		target_position_ = target;

	}
}
void Motor::VelocityMode(double target){
	if(!has_encoder_){return;}
	else{
		current_mode_ = VELOCITY;
		target_velocity_ = target;
	}
}

void Motor::OpenLoopMode(int16_t pwm){
	current_mode_ = OPEN_LOOP;
	target_pwm_ = pwm;
}

void Motor::SetPPR(double ppr){
	pulse_per_rev_ = ppr;
}

void Motor::Update(double dt){
	UpdateData(dt);
	switch(current_mode_){
		case OPEN_LOOP:
			RampPwm(dt);
			break;
		case POSITION:
			target_velocity_ = pid.position.calculate(target_position_, encoder_position_, dt);

			target_pwm_ = pid.velocity.calculate(target_velocity_, encoder_speed_, dt);
			RampPwm(dt);

			break;
		case VELOCITY:
			target_pwm_ = pid.velocity.calculate(target_velocity_, encoder_speed_, dt);
			RampPwm(dt);
			break;
	}
}

void Motor::UpdateData(double dt){
	if(!has_encoder_){return;}
	else{
		uint16_t current_encoder_value = encoder_port_ -> CNT;
		int16_t delta = (int16_t)(current_encoder_value - last_encoder_value_);

	    if (reverse_encoder_) {
	        delta = -delta;
	    }
	    double raw_speed = static_cast<double>(delta) / dt;
	    raw_speed = raw_speed / pulse_per_rev_ * 60.0;

	    encoder_speed_ = (double) VEL_EMA_ALPHA * raw_speed + (1.0 - VEL_EMA_ALPHA) * encoder_speed_;

	    if(delta == 0){encoder_speed_ = 0;}

		encoder_position_ += delta;
		last_encoder_value_ = current_encoder_value;
	}
}

void Motor::SetPwm(int16_t pwm){
	//clamp pwm
	if(pwm > MAX_DUTY){
		pwm = MAX_DUTY;
	}
	else if(pwm < -MAX_DUTY){
		pwm = -MAX_DUTY;
	}

	if(pwm >= 0){
		// set left side driver to GND
	    switch (L_CHN) {
	        case 1: L_TIM->CCR1 = 0; break;
	        case 2: L_TIM->CCR2 = 0; break;
	        case 3: L_TIM->CCR3 = 0; break;
	        case 4: L_TIM->CCR4 = 0; break;
	        default: break;
	    }
	    // set right side driver to pwm
	    switch (R_CHN) {
	        case 1: R_TIM->CCR1 = pwm; break;
	        case 2: R_TIM->CCR2 = pwm; break;
	        case 3: R_TIM->CCR3 = pwm; break;
	        case 4: R_TIM->CCR4 = pwm; break;
	        default: break;
	    }
	}
	else{
		// set left side driver to pwm
	    switch (L_CHN) {
	        case 1: L_TIM->CCR1 = -pwm; break;
	        case 2: L_TIM->CCR2 = -pwm; break;
	        case 3: L_TIM->CCR3 = -pwm; break;
	        case 4: L_TIM->CCR4 = -pwm; break;
	        default: break;
	    }
	    // set right side driver to GND
	    switch (R_CHN) {
	        case 1: R_TIM->CCR1 = 0; break;
	        case 2: R_TIM->CCR2 = 0; break;
	        case 3: R_TIM->CCR3 = 0; break;
	        case 4: R_TIM->CCR4 = 0; break;
	        default: break;
	    }
	}
}

void Motor::RampPwm(double dt){
	double delta = (double) RAMP_SPEED * dt;
	if(current_pwm_ < target_pwm_){
		if(current_pwm_ + delta > target_pwm_){current_pwm_ = target_pwm_;}
		else{current_pwm_ += delta;}
	}
	else if (current_pwm_ > target_pwm_){
		if(current_pwm_ - delta < target_pwm_){current_pwm_ = target_pwm_;}
		else{current_pwm_ -= delta;}
	}
	SetPwm(current_pwm_);
}
