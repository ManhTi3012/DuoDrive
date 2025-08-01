/*
 * motor_control.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */

#include "motor_control.h"

bool Motor::Init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right, TIM_TypeDef* pid_timer){
	if(channel_right > 4 || channel_left > 4){
		return false;
	}
	L_TIM = timer_left;
	L_CHN = channel_left;
	R_TIM = timer_right;
	R_CHN = channel_right;

	pid_timer_ = pid_timer;

	LoadConfig(default_motor);
	return true;
}

void Motor::InitEncoder(TIM_TypeDef *timer_encoder){
	has_encoder_ = true;
	encoder_port_ = timer_encoder;
}

void Motor::LoadConfig(MotorConfig config){
	this->config_ = config;

    pid.position.setGains(config_.position_kp, config_.position_ki, config_.position_kd);
    pid.position.setOutputLimits(-config_.max_velocity, config_.max_velocity);

    pid.velocity.setGains(config_.velocity_kp, config_.velocity_ki, config_.velocity_kd);
    pid.velocity.setOutputLimits(-MAX_DUTY, MAX_DUTY);
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

void Motor::ResetPos(){
	encoder_port_->CNT = 0;
	last_encoder_value_ = 0;
	encoder_position_ = 0;
}

void Motor::ResetPos(int64_t new_position){
	encoder_port_->CNT = 0;
	last_encoder_value_ = 0;
	encoder_position_ = new_position;
}

void Motor::SetMaxVel(double max_vel){
	config_.max_velocity = max_vel;
}

void Motor::SetAlpha(double alpha){
	config_.smoothing_alpha = alpha;
}

void Motor::ReverseEncoder(bool reverse){
	config_.reverse_encoder = reverse;
}

void Motor::SetARR(uint16_t arr){
	config_.pid_arr = arr;
}

void Motor::SetPScale(uint16_t pscale){
	config_.pos_scale = pscale;
}

void Motor::SetPPR(uint16_t ppr){
	config_.encoder_ppr = ppr;
}
void Motor::SetRampSpeed(uint16_t ramp_speed){
	config_.ramp_speed = ramp_speed;
}

double Motor::GetMaxVel(){
	return config_.max_velocity;
}

double Motor::GetAlpha(){
	return config_.smoothing_alpha;
}


uint16_t Motor::GetPScale(){
	return config_.pos_scale;
}

uint16_t Motor::GetARR(){
	return config_.pid_arr;
}

bool Motor::IsEncoderReversed(){
	return config_.reverse_encoder;
}

uint16_t Motor::GetPPR(){
	return config_.encoder_ppr;
}

uint16_t Motor::GetRampSpeed(){
	return config_.ramp_speed;
}

int64_t Motor::GetPosition(){
	if(has_encoder_){
		return encoder_position_;
	}
	else{
		return 0;
	}
}

int16_t Motor::GetPwm(){
	return current_pwm_;
}

double Motor::GetRPM(){
	if(has_encoder_){
		return encoder_speed_;
	}
	else{
		return 0;
	}
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

void Motor::PositionLoop(double dt){
	if(current_mode_ == POSITION){
		target_velocity_ = pid.position.calculate(target_pwm_, encoder_position_, dt);
	}
	else{
		return;
	}
}

void Motor::VelocityLoop(double dt){
	if(current_mode_ == OPEN_LOOP){
		RampPwm(dt);
	}
	else{
		target_pwm_ = pid.velocity.calculate(target_velocity_, encoder_speed_, dt);
		RampPwm(dt);
	}
}

void Motor::UpdateData(double dt){
	if(!has_encoder_){return;}
	else{
		uint16_t current_encoder_value = encoder_port_ -> CNT;
		int16_t delta = (int16_t)(current_encoder_value - last_encoder_value_);

	    if (config_.reverse_encoder) {
	        delta = -delta;
	    }
	    double raw_speed = static_cast<double>(delta) / dt;
	    raw_speed = raw_speed / config_.encoder_ppr * 60.0;

	    encoder_speed_ = (double) config_.smoothing_alpha * raw_speed + (1.0 - config_.smoothing_alpha) * encoder_speed_;

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
	double delta = (double) config_.ramp_speed * dt;
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
