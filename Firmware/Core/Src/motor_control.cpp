/*
 * motor_control.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */

#include "motor_control.h"

bool MOTOR::init(TIM_TypeDef *timer_left, uint8_t channel_left,TIM_TypeDef *timer_right, uint8_t channel_right){
	if(channel_right > 4 || channel_left > 4){
		return false;
	}
	L_TIM = timer_left;
	L_CHN = channel_left;
	R_TIM = timer_right;
	R_CHN = channel_right;
	return true;
}

void MOTOR::initEncoder(int16_t ppr,uint8_t port){
	hasEncoder = true;
	pulsePerRev = ppr;
	encoderPort = port;
}

void MOTOR::setPwm(int16_t pwm){
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

int64_t MOTOR::getPosition(){
	if(hasEncoder){
		return encoderPosition;
	}
	else{
		return 0;
	}
}

int32_t MOTOR::getRPM(){
	if(hasEncoder){
		return encoderSpeed;
	}
	else{
		return 0;
	}
}

void MOTOR::positionMode(){
	if(!hasEncoder){return;}
	else{

	}
}
void MOTOR::velocityMode(){

}
