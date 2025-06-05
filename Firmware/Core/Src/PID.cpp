/*
 * PID.cpp
 *
 *  Created on: May 26, 2025
 *      Author: Ti Manh
 */
#include "PID.h"

double constrain(double input, double lowerLimit, double upperLimit) {
    if (input < lowerLimit) {
        return lowerLimit;
    } else if (input > upperLimit) {
        return upperLimit;
    } else {
        return input;
    }
}

void PID::setGains(double set_Kp, double set_Ki, double set_Kd){
	Kp = set_Kp;
	Ki = set_Ki;
	Kd = set_Kd;
}

void PID::setOutputLimits(double lowerLimit, double upperLimit){
	output_lim_[0] = lowerLimit;
	output_lim_[1] = upperLimit;
}

double PID::calculate(double setpoint, double actual, double dt){
	double error = setpoint - actual;

	// Proportional
	double P = Kp * error;

	// Integral (with anti-windup)
	integral_ += Ki * error * dt;
	integral_ = constrain(integral_, output_lim_[0], output_lim_[1]);

	// Derivative (filtered)
	double D = Kd * (error - prev_error_) / dt;
	prev_error_ = error;

	return constrain(P + integral_ + D, output_lim_[0], output_lim_[1]);
}

