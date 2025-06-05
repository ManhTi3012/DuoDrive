/*
 * PID.h
 *
 *  Created on: May 26, 2025
 *      Author: Ti Manh
 */

#ifndef INC_PID_H_
#define INC_PID_H_

class PID {
public:
    double calculate(double setpoint, double actual, double dt);

    void setGains(double set_Kp, double set_Ki, double set_Kd);
    void setOutputLimits(double lowerLimit, double upperLimit);

private:
    double Kp, Ki, Kd;
    double integral_ = 0;
    double prev_error_ = 0;
    double output_lim_[2] = {-1e6, 1e6}; // Default: no limits
};

#endif /* INC_PID_H_ */
