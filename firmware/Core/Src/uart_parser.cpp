/*
 * uart_parser.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */
#include "uart_parser.h"
#include "flash_store_data.h"
#include "main.h"
#include <cstring>
#include <stdlib.h>
#include "motor_control.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern Motor motor1;
extern Motor motor2;
extern void save_config_to_flash();

void uart_print(const char *msg) {
    HAL_UART_Transmit(&huart3, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

bool parse(char *command) {
    char *argvalue[6];
    int argcount = 0;
    // split command to tokens
    char *token = strtok(command, " ");
    while (token != NULL && argcount < 6) {
    	argvalue[argcount++] = token;
        token = strtok(NULL, " ");
    }

    if (argcount <= 1){
    	uart_print("COMMAND PARSING FAILED: NOT ENOUGH ARGUMENT\r\n");
    	return false;
    }

	int motor_id = atoi(argvalue[1]);
	Motor *motor = NULL;
	if(motor_id == 1){motor = &motor1;}
	else if(motor_id == 2){motor = &motor2;}
	else {
		uart_print("Motor id out of range\r\n");
		return false;
	}


    if (strcmp(argvalue[0], "POS") == 0) {
    	if (argcount == 2) {
    		printf("%lld\r\n" , motor->GetPosition());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Position Set:%d\r\n",motor_id, target);
            motor->PositionMode(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}

    	return true;
    }
    else if (strcmp(argvalue[0], "VEL") == 0) {
    	if (argcount == 2) {
    		printf("%f\r\n" , motor->GetRPM());
        }
    	else if (argcount == 3){
            double target = atof(argvalue[2]);
            motor->VelocityMode(target);
            printf("M%d Velocity Set:%.3f\r\n",motor_id, target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }
    else if (strcmp(argvalue[0], "PWM") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetPwm());
        }
    	else if (argcount == 3){
            int speed = atoi(argvalue[2]);
            printf("M%d PWM Set:%d\r\n",motor_id, speed);
            motor->OpenLoopMode(speed);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}

    	return true;
    }

    else if (strcmp(argvalue[0], "ZERO") == 0) {
    	if (argcount == 2) {
    		motor->ResetPos();
    		printf("M%d Encoder Pos Set: 0\r\n",motor_id);
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            motor->ResetPos(target);
            printf("M%d Encoder Pos Set: %d\r\n",motor_id, target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "MVEL") == 0) {
    	if (argcount == 2) {
    		printf("%lf\r\n" , motor->GetMaxVel());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Maximum Velocity Set:%d\r\n",motor_id, target);
            motor->SetMaxVel(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }
    else if (strcmp(argvalue[0], "ALP") == 0) {
    	if (argcount == 2) {
    		printf("%lf\r\n" , motor->GetAlpha());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Smoothing Alpha Set:%d\r\n",motor_id, target);
            motor->SetAlpha(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "RVS") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->IsEncoderReversed());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Reverse Encoder:%d\r\n",motor_id, target);
            motor->ReverseEncoder(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "ARR") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetARR());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d PID ARR Set:%d\r\n",motor_id, target);
            motor->ReverseEncoder(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "VSCL") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetVScale());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d VPID ARR Set:%d\r\n",motor_id, target);
            motor->SetVScale(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }
    else if (strcmp(argvalue[0], "PSCL") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetPScale());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d PPID ARR Set:%d\r\n",motor_id, target);
            motor->SetPScale(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "PPR") == 0) {
    	if (argcount == 2) {
    		printf("%ld\r\n" , motor->GetPPR());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Encoder PPR Set:%d\r\n",motor_id, target);
            motor->SetPPR(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "RAMP") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetRampSpeed());
        }
    	else if (argcount == 3){
            int target = atoi(argvalue[2]);
            printf("M%d Ramp Speed Set:%d\r\n",motor_id, target);
            motor->SetRampSpeed(target);
        }
    	else{
        	uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
        	return false;
    	}
    	return true;
    }

    else if (strcmp(argvalue[0], "PIDP") == 0) {
    	if(argcount == 5){
            double kp = atof(argvalue[2]);
            double ki = atof(argvalue[3]);
            double kd = atof(argvalue[4]);

            motor->SetGainP(kp, ki, kd);

            printf("M%d Position PID set: KP=%.3f, KI=%.3f, KD=%.3f\r\n",motor_id, kp, ki, kd);

            return true;
    	}
    	else if(argcount == 2){
    		double kp,ki,kd;
    		motor->GetGainP(kp, ki, kd);
    		printf("%.3f %.3f %.3f\r\n", kp, ki, kd);
    		return true;
    	}
    	else{
        	uart_print("COMMAND PARSING FAILED: WRONG NUMBER OF ARGUMENT\r\n");
        	return false;
    	}
    }

    else if (strcmp(argvalue[0], "PIDV") == 0) {
    	if(argcount == 5){
    		double kp = atof(argvalue[2]);
    		double ki = atof(argvalue[3]);
    		double kd = atof(argvalue[4]);

            motor->SetGainV(kp, ki, kd);

            printf("M%d Velocity PID set: KP=%.3f, KI=%.3f, KD=%.3f\r\n",motor_id, kp, ki, kd);

            return true;
    	}
    	else if(argcount == 2){
    		double kp,ki,kd;
    		motor->GetGainV(kp, ki, kd);
    		printf("%.3f %.3f %.3f\r\n", kp, ki, kd);
    		return true;
    	}
    	else{
        	uart_print("COMMAND PARSING FAILED: WRONG NUMBER OF ARGUMENT\r\n");
        	return false;
    	}
    }

    else if (strcmp(argvalue[0], "INFO") == 0) {
        uart_print("COMMAND PARSING FAILED: TOO MUCH ARGUMENT\r\n");
    	return true;
    }
    else if (strcmp(argvalue[0], "SAVE") == 0) {
    	MotorConfig m1 = motor1.GetConfig();
    	MotorConfig m2 = motor2.GetConfig();

    	motor1.GetGainP(m1.position_kp, m1.position_ki, m1.position_kd);
    	motor1.GetGainV(m1.velocity_kp, m1.velocity_ki, m1.velocity_kd);

    	motor2.GetGainP(m2.position_kp, m2.position_ki, m2.position_kd);
    	motor2.GetGainV(m2.velocity_kp, m2.velocity_ki, m2.velocity_kd);

    	current_config.motor1 = m1;
    	current_config.motor2 = m2;
    	save_config_to_flash();
    	uart_print("CONFIG SAVED\r\n");
    	return true;
    }
    uart_print("WRONG COMMAND\r\n");
    return false;
}
bool machineParse(char *command) {
    char *argvalue[3];
    int argcount = 0;
	Motor *motor = NULL;
	char cmd;
    // split command to tokens
    char *token = strtok(command, " ");
    while (token != NULL && argcount < 6) {
    	argvalue[argcount++] = token;
        token = strtok(NULL, " ");
    }

    if (strlen(argvalue[0]) == 2) {
        int motor_id = argvalue[0][1] - '0';
        cmd = argvalue[0][0];
    	if(motor_id == 1){motor = &motor1;}
    	else if(motor_id == 2){motor = &motor2;}
    	else {
    		uart_print("Motor id out of range\r\n");
    		return false;
    	}
    }
    else{
    	return false;
    }

    if (cmd == 'P') {
    	if (argcount == 1) {
    		printf("%lld\r\n" , motor->GetPosition());
        }
    	else if (argcount == 2){
            float target = atof(argvalue[1]);
            motor->PositionMode(target);
        }
    	else{
        	uart_print("E1\r\n");
        	return false;
    	}
    	return true;
    }
    else if (cmd == 'V') {
    	if (argcount == 1) {
    		printf("%f\r\n" , motor->GetRPM());
        }
    	else if (argcount == 2){
            float target = atof(argvalue[1]);
            motor->VelocityMode(target);
        }
    	else{
        	uart_print("E1\r\n");
        	return false;
    	}
    	return true;
    }
    else if (cmd == 'S') {
    	if (argcount == 1) {
    		printf("%d\r\n" , motor->GetPwm());
        }
    	else if (argcount == 2){
            int target = atoi(argvalue[1]);
            motor->OpenLoopMode(target);
        }
    	else{
        	uart_print("E1\r\n");
        	return false;
    	}
    	return true;
    }

    uart_print("E2\r\n");
    return false;
}
