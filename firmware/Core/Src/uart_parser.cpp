/*
 * uart_parser.c
 *
 *  Created on: Apr 29, 2025
 *      Author: Ti Manh
 */
#include "uart_parser.h"
#include "main.h"
#include <cstring>
#include <stdlib.h>
#include "motor_control.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern Motor motor1;
extern Motor motor2;

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

    if (strcmp(argvalue[0], "led") == 0) {
    	// test function, beware when using
        if (strcmp(argvalue[1], "on") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else if (strcmp(argvalue[1], "off") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
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

    else if (strcmp(argvalue[0], "POS") == 0) {
    	if (argcount == 2) {
    		printf("%f\r\n" , motor->GetPosition());
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

    else if (strcmp(argvalue[0], "PPR") == 0) {
    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetPPR());
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
    else if (strcmp(argvalue[0], "PIDP") == 0) {
    	if(argcount == 5){
            double kp = atof(argvalue[2]);
            double ki = atof(argvalue[3]);
            double kd = atof(argvalue[4]);

            motor->pid.position.setGains(kp, ki, kd);

            printf("M%d Position PID set: KP=%.3f, KI=%.3f, KD=%.3f\r\n",motor_id, kp, ki, kd);

            return true;
    	}
    	else if(argcount == 2){
    		double kp,ki,kd;
    		motor->pid.position.getGains(kp, ki, kd);
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

            motor->pid.velocity.setGains(kp, ki, kd);

            printf("M%d Velocity PID set: KP=%.3f, KI=%.3f, KD=%.3f\r\n",motor_id, kp, ki, kd);

            return true;
    	}
    	else if(argcount == 2){
    		double kp,ki,kd;
    		motor->pid.velocity.getGains(kp, ki, kd);
    		printf("%.3f %.3f %.3f\r\n", kp, ki, kd);
    		return true;
    	}
    	else{
        	uart_print("COMMAND PARSING FAILED: WRONG NUMBER OF ARGUMENT\r\n");
        	return false;
    	}
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
    return true;
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
    		printf("%f\r\n" , motor->GetPosition());
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
