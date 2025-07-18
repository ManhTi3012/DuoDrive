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
    char *argvalue[4];
    int argcount = 0;
    // split command to tokens
    char *token = strtok(command, " ");
    while (token != NULL && argcount < 6) {
    	argvalue[argcount++] = token;
        token = strtok(NULL, " ");
    }

    if (argcount == 0) return false;

    if (strcmp(argvalue[0], "led") == 0 && argcount > 1) {
    	// test function
        if (strcmp(argvalue[1], "on") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else if (strcmp(argvalue[1], "off") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
    }
    else if (strcmp(argvalue[0], "PWM") == 0 && argcount > 1) {
    	int motor_id = atoi(argvalue[1]);
    	Motor *motor = NULL;
    	if(motor_id == 1){motor = &motor1;}
    	else if(motor_id == 2){motor = &motor2;}
    	else {
    		uart_print("Motor id out of range\r\n");
    		return false;
    	}

    	if (argcount == 2) {
    		printf("%d\r\n" , motor->GetPwm());
        }
    	else {
            int speed = atoi(argvalue[2]);
            motor->OpenLoopMode(speed);
        }

    	return true;
    }

    else if (strcmp(argvalue[0], "POS") == 0 && argcount > 1) {
    	int motor_id = atoi(argvalue[1]);
    	Motor *motor = NULL;
    	if(motor_id == 1){motor = &motor1;}
    	else if(motor_id == 2){motor = &motor2;}
    	else {
    		uart_print("Motor id out of range\r\n");
    		return false;
    	}

    	if (argcount == 2) {
    		printf("%ld\r\n" , (long) motor->GetPosition());
        }
    	else {
            int target = atoi(argvalue[2]);
            motor->PositionMode(target);
        }

    	return true;
    }
    else if (strcmp(argvalue[0], "VEL") == 0 && argcount > 1) {
    	int motor_id = atoi(argvalue[1]);
    	Motor *motor = NULL;
    	if(motor_id == 1){motor = &motor1;}
    	else if(motor_id == 2){motor = &motor2;}
    	else {
    		uart_print("Motor id out of range\r\n");
    		return false;
    	}

    	if (argcount == 2) {
    		printf("%lf\r\n" , motor->GetRPM());
        }
    	else {
            int target = atoi(argvalue[2]);
            motor->VelocityMode(target);
        }
    	return true;
    }

    return true;
}
