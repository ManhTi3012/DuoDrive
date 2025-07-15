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

extern Motor motor1;
extern Motor motor2;

bool parse(char *command) {
    char *argvalue[4];
    int argcount = 0;
    // split command to tokens
    char *token = strtok(command, " ");
    while (token != NULL && argcount < 4) {
    	argvalue[argcount++] = token;
        token = strtok(NULL, " ");
    }

    if (argcount == 0) return false;

    if (strcmp(argvalue[0], "led") == 0 && argcount > 1) {
        if (strcmp(argvalue[1], "on") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
        } else if (strcmp(argvalue[1], "off") == 0) {
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        }
    }
    else if (strcmp(argvalue[0], "PWM") == 0 && argcount > 1) {
    	motor_id = atoi(argvalue[1])
    	if (strcmp(argvalue[1], "1") == 0){
    		int speed = atoi(argvalue[2]);
    		motor1.OpenLoopMode(speed);
    	}
        int speed = atoi(argvalue[1]);
        motor1.OpenLoopMode(speed);
    }
    else if (strcmp(argvalue[0], "POS") == 0 && argcount > 1) {
        int target = atoi(argvalue[1]);
        motor1.PositionMode(target);
    }
    else if (strcmp(argvalue[0], "VEL") == 0 && argcount > 1) {
        int target = atoi(argvalue[1]);
        motor1.VelocityMode(target);
    }
    return true;
}
