/*
 * led_control.cpp
 *
 *  Created on: Apr 30, 2025
 *      Author: Ti Manh
 */

#include "led_control.h"

bool LED::init( GPIO_TypeDef* group, uint16_t gpio){
	gpio_pin = gpio;
	gpio_group = group;

	return true;
}

void LED::setCycleTime(int time){
	cycle_time = time;
}

// set blink pattern for each led
// 0 = fully on, 10 = off, 11 = no control
// 1-9 = blink, smaller value blink faster
void LED::tick(){

	switch (cycle_time) {
		case 0:
			HAL_GPIO_WritePin(gpio_group, gpio_pin, GPIO_PIN_SET);
			break;
		case 10:
			HAL_GPIO_WritePin(gpio_group, gpio_pin, GPIO_PIN_RESET);
			break;
		case 11:
			break;
		default:
			if(counter > 0){
				counter -= 1;
			}
			else{
				HAL_GPIO_TogglePin(gpio_group, gpio_pin);
				counter = cycle_time;
			}
			break;
	}
}
void LED::tickper100ms(){
	if(HAL_GetTick() - last_tick > 100){
		tick();
		last_tick = HAL_GetTick();
	}
}


