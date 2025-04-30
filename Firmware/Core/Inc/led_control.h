/*
 * led_control.h
 *
 *  Created on: Apr 30, 2025
 *      Author: Ti Manh
 */

#ifndef INC_LED_CONTROL_H_
#define INC_LED_CONTROL_H_

#include "stm32g4xx_hal.h"

class LED {
public:
	bool init( GPIO_TypeDef* group, uint16_t gpio);
	void setCycleTime(int time);
	void tick();
	void tickper100ms();
private:
	int cycle_time;
	uint16_t gpio_pin;
	GPIO_TypeDef* gpio_group;
	int counter;
	uint32_t last_tick = 0;
};

#endif /* INC_LED_CONTROL_H_ */
