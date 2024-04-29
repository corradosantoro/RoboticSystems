/*
 * motor.c
 *
 *  Created on: Apr 22, 2024
 *      Author: corrado
 */

#include <stdlib.h>
#include "main.h"
extern TIM_HandleTypeDef htim4;

uint16_t last_counter = 0;
float current_position = 0;

// encoder + gearbox = 40000 ticks/revolution

uint16_t read_encoder(void)
{
	return TIM3->CNT;
}

void read_sensors(float dt, float * speed, float * pos)
{
	uint16_t cnt = read_encoder();
	int16_t dx = cnt - last_counter;
	last_counter = cnt;

	float delta_angle = dx * 360.0 / 40000.0;

	*speed = delta_angle/dt;
	current_position += delta_angle;
	*pos = current_position;
}

void set_pwm(int value)
{
	if (value >= 0) {
		HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 0);
		HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 1);
	}
	else {
		HAL_GPIO_WritePin(DIR_1_GPIO_Port, DIR_1_Pin, 1);
		HAL_GPIO_WritePin(DIR_2_GPIO_Port, DIR_2_Pin, 0);
	}
	TIM4->CCR1 = abs(value);
}
