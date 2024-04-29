/*
 * pid.c
 *
 *  Created on: Apr 22, 2024
 *      Author: corrado
 */

#include "pid.h"

void pid_init(t_pid * pid, float kp, float ki, float kd, float sat)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->saturation = sat;
	pid->integral_term = 0;
	pid->error_old = 0;
	pid->in_saturation = 0;
}

float pid_evaluate(t_pid * pid, float delta_t, float target, float current)
{
	float error = target - current;

	if (!pid->in_saturation) {
		pid->integral_term += error * delta_t;
	}

	float deriv = (error - pid->error_old) / delta_t;

	pid->error_old = error;

	float output = pid->kp * error +
					pid->ki * pid->integral_term +
					pid->kd * deriv;

	if (output >= pid->saturation) {
		output = pid->saturation;
		pid->in_saturation = 1;
	}
	else if (output <= -pid->saturation) {
		output = -pid->saturation;
		pid->in_saturation = 1;
	}
	else
		pid->in_saturation = 0;

	return output;
}
