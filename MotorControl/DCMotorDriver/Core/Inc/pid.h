/*
 * pid.h
 *
 *  Created on: Apr 22, 2024
 *      Author: corrado
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float kp, ki, kd, saturation;
	float error_old;
	float integral_term;
	int in_saturation;
} t_pid;

void pid_init(t_pid * pid, float kp, float ki, float kd, float sat);
float pid_evaluate(t_pid * pid, float delta_t, float target, float current);

#endif /* INC_PID_H_ */
