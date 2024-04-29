/*
 * motor.h
 *
 *  Created on: Apr 22, 2024
 *      Author: corrado
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

void set_pwm(int value);
uint16_t read_encoder(void);
void read_sensors(float dt, float * speed, float * pos);



#endif /* INC_MOTOR_H_ */
