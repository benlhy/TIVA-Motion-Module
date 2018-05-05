/*
 * motor.h
 *
 *  Created on: Apr 30, 2018
 *      Author: Ben
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "includes.h"
#include "qei.h"

extern volatile int desiredPos0;
extern volatile int desiredPos1;
extern volatile int desiredVel0;
extern volatile int desiredVel1;

extern void setSpeed(int pwm, int state, int motor); // 0 = brake, 1 = CCW, 2 = CW, 3 = STOP, motor 0 and 1
extern void PWMconfig(int period);


// Controllers
extern void PIDPosupdate(void);
extern void PIDVelupdate(void);




#endif /* MOTOR_H_ */
