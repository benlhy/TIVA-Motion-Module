/*
 * motor.h
 *
 *  Created on: Apr 30, 2018
 *      Author: Ben
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "includes.h"
#include "gen_algo.h"


// Definitions
#define ENCODER_COUNT_1 14000 // set up to count 5 revolutions
#define ENCODER_COUNT_2 14000
#define COUNT_PER_REV 2800
#define TIME_TO_COUNT 0.01 // for velocity prediction

#define MAX_PWM 3000



extern void setSpeed(int pwm, int motor); // 0 = brake, 1 = CCW, 2 = CW, 3 = STOP, motor 0 and 1
extern void PWMconfig();

extern float counttoRPM(int count);
extern int angleToPosition(int angle);
extern int positionToAngle(int position);

extern int getMotor1Velocity(void);
extern int getMotor1Angle(void);
extern int getMotor1Counts(void);
extern void zeroMotor1(void);

extern int getMotor2Velocity(void);
extern int getMotor2Angle(void);
extern int getMotor2Counts(void);
extern void zeroMotor2(void);

extern int positionToAngle(int position);
extern int angleToPosition(int angle);

extern void QEIconfig(void);
extern void QEIvelocityConfig(void);





#endif /* MOTOR_H_ */
