/*
 * menu.c
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */

#include "menu.h"
#include "motor.h"
#include "control.h"


#define BUFFER_SIZE 50

extern absolute_data counts1;
extern absolute_data counts2;

extern pid_values motor_pid1;
extern pid_values motor_pid2;

extern int desiredPos1;
extern int desiredPos2;


void processUART(){

    char command[30];
    char buffer[50];
    uartRead(command, 1); // read the command
    switch(command[0]){
    case 'r': // reads the count
        sprintf(buffer,"Motor 1 counts: %d Motor 2 counts: %d\r\n",getMotor1Counts(),getMotor2Counts());
        uartWrite(buffer);
        break;
    case 'z': // zeros the counts
        counts1.modifier=0;
        counts2.modifier=0;
        zeroMotor1();
        zeroMotor2();
        sprintf(buffer,"Motor 1 angle: %d Motor 2 angle: %d\r\n",getMotor1Counts(),getMotor2Counts());
        uartWrite(buffer);
        break;
    case 'p': // sets a percentage of PWM
        uartRead(command, 10);
        int ppwm1;
        int ppwm2;
        sscanf(command,"%d %d",&ppwm1,&ppwm2);
        setSpeed(ppwm1,0);
        setSpeed(ppwm2,1);
        break;
    case 's': // zeros everything
        setSpeed(0,0);
        setSpeed(0,1);
        break;
    case 'g': // sets kp, ki, kd
        uartRead(command,30);
        int num;
        float kp;
        float ki;
        float kd;
        sscanf(command,"%d %f %f %f",&num,&kp,&ki,&kd);
        if (num == 0){
            motor_pid1.kp=kp;
            motor_pid1.ki=ki;
            motor_pid1.kd=kd;
        }
        else if (num == 1){
            motor_pid2.kp=kp;
            motor_pid2.ki=ki;
            motor_pid2.kd=kd;
        }
        break;

    case 'f': // sets motors to go to position
        uartRead(command,30);
        sscanf(command, "%d %d",&desiredPos1,&desiredPos2);
        break;

    default :
        sprintf(buffer,"%c\r\n",command[0]);
        uartWrite(buffer);
        break;

    }
}


