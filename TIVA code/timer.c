/*
 * timer.c
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */
#include "timer.h"
#include "includes.h"
#include "gen_algo.h"
#include "motor.h"


uint32_t ledFlag = 1;

extern pid_values motor_pid1;
extern pid_values motor_pid2;
extern int desiredPos1;

void TIMERconfig(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/100); // activate every 1/100 of a second 120/120/100 = 0.001s
    IntEnable(INT_TIMER2A);
    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER2_BASE, TIMER_A);

}


void
Timer2IntHandler(void)
{
    //
    // Clear the timer interrupt.
    //
    TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

    //
    // Toggle the flag for the first timer.
    //
    ledFlag ^= 1;

    //
    // Use the flags to Toggle the LED for this timer
    //
    int pwm1 = pid_controller(&motor_pid1,desiredPos1,getMotor1Counts());
    setSpeed(pwm1,1);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ledFlag<<0);



}


