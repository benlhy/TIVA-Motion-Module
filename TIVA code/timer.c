/*
 * timer.c
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */
#include "timer.h"
#include "includes.h"

uint32_t ledFlag = 1;


void TIMERconfig(void){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet()/4); // activate every 1/2 of a second 120/120/2 = 0.5s
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
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0, ledFlag<<0);

}


