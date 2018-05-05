//*****************************************************************************
// This project aims to build a simple motor controller around the TM4C123GXL
//
//
//
// PID controller
// Combined PWM and QEI
// PC5 - A, PC6 - B, Module 1
// PD6 - A, PD7 - B, Module 0
// Give it the limit to count to before rolling over
// current motor is 200:1 ratio with 7 pulses per revolution
// 100 * 7 * 4 = 700 * 4 = 2800 counts/ revolution
// Multiply it by 5 so that you get 5 revolutions of counting = 2800 * 5 = 14000
// Position counter is reset on value reset or seeing an index pulse
// clocking on edges is 4x decoding mode.
//
//
// Change stack size: Arm Linker -> Basic options
//
// PWM control
// PWM5: PF1 motor 0
// PWM6: PF2 motor 1
//
// Direction pins
// PE0, PE1 - motor 0
// PE2, PE3 - motor 1
//
// Encoder pins
// PD6 - A, PD7 - B, Module 0
// PC5 - A, PC6 - B, Module 1
//
// CAN pins
// CAN reference: https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/p/657164/2416622#pi320098=3
// WARNING, CURRENT CAN DOES NOT WORK, fix here: https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/551204#pi320098=2
// PB5 TX
// PB4 RX
//
// Messages to send over can..
// We need to know how many controllers are hooked up
// Transmit on all UARTs of each controller
// B - Controller 1 - A
//          |
// B - Controller 2 - A
// Need to set:
// 1. Velocity/position control
// 2. Velocity/position to go to
//
//
/*
 * PULSE
 * D1 - in
 * D2 - out
 *
 *
 * CAN message format:
 * Negotiating, on boot
 *
 * Bits[0:3]
 *
 * 0 - No number
 * 1 ... 7 - assigned number
 *
 * For one that is connected to UART, set flag, it is number 2
 *
 * HOW DO WE DETECT NEIGHBOUR NODES?
 * - Add 1 wire. Network node that is connected to UART will send out a pulse. Next node will subtract one from pulse.
 * - What if not connected to UART?
 *
 *
 * MODE:
 * 0. PID tuning
 * 1. PID control
 * 2. Lead-lag tuning
 * 3. Lead-lag control
 */
//*****************************************************************************
#include "includes.h"
#include "system.h"

#define BUFFER_SIZE 50

int main(void) {

    initAll();

    SysCtlDelay(SysCtlClockGet());

    char buffer[BUFFER_SIZE];
    int counts = 0;

    while (1)
    {

        sprintf(buffer,"Motor 1 angle: %d Motor 2 angle: %d\r\n",getMotor1Counts(),getMotor2Counts());
        uartWrite(buffer);

        SysCtlDelay(100);


    }
}







