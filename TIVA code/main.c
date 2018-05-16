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
#include "menu.h"

pid_values motor_pid1;
pid_values motor_pid2;
pid_values motor_vel_pid1;
absolute_data counts1;
absolute_data counts2;

int main(void) {

    initAll();

    pid_config(&motor_pid1,2.0,0.5,0,MAX_PWM,1000); // sets up the PID struct
    pid_config(&motor_pid2,1.0,1.0,0,MAX_PWM,500);
    pid_config(&motor_vel_pid1,30.0,1.0,0,MAX_PWM,3000); // max output, max integration factor
    absolute_to_relative_config(&counts1,14000,7000,getMotor1Counts());
    //char output[50];
    pid_auto_tune(&motor_pid1, 100, MAX_PWM, 1000,getMotor1Counts,setSpeed,zeroMotor1,1);
    //int result = pid_test(&motor_pid1,500,getMotor1Counts,setSpeed,1);
    setSpeed(0,1);
    //sprintf(output,"Result is %d\r\n",result);
    //uartWrite(output);




    while (1)
    {

        //int pwm1 = pid_controller(&motor_pid1,500,getMotor1Counts());
        //sprintf(output,"Motor int: %f Curr value is: %d, Commanded value is: %d\r\n",motor_pid1.ki,getMotor1Counts(),pwm1);
        //uartWrite(output);
        //setSpeed(pwm1,1);




        //int pwm1vel = pid_controller(&motor_vel_pid1,30,getMotor1Velocity());
        //sprintf(output,"PWM is: %d, Curr value is: %d \r\n",pwm1vel,getMotor1Velocity());
        //uartWrite(output);





        processUART();

    }
}







