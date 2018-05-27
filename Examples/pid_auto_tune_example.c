/*
This file demonstrates the use of the PID autotune function
*/
#include "includes.h" // include TIVA libraries
#include "system.h" // include written libraries
#include "menu.h" // include menu

pid_values motor_pid1; // sets up struct for PID
absolute_data counts1; // struct for absolute to relative


int main(void) {

    initAll(); // initialize all functions

    // The PID config is:
    // pid_config struct | P coefficient | I coefficient | D coefficient | maximum value to output | maximum value to integrate to

    pid_config(&motor_pid1,2.0,0.5,0,MAX_PWM,1000); 

    // The absolute to relative config is:
    // absolute_data | rollover value | rollover threshold | current value
    // The rollover value is when the absolute sensor resets to zero. The absolute_to_relative function adds this modifier each time a reset is detected. A threshold value is used to determine if a rollover has occured with the following formula: threshold = current value - previous value. This threshold is typically set to half of the rollover value

    absolute_to_relative_config(&counts1,14000,7000,getMotor1Counts());
    

    // The auto_tune inputs are:
    // pid struct | disturbance in counts | count function | set output function | zero motor function | motor number

    // This function assumes a certain form for the functions that it calls. The count function will return an integer, the zero function will set the encoder count to zero, and the set output function takes in a pwm value and a motor number and returns nothing.

    pid_auto_tune(&motor_pid1, 100, getMotor1Counts,setSpeed,zeroMotor1,1);

    // Send pwm = 0 to motor 1 to stop the motor if it is spinning
    setSpeed(0,1);





    while (1)
    {
        // start menu
        processUART();

    }
}







