# TIVA-Motion-Module
DC motor control with TM4C123. Each board controls 2 motors independently and can talk to other boards

# Use Cases
* A base module for a differential drive robot.
* 

# Features
* TM4C123 MCU
* TB6612FNG motor controller
* 2 channel encoder interface
* 2 channel 1A continuous, 1.3A max
* CAN Bus interface
* UART interface
* PID control library

# PID control library
The PID control library incorporates the use of an autotuning mechanism using the Nelder-Mead method whereby it searches through R3 space using a simplex model to reduce the scoring function, which in this case is the time it takes to settle.

There are some drawbacks using the Nelder-Mead method because the quality of the search depends on the initial values.

## Modifiable Values
The modifiable values for the PID control library are:
* **gen_algo.c**->testArray ~ This array stores 4 initial points to search
* **gen_algo.h**->TEST_PERIOD ~ This value is the length of time in cycles to measure the PID response
* **motor.h**->MAX_PWM ~ the maximum PWM counts. The range of PWM is from 0 to this value
* **motor.h**->TIME_TO_COUNT ~ the time to count the velocity. The encoder interface will wait this length of time to determine the velocity

## Function Descriptions
### Gen_Algo
* **absolute_to_relative**: takes a value from an absolute sensor (orientation/counts) and converts it to a relative value. 
* **pid_controller**: takes a sensed value and attempts to match it with a given desired value. Includes testing and autotuning algorithms.
* 

### Motor
* **setSpeed**: sets the speed of the motor based on the pwm counts given.
* **getMotorXCounts**: gets the counts from the encoder interface, note that absolute_to_relative function is already applied here
* **getMotorXVelocity**: gets the velocity in counts/time from the QEI interface
* **zeroMotorX**: zeros the motors and the relative counts


