# DC Control Module

## Todo
1. Change Timer 0 to Timer 1 because of PWM.

## Features
1. QEI mode counts: 2800
2. Rollover value: 14000

## Message Packet

### Mode
0. PID tuning
1. PID control
2. Lead-lag tuning
3. Lead-lag control

### PID control 
The user can send an angle in counts and the motor turns by the specified amount.

## Problems
* Not sure why when activating SEND and GET, there are periods where messages are dropped.
	* Turns out that having the same ID will cause a bus conflict
	* Issuing different IDs allow a much higher speed to be achieved (1Mbit/s)
* Not sure why resetting one before the other will cause messages to be dropped for the first but not for the other.
* Tuning might require a large amount of data to be set just because they are floats.
* How to write lead-lag control?