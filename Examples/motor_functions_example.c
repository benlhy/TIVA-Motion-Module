/*
This file demonstrates the use of motor functions
*/
#include "includes.h" // include TIVA libraries
#include "system.h" // include written libraries


int main(void) {

    initAll(); // initialize all functions

    // set up a char array to store our output
    char output[50]; 

    // set up a char array to store our input
    char input[10];


    int number =0;
    while (1)
    {
        // write output
        sprintf(output,"Starting spin up to 30\%!\r\n");
        uartWrite(output);
        int i;

        // demonstrate set speed
        for (i=0;i<100;i++){
            sprintf(output,"System at %d\% of commanded value!\r\n",i);
            uartWrite(output);
            // setSpeed uses the following inputs:
            // pwm | motor number
            setSpeed(i*10,1);
            delayMS(50);
        }
        // stop motor
        sprintf(output,"Stop! Reading values now...\r\n");
        uartWrite(output);
        setSpeed(0,1);

        sprintf(output,"Counts \t Angle \r\n");
        uartWrite(output);
        // Read counts and angle output
        sprintf(output,"%d \t %d \r\n",getMotor1Counts(),getMotor1Angle());
        uartWrite(output);
        delayMS(1000);




    }
}







