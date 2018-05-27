/*
This file demonstrates the use of UART 
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
        sprintf(output,"This is the last number you typed in! Number: %d", number);
        uartWrite(output);

        // write output
        sprintf(output,"Type in a new number: ");
        uartWrite(output);

        //Wait for 1 second for user to type in a new number
        delayMS(1000); 

        // read into our char array
        uartRead(input,10);

        // process our char array
        sscanf(command,"%d",&number);


    }
}







