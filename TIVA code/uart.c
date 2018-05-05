/*
 * uart.c
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */

#include "uart.h"
#include "includes.h"

void uartInit(void){
    // Enable GPIO port A which is used for UART0 pins.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure UART0 pins on port A0 and A1.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    // Enable UART0 so that we can configure the clock.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Select the alternate (UART) function for these pins.
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                            (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                             UART_CONFIG_PAR_NONE));
}


void
UARTIntHandler(void)
{
    uint32_t ui32Status;
    char charBuf[15];
    if (config_flag == 1){
        // we are in configuration mode
        interrupt_counter = 1;
    }


    //
    // Get the interrrupt status.
    //
    ui32Status = UARTIntStatus(UART0_BASE, true);

    //
    // Clear the asserted interrupts.
    // We need to clear the flag otherwise it will not interrupt again
    UARTIntClear(UART0_BASE, ui32Status);

    //
    // Loop while there are characters in the receive FIFO.
    //
    int count = 0;
    while(UARTCharsAvail(UART0_BASE))
    {
        //
        // Read the next character from the UART and write it back to the UART.
        //


        char newchar = UARTCharGet(UART0_BASE);
        charBuf[count] = newchar;
        count++;

    }

    //sscanf(charBuf,"%d %d %d %d", &desiredPos0,&desiredPos1,&desiredVel0,&desiredVel1);
    //desiredPos0 = angleToPosition(desiredPos0);
    //desiredPos1 = angleToPosition(desiredPos1);
    //UARTprintf("%d\n",pwmSpeed); // send it back.

}


void uartRead(char * message, int maxLength)
{
    char data = 0;
    int complete = 0, num_bytes = 0;
    // loop until you get a '\r' or '\n'
    while (!complete)
    {
        data = UARTCharGet(UART0_BASE); // read char when it becomes available
        if ((data == '\n') || (data == '\r')) {
            complete = 1;
        }
        else
        {
            message[num_bytes] = data;
            ++num_bytes;
            // roll over if the array is too small
            if (num_bytes >= maxLength) {
                num_bytes = 0;
            }
        }
    }
    // end the string
    message[num_bytes] = '\0';
}

void uartWrite(const char * string)
{
    while (*string != '\0') {
        UARTCharPut(UART0_BASE, *string); // send the data
        ++string;
    }
}


