/*
 * system.c
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */
#include "system.h"

void delayMS(int ms){
    SysCtlDelay((SysCtlClockGet() * ms) / 3000);
}


void sysInit(void){
    // Set the clock to 80Mhz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
}

void initAll(void){
    sysInit();
    uartInit();
    FPULazyStackingEnable();
    FPUEnable();
    IntMasterEnable();
    PWMconfig();
    //GAINSconfig();
    QEIconfig(); // zero based
    QEIvelocityConfig();

    //IntEnable(INT_UART0);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    GPIOconfig();
    TIMERconfig();
    PULSEconfig();



    //PULSEorderconfig(); // infinite loop until signal received
    CANconfig();
    CANRXmsgconfig();

}

void GPIOconfig(void){
    HWREG(GPIO_PORTF_BASE+GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE+GPIO_O_CR) |= GPIO_PIN_0;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_3);


    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, GPIO_PIN_3); // AN1
    //GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4); // AN1


}
