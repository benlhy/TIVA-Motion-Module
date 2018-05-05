#include "qei.h"

float counttoRPM(int count){
     return (float)count/(COUNT_PER_REV*TIME_TO_COUNT);
}

int angleToPosition(int angle){
    return (int)((float)angle/360.0*COUNT_PER_REV);
}

int positionToAngle(int position){
    return (int)((float)position/(float)COUNT_PER_REV*360);
}

int getMotor1Angle(){
    return (int)((float)getMotor1Counts()/(float)(ENCODER_COUNT_1-1)*360.0*ENCODER_COUNT_1/COUNT_PER_REV);
}

int getMotor1Counts(){
   int curr_pos = QEIPositionGet(QEI0_BASE);
   curr_pos = curr_pos - (ENCODER_COUNT_1-1)/2;
   return curr_pos;
}

void zeroMotor1(){
    QEIPositionSet(QEI0_BASE,(ENCODER_COUNT_1-1)/2);
}

int getMotor2Angle(){
    int some_val = (int)(((float)getMotor2Counts()/(float)(ENCODER_COUNT_2-1))*360.0*ENCODER_COUNT_2/COUNT_PER_REV);
    return some_val;

}

int getMotor2Counts(){
   int curr_pos = QEIPositionGet(QEI1_BASE);
   curr_pos = curr_pos - (ENCODER_COUNT_2-1)/2;
   return curr_pos;
}

void zeroMotor2(){
    QEIPositionSet(QEI1_BASE,(ENCODER_COUNT_2-1)/2);
}

void QEIconfig(){
    // QEI Config
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // We are using PD7 for PhB0, QEI module 0 phase B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC); // PC5 - A,PC6 - B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0); // QEI module 0 phase A
    GPIOPinConfigure(GPIO_PD7_PHB0); // QEI module 0 phase B

    //Set Pins to be PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1); // QEI module 1 phase A
    GPIOPinConfigure(GPIO_PC6_PHB1); // QEI module 1 phase B

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 |  GPIO_PIN_7);
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

    //Disable peripheral and int before configuration
    QEIDisable(QEI0_BASE);

    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), ENCODER_COUNT_1-1);
    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, (ENCODER_COUNT_1-1)/2);

    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), ENCODER_COUNT_2-1);
    // Enable the quadrature encoder.
    QEIEnable(QEI1_BASE);
    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI1_BASE, (ENCODER_COUNT_2-1)/2);
}

void QEIvelocityConfig(){
    QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,SysCtlClockGet()*TIME_TO_COUNT); // base, predivider, clock ticks to count for
    QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,SysCtlClockGet()*TIME_TO_COUNT);
    QEIVelocityEnable(QEI0_BASE);
    QEIVelocityEnable(QEI1_BASE);
    // QEIVelocityGet(QEI0_BASE); // returns the number of pulses captured in the time period
}
