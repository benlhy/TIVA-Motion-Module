/*
 *
 *  Created on: May 1, 2018
 *      Author: Ben
 */

#include "motor.h"

int max_pwm;

uint8_t stateTable[4] = {
                         GPIO_PIN_0, // CW
                         GPIO_PIN_1, // CCW
                         GPIO_PIN_0+GPIO_PIN_1,//brake 1
                         0, // STOP
};


void setSpeed(int pwm, int motor){ // 0 to CW, // 1 to CCW // 2 to brake // 4 to stop
    if (motor==1){ // motor 1
        if (pwm>0){
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,pwm);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1,stateTable[0]);
        }
        else if (pwm<0) {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,-pwm);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1,stateTable[1]);
        }
        else {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,0);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1,stateTable[2]);
        }
    }
    else if (motor == 2){
        if (pwm>0){
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,pwm);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3,stateTable[0]<<2);
        }
        else if (pwm<0) {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,-pwm);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3,stateTable[1]<<2);
        }
        else {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,0);
            GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3,stateTable[2]<<2);
        }
    }
}



void PWMconfig(){



    SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);


    GPIOPinConfigure(GPIO_PF1_M1PWM5); //PF1
    GPIOPinConfigure(GPIO_PF2_M1PWM6); //PF2

    // Set pin types
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2); // yep
    // PWM configuration
    PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, MAX_PWM);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, MAX_PWM);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,0);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,0);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT , true);
}

extern absolute_data counts1;


float counttoRPM(int count){
     return (float)count/(COUNT_PER_REV*TIME_TO_COUNT);
}

int angleToPosition(int angle){
    return (int)((float)angle/360.0*COUNT_PER_REV);
}

int positionToAngle(int position){
    return (int)((float)position/(float)COUNT_PER_REV*360);
}

int getMotor1Velocity() {
   int currvel = QEIVelocityGet(QEI0_BASE);
   return currvel;
}

int getMotor1Angle(){
    return (int)((float)getMotor1Counts()/(float)(ENCODER_COUNT_1-1)*360.0*ENCODER_COUNT_1/COUNT_PER_REV);
}

int getMotor1Counts(){
   int curr_pos = QEIPositionGet(QEI0_BASE);
   curr_pos = curr_pos - (ENCODER_COUNT_1-1)/2;
   curr_pos = absolute_to_relative(curr_pos,&counts1);
   return curr_pos;
}

void zeroMotor1(){
    QEIPositionSet(QEI0_BASE,(ENCODER_COUNT_1-1)/2);
}

int getMotor2Velocity() {
   int currvel = QEIVelocityGet(QEI1_BASE);
   return currvel;
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





