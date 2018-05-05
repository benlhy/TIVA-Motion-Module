/*
 *
 *  Created on: May 1, 2018
 *      Author: Ben
 */

#include "motor.h"

uint8_t stateTable[4] = {
                         GPIO_PIN_0, // CW
                         GPIO_PIN_1, // CCW
                         GPIO_PIN_0+GPIO_PIN_1,//brake 1
                         0, // STOP
};


volatile int currPosition0;
int dir0 = 1;
int pwmout0 = 0;
volatile int currPosition1;
int dir1 = 1;
int pwmout1 = 0;

// Velocity

volatile int currVel0;
int dirv0 = 1;
int pwmoutv0 = 0;
volatile int currVel1;
int dirv1 = 1;
int pwmoutv1 = 0;

// Variables
volatile int pwmSpeed=1;
volatile int desiredPos0=0;
volatile int desiredPos1=0;
volatile int motornum = 1;

volatile int desiredVel0=0;
volatile int desiredVel1=0;


// PID position gains
float Kp0 = 1;
float Kd0 = 0.1;
float Ki0 = 0.15;

float Kp1 = 1;
float Kd1 = 0.1;
float Ki1 = 0.15;


// Variables for 0
float error0 = 0;
int eprev0 = 0;
float ediv0 = 0;
float eint0 = 0;
int controlsig0 = 0;

// Variables for 1
float error1 = 0;
int eprev1 = 0;
float ediv1 = 0;
float eint1 = 0;
int controlsig1 = 0;

// PID velocity gains
float Kpv0 = 1;
float Kdv0 = 0.1;
float Kiv0 = 0.15;

float Kpv1 = 1;
float Kdv1 = 0.1;
float Kiv1 = 0.15;

// Variables for 0
float errorv0 = 0;
int eprevv0 = 0;
float edivv0 = 0;
float eintv0 = 0;
int controlsigv0 = 0;

// Variables for 1
float errorv1 = 0;
int eprevv1 = 0;
float edivv1 = 0;
float eintv1 = 0;
int controlsigv1 = 0;


void setSpeed(int pwm, int state, int motor){ // 0 to CW, // 1 to CCW // 2 to brake // 4 to stop
    if (motor==0){ // motor 1
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,pwm);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1,stateTable[state]);
    }
    else if (motor == 1){
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,pwm);
        GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2|GPIO_PIN_3,stateTable[state]<<2);
    }
}



void PWMconfig(int period){

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

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, period);
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, period);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,100);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,100);

    PWMGenEnable(PWM1_BASE, PWM_GEN_2);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT , true);
}



void PIDVelupdate(){

    currVel1 = QEIVelocityGet(QEI1_BASE);
    currVel0 = QEIVelocityGet(QEI0_BASE);

    errorv0 = (desiredVel0 - currVel0); // get current error
    eintv0 = eintv0 + errorv0; // integrate up the error
    edivv0 = errorv0-eprevv0; // no need to divide by time since you are multiplying by a constant.
    controlsigv0 = Kp0*errorv0+Kd0*edivv0+Ki0*eintv0;

    if (eintv0>200){
        eintv0 = 200;
    }
    else if(eintv0<-200){
        eintv0 = -200;
    }

    if (controlsigv0>100) {
        pwmoutv0 = 100;
        dirv0 = 0; // forward full speed
    }
    else if(controlsigv0<-100) {
        dirv0 = 1;
        pwmoutv0 = 100; // backwards full speed
    }
    else if((controlsigv0<0) && (controlsigv0>-100)){
        pwmoutv0 = -1*controlsigv0; // backwards at controlsig
        dirv0 = 1;
    }
    else if ((controlsigv0>0)&&(controlsigv0<100)){
        pwmoutv0 = controlsigv0; // forwards at controlsig
        dirv0 = 0;
    }

    errorv1 = (desiredVel1 - currVel1); // get current error
    eintv1 = eintv1 + errorv1; // integrate up the error
    edivv1 = errorv1-eprevv1; // no need to divide by time since you are multiplying by a constant.
    controlsigv1 = Kp1*errorv1+Kd1*edivv1+Ki1*eintv1;

    if (eintv1>200){
        eintv1 = 200;
    }
    else if(eintv1<-200){
        eintv1 = -200;
    }

    if (controlsigv1>100) {
        pwmoutv1 = 100;
        dirv1 = 0; // forward full speed
    }
    else if(controlsigv1<-100) {
        dirv1 = 1;
        pwmoutv1 = 100; // backwards full speed
    }
    else if((controlsigv1<0) && (controlsigv1>-100)){
        pwmoutv1 = -1*controlsigv1; // backwards at controlsig
        dirv1 = 1;
    }
    else if ((controlsigv1>0)&&(controlsigv1<100)){
        pwmoutv1 = controlsigv1; // forwards at controlsig
        dirv1 = 0;
    }
    eprev0 = errorv0; // update previous error
    eprev1 = errorv1; // update previous error

}

void PIDPosupdate(){
    currPosition0 = getMotor1Counts();
    currPosition1 = getMotor2Counts();


    error0 = (desiredPos0 - currPosition0); // get current error
    eint0 = eint0 + error0; // integrate up the error
    ediv0 = error0-eprev0; // no need to divide by time since you are multiplying by a constant.
    controlsig0 = Kp0*error0+Kd0*ediv0+Ki0*eint0;

    if (eint0>200){
        eint0 = 200;
    }
    else if(eint0<-200){
        eint0 = -200;
    }

    if (controlsig0>100) {
        pwmout0 = 100;
        dir0 = 0; // forward full speed
    }
    else if(controlsig0<-100) {
        dir0 = 1;
        pwmout0 = 100; // backwards full speed
    }
    else if((controlsig0<0) && (controlsig0>-100)){
        pwmout0 = -1*controlsig0; // backwards at controlsig
        dir0 = 1;
    }
    else if ((controlsig0>0)&&(controlsig0<100)){
        pwmout0 = controlsig0; // forwards at controlsig
        dir0 = 0;
    }

    error1 = (desiredPos1 - currPosition1); // get current error
    eint1 = eint1 + error1; // integrate up the error
    ediv1 = error1-eprev1; // no need to divide by time since you are multiplying by a constant.
    controlsig1 = Kp1*error1+Kd1*ediv1+Ki1*eint1;

    if (eint1>200){
        eint1 = 200;
    }
    else if(eint1<-200){
        eint1 = -200;
    }

    if (controlsig1>100) {
        pwmout1 = 100;
        dir1 = 0; // forward full speed
    }
    else if(controlsig1<-100) {
        dir1 = 1;
        pwmout1 = 100; // backwards full speed
    }
    else if((controlsig1<0) && (controlsig1>-100)){
        pwmout1 = -1*controlsig1; // backwards at controlsig
        dir1 = 1;
    }
    else if ((controlsig1>0)&&(controlsig1<100)){
        pwmout1 = controlsig1; // forwards at controlsig
        dir1 = 0;
    }
    eprev0 = error0; // update previous error
    eprev1 = error1; // update previous error

}



