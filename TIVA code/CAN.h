/*
 * CAN.h
 *
 *  Created on: May 1, 2018
 *      Author: Ben
 */

#ifndef CAN_H_
#define CAN_H_


#include "includes.h"
#include "motor.h"

#define MASTER 2 // rank of the master NODE

#define WIRE_GPIO_INT_PIN GPIO_INT_PIN_1
#define WIRE_GPIO_PIN GPIO_PIN_1

// CAN
tCANMsgObject sCANMessageTX;
uint32_t ui32MsgDataTX;
uint8_t *pui8MsgDataTX;
uint8_t ui8MsgDataTX[6];


tCANMsgObject sCANMessageRX;
uint32_t ui32MsgDataRX;
uint8_t *pui8MsgDataRX;
uint8_t ui8MsgDataRX[6];

extern volatile int config_flag;
extern uint32_t txID;

int currMode;
int desiredAngle1;
int desiredAngle2;

void CANIntHandler(void);

void PortDIntHandler(void);

extern volatile int interrupt_counter;
extern volatile uint32_t g_ui32MsgCount;
extern volatile bool g_bErrFlag;
extern volatile bool g_bRXFlag;


void PULSEconfig(void);
void CANconfig(void);

void CANsend();
void CANget();
void CANTXmsgconfig(uint8_t *pui8MsgData);
void CANRXmsgconfig();

// Pulse
void PULSEsend(void);
void PULSEorderconfig(void);

// Master-Slave communication over CAN
void tellAngle(int device,int mode,int angle1,int angle2);




#endif /* CAN_H_ */
