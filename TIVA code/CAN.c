/*
 * CAN.c
 *
 *  Created on: May 1, 2018
 *      Author: Ben
 */

#include "CAN.h"
// GPIO

char first_node_flag = 0;
volatile int interrupt_counter=0;
volatile int config_flag = 1;
uint32_t txID=0;

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile bool g_bRXFlag = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************

// Reference
// https://sites.google.com/site/luiselectronicprojects/tutorials/tiva-tutorials/tiva-gpio/digital-input-with-interrupt
void PortDIntHandler(void){
    uint32_t status=0;
    status = GPIOIntStatus(GPIO_PORTD_BASE,true);
    GPIOIntClear(GPIO_PORTD_BASE, WIRE_GPIO_INT_PIN);
    if( (status & WIRE_GPIO_INT_PIN) == WIRE_GPIO_INT_PIN){
      //Then there was a pin4 interrupt
        interrupt_counter = interrupt_counter+1;
    }

    if( (status & GPIO_INT_PIN_5) == GPIO_INT_PIN_5){
      //Then there was a pin5 interrupt
    }

}


void
CANIntHandler(void)
{
    uint32_t ui32Status;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ui32Status = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ui32Status == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ui32Status = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);
        //////UARTprintf("Status: 0x%08X",ui32Status);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //

    else if(ui32Status == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        //g_ui32MsgCount++;

        //
        // Since the message was sent, clear any error flags.
        //
        g_bErrFlag = 0;
    }
    else if(ui32Status == 2)
    {
        //////UARTprintf("Interrupt reached!");
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 2, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 2);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ui32MsgCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
        //////UARTprintf("Okay done.\n");
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        ////UARTprintf("Errortime\n");
        //
        // Spurious interrupt handling can go here.
        //
    }
}


void PULSEorderconfig(){
    while(interrupt_counter==0){
        ; // do nothing. It will only change if the input GPIO is interrupted OR UART is triggered, then it will be 1.
    }
    ////UARTprintf("Okay, counting!\n");
    SysCtlDelay(100000);
    interrupt_counter = interrupt_counter+1; // increment our counter, this is our ID
    txID = interrupt_counter;
    ////UARTprintf("ID is: %d\n", interrupt_counter);
    PULSEsend();
    ////UARTprintf("Sending complete.\n");

}

void PULSEconfig(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE, WIRE_GPIO_PIN);  // make F4 an input
    GPIOPadConfigSet(GPIO_PORTD_BASE,WIRE_GPIO_PIN,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);   // enable F4's pullup, the drive strength won't affect the input
    GPIOIntTypeSet(GPIO_PORTD_BASE,WIRE_GPIO_PIN,GPIO_FALLING_EDGE);
    GPIOIntRegister(GPIO_PORTD_BASE,PortDIntHandler);
    GPIOIntEnable(GPIO_PORTD_BASE, WIRE_GPIO_INT_PIN);
}

void PULSEsend(){
    // config output here
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
    int i;
    uint32_t state=0;
    for(i=0;i<interrupt_counter;i++){
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2, state);
        SysCtlDelay(1000);
        state^=GPIO_PIN_2;
        GPIOPinWrite(GPIO_PORTD_BASE,GPIO_PIN_2, state);
        SysCtlDelay(1000);
    }
    GPIOIntDisable(GPIO_PORTD_BASE,WIRE_GPIO_INT_PIN); // We no longer need the interrupt
    config_flag=0; // no longer configuring
}


// get CAN messages and process them
void CANget(){
    // unsigned int uIdx;
    //
    // If the flag is set, that means that the RX interrupt occurred and
    // there is a message ready to be read from the CAN
    //
    if(g_bRXFlag)
    {
        // ////UARTprintf("Function reached\n");
        //
        // Reuse the same message object that was used earlier to configure
        // the CAN for receiving messages.  A buffer for storing the
        // received data must also be provided, so set the buffer pointer
        // within the message object.
        //
        sCANMessageRX.pui8MsgData = ui8MsgDataRX;

        //
        // Read the message from the CAN.  Message object number 2 is used
        // (which is not the same thing as CAN ID).  The interrupt clearing
        // flag is not set because this interrupt was already cleared in
        // the interrupt handler.
        //
        CANMessageGet(CAN0_BASE, 2, &sCANMessageRX, 0);

        //
        // Clear the pending message flag so that the interrupt handler can
        // set it again when the next message arrives.
        //
        g_bRXFlag = 0;

        if(sCANMessageRX.ui32Flags & MSG_OBJ_DATA_LOST)
        {
            ////UARTprintf("CAN message loss detected\n");
        }

        //////UARTprintf("Msg ID=0x%08X len=%u",
        //           sCANMessageRX.ui32MsgID, sCANMessageRX.ui32MsgLen);



        uint8_t device = ui8MsgDataRX[0];
        uint8_t mode = ui8MsgDataRX[1];
        uint16_t data1 = (ui8MsgDataRX[2]<<8)|ui8MsgDataRX[3]; // 16 bits
        uint16_t data2 = (ui8MsgDataRX[4]<<8)|ui8MsgDataRX[5];
        //uint16_t data2 = (ui8MsgDataRX[4]<<8)|ui8MsgDataRX[5]; // 16 bits
        if (device==2){
            // master is sending the message
            currMode = mode;
            if (mode==1){
                desiredAngle1 = data1;
                desiredAngle2 = data2;

            }
        }
        //////UARTprintf("RX, Device: %d, Mode: %d, Angle 1:%d, Angle 2:%d ",device,mode,data1,data2);
        ////UARTprintf(" total count=%u\n", g_ui32MsgCount);
    }


}

void CANsend() {

    //////UARTprintf("Sending msg: 0x%02X %02X %02X %02X",
    //           pui8MsgDataTX[0], pui8MsgDataTX[1], pui8MsgDataTX[2],
    //           pui8MsgDataTX[3]);
    //////UARTprintf("data is %d",ui32MsgDataTX);
    //

    // Send the CAN message using object number 1 (not the same thing as
    // CAN ID, which is also 1 in this example).  This function will cause
    // the message to be transmitted right away.
    CANMessageSet(CAN0_BASE, 1, &sCANMessageTX, MSG_OBJ_TYPE_TX);
    //
    // Check the error flag to see if errors occurred
    //
    if(g_bErrFlag)
    {
        ////UARTprintf(" error - cable connected?\n");
    }
    else
    {
        //
        // If no errors then print the count of message sent
        //
        //////UARTprintf(" total count = %u\n", g_ui32MsgCount);
    }

    //ui32MsgDataTX++;
    SysCtlDelay(500);
}

void CANRXmsgconfig(){
    if (txID==2){ // if I am the controller
        sCANMessageRX.ui32MsgID = 0;  // accept all incoming data
    }
    else { // if I am not controller
        sCANMessageRX.ui32MsgID = 2;  // accept only commands from 2
    }
    sCANMessageRX.ui32MsgIDMask = 0;
    sCANMessageRX.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
    sCANMessageRX.ui32MsgLen = 6;
    CANMessageSet(CAN0_BASE, 2, &sCANMessageRX, MSG_OBJ_TYPE_RX);

}

void CANTXmsgconfig(uint8_t *pui8MsgData){
    sCANMessageTX.ui32MsgID = txID;
    sCANMessageTX.ui32MsgIDMask = 0;
    sCANMessageTX.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
    sCANMessageTX.ui32MsgLen = sizeof(ui8MsgDataTX);
    sCANMessageTX.pui8MsgData = ui8MsgDataTX;
    //CANMessageSet(CAN0_BASE, 1, &sCANMessageTX, MSG_OBJ_TYPE_TX);

}

void CANconfig(){
    pui8MsgDataTX=(uint8_t *)&ui8MsgDataTX; // point it to ui32MsgData because CAN only deals in 8 bytes
        pui8MsgDataRX=(uint8_t *)&ui8MsgDataRX; // point it to ui32MsgData because CAN only deals in 8 bytes
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_6); // standby pin
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0); // HIGH = standby, LOW = active

    GPIOPinConfigure(GPIO_PB4_CAN0RX);
    GPIOPinConfigure(GPIO_PB5_CAN0TX);
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
    CANInit(CAN0_BASE);
    //CANRetrySet(CAN0_BASE,1);
    // In this example, the CAN bus is set to 500 kHz.
    if(CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000)==0){
        while(1){
            ////UARTprintf("Bit rate set error!");
        }
    }
    CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
    IntEnable(INT_CAN0);
    CANEnable(CAN0_BASE);
}

/*
 * This function sets the CAN TX message for the master to tell slave devices the angle to turn to
 */
void tellAngle(int device, int mode, int angle1, int angle2){

    ui8MsgDataTX[0] = device;
    ui8MsgDataTX[1] = mode;
    ui8MsgDataTX[2] = (angle1 & 0b1111111100000000)>>8; // first 8
    ui8MsgDataTX[3] = angle1 & 0b0000000011111111;
    ui8MsgDataTX[4] = (angle2 & 0b1111111100000000)>>8; // first 8
    ui8MsgDataTX[5] = angle2 & 0b0000000011111111;
    //////UARTprintf("Size of array is: %d", sizeof(ui8MsgDataTX));
    //////UARTprintf("device: %d, mode: %d, angle 1: %d, angle 2: %d",ui8MsgDataTX[0],ui8MsgDataTX[1],
    //           (ui8MsgDataTX[2]<<8)|ui8MsgDataTX[3],(ui8MsgDataTX[4]<<8)|ui8MsgDataTX[5]);

}

/*
 * This function sets up the order in which the controllers are ordered.
 */



