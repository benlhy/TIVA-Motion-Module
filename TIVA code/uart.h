/*
 * uart.h
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */

#ifndef UART_H_
#define UART_H_


extern volatile int config_flag;
extern volatile int interrupt_counter;

void uartInit(void);
void uartIntHandler(void);
void uartRead(char * message, int maxLength);
void uartWrite(const char * string);




#endif /* UART_H_ */
