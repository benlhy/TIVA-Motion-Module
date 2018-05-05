/*
 * system.h
 *
 *  Created on: May 5, 2018
 *      Author: Ben
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#include "can.h"
#include "motor.h"
#include "timer.h"
#include "uart.h"
#include "includes.h"
#include "qei.h"

void sysInit(void);
void initAll(void);
void GPIOconfig(void);





#endif /* SYSTEM_H_ */
