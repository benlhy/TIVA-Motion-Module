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
#include "gen_algo.h"

void delayMS(int ms);
void sysInit(void);
void initAll(void);
void GPIOconfig(void);





#endif /* SYSTEM_H_ */
