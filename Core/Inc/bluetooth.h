/*
 * bluetooth.h
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "main.h"
#include <string.h>
#include <stdio.h>

extern char buffer[50];
extern uint8_t timer_count, buffer_index;
extern UART_HandleTypeDef huart2;

void message_handler();

#endif /* INC_BLUETOOTH_H_ */
