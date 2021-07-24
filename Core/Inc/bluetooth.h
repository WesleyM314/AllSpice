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

// Command values
#define DISPENSE 0x01
#define REGISTER 0x02
#define DELETE 0x03
#define REFILL 0x04
#define DISPENSE_SERIES 0x05
#define DELETE_ALL 0x06
#define LOW_SPICE 0x07
// TODO determine appropriate low spice threshold
#define LOW_SPICE_THRESHOLD 28

extern char buffer[50];
extern uint8_t timer_count, buffer_index;
extern UART_HandleTypeDef huart2;

void message_handler();

#endif /* INC_BLUETOOTH_H_ */
