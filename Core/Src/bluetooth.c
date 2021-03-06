/*
 * bluetooth.c
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#include "bluetooth.h"
#include "spices.h"
#include "flash.h"
#include "motors.h"

//spice_t spice_list[NUMCONTAINERS];
char buffer[50];
char pref[] = "YOU SENT: ";
uint8_t timer_count = 0, buffer_index = 0;
char low_spices[9];
uint8_t low_spice_index = 1;

// Bluetooth command bytes
//uint8_t DISPENSE = 0x01;
//uint8_t REGISTER = 0x02;
//uint8_t DELETE = 0x03;
//uint8_t REFILL = 0x04;
//uint8_t DISPENSE_SERIES = 0x05;
//uint8_t DELETE_ALL = 0x06;
//uint8_t LOW_SPICE = 0x07;

/**
 * BASIC COMMAND STRUCTURES
 *
 * DISPENSE
 * [0x01, <container number>, <units to dispense>]
 *
 * REGISTER
 * [0x02, <container number>, {name, max 24 bytes}]
 *
 * DELETE
 * [0x03, <container number>]
 *
 * REFILL
 * [0x04, <container number>]
 *
 * DISPENSE_SERIES
 * [0x05, <number of spices to dispense>, {<container number>, <units>, ...}]
 *
 */

void message_handler() {

//	if(buffer[0] == '1') {
//		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, SET);
//	} else if(buffer[0] == '0') {
//		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, RESET);
//	}
//
//	HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

	HAL_UART_Transmit(&huart2, (uint8_t*)pref, sizeof(pref), 500);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, sizeof(buffer), 500);

	char str[25];
	memset(str, 0, sizeof(str));

	char * done_str = "DONE\r\n";

	// Reset low spices
	memset(low_spices, 0xFF, sizeof(low_spices));
	low_spices[0] = LOW_SPICE;
	low_spice_index = 1;

	// Switch case for bluetooth commands
	switch (buffer[0]) {
	// Dispense
	int d;
	case DISPENSE:
		// Call to Noah's dispense function
		DispenseSpice(buffer[1], buffer[2]);
		// Update amount dispensed
		d = dispenseSpice(spice_list, buffer[1], buffer[2]);
		// TODO test low spice notification
		if(d > LOW_SPICE_THRESHOLD) {
			low_spices[low_spice_index] = buffer[1];
			low_spice_index += 1;
		}

		break;

		// Register - assumes container is free
	case REGISTER:
		// Make sure 25th character is 0x00
		buffer[26] = 0x00;
		strcpy(str, &buffer[2]);

		if (!addSpice(spice_list, buffer[1], str)) {
			// TODO add some error message
			break;
		}
		// If write successful, rewrite spice_list in flash
		flash_write();
		break;

		// Delete
	case DELETE:
		if (!removeSpice(spice_list, buffer[1])) {
			// TODO add error message
			break;
		}
		// If remove successful, rewrite spice_list in flash
		flash_write();
		break;

		// Refill
	case REFILL:
		if (!refillSpice(spice_list, buffer[1])) {
			// TODO error message
			break;
		}
		// Send 'done' message
		HAL_UART_Transmit(&huart2, (uint8_t*)done_str, sizeof(done_str), 500);
		flash_write();
		break;

		// Dispense series
	case DISPENSE_SERIES:
		for (int i = 0; i < buffer[1]; i++) {
			int c = buffer[2 + 2 * i];
			int a = buffer[3 + 2 * i];

			// Call to Noah's dispense function
			DispenseSpice(c, a);
			// Update amount dispensed
			d = dispenseSpice(spice_list, c, a);
			HAL_Delay(0);
			// TODO test low spice notification
			if(d > LOW_SPICE_THRESHOLD) {
				low_spices[low_spice_index] = buffer[1];
				low_spice_index += 1;
			}
		}
		flash_write();
		break;

		// Reset all spices
	case DELETE_ALL:
		initList(spice_list);
		flash_write();
		break;
	}

	// Send low spice array
	// TODO test low spice notification
	if(low_spices[1] != 0xFF) {
		low_spices[low_spice_index] = '\n';
		HAL_Delay(250);
		HAL_UART_Transmit(&huart2, (uint8_t*)low_spices, sizeof(low_spices), 500);
	}

	memset(buffer, 0, sizeof(buffer));

	buffer_index = 0;
	timer_count = 0;

}
