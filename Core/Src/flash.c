/*
 * flash.c
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */


#include "main.h"
#include "flash.h"
#include "spices.h"
#include <stdbool.h>

uint8_t FLASH_READ = 0x03;
uint8_t FLASH_WRITE_EN = 0x06;
uint8_t FLASH_WRITE_DISABLE = 0x04;
uint8_t FLASH_SECTOR_ERASE = 0x20;
uint8_t FLASH_WRITE_PAGE = 0x02;
uint8_t FLASH_READ_REG1 = 0x05;
uint8_t FLASH_READ_REG2 = 0x35;
uint8_t FLASH_READ_REG3 = 0x15;
uint8_t FLASH_CHIP_ERASE = 0x60;
uint8_t FLASH_EXIT_QPI = 0xFF;

uint32_t addr = 0x00000000;
uint8_t valid;
uint8_t wip;

/**
 * Check if first byte of flash memory == 1, meaning a valid
 * spice list is saved there. If invalid, return false
 */
bool flash_read() {
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &FLASH_READ, 1, 1000);			// Read command
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, 3, 1000);	// Read address
	HAL_SPI_Receive(&hspi1, &valid, 1, 1000);		// Check valid bit

	if (valid != 0x01)
		return false;
	HAL_SPI_Receive(&hspi1, (uint8_t*) spice_list, sizeof(spice_list), 1000);// Read spice_list
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// TESTING
	char rxData[100];
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &FLASH_READ, 1, 1000);			// Read command
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, 3, 1000);	// Read address
	HAL_SPI_Receive(&hspi1, (uint8_t*) rxData, sizeof(rxData), 1000);// Read spice_list
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	return true;
}

/**
 * Write the current spice_list to flash memory
 */
void flash_write() {
	// Make sure qpi mode off
//	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
//	HAL_SPI_Transmit(&hspi1, (uint8_t*) &FLASH_EXIT_QPI, 1, 1000);
//	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// Write enable latch
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &FLASH_WRITE_EN, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// Erase chip
//	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
//	HAL_SPI_Transmit(&hspi1, &FLASH_CHIP_ERASE, 1, 1000);
//	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// Erase sector
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &FLASH_SECTOR_ERASE, 1, 1000);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, 3, 1000);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// Wait until wip bit cleared
	wip = 1;
	while (wip) {
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &FLASH_READ_REG1, 1, 1000);
		HAL_SPI_Receive(&hspi1, &wip, 1, 1000);
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
		wip = wip & 0x01;
	}

	// Write enable
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &FLASH_WRITE_EN, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	// Write valid byte and spice_list
	valid = 0x01;
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &FLASH_WRITE_PAGE, 1, 1000);// Page Program Command
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, 3, 1000);	// Write Address
	HAL_SPI_Transmit(&hspi1, &valid, 1, 1000);		// Write valid byte
	HAL_SPI_Transmit(&hspi1, (uint8_t *)spice_list, sizeof(spice_list), 1000);	// Write spice_list
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

	wip = 1;
	while (wip) {
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*) &FLASH_READ_REG1, 1, 1000);
		HAL_SPI_Receive(&hspi1, &wip, 1, 1000);
		HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);
		wip = wip & 0x01;
	}

	// TESTING
	char rxData[100];
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi1, &FLASH_READ, 1, 1000);			// Read command
	HAL_SPI_Transmit(&hspi1, (uint8_t*) &addr, 3, 1000);	// Read address
	HAL_SPI_Receive(&hspi1, (uint8_t*) rxData, sizeof(rxData), 1000);// Read spice_list
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, SET);

}

