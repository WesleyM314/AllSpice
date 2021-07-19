/*
 * flash.h
 *
 *  Created on: Jul 19, 2021
 *      Author: lastl
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

extern SPI_HandleTypeDef hspi1;

extern uint8_t FLASH_READ;
extern uint8_t FLASH_WRITE_EN;
extern uint8_t FLASH_WRITE_DISABLE;
extern uint8_t FLASH_SECTOR_ERASE;
extern uint8_t FLASH_WRITE_PAGE;
extern uint8_t FLASH_READ_REG1;
extern uint8_t FLASH_READ_REG2;
extern uint8_t FLASH_READ_REG3;
extern uint8_t FLASH_CHIP_ERASE;
extern uint8_t FLASH_EXIT_QPI;

bool flash_read();
void flash_write();

#endif /* INC_FLASH_H_ */
