/*
 * SST25VF016B.h
 *
 *  Created on: Nov 30, 2021
 *      Author: evg82
 */
#include "stm32f4xx_hal.h"
#include <string.h>
#ifndef SST25VF016B_H_
#define SST25VF016B_H_



#endif /* SST25VF016B_H_ */

void Flash_Init (uint16_t cs_pin, SPI_HandleTypeDef *handler_spi);

void Flash_Read(uint32_t addr, uint8_t *RX_buf, uint32_t count);

void Flash_Erase4k(uint32_t addr);

void Flash_EraseAll();

void Flash_Write(uint32_t addr, const uint8_t *write_buf, uint32_t count);

