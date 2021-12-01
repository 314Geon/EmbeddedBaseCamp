/*
 * SST25VF016B.c
 *
 *  Created on: Nov 30, 2021
 *      Author: evg82
 */
#include "SST25VF016B.h"
#include "defines.h"

uint16_t CS_Pin = 0;

SPI_HandleTypeDef *handler;

void Flash_WriteStatus(uint8_t status) {
	uint8_t TX_buf[2] = {OP_ENWR_SR};
	CS_LOW;
	HAL_SPI_Transmit(handler, TX_buf, 1, 1000);
	CS_HIGH;
	TX_buf[0] = OP_WRITE_SR;
	TX_buf[1] = status;
	CS_LOW;
	HAL_SPI_Transmit(handler, TX_buf, 2, 1000);
	CS_HIGH;
}

void Flash_Lock() {
	Flash_WriteStatus(0x1C);
}

void Flash_Unlock() {
	Flash_WriteStatus(0x80);
}

void Flash_Init (uint16_t cs_pin, SPI_HandleTypeDef *handler_spi)
{
	handler = handler_spi;
	CS_Pin = cs_pin;
}

void Flash_Read(uint32_t addr, uint8_t *RX_buf, uint32_t count) {
	uint8_t TX_buf[4] = {OP_READ, (addr & 0xFFFFFF) >> 16, (addr & 0xFFFF) >> 8, (addr & 0xFF)};
	CS_LOW;
	HAL_SPI_Transmit(handler, TX_buf, 4, 1000);
	HAL_SPI_Receive(handler, RX_buf, count, 1000);
	CS_HIGH;
}

void Flash_WriteSet(uint8_t enable) {
	uint8_t TX_buf[2];
	TX_buf[0] = enable ? OP_WRITE_ENABLE : OP_WRITE_DISABLE;

	CS_LOW;
	HAL_SPI_Transmit(handler, TX_buf, 1, 1);
	CS_HIGH;
}


void Flash_Erase4k(uint32_t addr) {
	Flash_Unlock();
	Flash_WriteSet(1);
	uint8_t TX_buf[4] = {OP_ERASE_4K, (addr & 0xFFFFFF) >> 16, (addr & 0xFFFF) >> 8, (addr & 0xFF)};
	uint8_t RX_buf[2];
	CS_LOW;
	HAL_SPI_TransmitReceive(handler, TX_buf, RX_buf, sizeof(TX_buf), 1000);
	CS_HIGH;
	HAL_Delay(25);
	Flash_Lock();
}

void Flash_EraseAll() {
	Flash_Unlock();
	Flash_WriteSet(1);
	uint8_t TX_buf[2];
	uint8_t RX_buf[2];

	TX_buf[0] = OP_ERASE_ALL;
	CS_LOW;
	HAL_SPI_TransmitReceive(handler, TX_buf, RX_buf, sizeof(TX_buf), 1);
	CS_HIGH;
	HAL_Delay(50);
	Flash_Lock();
}

void Flash_Write(uint32_t addr, const uint8_t *write_buf, uint32_t count) {
	Flash_Unlock();
	for (int i = 0; i<count; i++) {
		Flash_WriteSet(1);
		uint32_t write_addr = addr+i;
		uint8_t TX_buf[5] = {OP_PROG_BYTE, (write_addr & 0xFFFFFF) >> 16, (write_addr & 0xFFFF) >> 8, (write_addr & 0xFF), write_buf[i]};
		CS_LOW;
		HAL_SPI_Transmit(handler, TX_buf, sizeof(TX_buf), 1000);
		CS_HIGH;
		HAL_Delay(1);
	}
	Flash_Lock();
}
