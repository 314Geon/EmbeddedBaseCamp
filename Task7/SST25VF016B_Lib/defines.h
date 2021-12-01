/*
 * defines.h
 *
 *  Created on: Dec 1, 2021
 *      Author: evg82
 */

#ifndef DEFINES_H_
#define DEFINES_H_



#endif /* DEFINES_H_ */
#define OP_READ 0x03
#define OP_READ_FAST 0x0B
#define OP_ERASE_4K 0x10
#define OP_ERASE_32K 0x52
#define OP_ERASE_64K 0xD8
#define OP_ERASE_ALL 0x60
#define OP_PROG_BYTE 0x02
#define OP_PROG_WORD 0xAD
#define OP_READ_SR 0x05
#define OP_ENWR_SR 0x50
#define OP_WRITE_SR 0x01
#define OP_WRITE_ENABLE 0x06
#define OP_WRITE_DISABLE 0x04
#define OP_READ_ID 0xAB
#define OP_JEDEC_ID 0x9F
#define OP_EN_BSY 0x70
#define OP_DIS_BSY 0x80

#define CS_LOW (HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_RESET))
#define CS_HIGH (HAL_GPIO_WritePin(GPIOD, CS_Pin, GPIO_PIN_SET))
