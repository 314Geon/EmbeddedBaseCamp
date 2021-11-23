/*
 * PCA9685.c
 *
 *  Created on: Nov 17, 2021
 *      Author: evg82
 */
#include <pca9685.h>

#define DEV_ADRRESS 0x80
#define I2C_TIMEOUT 1

#define MODE1_REG 			0x00
#define LED0_ON_L_REG 		0x06
#define ALL_LED_ON_L_REG 	0xfa
#define ALL_LED_ON_H_REG 	0xfb
#define ALL_LED_OFF_L_REG 	0xfc
#define ALL_LED_OFF_H_REG 	0xfd
#define PRESCALER_REG 		0xfe

I2C_HandleTypeDef *handler;

void pca9685_WriteData (uint8_t data[])
{
	uint8_t buff[2];

	for(int i = 0; i < 4; i++)
	{
		buff[0] = data[0] + i;
		buff[1] = data[i+1];
		HAL_I2C_Master_Transmit(handler, DEV_ADRRESS, buff, 2, I2C_TIMEOUT);
	}
}

void pca9685_Init (I2C_HandleTypeDef *handleri2c)
{
	handler = handleri2c;

	uint8_t data[2] = {MODE1_REG, 0x01};

	HAL_I2C_Master_Transmit(handler, DEV_ADRRESS, data, 2, I2C_TIMEOUT);
}

void pca9685_SetDutyCycle (uint8_t channel, uint8_t duty_cycle, uint8_t delay_time)
{
	uint16_t delay = (4096 * delay_time / 100);
	uint16_t duty = (4096 * duty_cycle / 100) + delay;

	uint8_t on_l = (delay & 0x00ff);
	uint8_t on_h = (delay & 0x0f00) >> 8u;

	uint8_t off_l = (duty & 0x00ff);
	uint8_t off_h = (duty & 0x0f00) >> 8u;

	uint8_t chanell_reg = LED0_ON_L_REG + channel * 4;

	uint8_t data[5] = {chanell_reg, on_l, on_h, off_l, off_h};

	pca9685_WriteData(data);
}
void pca9685_DisableALL ()
{
	uint8_t data [5] = {ALL_LED_ON_L_REG, 0x00, 0x00, 0x00, 0x00};
	pca9685_WriteData(data);
}
