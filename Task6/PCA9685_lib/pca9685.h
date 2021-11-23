/*
 * PCA9685.h
 *
 *  Created on: Nov 17, 2021
 *      Author: evg82
 */
#include "stm32f4xx_hal.h"
#ifndef PCA9685_H_
#define PCA9685_H_

void pca9685_Init (I2C_HandleTypeDef *handler);
void pca9685_SetDutyCycle (uint8_t channel, uint8_t duty_cycle, uint8_t delay_time);
void pca9685_DisableALL ();



#endif /* PCA9685_H_ */
