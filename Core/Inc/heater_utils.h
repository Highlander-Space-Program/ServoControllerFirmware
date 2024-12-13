/*
 * heater_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_HEATER_UTILS_H_
#define INC_HEATER_UTILS_H_

#include "stm32f0xx_hal.h"
#include "main.h"


void Heater_On() {
	HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_SET);
}

void Heater_Off() {
	HAL_GPIO_WritePin(HEATER_EN_GPIO_Port, HEATER_EN_Pin, GPIO_PIN_RESET);
}

void Heater_Auto() {
	//TODO build this function
}

double Get_Temperature(uint32_t adc_val) {
	// TODO: Test this
	double voltage = adc_val * (3.3 / 4096);
	return (voltage - 3.3) / 0.005;
}

#endif /* INC_HEATER_UTILS_H_ */
