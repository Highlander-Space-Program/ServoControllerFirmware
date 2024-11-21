/*
 * board_utils.h
 *
 *  Created on: Nov 20, 2024
 *      Author: zande
 */

#ifndef INC_BOARD_UTILS_H_
#define INC_BOARD_UTILS_H_

#include "stm32f0xx_hal.h"
#include "main.h"

void STATUS_IND_On() {
	HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_SET);
}

void STATUS_IND_Off() {
	HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, GPIO_PIN_RESET);
}

void STATUS_IND_Toggle() {
	HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
}

void WARN_IND_On() {
	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_SET);
}

void WARN_IND_Off() {
	HAL_GPIO_WritePin(WARN_IND_GPIO_Port, WARN_IND_Pin, GPIO_PIN_RESET);
}

void WARN_IND_Toggle() {
	HAL_GPIO_TogglePin(WARN_IND_GPIO_Port, WARN_IND_Pin);
}

#endif /* INC_BOARD_UTILS_H_ */
