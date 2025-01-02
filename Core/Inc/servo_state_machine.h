/*
 * servo_state_machine.h
 *
 *  Created on: Jan 2, 2025
 *      Author: brandonmarcus
 */

#ifndef INC_SERVO_STATE_MACHINE_H_
#define INC_SERVO_STATE_MACHINE_H_

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f0xx_hal.h" // Assuming this is your HAL header
#include "servo_utils.h"

#define HSP_SERVO_MIN_PULSE_WIDTH 500
#define HSP_SERVO_MAX_PULSE_WIDTH 2500
#define HSP_SERVO_PWM_PERIOD 20000
#define HSP_SERVO_WAIT_TIME 10000 // in milliseconds

// Forward declarations
typedef enum {
	ON_CLOSED = 0,
	OFF_CLOSED = 1,
	ON_OPEN = 2,
	OFF_OPEN = 3
} SERVO_STATE;

typedef enum {
	OPEN_SERVO = 0,
	CLOSE_SERVO = 1,
	CHECK_SERVO = 2
} SERVO_CMD;

struct Servo;
struct ServoData;

struct Servo {
  const char* pnid;
  uint8_t closed_deg;
  uint8_t open_deg;
  SERVO_STATE state;
  uint32_t last_tick_update; // Use HAL_GetTick() for tracking time
  const TIM_HandleTypeDef *timer;
  volatile uint32_t *ccr;
};

struct ServoData {
    uint32_t uid[3];
    char* pnid;
    uint8_t closed_deg;
    uint8_t open_deg;
};

//Function Prototypes
void Tick_SERVO (uint8_t cmd, struct Servo* servo);
struct Servo* construct_servo (const uint32_t board_uid[3], const TIM_HandleTypeDef *timer);

struct ServoData servo_lookup_table[] = {
    {{0x39002c, 0x48585314, 0x20373733}, "FV-02", 0, 45},
    {{0x1234, 0x4567, 0x6789}, "FV-03", 0, 135}
};

/**
  * @brief  The construct_servo function is used to create a servo.
  * @param  uid This universally unique identification is based on the
  * 		microcontroller on the servo board to allow for different
  * 		positions of each servo
  * @param  timer This is the timer used for the PWM as well as the 10
  * 		second counter which determines when to turn the servo off
  *
  * @retval struct Servo* returns a pointer to the constructed servo.
  */
struct Servo* construct_servo (const uint32_t board_uid[3], const TIM_HandleTypeDef *timer) {
	for (uint8_t i = 0; i < sizeof(servo_lookup_table) / sizeof(servo_lookup_table[0]); i++) {
		if (memcmp(board_uid, servo_lookup_table[i].uid, sizeof(uint32_t) * 3) == 0) {
            // uid found, allocate memory for struct Servo
            struct Servo *servo = (struct Servo*)malloc(sizeof(struct Servo));
            if (servo == NULL) {
                // Handle memory allocation failure
                return NULL;
            }

            // Initialize the members of the struct individually
            servo->pnid = servo_lookup_table[i].pnid;
            servo->closed_deg = servo_lookup_table[i].closed_deg;
            servo->open_deg = servo_lookup_table[i].open_deg;
            servo->state = ON_CLOSED;
            servo->last_tick_update = HAL_GetTick(); // Initialize with current tick
            servo->timer = timer;
            servo->ccr = &(timer->Instance->CCR1);

            return servo;
		}
	}

	return NULL;
}

void Tick_SERVO (uint8_t cmd, struct Servo* servo) {
	// Changing the states
	switch (cmd){
		case OPEN_SERVO:
			if (servo->state == ON_CLOSED) {
				servo->state = ON_OPEN;
				servo->last_tick_update = HAL_GetTick();
			}
			else if (servo->state == OFF_CLOSED) {
				servo->state = ON_OPEN;
				servo->last_tick_update = HAL_GetTick();
			}
			else if (servo->state == ON_OPEN) {/*Not Used*/}
			else if (servo->state == OFF_OPEN) {/*Not Used*/}
			break;
		case CLOSE_SERVO:
			if (servo->state == ON_CLOSED) {/*Not Used*/}
			else if (servo->state == OFF_CLOSED) {/*Not Used*/}
			else if  (servo->state == ON_OPEN) {
				servo->state = ON_CLOSED;
				servo->last_tick_update = HAL_GetTick();
			}
			else if  (servo->state == OFF_OPEN) {
				servo->state = ON_CLOSED;
				servo->last_tick_update = HAL_GetTick();
			}
			break;
	}

    // Check if 10 seconds have passed and turn off servo
    if ((HAL_GetTick() - servo->last_tick_update) >= HSP_SERVO_WAIT_TIME) {
        if (servo->state == ON_CLOSED) {
            servo->state = OFF_CLOSED;
        } else if (servo->state == ON_OPEN) {
            servo->state = OFF_OPEN;
        }
        // Reset the tick update time even if the servo is already off
        servo->last_tick_update = HAL_GetTick();
    }

	// acting based on the state
	switch (servo->state) {
		case ON_CLOSED:
			Turn_On_Servo ();
			STATUS_IND_On ();
			Actuate_Servo (servo->closed_deg, servo->timer);
			break;
		case OFF_CLOSED:
			Turn_Off_Servo ();
			STATUS_IND_Off ();
			break;
		case ON_OPEN:
			Turn_On_Servo ();
			STATUS_IND_On ();
			Actuate_Servo (servo->open_deg, servo->timer);
			break;
		case OFF_OPEN:
			Turn_Off_Servo ();
			STATUS_IND_Off ();
			break;
	}
}

#endif /* INC_SERVO_STATE_MACHINE_H_ */
