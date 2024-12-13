/*
 * heater_stateMachine.h
 *
 *  Created on: Dec 5, 2024
 *      Author: Alan Sanchez Rodriguez
 */

#ifndef INC_HEATER_STATEMACHINE_H_
#define INC_HEATER_STATEMACHINE_H_

#include "heater_enum.h"

void Tick_Heater (uint8_t CMD) {

	//----------TRANSITIONS----------
	switch (heaterState) {
		case OFF:
	  		  if (CMD == H_ON) {
	  			  heaterState = ON;

	  		  } else if (CMD == H_AUTO) {
	  			  heaterState = AUTO;
	  		  }

		break;

		case ON:
	  		  if (CMD == H_OFF) {
	  			  heaterState = OFF;
	  		  } else if (CMD == H_AUTO) {
	  			  heaterState = AUTO;
	  		  }

	  	break;

		case AUTO:
	  		  if (CMD == H_ON) {
	  			  heaterState = ON;
	  		  } else if (CMD == H_OFF) {
	  			  heaterState = OFF;
	  		  }

	  	break;
	}

	//------ACTIONS----------
	switch (heaterState) {
		case OFF:
			Heater_Off();
		break;

		case ON:
			Heater_On();
		break;

		case AUTO:
			Heater_Auto();
		break;
	}



}


#endif /* INC_HEATER_STATEMACHINE_H_ */
