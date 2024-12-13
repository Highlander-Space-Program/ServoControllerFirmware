/*
 * heater_enum.h
 *
 *  Created on: Dec 5, 2024
 *      Author: Alan Sanchez Rodriguez
 */

#ifndef INC_HEATER_ENUM_H_
#define INC_HEATER_ENUM_H_


//States of the heater
enum HEATER_STATE{
	  OFF = 0,
	  ON = 1,
	  AUTO = 2

} heaterState = OFF;


//commands for the heater
enum COMMANDS {
  H_ON = 0,
  H_OFF = 1,
  H_AUTO = 2

};



#endif /* INC_HEATER_ENUM_H_ */
