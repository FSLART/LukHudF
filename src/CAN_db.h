/**
* @file CAN_db.h
* @brief CAN database header file
* @details This file contains the mapping used to extract to encode and decode CAN messages
* @author João Vieira
**/
#ifndef CAN_DB_H
#define CAN_DB_H

#ifdef __LART_T24__
	/**
	* @brief T24 uses a little endian architecture when it comes to CAN messages
	**/
	#define CAN_TEMPS_DRIVE 0x21
	/**======================================================================**/
	#define MAP_RPM(x) ((x[7] << 8 | x[6])) 
	#define MAP_MOTOR_TEMPERATURE(x) ((x[5] << 8 | x[4]) +40)
	#define MAP_INVERTER_TEMPERATURE(x) ((x[3] << 8 | x[2]) +40)
	#define MAP_INVERTER_VOLTAGE(x) ((x[1] << 8 | x[0]))
	/**======================================================================**/
#endif

#endif // CAN_DB_H