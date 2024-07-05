/************************************************************/
/************************************************************/
/********************* Driver : CAN	          ***************/
/********************* Layer  : HAL_Config    ***************/
/********************* Date   : 01 may 2022   ***************/
/********************* Version: V01           ***************/
/********************* Author : Ahmed Mostafa ***************/
/********************* Company: Aero          ***************/
/************************************************************/
/************************************************************/

/* Preprocessor File Guard */

#ifndef CAN_CONFIG_H_
#define CAN_CONFIG_H_

/* Select BaudRate of MCP2515 */
/* Note: the BaudRate of all nodes in the system connection 
 *       must be same.
 */

#define CAN_BAUD_RATE					CAN_BR_500KB_PER_SEC

/* BaudRate Options: */

#define CAN_BR_500KB_PER_SEC			1
#define CAN_BR_250KB_PER_SEC			2
#define CAN_BR_125KB_PER_SEC			3

/* Frame Type */
#define CAN_FRAME_TYPE					CAN_STANDARD_MESSAGE_TYPE

/* Frame Options */
#define CAN_STANDARD_MESSAGE_TYPE 		1
#define CAN_EXTENDED_MESSAGE_TYPE 		2
#endif