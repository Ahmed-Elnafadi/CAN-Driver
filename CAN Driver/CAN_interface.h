/************************************************************/
/************************************************************/
/********************* Driver : CAN	          ***************/
/********************* Layer  : HAL_interface ***************/
/********************* Date   : 01 may 2022   ***************/
/********************* Version: V01           ***************/
/********************* Author : Ahmed Mostafa ***************/
/********************* Company: Aero          ***************/
/************************************************************/
/************************************************************/

/* Preprocessor File Guard */

#ifndef CAN_INTERFACE_H_
#define CAN_INTERFACE_H_
#include "CAN_private.h"
/* APIs Definition */

void CAN_Init(void);
/**************************************************************************************************************/
void CAN_SelectOperationMode(uint8_t copy_u8Mode);
/**************************************************************************************************************/
void CAN_SelectBaudRate(void);
/**************************************************************************************************************/
void CAN_Transmit(uint8_t copy_u8Buffer, uint8_t copy_u8Priority, uCAN_MSG *copy_uCANMSGFrame);
/**************************************************************************************************************/
void CAN_Receive(uCAN_MSG *copy_ReceiveMessage);
/**************************************************************************************************************/

/* Macros Definition */

#define CAN_NORMAL_MODE				0
#define CAN_SLEEP_MODE				1
#define CAN_LOOP_BACK_MODE			2
#define CAN_LISTEN_ONLY_MODE		3
#define CAN_CONFIG_MODE				4
/****************************************************************/

#define CAN_TXB0_BUFFER				1
#define CAN_TXB1_BUFFER				2
#define CAN_TXB2_BUFFER				3

#define CAN_MESSAGE_HIGHEST_PRIORITY							0
#define CAN_MESSAGE_HIGH_INTERMEDIATE_PRIORITY					1
#define CAN_MESSAGE_LOWEST_INTERMEDIATE_PRIORITY				2
#define CAN_MESSAGE_LOWEST_PRIORITY								3
/****************************************************************/


/****************************************************************/
#endif
