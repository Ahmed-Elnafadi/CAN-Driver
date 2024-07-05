/************************************************************/
/************************************************************/
/********************* Driver : CAN	          ***************/
/********************* Layer  : HAL_Program   ***************/
/********************* Date   : 01 may 2022   ***************/
/********************* Version: V01           ***************/
/********************* Author : Ahmed Mostafa ***************/
/********************* Company: Aero          ***************/
/************************************************************/
/************************************************************/


#include "stm32f1xx_hal.h"
#include "MCP2515_interface.h"
#include "MCP2515_private.h"
#include "MCP2515_config.h"
#include "CAN_interface.h"
#include "CAN_private.h"
#include "CAN_config.h"

void CAN_SelectOperationMode(uint8_t copy_u8Mode){
   /* API Information:
	* This API is used to determine which mode, the MCP2515 will be on.
	* There are Five Modes in MCP2515:
	* 1- Normal Mode: At which we can transmit and receive messages by TXCAN and RXCAN.
	* 2- Sleep Mode:  At which the MCP2515 will in silent mode and the oscillator will be off.
	* 	 In this mode the MCP2515 monitors the RXCAN, if any message comes the wake-up interrupt will change this mode
	*    to the listen-only mode. TXCAN isn't working in the sleep mode. The first message comes on RXCAN to wake-up the
	*	 the system will be ignored.
	* 3- Loop-Back Mode: This mode is used to test and debug the MCP2515, it put the message loaded in the TX-Buffer in RX-buffer.
	*    So, the TXCAN and RXCAN are not used in this mode.
	* 4- Listen-only Mode: This mode is used in determining the baudrate of the system, but there must be other two nodes that communicates
	*    with each other.
	* 5- Configuration Mode: This mode is set automatically when the MCP2515 power-on. It is used to initialize some registers.
	*    It is set in the initialization of CAN.
	*/
	switch(copy_u8Mode){
		case CAN_NORMAL_MODE: 
		     MCP2515_WriteByte(0x00, MCP2515_CANCTRL);						// Configure the MCP2515 in Normal Mode.
			 while((MCP2515_ReadByte(MCP2515_CANSTAT)& 0xE0) != 0x00);		// Polling until the MCP2515 becomes in Normal Mode.
			 break;
		case CAN_SLEEP_MODE: 
			 MCP2515_BitModify(MCP2515_CANINTF, 0X40, 0X00);				// Clear the wake-up interrupt flag.
			 MCP2515_BitModify(MCP2515_CANINTE, 0x40, 0x40);				// Enable the wake-up interrupt before the Sleep Mode.
			 MCP2515_WriteByte(0x20, MCP2515_CANCTRL);						// Configure the MCP2515 in sleep mode.
			 while((MCP2515_ReadByte(MCP2515_CANSTAT)& 0xE0)!= 0x20);		// Polling until the MCP2515 becomes in Sleep Mode.
			 break;
		case CAN_LOOP_BACK_MODE: 
			 MCP2515_WriteByte(0x40, MCP2515_CANCTRL);						// Configure the MCP2515 in Loop-Back Mode.
			 while((MCP2515_ReadByte(MCP2515_CANSTAT)& 0xE0)!= 0x40);		// Polling until the MCP2515 becomes in Loop-Back Mode.
			 break;
		case CAN_LISTEN_ONLY_MODE: 
			 MCP2515_WriteByte(0x60, MCP2515_CANCTRL); 						// Configure the MCP2515 in Listen-only mode.
			 while((MCP2515_ReadByte(MCP2515_CANSTAT)& 0xE0)!= 0x60);		// Polling until the MCP2515 becomes in Listen-only mode.
			 break;
		case CAN_CONFIG_MODE: 
			 MCP2515_WriteByte(0x80, MCP2515_CANCTRL); 						// Configure the MCP2515 in Configuration Mode.
			 while((MCP2515_ReadByte(MCP2515_CANSTAT)& 0xE0)!= 0x80);		// Polling until the MCP2515 becomes in Configuration Mode.
			 break;
		default: break;
	}
}
/**************************************************************************************************************/
void CAN_SelectBaudRate(void){
   /* API Information:
	* This API is used to determine the BaudRate of the MCP2515 module between other nodes.
	* We are working on 500Kb/sec, 250Kb/sec and 125Kb/sec.
	* In order to set the baud rate you must calculate The Time Quanta (TQ).
	* TQ = (BRP+1)/(FOSC), Note: BRP is the BaudRate Prescaler, FOSC is the oscillator Frequency in my case, it is 8MHz.
	* tbit = N * TQ
	* N = SyncSeg + PropSeg + PS1 + PS2
	* In our case: SyncSeg = 1*TQ, PropSeg = 2*TQ, PS1 = 2*TQ, PS2 = 3*TQ, SJW = 1
	* Notes:
	* PropSeg + PS1 > PS2
	* PropSeg + PS1 > TDelay, TDelay = (1~2)*TQ
	* PS2 > SJW
	* Sample Point = 62.5%
	* Good Sample Point = (60% - 70%)
	*/
	switch(CAN_BAUD_RATE){
		case CAN_BR_500KB_PER_SEC:
			 MCP2515_WriteByte(0x00, MCP2515_CNF1);  // SJW = 1*TQ, BRP(BaudRate Prescaler) = 0
			 MCP2515_WriteByte(0x89, MCP2515_CNF2);  // PS1 = 2TQ, PropSeg = 2TQ
			 MCP2515_WriteByte(0x02, MCP2515_CNF3);  // PS2 = 3TQ
			 break;
		case CAN_BR_250KB_PER_SEC:
			 MCP2515_WriteByte(0x01, MCP2515_CNF1);  // SJW = 1*TQ, BRP(BaudRate Prescaler) = 1
			 MCP2515_WriteByte(0x89, MCP2515_CNF2);  // PS1 = 2TQ, PropSeg = 2TQ
			 MCP2515_WriteByte(0x02, MCP2515_CNF3);  // PS2 = 3TQ
			 break;
		case CAN_BR_125KB_PER_SEC:
			 MCP2515_WriteByte(0x03, MCP2515_CNF1);  // SJW = 1*TQ, BRP(BaudRate Prescaler) = 3
			 MCP2515_WriteByte(0x89, MCP2515_CNF2);  // PS1 = 2TQ, PropSeg = 2TQ
			 MCP2515_WriteByte(0x02, MCP2515_CNF3);  // PS2 = 3TQ
			 break;
		default: /* Do Nothing */ break;
	}
	
}
/**************************************************************************************************************/
void CAN_Init(void){
	/* This API is used to initialize two things:
	 * 1. BaudRate of transmission and Receiving through three registers:
	 *		- CNF1.
	 *		- CNF2.
	 *		- CNF3.
	 * 2. Filters and Masks of Receive Buffer.
	 * All these are done in configuration mode.
	 * After finishing all these configurations, change to Normal Mode.
	*/
	
	/* Set the MCP2515 in configuration Mode */
	CAN_SelectOperationMode(CAN_CONFIG_MODE);
	/* Configure BaudRate */
	CAN_SelectBaudRate();
	/* Initialize All Filters and Masks */
	/* Note: If all masks become zero, all messages will be accepted */
	/* Mask Registers for RXB0 and RXB1 */
	MCP2515_WriteByte(0x00, MCP2515_RXM0SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXM0SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXM0EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXM0EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXM1SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXM1SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXM1EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXM1EID0);
	/* Filter Registers */
	MCP2515_WriteByte(0x00, MCP2515_RXF0SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF0SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF0EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF0EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXF1SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF1SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF1EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF1EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXF2SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF2SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF2EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF2EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXF3SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF3SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF3EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF3EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXF4SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF4SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF4EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF4EID0);
	MCP2515_WriteByte(0x00, MCP2515_RXF5SIDH);
	MCP2515_WriteByte(0x00, MCP2515_RXF5SIDL);
	MCP2515_WriteByte(0x00, MCP2515_RXF5EID8);
	MCP2515_WriteByte(0x00, MCP2515_RXF5EID0);
	/* Set the MCP2515 in Normal Mode to make Bus available for transmission and receiving */
	CAN_SelectOperationMode(CAN_NORMAL_MODE);
}
/**************************************************************************************************************/
void CAN_Transmit(uint8_t copy_u8Buffer, uint8_t copy_u8Priority, uCAN_MSG *copy_uCANMSGFrame){
	/* When the transmission initiated by sending Request message,
	 * the TXREQ will be set and if TXREQ was set, the ABTF, MLOA and TXERR bits will be cleared.
	 * So, we must clear TXREQ, before initiating the transmission.
	 * Function is pending until the transmission was completed as TXREQ = 0, TXnIF = 1.
	 * Then clear The TXnIF before leaving this function.
	*/
	/* Putting Frame in the Registers */
	if(CAN_FRAME_TYPE == CAN_STANDARD_MESSAGE_TYPE){
		switch(copy_u8Buffer){
				case CAN_TXB0_BUFFER:
					/* Enable Interrupt That will be generated if message was transmitted */
					MCP2515_BitModify(MCP2515_CANINTE, 0x04, 0x04);			// Enable TX0IE.
					/* Clearing TXREQ of TXB0 before writing on it */
					MCP2515_BitModify(MCP2515_TXB0CTRL, 0x08, 0x00);
					/* Set The Priority of the message */
					MCP2515_BitModify(MCP2515_TXB0CTRL, 0x03, copy_u8Priority);
					/* Send ID */
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0x07)<< 5), MCP2515_TXB0SIDL);  		// Send Part of ID of Transmitted Message to SIDL Register.
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0xF8)>>3), MCP2515_TXB0SIDH);			// Send Part of ID of Transmitted Message to SIDH Register.
					/* Send DLC */
					MCP2515_WriteByte((uint8_t)((copy_uCANMSGFrame->frame.dlc)&0x0F), MCP2515_TXB0DLC);
					/* Send Zeros to EID0 and EID8 */
					MCP2515_WriteByte(0x00, MCP2515_TXB0EID8);
					MCP2515_WriteByte(0x00, MCP2515_TXB0EID0);
					/* Send Data Frame */
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data0, MCP2515_TXB0D0);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data1, MCP2515_TXB0D1);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data2, MCP2515_TXB0D2);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data3, MCP2515_TXB0D3);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data4, MCP2515_TXB0D4);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data5, MCP2515_TXB0D5);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data6, MCP2515_TXB0D6);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data7, MCP2515_TXB0D7);
					
					/* Sending Request to Send Command to TXB0 */
					MCP2515_RequestToSend(0x81);
					
					/* Pending until the transmission completed successfully i.e. TXREQ = 0 and TX0IF = 1*/
					while(((MCP2515_ReadByte(MCP2515_TXB0CTRL) & 0x08)>>3) != 0 && ((MCP2515_ReadByte(MCP2515_CANINTF) & 0x04)>>2) != 1);
					/* Clear TX0IF bit */
					MCP2515_BitModify(MCP2515_CANINTF, 0x04, 0x00);
					break;
				case CAN_TXB1_BUFFER:
					/* Enable Interrupt That will be generated if message was transmitted */
					MCP2515_BitModify(MCP2515_CANINTE, 0x08, 0x08);			// Enable TX1IE.
					/* Clearing TXREQ of TXB1 before writing on it */
					MCP2515_BitModify(MCP2515_TXB1CTRL, 0x08, 0x00);
					/* Set The Priority of the message */
					MCP2515_BitModify(MCP2515_TXB1CTRL, 0x03, copy_u8Priority);
					/* Send ID */
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0x07)<< 5), MCP2515_TXB1SIDL);  		// Send Part of ID of Transmitted Message to SIDL Register.
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0xF8)>>3), MCP2515_TXB1SIDH);			// Send Part of ID of Transmitted Message to SIDH Register.
					/* Send DLC */
					MCP2515_WriteByte((uint8_t)((copy_uCANMSGFrame->frame.dlc)&0x0F), MCP2515_TXB1DLC);
					/* Send Zeros to EID0 and EID8 */
					MCP2515_WriteByte(0x00, MCP2515_TXB1EID8);
					MCP2515_WriteByte(0x00, MCP2515_TXB1EID0);
					/* Send Data Frame */
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data0, MCP2515_TXB1D0);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data1, MCP2515_TXB1D1);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data2, MCP2515_TXB1D2);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data3, MCP2515_TXB1D3);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data4, MCP2515_TXB1D4);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data5, MCP2515_TXB1D5);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data6, MCP2515_TXB1D6);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data7, MCP2515_TXB1D7);
					
					/* Sending Request to Send Command to TXB1 */
					MCP2515_RequestToSend(0x82);
					/* Pending until the transmission completed successfully i.e. TXREQ = 0 and TX1IF = 1*/
					while(((MCP2515_ReadByte(MCP2515_TXB1CTRL) & 0x08)>>3) != 0 && ((MCP2515_ReadByte(MCP2515_CANINTF) & 0x08)>>3) != 1);
					/* Clear TX1IF bit */
					MCP2515_BitModify(MCP2515_CANINTF, 0x08, 0x00);
					break;
				case CAN_TXB2_BUFFER:
					/* Enable Interrupt That will be generated if message was transmitted */
					MCP2515_BitModify(MCP2515_CANINTE, 0x10, 0x10);			// Enable TX2IE.
					/* Clearing TXREQ of TXB2 before writing on it */
					MCP2515_BitModify(MCP2515_TXB2CTRL, 0x08, 0x00);
					/* Set The Priority of the message */
					MCP2515_BitModify(MCP2515_TXB2CTRL, 0x03, copy_u8Priority);
					/* Send ID */
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0x07)<< 5), MCP2515_TXB2SIDL);  		// Send Part of ID of Transmitted Message to SIDL Register.
					MCP2515_WriteByte((uint8_t)((((uint16_t)(copy_uCANMSGFrame->frame.id)) & 0xF8)>>3), MCP2515_TXB2SIDH);			// Send Part of ID of Transmitted Message to SIDH Register.
					/* Send DLC */
					MCP2515_WriteByte((uint8_t)((copy_uCANMSGFrame->frame.dlc)&0x0F), MCP2515_TXB2DLC);
					/* Send Zeros to EID0 and EID8 */
					MCP2515_WriteByte(0x00, MCP2515_TXB2EID8);
					MCP2515_WriteByte(0x00, MCP2515_TXB2EID0);
					/* Send Data Frame */
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data0, MCP2515_TXB2D0);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data1, MCP2515_TXB2D1);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data2, MCP2515_TXB2D2);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data3, MCP2515_TXB2D3);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data4, MCP2515_TXB2D4);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data5, MCP2515_TXB2D5);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data6, MCP2515_TXB2D6);
					MCP2515_WriteByte(copy_uCANMSGFrame->frame.data7, MCP2515_TXB2D7);
					
					/* Sending Request to Send Command to TXB2 */
					MCP2515_RequestToSend(0x84);
					/* Pending until the transmission completed successfully i.e. TXREQ = 0 and TX2IF = 1*/
					while(((MCP2515_ReadByte(MCP2515_TXB2CTRL) & 0x08)>>3) != 0 && ((MCP2515_ReadByte(MCP2515_CANINTF) & 0x10)>>4) != 1);
					/* Clear TX2IF bit */
					MCP2515_BitModify(MCP2515_CANINTF, 0x10, 0x00);
					break;
				default: break;
		}
	}
	else if(CAN_FRAME_TYPE == CAN_EXTENDED_MESSAGE_TYPE){
		/****************************************/
	}
}
/**************************************************************************************************************/
void CAN_Receive(uCAN_MSG *copy_ReceiveMessage){
	/* Turn Masks, Filters On and Disable Rollover in RXB0 */
	MCP2515_BitModify(MCP2515_RXB0CTRL, 0x6D, 0x00);
	/* Turn Masks, Filters On in RXB1 */
	//MCP2515_BitModify(MCP2515_RXB1CTRL, 0x6F, 0x60);
	/* Enable Interrupt For RXB0 and RXB1 */
	//MCP2515_BitModify(MCP2515_CANINTE, 0x03, 0x03);
	if(((MCP2515_ReadByte(MCP2515_CANINTF)&0x01)) == 1){
		/* Read Message on RXB0 as this buffer is Full now */
		/* Read ID of the message received */
		uint32_t MSB_Id = MCP2515_ReadByte(MCP2515_RXB0SIDH);
		uint32_t LSB_Id = MCP2515_ReadByte(MCP2515_RXB0SIDL);
		copy_ReceiveMessage->frame.id = (MSB_Id<<3)|(LSB_Id>>5);
		/* Read DLC of the message received */
		copy_ReceiveMessage->frame.dlc = (MCP2515_ReadByte(MCP2515_RXB0DLC)& 0x0F);
		/* Read Data of the message recieved */
		copy_ReceiveMessage->frame.data0 = MCP2515_ReadByte(MCP2515_RXB0D0);
		copy_ReceiveMessage->frame.data1 = MCP2515_ReadByte(MCP2515_RXB0D1);
		copy_ReceiveMessage->frame.data2 = MCP2515_ReadByte(MCP2515_RXB0D2);
		copy_ReceiveMessage->frame.data3 = MCP2515_ReadByte(MCP2515_RXB0D3);
		copy_ReceiveMessage->frame.data4 = MCP2515_ReadByte(MCP2515_RXB0D4);
		copy_ReceiveMessage->frame.data5 = MCP2515_ReadByte(MCP2515_RXB0D5);
		copy_ReceiveMessage->frame.data6 = MCP2515_ReadByte(MCP2515_RXB0D6);
		copy_ReceiveMessage->frame.data7 = MCP2515_ReadByte(MCP2515_RXB0D7);
		/* Then Clear RX0IF bit in order to say that this buffer is empty */
		MCP2515_BitModify(MCP2515_CANINTF, 0x01, 0x00);
	}
	
}
/**************************************************************************************************************/
