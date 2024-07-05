/************************************************************/
/************************************************************/
/********************* Driver : MCP2515       ***************/
/********************* Layer  : HAL_Interface ***************/
/********************* Date   : 01 may 2022   ***************/
/********************* Version: V01           ***************/
/********************* Author : Ahmed Mostafa ***************/
/********************* Company: Aero          ***************/
/************************************************************/
/************************************************************/

/* Preprocessor File Guard */

#ifndef MCP2515_INTERFACE_H_
#define MCP2515_INTERFACE_H_

/* APIs Definition */
/* SPI Commmands/Instructions to MCP2515 CAN Controller */

void MCP2515_Reset(void);
/**************************************************************************************************************/
uint8_t MCP2515_ReadByte(uint8_t copy_u8Address);
/**************************************************************************************************************/
void MCP2515_ReadRxSequence(uint8_t copy_u8Instruction, uint8_t *copy_p8Data, uint8_t copy_u8Length);
/**************************************************************************************************************/
void MCP2515_WriteByte(uint8_t copy_u8Data, uint8_t copy_u8Address);
/**************************************************************************************************************/
void MCP2515_WriteBuffer(uint8_t copy_u8StartAddress, uint8_t copy_u8EndAddress, uint8_t *copy_p8Data);
/**************************************************************************************************************/
void MCP2515_LoadTxBuffer(uint8_t copy_u8Instruction, uint8_t *copy_p8IdReg, uint8_t copy_u8Dlc, uint8_t *copy_u8Data);
/**************************************************************************************************************/
void MCP2515_RequestToSend(uint8_t copy_u8Instruction);

/* Instruction Macros */

#define MCP2515_TXB0_INIT_TRANSMISSION			0x81		// This instruction will set the TXREQ bit in TXB0CTRL which will initialize the transmission for the TXB0 Buffer.
#define MCP2515_TXB1_INIT_TRANSMISSION			0x82		// This instruction will set the TXREQ bit in TXB1CTRL which will initialize the transmission for the TXB1 Buffer.
#define MCP2515_TXB2_INIT_TRANSMISSION			0x84		// This instruction will set the TXREQ bit in TXB2CTRL which will initialize the transmission for the TXB2 Buffer.
/**************************************************************************************************************/
uint8_t MCP2515_ReadStatus(void);
/**************************************************************************************************************/
uint8_t MCP2515_Rx_Status(void);
/**************************************************************************************************************/
void MCP2515_BitModify(uint8_t copy_u8Address, uint8_t copy_u8Mask, uint8_t copy_u8Data);

#endif
