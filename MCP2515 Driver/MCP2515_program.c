/************************************************************/
/************************************************************/
/********************* Driver : MCP2515       ***************/
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

extern SPI_HandleTypeDef hspi2;
/* APIs Implementation */
/* Note those APIs deal with SPI2 in stm32f103c8t6
 * If you want another SPI like SPI1 change in each function if found 
 * the (&hspix) to &hspi1.
 * SPI here send from Most significant bit (MSB)
*/

/**************************************************************************************************************/
void MCP2515_Reset(void){
   /* API Information:
	* Reset Instruction is sent to MCP2515 by Node MCU in order to initialize
	* the internal registers of MCP2515 to the default state.
	* It also sets the configuration mode.
	* This instruction is a byte instruction = 1100 0000.
	* It is highly recommended that the RESET Command be sent as part of
	* the power on initialization sequence.
	*/
	uint8_t data = 0xC0;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   // Set the chip select pin to LOW
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);			// Send reset instruction
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET); // Set the chip select pin to HIGH
}
/**************************************************************************************************************/
uint8_t MCP2515_ReadByte(uint8_t copy_u8Address){
   /* API Information:
	* Read Instruction is used to read byte from certain register in MCP2515.
	* Each register in the MCP2515 is 1 Byte (8-Bits).
	* First send read instruction then send the address you want to read.
	* Then receive the data comes on MISO Pin from the MCP2515 Register of
	* this address.
	* The Read operation is terminated or ended by raising the Chip Select pin.
	*/
	uint8_t retByte;
	uint8_t data = 0x03;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   // Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);			  // Send Read instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8Address, 1, SPI_TIMEOUT);    // Send Address you want to read byte from it.
	HAL_SPI_Receive(&hspi2, &retByte, 1, SPI_TIMEOUT);			  // Receive Byte data comes from these address.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     // Set the chip select pin to HIGH.
	return retByte;
}
/**************************************************************************************************************/
void MCP2515_ReadRxSequence(uint8_t copy_u8Instruction, uint8_t *copy_p8Data, uint8_t copy_u8Length){
   /* API Information:
	* Read RX Sequence is an instruction that used read from MCP2515 more than one byte i.e Buffer.
	* First transmit certain instruction (1001 0nm0) to MCP2515.
	* nm may be 00, 01, 10, 11 to indicate the address pointer location.
	* Then receive the buffer with certain length.
	*/
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &copy_u8Instruction, 1, SPI_TIMEOUT);		// Send Instruction.
	HAL_SPI_Receive(&hspi2, copy_p8Data, copy_u8Length, SPI_TIMEOUT);	// Receive Buffer with certain length from MCP2515.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
void MCP2515_WriteByte(uint8_t copy_u8Data, uint8_t copy_u8Address){
   /* API Information:
	* Write Instruction is an instruction used to write data in a certain register in MCP2515.
	* This instruction can send one byte by sending the instruction 0x02, then send address of
	* register you want to write in, then send a byte of data, then raise chip select to high.
	* This instruction can send a buffer that contains more than one byte. 
	*/
	uint8_t data = 0x02;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);			  		// Send Write instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8Address, 1, SPI_TIMEOUT);    		// Send Address you want to write byte to it.
	HAL_SPI_Transmit(&hspi2, &copy_u8Data, 1, SPI_TIMEOUT);				// Send a Byte of Data to this register.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
void MCP2515_WriteBuffer(uint8_t copy_u8StartAddress, uint8_t copy_u8EndAddress, uint8_t *copy_p8Data){
   /* API Information:
	* Write Instruction is an instruction used to write data in a certain register in MCP2515.
	* This instruction can send many bytes by sending the instruction 0x02, then send first address of
	* registers you want to write in then send length of them and buffer of data, then raise chip select to high.
	* This instruction can send a buffer that contains more than one byte. 
	*/
	uint8_t data = 0x02;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   										// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);			  										// Send Write instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8StartAddress, 1, SPI_TIMEOUT);    									// Send Address you want to write byte to it.
	HAL_SPI_Transmit(&hspi2, copy_p8Data, (copy_u8EndAddress - copy_u8StartAddress +1), SPI_TIMEOUT);	// Send a Byte of Data to this register.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     										// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
void MCP2515_LoadTxBuffer(uint8_t copy_u8Instruction, uint8_t *copy_p8IdReg, uint8_t copy_u8Dlc, uint8_t *copy_u8Data){
   /* API Information:
	* This instruction is used to send ID, DLC and data frame to Transmit Buffer to be sent by CAN.
	* First we send instruction (0100 0abc) which indicates the address pointer to one of six locations.
	* then send ID, then send DLC and then data itself.
	*/
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &copy_u8Instruction, 1, SPI_TIMEOUT);		// Send the instruction of Load TX Buffer.
	HAL_SPI_Transmit(&hspi2, copy_p8IdReg, 4, SPI_TIMEOUT);				// Send ID of this message.
	HAL_SPI_Transmit(&hspi2, &copy_u8Dlc, 1, SPI_TIMEOUT);				// Send Data Length of this message.
	HAL_SPI_Transmit(&hspi2, copy_u8Data, copy_u8Dlc, SPI_TIMEOUT);		// Send Data it self.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
void MCP2515_RequestToSend(uint8_t copy_u8Instruction){
   /* API Information:
	* This instruction is used to initiate the transmission for one or more than one transmit buffer.
	* The last three bit indicates which transmission buffer, we will initiate a message transmission for it.
	* The instruction is (1000 0nnn) first (n) is Request to send for TXB2, the second (n) is Request to send for TXB1
	* and the third (n) is Request to send for TXB0.
	*/
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &copy_u8Instruction, 1, SPI_TIMEOUT);		// Send the instruction of RTS.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
uint8_t MCP2515_ReadStatus(void){
   /* API Information:
	* This instruction is used to read status bits of transmit and receive functions.
	* First send the instruction of read status, then receive the status bits in one byte.
	* If you don't raise the chip select to high, the clock will be generated. Therefore;
	* the more status you can receive.
	*/
	uint8_t data = 0xA0;
	uint8_t retStatus;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);						// Send the Read Status instruction.
	HAL_SPI_Receive(&hspi2, &retStatus, 1, SPI_TIMEOUT);				// Read the status
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
	return retStatus;
}
/**************************************************************************************************************/
uint8_t MCP2515_Rx_Status(void){
   /* API Information:
	* This instruction is used to determine the filter match and message type
	* (standard, extended or remote) by sending this command then mcp2515 will send
	* the status bits for that to the controller by SO Pin.
	*/
	uint8_t data = 0xB0;
	uint8_t retStatus;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);						// Send the Read Status instruction.
	HAL_SPI_Receive(&hspi2, &retStatus, 1, SPI_TIMEOUT);				// Read the status
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
	return retStatus;
}
/**************************************************************************************************************/
void MCP2515_BitModify(uint8_t copy_u8Address, uint8_t copy_u8Mask, uint8_t copy_u8Data){
   /* API Information:
	* This instruction is used to write in bits specifically in registers that can be modified by bit.
	* Then we send three things: Address of this register that will be modified, Mask that will determine which bit will be modified,
	* data it self.
	* if the bit in mask is (1) then the bit corrosponding to it will be modified, if (0), then it will not be modified.
	*/
	uint8_t data = 0x05;
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_RESET);   		// Set the chip select pin to LOW.
	HAL_SPI_Transmit(&hspi2, &data, 1, SPI_TIMEOUT);						// Send the Bit Modify instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8Address, 1, SPI_TIMEOUT);			// Send the address instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8Mask, 1, SPI_TIMEOUT);				// Send the Mask instruction.
	HAL_SPI_Transmit(&hspi2, &copy_u8Data, 1, SPI_TIMEOUT);				// Send the Data instruction.
	HAL_GPIO_WritePin(SPI_CS_PORT, SPI_CS_PIN, GPIO_PIN_SET);     		// Set the chip select pin to HIGH.
}
/**************************************************************************************************************/
