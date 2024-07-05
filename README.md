# CAN-Driver
MCAL driver for CAN protocol MCP2515 module with STM32 Blue Pill.
# There are two folders and main code file:
1- CAN Driver
   - CAN_config.h
   - CAN_interface.h
   - CAN_private.h
   - CAN_program.c

2- MCP2515 Driver
   - MCP2515_config.h
   - MCP2515_interface.h
   - MCP2515_private.h
   - MCP2515_program.c

3- main.c

In this file, you can edit it using APIs in CAN libraries to match your settings.
# Note:
You must compile all those files with main.c to read CAN an MCP2515 libraries.
CAN library uses APIs found in MCP2515 as MCP2515 Libraries access MCP2515 device commands.
