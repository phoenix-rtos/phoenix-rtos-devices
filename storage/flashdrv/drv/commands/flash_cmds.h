/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Macronix MT25 flash commands
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_CMDS_H_
#define _FLASH_CMDS_H_


/* Status register */

#define FLASH_SR_WIP 0x01u /* Write in progress */
#define FLASH_SR_WEL 0x02u /* Write enable latch */

/* ID */

#define FLASH_CMD_RDID 0x9Fu /* Read Identification */

/* Registers */

#define FLASH_CMD_RDSR    0x05u /* Read Status Register */
#define FLASH_CMD_RDEAR   0xC8u /* Read Extended Address Register */

/* Write Register */

#define FLASH_CMD_WREAR   0xC5u /* Write Extended Address Register */
#define FLASH_CMD_WRITE_SR 0x1u /* Write Status Register */

/* Read */

#define FLASH_CMD_READ 0x03u /* Read */
#define FLASH_CMD_FASTREAD 0x0Bu /* Fast Read */
#define FLASH_CMD_DOUTPUT_FASTREAD 0x3Bu /* Dual Output Fast Read */
#define FLASH_CMD_DIO_FASTREAD 0xBBu /* Dual Input Output Fast Read */
#define FLASH_CMD_QOUTPUT_FASTREAD 0x6B /* Quad Output Fast Read */
#define FLASH_CMD_QIO_FASTREAD 0xEBu /* Quad Input Output Fast Read */

/* Read with 4-Byte Address */

#define FLASH_CMD_4B_READ 0x13u /* 4-Byte Read */
#define FLASH_CMD_4B_FASTREAD 0x0Cu /* 4-Byte Fast Read */
#define FLASH_CMD_4B_DOUTPUT_FASTREAD 0x3Cu /* 4-Byte Dual Output Fast Read */
#define FLASH_CMD_4B_DIO_FASTREAD 0xBCu /* 4-Byte Dual Input Output Fast Read */
#define FLASH_CMD_4B_QOUTPUT_FASTREAD 0x6Cu /* 4-Byte Quad Output Fast Read */
#define FLASH_CMD_4B_QIO_FASTREAD 0xECu /* 4-Byte Quad Input Output Fast Read */

/* Program Operations */

#define FLASH_CMD_WREN 0x06u /* Write Enable */
#define FLASH_CMD_WRDI 0x04u /* Write Disable */
#define FLASH_CMD_PP 0x02u /* Page Program */

/* Program Operations with 4-Byte Address */

#define FLASH_CMD_4B_PP 0x12u /* 4-Byte Page Program */
#define FLASH_CMD_4B_QIN_FP 0x34u /* 4-Byte Extended Quad Input Fast Program */

/* Erase Operations */

#define FLASH_CMD_BE32K 0x52u /* Subsector Erase 32kB */
#define FLASH_CMD_SE 0x20u /* Subsector Erase 4 kB */
#define FLASH_CMD_BE 0xD8u /* Block Erase 64 kB */

/* Erase Operations with 4-Byte Address */

#define FLASH_CMD_4B_BE32K 0x5Cu /* Subsector Erase 32kB */
#define FLASH_CMD_4B_SE 0x21u /* Subsector Erase 4 kB */
#define FLASH_CMD_4B_BE 0xDCu /* Block Erase 64 kB */

/* Address Mode */

#define FLASH_CMD_ENTER_4B 0xB7u /* Enter 4-Byte Address Mode */
#define FLASH_CMD_EXIT_4B 0xE9u /* Exit 4-Byte Address Mode */

/* Quad Protocol */

#define FLASH_CMD_ENTER_QUADIO 0x35u /* Enter Quad Input Output Mode */
#define FLASH_CMD_RESET_QUADIO 0xF5u /* Reset Quad Input Output Mode */

/* Mode Setting */

#define FLASH_CMD_DP    0xB9u /* Deep Power Down */
#define FLASH_CMD_RDP   0xABu /* Release from Deep Power Down */

/* Reset */

#define FLASH_CMD_RSTEN  0x66u /* Reset Enable */
#define FLASH_CMD_RST    0x99u /* Reset Memory */
#define FLASH_CMD_RSTQIO 0xF5u /* Reset Quad I/O */
#define FLASH_CMD_NOP    0x00u /* No Operation */


/// MT25 ///

/* Write Registers */

#define FLASH_CMD_WRITE_NVCONFREG 0xB1u /* Write Nonvolatile Configuration Register */
#define FLASH_CMD_WRITE_VCONFREG 0x81u /* Write Volatile Configuration Register */

/* Program Operations */

#define FLASH_CMD_DIN_FP 0xA2 /* Dual Input Fast Program */
#define FLASH_CMD_QIN_FP 0x32u /* Quad Input Fast Program */

/* Erase Operations */

#define FLASH_CMD_DE 0xC4u /* Die Erase */


/// MX25 ///

/* Erase Operations */

#define FLASH_CMD_CE    0x60u /* Chip Erase */


#endif /* _FLASH_CMDS_H_ */