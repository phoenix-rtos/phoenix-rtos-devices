/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * MT25QU01GB Micron flash commands
 *
 * Copyright 2026 Phoenix Systems
 * Author: Amelia Waszkowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _MICRON_FLASH_CMDS_H_
#define _MICRON_FLASH_CMDS_H_


/* Read Registers */

#define FLASH_CMD_RDEVCR 0x65u /* Read Enhanced Volatile Configuration Register */


/* Write Registers */

#define FLASH_CMD_WRITE_NVCONFREG 0xB1u /* Write Nonvolatile Configuration Register */
#define FLASH_CMD_WRITE_VCONFREG 0x81u /* Write Volatile Configuration Register */
#define FLASH_CMD_WRITE_EVCR 0x61u /* Write Enhanced Volatile Configuration Register */

/* Program Operations */

#define FLASH_CMD_DIN_FP 0xA2u /* Dual Input Fast Program */
#define FLASH_CMD_QIN_FP 0x32u /* Quad Input Fast Program */
#define FLASH_CMD_EXTENDED_QIN_FP 0x38u /* Extended Quad Input Fast Program */

/* Program Operations with 4-Byte Address */

#define FLASH_CMD_4B_QIN_FP 0x34u /* 4-Byte Quad Input Fast Program */

/* Erase Operations */

#define FLASH_CMD_DE 0xC4u /* Die Erase */



#endif /* _MICRON_FLASH_CMDS_H_ */