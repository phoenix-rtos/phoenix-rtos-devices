/*
 * Phoenix-RTOS
 *
 * GRLIB FTMCTRL Flash driver
 *
 * Flash commands
 *
 * Copyright 2024 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FTMCTRL_CMDS_H_
#define _FTMCTRL_CMDS_H_


/* Common flash commands */
#define CMD_RD_QUERY  0x98u /* Read/Enter Query */
#define CMD_RD_STATUS 0x70u /* Read Status Register */

/* Intel command set */
#define INTEL_CMD_RESET      0xffu /* Reset/Read Array */
#define INTEL_CMD_WR_BUF     0xe8u /* Write to Buffer */
#define INTEL_CMD_WR_CONFIRM 0xd0u /* Write Confirm */
#define INTEL_CMD_CLR_STATUS 0x50u /* Clear Status Register */
#define INTEL_CMD_BE_CYC1    0x20u /* Block Erase (1st bus cycle) */

/* AMD command set */
#define AMD_CMD_RESET      0xf0u /* Reset/ASO Exit */
#define AMD_CMD_WR_BUF     0x25u /* Write to Buffer */
#define AMD_CMD_WR_CONFIRM 0x29u /* Write Confirm */
#define AMD_CMD_CLR_STATUS 0x71u /* Clear Status Register */
#define AMD_CMD_CE_CYC1    0x80u /* Chip Erase (1st bus cycle) */
#define AMD_CMD_CE_CYC2    0x10u /* Chip Erase (2nd bus cycle) */
#define AMD_CMD_BE_CYC1    0x80u /* Block Erase (1st bus cycle) */
#define AMD_CMD_BE_CYC2    0x30u /* Block Erase (2nd bus cycle) */
#define AMD_CMD_EXIT_QUERY 0xf0u /* Exit Query */


#endif
