/*
 * Phoenix-RTOS
 *
 * GR712RC Flash driver
 *
 * Intel StrataFlash J3 commands
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PROM_H_
#define _PROM_H_


#define FLASH_RD_ARRAY   0xffu /* Read Array */
#define FLASH_RD_ID      0x90u /* Read Identifier Codes */
#define FLASH_RD_QUERY   0x98u /* Read Query */
#define FLASH_RD_STATUS  0x70u /* Read Status Register */
#define FLASH_CLR_STATUS 0x50u /* Clear Status Register */

#define FLASH_WR_BUF      0xe8u /* Write to Buffer */
#define FLASH_WR_CONFIRM  0xd0u /* Write Confirm */
#define FLASH_WR_BYTE     0x40u /* Byte Program */
#define FLASH_BE_CYC1     0x20u /* Block Erase (1st bus cycle) */
#define FLASH_BE_PROG_SUS 0xb0u /* Block Erase, Program Suspend */
#define FLASH_BE_PROG_RES 0xd0u /* Block Erase, Program Resume */

#define FLASH_CFG_CYC1    0xb8u /* Configuration Register (1st bus cycle) */
#define FLASH_CFG_CYC2    0xccu /* Configuration Register (2nd bus cycle) */
#define FLASH_LOCK_CYC1   0x60u /* Set Block Lock-Bit (1st bus cycle) */
#define FLASH_LOCK_CYC2   0x01u /* Set Block Lock-Bit (2nd bus cycle) */
#define FLASH_UNLOCK_CYC1 0x60u /* Clear Block Lock-Bit (1st bus cycle) */
#define FLASH_UNLOCK_CYC2 0xd0u /* Clear Block Lock-Bit (2nd bus cycle) */
#define FLASH_PROT_CYC1   0xc0u /* Protection Program (1st bus cycle) */


#endif
