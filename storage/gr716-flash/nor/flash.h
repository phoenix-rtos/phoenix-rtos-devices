/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Macronix MX25L flash commands
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _FLASH_H_
#define _FLASH_H_


/* Status register */

#define FLASH_SR_WIP 0x01u /* Write in progress */
#define FLASH_SR_WEL 0x02u /* Write enable latch */

/* ID */

#define FLASH_CMD_RDID 0x9Fu /* Read Identification */
#define FLASH_CMD_RES  0xABu /* Read Electronic ID */
#define FLASH_CMD_REMS 0x90u /* Read Electronic & Device ID */

/* Register */

#define FLASH_CMD_WRSR    0x01u /* Write Status Register */
#define FLASH_CMD_RDSR    0x05u /* Read Status Register */
#define FLASH_CMD_WRSCUR  0x2Fu /* Write Security Register */
#define FLASH_CMD_RDSCUR  0x2Bu /* Read Security Register */
#define FLASH_CMD_RDCR    0x15u /* Read Configuration Register */
#define FLASH_CMD_RDEAR   0xC8u /* Read Extended Address Register */
#define FLASH_CMD_WREAR   0xC5u /* Write Extended Address Register */
#define FLASH_CMD_RDFBR   0x16u /* Read Fast Boot Register */
#define FLASH_CMD_WRFBR   0x17u /* Write Fast Boot Register */
#define FLASH_CMD_ESFBR   0x18u /* Erase Fast Boot Register */
#define FLASH_CMD_WRLR    0x2Cu /* Write Lock Register */
#define FLASH_CMD_RDLR    0x2Du /* Read Lock Register */
#define FLASH_CMD_RDSPB   0xE2u /* Read SPB Status */
#define FLASH_CMD_WRSPB   0xE3u /* Write SPB Bit */
#define FLASH_CMD_ESSPB   0xE4u /* Erase All SPB Status */
#define FLASH_CMD_SPBLK   0xA6u /* SPB lock Set */
#define FLASH_CMD_RDSPBLK 0xA7u /* Read SPB lock Register */
#define FLASH_CMD_WRPASS  0x28u /* Write Password Register */
#define FLASH_CMD_RDPASS  0x27u /* Read Password Register */
#define FLASH_CMD_PASSULK 0x29u /* Password Unlock */
#define FLASH_CMD_RDDPB   0xE0u /* Read DPB Register */
#define FLASH_CMD_WRDPB   0xE1u /* Write DPB Register */

/* Read */

#define FLASH_CMD_READ     0x03u /* Read */
#define FLASH_CMD_FASTREAD 0x0Bu /* Fast Read */
#define FLASH_CMD_RDSFDP   0x5Au /* Read SFDP */

/* Program */

#define FLASH_CMD_WREN 0x06u /* Write Enable */
#define FLASH_CMD_WRDI 0x04u /* Write Disable */
#define FLASH_CMD_PP   0x02u /* Page Program */

/* Erase */

#define FLASH_CMD_SE    0x20u /* Sector Erase */
#define FLASH_CMD_BE32K 0x52u /* Block Erase 32kB */
#define FLASH_CMD_BE    0xD8u /* Block Erase */
#define FLASH_CMD_CE    0x60u /* Chip Erase */

/* Mode setting */

#define FLASH_CMD_DP    0xB9u /* Deep Power Down */
#define FLASH_CMD_RDP   0xABu /* Release from Deep Power Down */
#define FLASH_CMD_ENSO  0xB1u /* Enter Secured OTP */
#define FLASH_CMD_EXSO  0xC1u /* Exit Secured OTP */
#define FLASH_CMD_WPSEL 0x68u /* Enable block protect mode */
#define FLASH_CMD_SBL   0xC0u /* SBL Set Burst Length */

/* Reset */

#define FLASH_CMD_RSTEN  0x66u /* Reset Enable */
#define FLASH_CMD_RST    0x99u /* Reset Memory */
#define FLASH_CMD_RSTQIO 0xF5u /* Reset Quad I/O */
#define FLASH_CMD_NOP    0x00u /* No Operation */


#endif /* _FLASH_H_ */
