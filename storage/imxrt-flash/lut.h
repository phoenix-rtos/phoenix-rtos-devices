/*
 * Phoenix-RTOS
 *
 * i.MX RT LUT (lookup table) instructions
 *
 * Copyright 2015, 2020 Phoenix Systems
 * Author: Hubert Buczynski, Katarzyna Baranowska
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMXRT_LUT_H_
#define _IMXRT_LUT_H_


#define LUT_INSTR(opcode0, pads0, operand0, opcode1, pads1, operand1) \
				 (((opcode1 & 0x3f) << 26) | ((pads1 & 0x3) << 24) | (operand1 & 0xff) << 16) \
				 | (((opcode0 & 0x3f) << 10) | ((pads0 & 0x3) << 8) | (operand0 & 0xff))



#define LUT_MAGIC_KEY 0x5AF05AF0

/* Instruction opcodes in SDR mode, based on IMXRT1064RM Rev. 0.1, 12/2018 chapter 26.7.8  */
#define LUT_CMD_STOP        0x00       /* Stop execution */
#define LUT_CMD_CMD         0x01       /* Transmit command code to flash */
#define LUT_CMD_ADDR        0x02       /* Transmit row address to flash */
#define LUT_CMD_CADDR       0x03       /* Transmit column address to flash */
#define LUT_CMD_MODE        0x04       /* Transmit 1 - bit mode to flash */
#define LUT_CMD_MODE2       0x05       /* Transmit 2 - bit mode to flash*/
#define LUT_CMD_MODE4       0x06       /* Transmit 4 - bit mode to flash*/
#define LUT_CMD_MODE8       0x07       /* Transmit 8 - bit mode to flash*/
#define LUT_CMD_WRITE       0x08       /* Transmit programming data to flash */
#define LUT_CMD_READ        0x09       /* Receive read data from flash */
#define LUT_CMD_LEARN       0x0a       /* Receive Read Data or Preamble bit from Flash */
#define LUT_CMD_DATASZ      0x0b       /* Transmit Read/Program Data size (byte number) to Flash */
#define LUT_CMD_DUMMY       0x0c       /* Leave data lines undriven by FlexSPI controller */
#define LUT_CMD_DUMMY_RWDS  0x0d       /* This instruction is similar as - DUMMY instruction, dummy cycles decided by RWDS */
#define LUT_CMD_JUMP_ON_CS  0x1f       /* Stop execution, deassert CS and save operand[7:0] */


#define LUT_PAD1  0
#define LUT_PAD2  1
#define LUT_PAD4  2
#define LUT_PAD8  3


#define LUT_3B_ADDR  0x18    /* 4 byte address */
#define LUT_4B_ADDR  0x20    /* 3 byte address */



/* Instruction operands, based on:
 * - Windbond W25Q32JV ref. manual 2018 - Rev. G,
 * - Micron MT25QL512ABB - Rev. F/18
*/
#define FLASH_SPANSION_CMD_READ_ID    0x90      /* Read manufacturer/device ID */
#define FLASH_SPANSION_CMD_RDID       0x9f      /* Read JEDEC ID */
#define FLASH_SPANSION_CMD_RES        0xab      /* Release from deep power - down*/
#define FLASH_SPANSION_CMD_RDSR1      0x05      /* Read Status Register - 1 */
#define FLASH_SPANSION_CMD_RDSR2      0x35      /* Read Status Register - 2 */
#define FLASH_SPANSION_CMD_RDSR3      0x15      /* Read Status Register - 3 */
#define FLASH_SPANSION_CMD_EQM        0x35      /* Enebale quad input/output mode */
#define FLASH_SPANSION_CMD_RQM        0xF5      /* Reset quad input/output mode */
#define FLASH_SPANSION_CMD_WRR1       0x01      /* Write Status Register - 1 */
#define FLASH_SPANSION_CMD_WRR2       0x31      /* Write Status Register - 2 */
#define FLASH_SPANSION_CMD_WRR3       0x11      /* Write Status Register - 3 */
#define FLASH_SPANSION_CMD_WRDI       0x04      /* Write enable */
#define FLASH_SPANSION_CMD_WREN       0x06      /* Write disable */
#define FLASH_SPANSION_CMD_READ       0x03      /* Read data */
#define FLASH_SPANSION_CMD_4READ      0x13      /* 4 byte read data */
#define FLASH_SPANSION_CMD_FAST_READ  0x0b      /* Fast read data */
#define FLASH_SPANSION_CMD_4FAST_READ 0x0c      /* 4 byte fast read data */
#define FLASH_SPANSION_CMD_DDRFR      0x0d      /* DTR fast read data */
#define FLASH_SPANSION_CMD_4DDRFR     0x0e      /* 4 byte DTR fast read data */
#define FLASH_SPANSION_CMD_DOR        0x3b      /* Dual output fast read data */
#define FLASH_SPANSION_CMD_4DOR       0x3c      /* 4 byte dual output read data */
#define FLASH_SPANSION_CMD_QOR        0x6b      /* Quad output read data */
#define FLASH_SPANSION_CMD_4QOR       0x6c      /* 4 byte quad output read data */
#define FLASH_SPANSION_CMD_DIOR       0xbb      /* Dual input/output fast read data */
#define FLASH_SPANSION_CMD_4DIOR      0xbc      /* 4 byte dual input/output fast read data */
#define FLASH_SPANSION_CMD_DDRDIOR    0xbd      /* DTR dual input/output fast read data */
#define FLASH_SPANSION_CMD_4DDRDIOR   0xbe      /* 4 byte DTR dual input/output fast read data */
#define FLASH_SPANSION_CMD_QIOR       0xeb      /* Quad input/output fast read data */
#define FLASH_SPANSION_CMD_4QIOR      0xec      /* 4 byte Quad input/output fast read data */
#define FLASH_SPANSION_CMD_DDRQIOR    0xed      /* DTR quad input/output fast read data */
#define FLASH_SPANSION_CMD_4DDRQIOR   0xee      /* 4 byte DTR quad input/output fast read data */
#define FLASH_SPANSION_CMD_PP         0x02      /* Page program */
#define FLASH_SPANSION_CMD_4PP        0x12      /* 4 byte page program */
#define FLASH_SPANSION_CMD_QPP        0x32      /* Quad input fast program */
#define FLASH_SPANSION_CMD_4QPP       0x34      /* 4 byte quad input fast program */
#define FLASH_SPANSION_CMD_4QEPP      0x3e      /* 4 byte extended quad input fast program */
#define FLASH_SPANSION_CMD_PGSP       0x85      /* Read volatile configuration */
#define FLASH_SPANSION_CMD_P4E        0x20      /* 4KB sector erase */
#define FLASH_SPANSION_CMD_4P4E       0x21      /* 4 byte 4 KB sector erase */
#define FLASH_SPANSION_CMD_BE         0x60      /* Chip erase */
#define FLASH_SPANSION_CMD_SE         0xd8      /* 64KB sector erase*/
#define FLASH_SPANSION_CMD_4SE        0xdc      /* 4 byte 64KB sector erase */
#define FLASH_SPANSION_CMD_ERSP       0x75      /* Program/erase suspend */
#define FLASH_SPANSION_CMD_ERRS       0x7a      /* Program/erase resume */


#endif // _IMXRT_LUT_H_
