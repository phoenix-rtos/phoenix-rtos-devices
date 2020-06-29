/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash Configurator
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdlib.h>
#include <errno.h>

#include "lut.h"
#include "flash_config.h"


static int flash_getWindbondConfig(flash_context_t *ctx)
{
	switch (GET_DEVICE_ID(ctx->flashID)) {
		case WINDBOND_W25Q32JV_IQ:
			ctx->properties.size = 0x400000;
			ctx->properties.page_size = 0x100;
			ctx->properties.sector_size = 0x1000;
			ctx->buff = malloc(ctx->properties.sector_size);

			if (ctx->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


static void flash_setWindbondLUT(flash_context_t *ctx)
{
	/* QUAD Fast Read */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_QIOR, LUT_CMD_ADDR, LUT_PAD4, 0x18); /* 0xa1804eb */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 1] = LUT_INSTR(LUT_CMD_MODE8, LUT_PAD4, 0x04, LUT_CMD_DUMMY, LUT_PAD4, 0x04); /* 0x32061ef4 */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 2] = LUT_INSTR(LUT_CMD_READ, LUT_PAD4, 0x04, 0, 0, 0); /* 0x2604 */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, QUAD_FAST_READ_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID], 1);

	/* Read Status Register */
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_RDSR1, LUT_CMD_READ, LUT_PAD1, 0x04); /* 0x24040405 */
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, READ_STATUS_REG_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID], 1);

	/* Write Enable */
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_WREN, 0, 0, 0); /* 0x406 */
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, WRITE_ENABLE_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID], 1);

	/* Sector Erase */
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_P4E, LUT_CMD_ADDR, LUT_PAD1, LUT_3B_ADDR); /* 0x8180420 */
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 3] = 0;

	/* Block Erase */
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_SE, LUT_CMD_ADDR, LUT_PAD1, LUT_3B_ADDR); /* 0x81804d8 */
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 3] = 0;

	/* Quad Input Page Program */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_QPP, LUT_CMD_ADDR, LUT_PAD1, LUT_3B_ADDR); /* 0x8180432 */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 1] = LUT_INSTR(LUT_CMD_WRITE, LUT_PAD4, 0x04, 0,0,0); /* 0x2204 */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, PAGE_PROGRAM_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID], 1);

	/* Chip Erase */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_BE, 0, 0, 0); /* 0x0460 */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 1] = LUT_INSTR(LUT_CMD_WRITE, LUT_PAD1, 0x04, 0, 0, 0); /* 0x2004 */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 3] = 0;

	/* READ JEDEC ID*/
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_RDID, LUT_CMD_READ, LUT_PAD1, 0x04); /* 0x2404049f */
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, READ_JEDEC_ID_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID], 1);
}


static int flash_getIssiConfig(flash_context_t *ctx)
{
	switch (GET_DEVICE_ID(ctx->flashID)) {
		case ISSI_DEV_IS25WP064A:
			ctx->properties.size = 0x800000;
			ctx->properties.page_size = 0x100;
			ctx->properties.sector_size = 0x1000;
			ctx->buff = malloc(ctx->properties.sector_size);

			if (ctx->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


static void flash_setIssiLut(flash_context_t *ctx)
{
	// TODO: set ISSI Lut
}


static int flash_getMicronConfig(flash_context_t *ctx)
{
	switch (GET_DEVICE_ID(ctx->flashID)) {
		case MICRON_MT25QL512ABB:
			ctx->properties.size = 0x4000000;
			ctx->properties.page_size = 0x100;
			ctx->properties.sector_size = 0x1000;
			ctx->buff = malloc(ctx->properties.sector_size);

			if (ctx->buff == NULL)
				return -ENOMEM;
			break;

		default :
			return -ENODEV;
	}

	return EOK;
}


static void flash_setMicronLUT(flash_context_t *ctx)
{
	/* 4 - Byte QUAD Fast Read */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_4QIOR, LUT_CMD_ADDR, LUT_PAD4, LUT_4B_ADDR); /* 0xa2004ec */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 1] = LUT_INSTR(LUT_CMD_DUMMY, LUT_PAD4, 0xa, LUT_CMD_READ, LUT_PAD4, 0x04); /* 0x2604320a */
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, QUAD_FAST_READ_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * QUAD_FAST_READ_SEQ_ID], 1);

	/* Read Status Register */
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_RDSR1, LUT_CMD_READ, LUT_PAD1, 0x04); /* 0x24040405 */
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, READ_STATUS_REG_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * READ_STATUS_REG_SEQ_ID], 1);

	/* Write Enable */
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_WREN, 0, 0, 0); /* 0x406 */
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, WRITE_ENABLE_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * WRITE_ENABLE_SEQ_ID], 1);

	/* 4 - Byte Sector Erase */
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_4P4E, LUT_CMD_ADDR, LUT_PAD1, LUT_4B_ADDR); /* 0x8200421 */
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * SECTOR_ERASE_SEQ_ID + 3] = 0;

	/* 4 - Byte Block Erase */
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_4SE, LUT_CMD_ADDR, LUT_PAD1, LUT_4B_ADDR); /* 0x82004dc */
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * BLOCK_ERASE_SEQ_ID + 3] = 0;

	/* 4 - Byte Quad Extended Page Program */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_4QEPP, LUT_CMD_ADDR, LUT_PAD4, LUT_4B_ADDR); /* 0xa20043e */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 1] =  LUT_INSTR(LUT_CMD_WRITE, LUT_PAD4, 0x04, 0, 0, 0); /* 0x2204 */
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, PAGE_PROGRAM_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * PAGE_PROGRAM_SEQ_ID], 1);;

	/* Chip Erase */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_BE, 0, 0, 0); /* 0x0460 */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 1] = LUT_INSTR(LUT_CMD_WRITE, LUT_PAD1, 0x04, 0, 0, 0); /* 0x2004 */
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * CHIP_ERASE_SEQ_ID + 3] = 0;

	/* READ JEDEC ID*/
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID] = LUT_INSTR(LUT_CMD_CMD, LUT_PAD1, FLASH_SPANSION_CMD_RDID, LUT_CMD_READ, LUT_PAD1, 0x04); /* 0x2404049f */
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 1] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 2] = 0;
	ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID + 3] = 0;
	flexspi_norFlashUpdateLUT(ctx->instance, READ_JEDEC_ID_SEQ_ID, (const uint32_t *)&ctx->config.mem.lut[4 * READ_JEDEC_ID_SEQ_ID], 1);
}


int flash_getConfig(flash_context_t *ctx)
{
	switch (GET_MANUFACTURE_ID(ctx->flashID)) {
		case flash_windbond:
			if (flash_getWindbondConfig(ctx) < 0)
				return -ENODEV;
			flash_setWindbondLUT(ctx);
			break;

		case flash_issi:
			if (flash_getIssiConfig(ctx) < 0)
				return -ENODEV;
			flash_setIssiLut(ctx);
			break;

		case flash_micron:
			if (flash_getMicronConfig(ctx) < 0)
				return -ENODEV;
			flash_setMicronLUT(ctx);
			break;

		default:
			return -ENODEV;
	}

	return EOK;
}
