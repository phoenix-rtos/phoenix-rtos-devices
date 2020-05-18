/*
 * Phoenix-RTOS
 *
 * i.MX RT flash driver tests
 *
 * Copyright 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include "../flashdrv.h"


#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)


int test_flashdrv_init(uint32_t addr)
{
	int res = EOK;
	flash_context_t ctx;

	ctx.address = addr;
	if ((res = flash_init(&ctx)) < 0) {
		flash_contextDestroy(&ctx);
		return res;
	}

	/* Verification */
	if (ctx.properties.page_size == 0 || ctx.properties.sector_size == 0 || ctx.properties.size == 0){
		flash_contextDestroy(&ctx);
		return -1;
	}

	flash_contextDestroy(&ctx);

	return res;

}


int test_flashdrv_writeAndReadPage(uint32_t addr)
{
	int res = EOK;
	size_t offs = 0x3000;

	flash_context_t ctx;

	char *writeBuff;
	char *readBuff;

	/* Initialize */
	ctx.address = addr;
	if ((res = flash_init(&ctx)) < 0) {
		flash_contextDestroy(&ctx);
		return res;
	}

	if ((writeBuff = (char *)malloc(ctx.properties.page_size)) == NULL) {
		LOG_ERROR("cannot allocate memory.");
		flash_contextDestroy(&ctx);
		return -ENOMEM;
	}

	memset(writeBuff, 0xbc, ctx.properties.page_size);

	/* Write page */
	if (flash_bufferedPagesWrite(&ctx, offs, writeBuff, ctx.properties.page_size) != ctx.properties.page_size){
		LOG_ERROR("writing page failed.");
		flash_contextDestroy(&ctx);
		free(writeBuff);
		return -1;
	}

	flash_sync(&ctx);

	/* Read data */
	if ((readBuff = (char *)malloc(ctx.properties.page_size)) == NULL) {
		LOG_ERROR("cannot allocate memory.");
		flash_contextDestroy(&ctx);
		free(writeBuff);
		return -ENOMEM;
	}

	flash_readData(&ctx, offs, readBuff, ctx.properties.page_size);

	/* Verification */
	if (memcmp(readBuff, writeBuff, ctx.properties.page_size) != 0)
		res = -1;

	flash_contextDestroy(&ctx);
	free(writeBuff);
	free(readBuff);

	return res;
}


int test_flashdrv_writeAndReadBytes(uint32_t addr)
{
	int res = EOK, i;
	size_t offs = 0x100;
	flash_context_t ctx;

	const uint8_t CHECK_VALUE = 0xcf;
	const uint8_t BUFF_SIZE = 12;
	char buff[BUFF_SIZE];


	/* Initialize */
	ctx.address = addr;
	if ((res = flash_init(&ctx)) < 0) {
		flash_contextDestroy(&ctx);
		return res;
	}

	memset(buff, CHECK_VALUE, BUFF_SIZE);

	/* Write page */
	if (flash_directBytesWrite(&ctx, offs, buff, BUFF_SIZE) != BUFF_SIZE){
		LOG_ERROR("writing page failed, err - %d.", res);
		flash_contextDestroy(&ctx);
		return -1;
	}

	/* Read data */
	memset(buff, 0, BUFF_SIZE);

	flash_readData(&ctx, offs, buff, BUFF_SIZE);

	/* Verification */
	for (i = 0; i < BUFF_SIZE; ++i) {
		if (buff[i] != CHECK_VALUE) {
			res = -1;
			break;
		}
	}

	flash_contextDestroy(&ctx);

	return res;
}

int test_flashdrv_eraseSector(uint32_t addr)
{
	int res = EOK, i;
	uint32_t sector = 5;;
	flash_context_t ctx;

	char *readBuff;

	/* Initialize */
	ctx.address = addr;
	if ((res = flash_init(&ctx)) < 0) {
		flash_contextDestroy(&ctx);
		return res;
	}

	flash_sync(&ctx);
	flash_sectorErase(&ctx, sector * ctx.properties.sector_size);

	/* Read data */
	readBuff = (void *)(addr + sector * ctx.properties.sector_size);

	/* Verification */
	for (i = 0; i < ctx.properties.sector_size; ++i) {
		if (readBuff[i] != 0xff) {
			res = -1;
			break;
		}
	}

	flash_contextDestroy(&ctx);

	return res;
}


int test_flashdrv_eraseChip(uint32_t addr)
{
	int res = EOK, i, j;
	uint16_t pageID;
	flash_context_t ctx;

	const int MAX_BUFFERS = 5;

	char *readBuffs[MAX_BUFFERS];

	/* Initialize */
	ctx.address = addr;
	if ((res = flash_init(&ctx)) < 0) {
		flash_contextDestroy(&ctx);
		return res;
	}

	/* Erase whole chip */
	flash_sync(&ctx);

	res = flash_chipErase(&ctx);

	/* Check random pages */
	for (i = 0, pageID = 0; i < MAX_BUFFERS - 2; ++i) {
		pageID += 34;
		readBuffs[i] = (void *)(addr + pageID * ctx.properties.page_size);
	}

	readBuffs[3] = (void *)(addr + 0x3000);
	readBuffs[4] = (void *)(addr + 0x8000);

	/* Verification */
	for (i = 0; i < ctx.properties.page_size; ++i) {
		for (j = 0; j < MAX_BUFFERS; ++j){
			if (readBuffs[j][i] != 0xff) {
				res = -1;
				break;
			}
		}
	}

	flash_contextDestroy(&ctx);

	return res;
}
