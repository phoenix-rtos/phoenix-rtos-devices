/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash server
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <meterfs.h>
#include <ptable.h>

#include "flashdrv.h"
#include "imxrt-flashsrv.h"


#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)

#define METERFS_STACKSZ              1024
#define THREAD_STACKSZ               512
#define THREAD_PRIORITY              4

#define FLASH_MEMORIES_NO            2


enum { flash_memory_unactive = 0, flash_memory_active = 0xff };


typedef struct {
	ptable_partition_t *pHeader;

	oid_t oid;
	uint8_t fID;
	uint8_t pStatus;

	void *fsCtx;
} flashsrv_partition_t;


typedef struct {
	flash_context_t ctx;
	flashsrv_partition_t *parts;

	uint32_t pCnt;
	uint8_t fStatus;
	uint8_t currPart;
	uint8_t rawActive;

	oid_t fOid;
	uint32_t rawPort;

	handle_t lock;
} flash_memory_t;


struct {
	flash_memory_t flash_memories[FLASH_MEMORIES_NO];
	uint32_t flexspi_addresses[FLASH_MEMORIES_NO];
} flashsrv_common;



/* Flash functions */

static ssize_t flashsrv_bufferedPagesWrite(uint8_t fID, size_t offset, const char *data, size_t size)
{
	flash_memory_t *flash_memory;

	if (FLASH_MEMORIES_NO <= fID)
		return -EINVAL;

	flash_memory = flashsrv_common.flash_memories + fID;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_bufferedPagesWrite(&flash_memory->ctx, offset, data, size);
}


static ssize_t flashsrv_read(uint8_t fID, size_t offset, char *data, size_t size)
{
	flash_memory_t *flash_memory;

	if (FLASH_MEMORIES_NO <= fID)
		return -EINVAL;

	flash_memory = flashsrv_common.flash_memories + fID;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_readData(&flash_memory->ctx, offset, data, size);
}


static int flashsrv_eraseSector(unsigned char fID, unsigned int offs)
{
	flash_memory_t *flash_memory;

	if (FLASH_MEMORIES_NO <= fID)
		return -EINVAL;

	flash_memory = flashsrv_common.flash_memories + fID;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_sectorErase(&flash_memory->ctx, offs);
}


static int flashsrv_chipErase(unsigned char fID)
{
	flash_memory_t *flash_memory;

	if (FLASH_MEMORIES_NO <= fID)
		return -EINVAL;

	flash_memory = flashsrv_common.flash_memories + fID;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_chipErase(&flash_memory->ctx);
}


/* Callbacks to meterfs - wrappers for basic flash functions */

static ssize_t flashsrv_fsWritef0(unsigned int addr, const void *buff, size_t bufflen)
{
	flash_memory_t *flash_memory = flashsrv_common.flash_memories + 0;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_directBytesWrite(&flash_memory->ctx, addr, buff, bufflen);
}


static ssize_t flashsrv_fsWritef1(unsigned int addr, const void *buff, size_t bufflen)
{
	flash_memory_t *flash_memory = flashsrv_common.flash_memories + 1;

	if (flash_memory->fStatus == flash_memory_unactive)
		return -EINVAL;

	return flash_directBytesWrite(&flash_memory->ctx, addr, buff, bufflen);
}


static ssize_t flashsrv_fsReadf0(unsigned int addr, void *buff, size_t bufflen)
{
	return flashsrv_read(0, addr, buff, bufflen);
}


static ssize_t flashsrv_fsReadf1(unsigned int addr, void *buff, size_t bufflen)
{
	return flashsrv_read(1, addr, buff, bufflen);
}


static int flashsrv_fsEraseSectorf0(unsigned int addr)
{
	return flashsrv_eraseSector(0, addr);
}


static int flashsrv_fsEraseSectorf1(unsigned int addr)
{
	return flashsrv_eraseSector(1, addr);
}


static int flashsrv_fsPartitionErase(uint8_t fID)
{
	int i, err;
	uint32_t sectorsNb;
	flash_memory_t *flash_memory;

	if (FLASH_MEMORIES_NO <= fID)
		return -EINVAL;

	flash_memory = flashsrv_common.flash_memories + fID;
	sectorsNb = flash_memory->parts[flash_memory->currPart].pHeader->size / flash_memory->ctx.properties.sector_size;

	for (i = 0; i < sectorsNb; ++i) {
		if ((err = flashsrv_eraseSector(fID, flash_memory->parts[flash_memory->currPart].pHeader->offset + flash_memory->ctx.properties.sector_size * i)) < 0)
			return err;
	}

	return 0;
}


static int flashsrv_fsPartitionErasef0(void)
{
	return flashsrv_fsPartitionErase(0);
}


static int flashsrv_fsPartitionErasef1(void)
{
	return flashsrv_fsPartitionErase(1);
}


static void flashsrv_devCtl(flash_memory_t *memory, msg_t *msg)
{
	uint8_t fID;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	if (memory == NULL) {
		odevctl->err = -EINVAL;
		return;
	}

	if ((fID = idevctl->oid.id) >= FLASH_MEMORIES_NO) {
		odevctl->err = -EINVAL;
		return;
	}

	switch (idevctl->type) {
		case flashsrv_devctl_properties:
			TRACE("imxrt-flashsrv: flashsrv_devctl_properties, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			odevctl->properties.size = memory->ctx.properties.size;
			odevctl->properties.psize = memory->ctx.properties.page_size;
			odevctl->properties.ssize = memory->ctx.properties.sector_size;
			odevctl->properties.offs = 0;
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_sync:
			TRACE("imxrt-flashsrv: flashsrv_devctl_sync, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			flash_sync(&memory->ctx);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_eraseSector:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseSector - adrr: %u, id: %u, port: %u.", idevctl->erase.addr, idevctl->oid.id, idevctl->oid.port);
			if (idevctl->erase.addr >= memory->ctx.properties.size) {
				odevctl->err = -EINVAL;
				break;
			}
			flashsrv_eraseSector(memory->fOid.id, idevctl->erase.addr);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_erasePartition:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseChip, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			flashsrv_chipErase(memory->fOid.id);
			odevctl->err = EOK;
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


static void flashsrv_rawCtl(flash_memory_t *memory, msg_t *msg)
{
	int i;
	uint32_t sNb;
	uint8_t partID;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	if (memory == NULL) {
		odevctl->err = -EINVAL;
		return;
	}

	if ((partID = idevctl->oid.id) >= memory->pCnt) {
		odevctl->err = -EINVAL;
		return;
	}

	if (memory->parts[partID].pStatus == flash_memory_unactive) {
		odevctl->err = -EINVAL;
		return;
	}

	switch (idevctl->type) {
		case flashsrv_devctl_properties:
			TRACE("imxrt-flashsrv: flashsrv_devctl_properties, id: %u, port: %u.", partID, idevctl->oid.port);
			odevctl->properties.size = memory->parts[partID].pHeader->size;
			odevctl->properties.psize = memory->ctx.properties.page_size;
			odevctl->properties.ssize = memory->ctx.properties.sector_size;
			odevctl->properties.offs = memory->parts[partID].pHeader->offset;
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_sync:
			TRACE("imxrt-flashsrv: flashsrv_devctl_sync, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			flash_sync(&memory->ctx);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_eraseSector:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseSector - adrr: %u, id: %u, port: %u.", idevctl->erase.addr + memory->parts[partID].pHeader->offset,
					partID, idevctl->oid.port);

			if (idevctl->erase.addr	> memory->parts[idevctl->oid.id].pHeader->size) {
				odevctl->err = -EINVAL;
				break;
			}

			flashsrv_eraseSector(memory->fOid.id, memory->parts[partID].pHeader->offset + idevctl->erase.addr);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_erasePartition:
			TRACE("imxrt-flashsrv: flashsrv_devctl_erasePartition, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			sNb = memory->parts[partID].pHeader->size / memory->ctx.properties.sector_size;

			for (i = 0; i < sNb; ++i)
				flashsrv_eraseSector(memory->fOid.id, memory->parts[partID].pHeader->offset + i * memory->ctx.properties.sector_size);

			odevctl->err = EOK;
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


/* Threads */

static void flashsrv_meterfsThread(void *arg)
{
	msg_t msg;
	unsigned long rid;

	flashsrv_partition_t *part = (flashsrv_partition_t *)arg;

	meterfs_i_devctl_t *idevctl = (meterfs_i_devctl_t *)msg.i.raw;
	meterfs_o_devctl_t *odevctl = (meterfs_o_devctl_t *)msg.o.raw;

	while (1) {
		while (msgRecv(part->oid.port, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtRead:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_readFile(msg.i.io.oid.id, msg.i.io.offs, msg.o.data, msg.o.size, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtWrite:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_writeFile(msg.i.io.oid.id, msg.i.data, msg.i.size, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtLookup:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.lookup.err = meterfs_lookup(msg.i.data, &msg.o.lookup.fil.id, (meterfs_ctx_t *)part->fsCtx);
				msg.o.lookup.fil.port = part->oid.port;
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				memcpy(&msg.o.lookup.dev, &msg.o.lookup.fil, sizeof(oid_t));
				break;

			case mtOpen:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_open(msg.i.openclose.oid.id, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtClose:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_close(msg.i.openclose.oid.id, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtDevCtl:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				odevctl->err = meterfs_devctl(idevctl, odevctl, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}
		msgRespond(part->oid.port, &msg, rid);
	}
}


static int flashsrv_verifyRawIO(const flash_memory_t *memory, msg_t *msg, size_t *size)
{
	if (msg->i.io.oid.id >= memory->pCnt) {
		msg->o.io.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->i.io.oid.id].pStatus == flash_memory_unactive) {
		msg->o.io.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->i.io.oid.id].pHeader->size <= msg->i.io.offs) {
		msg->o.io.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->i.io.oid.id].pHeader->size < (msg->i.io.offs + *size))
		*size = memory->parts[msg->i.io.oid.id].pHeader->size - msg->i.io.offs;

	return EOK;
}


static void flashsrv_rawThread(void *arg)
{
	msg_t msg;
	unsigned long rid;
	uint32_t beginAddr;

	flash_memory_t *memory = (flash_memory_t *)arg;

	while (1) {
		while (msgRecv(memory->rawPort, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtRead:
				if (flashsrv_verifyRawIO(memory, &msg, &msg.o.size) < 0)
					break;

				beginAddr = memory->parts[msg.i.io.oid.id].pHeader->offset;
				mutexLock(memory->lock);
				msg.o.io.err = flashsrv_read(memory->fOid.id, beginAddr + msg.i.io.offs, msg.o.data, msg.o.size);
				mutexUnlock(memory->lock);
				break;

			case mtWrite:
				if (flashsrv_verifyRawIO(memory, &msg, &msg.i.size) < 0)
					break;

				beginAddr = memory->parts[msg.i.io.oid.id].pHeader->offset;
				mutexLock(memory->lock);
				msg.o.io.err = flashsrv_bufferedPagesWrite(memory->fOid.id, beginAddr + msg.i.io.offs, msg.i.data, msg.i.size);
				mutexUnlock(memory->lock);
				break;

			case mtDevCtl:
				mutexLock(memory->lock);
				flashsrv_rawCtl(memory, &msg);
				mutexUnlock(memory->lock);
				break;
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}
		msgRespond(memory->rawPort, &msg, rid);
	}
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	unsigned long rid;

	flash_memory_t *memory = (flash_memory_t *)arg;

	while (1) {
		while (msgRecv(memory->fOid.port, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtRead:
				mutexLock(memory->lock);
				msg.o.io.err = flashsrv_read(memory->fOid.id, msg.i.io.offs, msg.o.data, msg.o.size);
				mutexUnlock(memory->lock);
				break;

			case mtWrite:
				mutexLock(memory->lock);
				msg.o.io.err = flashsrv_bufferedPagesWrite(memory->fOid.id, msg.i.io.offs, msg.i.data, msg.i.size);
				mutexUnlock(memory->lock);
				break;

			case mtDevCtl:
				mutexLock(memory->lock);
				flashsrv_devCtl(memory, &msg);
				mutexUnlock(memory->lock);
				break;
			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;
			default:
				msg.o.io.err = -ENOSYS;
				break;
		}
		msgRespond(memory->fOid.port, &msg, rid);
	}
}


/* Initialization functions */

static int flashsrv_initMeterfs(flashsrv_partition_t *part)
{
	meterfs_ctx_t *ctx;

	if ((part->fsCtx = calloc(1, sizeof(meterfs_ctx_t))) == NULL) {
		LOG_ERROR("imxrt-flashsrv: cannot allocate memory.");
		return -ENOMEM;
	}

	ctx = (meterfs_ctx_t *)part->fsCtx;

	ctx->sz = part->pHeader->size;
	ctx->offset = part->pHeader->offset;
	ctx->sectorsz = flashsrv_common.flash_memories[part->fID].ctx.properties.sector_size;

	if (part->fID == 0) {
		ctx->read = flashsrv_fsReadf0;
		ctx->write = flashsrv_fsWritef0;
		ctx->partitionErase = flashsrv_fsPartitionErasef0;
		ctx->eraseSector = flashsrv_fsEraseSectorf0;
	}
	else if (part->fID == 1) {
		ctx->read = flashsrv_fsReadf1;
		ctx->write = flashsrv_fsWritef1;
		ctx->partitionErase = flashsrv_fsPartitionErasef1;
		ctx->eraseSector = flashsrv_fsEraseSectorf1;
	}

	if (meterfs_init(ctx) < 0) {
		LOG_ERROR("imxrt-flashsrv: init meterfs at flash: %u, partition: %u.", part->fID, part->oid.id);
		return -1;
	}

	return 0;
}


static int flashsrv_mountPart(flashsrv_partition_t *part)
{
	void *mem;
	int res = EOK;
	const uint8_t MAX_PATH_SIZE = 32;
	char path[MAX_PATH_SIZE];

	snprintf(path, MAX_PATH_SIZE, "/dev/flash%u.%s", part->fID, part->pHeader->name);

	switch (part->pHeader->type) {
		/* Raw partitions locate within single flash chip are handled by one thread */
		case ptable_raw:
			part->fsCtx = NULL;
			part->oid.port = flashsrv_common.flash_memories[part->fID].rawPort;

			if ((res = create_dev(&part->oid, path)) < 0) {
				LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, res);
				return res;
			}

			if (!flashsrv_common.flash_memories[part->fID].rawActive) {
				flashsrv_common.flash_memories[part->fID].rawActive = 1;
				if ((mem = malloc(THREAD_STACKSZ)) == NULL) {
					LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
					return -ENOMEM;
				}

				beginthread(flashsrv_rawThread, THREAD_PRIORITY, mem, THREAD_STACKSZ, (void *)&flashsrv_common.flash_memories[part->fID]);
			}
			part->pStatus = flash_memory_active;
			break;

		/* Each meterfs partiton is handled by separete thread */
		case ptable_meterfs:
			if ((res = flashsrv_initMeterfs(part)) < 0)
				return res;

			portCreate(&part->oid.port);
			portRegister(part->oid.port, path, NULL);
			if ((res = create_dev(&part->oid, path)) < 0) {
				LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, res);
				return res;
			}

			if ((mem = malloc(METERFS_STACKSZ)) == NULL) {
				LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
				return -ENOMEM;
			}

			beginthread(flashsrv_meterfsThread, THREAD_PRIORITY, mem, METERFS_STACKSZ, (void *)part);
			part->pStatus = flash_memory_active;
			break;

		default:
			res = -EINVAL;
			break;
	}

	return res;
}


static int flashsrv_partsInit(void)
{
	int i, j;

	flash_memory_t *memory;
	memory_properties_t memProp;
	ptable_partition_t *pHeaders;

	for (i = 0; i < FLASH_MEMORIES_NO; ++i) {
		memory = flashsrv_common.flash_memories + i;
		if (memory->fStatus != flash_memory_active)
			continue;

		/* Read partition table */
		memProp.memSize = memory->ctx.properties.size;
		memProp.sectorSize = memory->ctx.properties.sector_size;
		memProp.read = i ? flashsrv_fsReadf1 : flashsrv_fsReadf0 ;

		if ((pHeaders = ptable_readPartitions(&memory->pCnt, &memProp)) == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot initialize partition table on flash %d, wrong attributes", i);
			continue;
		}

		if ((memory->parts = calloc(memory->pCnt, sizeof(flashsrv_partition_t))) == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot allocate memory.");
			free(pHeaders);
			return -ENOMEM;
		}

		/* Initialize partitions */
		for (j = 0; j < memory->pCnt; ++j) {
			memory->parts[j].fID = i;
			memory->parts[j].oid.id = j;
			memory->parts[j].pHeader = pHeaders + j;

			if (flashsrv_mountPart(&memory->parts[j]) < 0) {
				LOG_ERROR("imxrt-flashsrv: partition %s at flash %d is not mounted.", pHeaders[j].name, i);
				memory->parts[j].pStatus = flash_memory_unactive;
				free(memory->parts[j].fsCtx);
			}
		}

		flash_sync(&memory->ctx);
	}

	return EOK;
}


static int flashsrv_flashMemoriesInit(void)
{
	char path[32];
	int i, err = EOK;
	oid_t odir = { 0 };

	flash_memory_t *memory;

	flashsrv_common.flexspi_addresses[0] = FLASH_EXT_DATA_ADDRESS;
	flashsrv_common.flexspi_addresses[1] = FLASH_INTERNAL_DATA_ADDRESS;

	/* Wait on root */
	while (lookup("/", NULL, &odir) < 0)
		usleep(100000);

	/* Initialize flash memories */
	for (i = 0; i < FLASH_MEMORIES_NO; ++i) {
		memory = flashsrv_common.flash_memories + i;

		if (mutexCreate(&memory->lock) != EOK)
			return -ENOENT;

		portCreate(&memory->fOid.port);
		portCreate(&memory->rawPort);

		memory->fOid.id = i;
		memory->rawActive = 0;
		memory->pCnt = 0;

		sprintf(path, "/dev/flash%d", memory->fOid.id);

		if ((err = create_dev(&memory->fOid, path)) < 0) {
			LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, err);
			return err;
		}

		memory->ctx.address = flashsrv_common.flexspi_addresses[i];

		if ((err = flash_init(&memory->ctx)) == EOK) {
			memory->fStatus = flash_memory_active;
		}
		else {
			memory->fStatus = flash_memory_unactive;
			LOG_ERROR("imxrt-flashsrv: %s - has not been initialized correctly, err: %d.", path, err);
		}
	}

	return EOK;
}


int main(void)
{
	void *mem;

	if (flashsrv_flashMemoriesInit() != EOK) {
		LOG_ERROR("imxrt-flashsrv: flash memories were not initialized correctly.\n");
		return -1;
	}

	if (flashsrv_partsInit() != EOK) {
		LOG_ERROR("imxrt-flashsrv: partitions were not initialized correctly.\n");
		return -1;
	}

	printf("imxrt-flashsrv: initialized.\n");

	/* flashsrv_devThread handles requests associated with the whole single flash chip */
	if (flashsrv_common.flash_memories[1].fStatus == flash_memory_active) {
		if ((mem = malloc(THREAD_STACKSZ)) == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
			return 0;
		}
		beginthread(flashsrv_devThread, THREAD_PRIORITY, mem, THREAD_STACKSZ, (void *)&flashsrv_common.flash_memories[0]);
		flashsrv_devThread(&flashsrv_common.flash_memories[1]);
	}
	else if (flashsrv_common.flash_memories[0].fStatus == flash_memory_active) {
		flashsrv_devThread(&flashsrv_common.flash_memories[0]);
	}
	else {
		printf("imxrt-flashsrv: end\n");
	}


	return EOK;
}
