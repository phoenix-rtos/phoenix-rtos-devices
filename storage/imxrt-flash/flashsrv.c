/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash server
 *
 * Copyright 2019-2023 Phoenix Systems
 * Author: Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
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
#include <board_config.h>

#include "fspi.h"
#include "flashdrv.h"
#include "imxrt-flashsrv.h"


/* clang-format off */
#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */

#define METERFS_STACKSZ   1024
#define THREAD_STACKSZ    1024
#define FLASH_MEMORIES_NO (FLEXSPI_COUNT)

#ifndef IMXRT_FLASH_PRIO
/*
 * Threads/processes including and below this priority must be run only from RAM to avoid preempting imxrt-flash
 * during write to flash, threads/processes above that priority shall be run either from XIP or RAM.
 */
#define IMXRT_FLASH_PRIO 3
#endif

/* clang-format off */
enum { flashsrv_memory_inactive = 0, flashsrv_memory_active = 0xff };
/* clang-format on */

typedef struct {
	ptable_part_t *pHeader;

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
} flashsrv_memory_t;


struct {
	flashsrv_memory_t flash_memories[FLASH_MEMORIES_NO];
	uint32_t flexspi_addresses[FLASH_MEMORIES_NO];
	flashsrv_partitionOps_t ops;
} flashsrv_common;


/* Flash functions */

static int flashsrv_getFlashMemory(uint8_t fID, flashsrv_memory_t **pFlashMemory)
{
	flashsrv_memory_t *mem;

	if (FLASH_MEMORIES_NO <= fID) {
		return -EINVAL;
	}

	mem = flashsrv_common.flash_memories + fID;

	if (mem->fStatus == flashsrv_memory_inactive) {
		return -ENODEV;
	}

	*pFlashMemory = mem;

	return EOK;
}

/* Buffered access */

static ssize_t flashsrv_bufferedWrite(uint8_t fID, size_t offset, const void *data, size_t size)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_bufferedWrite(&mem->ctx, offset, data, size) : res;
}


static ssize_t flashsrv_bufferedRead(uint8_t fID, size_t offset, void *data, size_t size)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_bufferedRead(&mem->ctx, offset, data, size) : res;
}


static ssize_t flashsrv_bufferedSync(uint8_t fID)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_sync(&mem->ctx) : res;
}


/* Direct access */

static ssize_t flashsrv_directWrite(uint8_t fID, size_t offset, const void *data, size_t size)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_directWrite(&mem->ctx, offset, data, size) : res;
}


static ssize_t flashsrv_directRead(uint8_t fID, size_t offset, void *data, size_t size)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_directRead(&mem->ctx, offset, data, size) : res;
}


static int flashsrv_eraseSector(uint8_t fID, size_t offs)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_sectorErase(&mem->ctx, offs) : res;
}


static int flashsrv_chipErase(uint8_t fID)
{
	flashsrv_memory_t *mem;
	int res = flashsrv_getFlashMemory(fID, &mem);
	return (res == EOK) ? flash_chipErase(&mem->ctx) : res;
}


/* Callbacks to meterfs - wrappers for basic flash functions */

static ssize_t flashsrv_fsWritef0(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	(void)devCtx;

	return flashsrv_directWrite(0, offs, buff, bufflen);
}


static ssize_t flashsrv_fsWritef1(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	(void)devCtx;

	return flashsrv_directWrite(1, offs, buff, bufflen);
}


static ssize_t flashsrv_fsReadf0(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	(void)devCtx;

	return flashsrv_directRead(0, offs, buff, bufflen);
}


static ssize_t flashsrv_fsReadf1(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	(void)devCtx;

	return flashsrv_directRead(1, offs, buff, bufflen);
}


static int flashsrv_fsEraseSectorf0(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	(void)devCtx;

	return flashsrv_eraseSector(0, offs);
}


static int flashsrv_fsEraseSectorf1(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	(void)devCtx;

	return flashsrv_eraseSector(1, offs);
}


static void flashsrv_devCtl(flashsrv_memory_t *memory, msg_t *msg)
{
	uint8_t fID;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	if (memory == NULL) {
		odevctl->err = -EINVAL;
		return;
	}

	fID = idevctl->oid.id;
	if (fID >= FLASH_MEMORIES_NO) {
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
			odevctl->err = flash_sync(&memory->ctx);
			break;

		case flashsrv_devctl_eraseSector:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseSector - addr: %u, id: %u, port: %u.", idevctl->addr, idevctl->oid.id, idevctl->oid.port);
			if (idevctl->addr >= memory->ctx.properties.size) {
				odevctl->err = -EINVAL;
				break;
			}
			flashsrv_eraseSector(memory->fOid.id, idevctl->addr);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_erasePartition:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseChip, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			flashsrv_chipErase(memory->fOid.id);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_directWrite:
			TRACE("imxrt-flashsrv: flashsrv_devctl_directWrite, addr: %u, size: %u, id: %u, port: %u.",
				idevctl->addr, msg->i.size, idevctl->oid.id, idevctl->oid.port);

			if (idevctl->addr >= memory->ctx.properties.size) {
				odevctl->err = -EINVAL;
				break;
			}

			odevctl->err = flashsrv_directWrite(memory->fOid.id, idevctl->addr, msg->i.data, msg->i.size);
			break;

		case flashsrv_devctl_directRead:
			TRACE("imxrt-flashsrv: flashsrv_devctl_directRead, addr: %u, size: %u, id: %u, port: %u.",
				idevctl->addr, msg->o.size, idevctl->oid.id, idevctl->oid.port);

			if (idevctl->addr >= memory->ctx.properties.size) {
				odevctl->err = -EINVAL;
				break;
			}

			odevctl->err = flashsrv_directRead(memory->fOid.id, idevctl->addr, msg->o.data, msg->o.size);
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


static void flashsrv_rawCtl(flashsrv_memory_t *memory, msg_t *msg)
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

	partID = idevctl->oid.id;
	if (partID >= memory->pCnt) {
		odevctl->err = -EINVAL;
		return;
	}

	if (memory->parts[partID].pStatus == flashsrv_memory_inactive) {
		odevctl->err = -ENODEV;
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
			odevctl->err = flash_sync(&memory->ctx);
			break;

		case flashsrv_devctl_eraseSector:
			TRACE("imxrt-flashsrv: flashsrv_devctl_eraseSector - addr: %u, id: %u, port: %u.",
				idevctl->addr + memory->parts[partID].pHeader->offset, partID, idevctl->oid.port);

			if (idevctl->addr >= memory->parts[idevctl->oid.id].pHeader->size) {
				odevctl->err = -EINVAL;
				break;
			}

			flashsrv_eraseSector(memory->fOid.id, memory->parts[partID].pHeader->offset + idevctl->addr);
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_erasePartition:
			TRACE("imxrt-flashsrv: flashsrv_devctl_erasePartition, id: %u, port: %u.", idevctl->oid.id, idevctl->oid.port);
			sNb = memory->parts[partID].pHeader->size / memory->ctx.properties.sector_size;

			for (i = 0; i < sNb; ++i) {
				flashsrv_eraseSector(memory->fOid.id, memory->parts[partID].pHeader->offset + i * memory->ctx.properties.sector_size);
			}

			odevctl->err = EOK;
			break;

		case flashsrv_devctl_directWrite:
			TRACE("imxrt-flashsrv: flashsrv_devctl_directWrite, addr: %u, size: %u, id: %u, port: %u.",
				idevctl->addr + memory->parts[partID].pHeader->offset, msg->i.size, idevctl->oid.id, idevctl->oid.port);

			if (idevctl->addr >= memory->parts[idevctl->oid.id].pHeader->size) {
				odevctl->err = -EINVAL;
				break;
			}

			odevctl->err = flashsrv_directWrite(memory->fOid.id, memory->parts[partID].pHeader->offset + idevctl->addr, msg->i.data, msg->i.size);
			break;

		case flashsrv_devctl_directRead:
			TRACE("imxrt-flashsrv: flashsrv_devctl_directRead, addr: %u, size: %u, id: %u, port: %u.",
				idevctl->addr + memory->parts[partID].pHeader->offset, msg->o.size, idevctl->oid.id, idevctl->oid.port);

			if (idevctl->addr >= memory->parts[idevctl->oid.id].pHeader->size) {
				odevctl->err = -EINVAL;
				break;
			}

			odevctl->err = flashsrv_directRead(memory->fOid.id, memory->parts[partID].pHeader->offset + idevctl->addr, msg->o.data, msg->o.size);
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
	msg_rid_t rid;

	flashsrv_partition_t *part = (flashsrv_partition_t *)arg;

	meterfs_i_devctl_t *idevctl = (meterfs_i_devctl_t *)msg.i.raw;
	meterfs_o_devctl_t *odevctl = (meterfs_o_devctl_t *)msg.o.raw;

	for (;;) {
		if (msgRecv(part->oid.port, &msg, &rid) < 0) {
			continue;
		}

		mutexLock(flashsrv_common.flash_memories[part->fID].lock);

		switch (msg.type) {
			case mtRead:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_readFile(msg.i.io.oid.id, msg.i.io.offs, msg.o.data, msg.o.size, (meterfs_ctx_t *)part->fsCtx);
				break;

			case mtWrite:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_writeFile(msg.i.io.oid.id, msg.i.data, msg.i.size, (meterfs_ctx_t *)part->fsCtx);
				break;

			case mtLookup:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.lookup.err = meterfs_lookup(msg.i.data, &msg.o.lookup.fil.id, (meterfs_ctx_t *)part->fsCtx);
				msg.o.lookup.fil.port = part->oid.port;
				memcpy(&msg.o.lookup.dev, &msg.o.lookup.fil, sizeof(oid_t));
				break;

			case mtOpen:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_open(msg.i.openclose.oid.id, (meterfs_ctx_t *)part->fsCtx);
				break;

			case mtClose:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.io.err = meterfs_close(msg.i.openclose.oid.id, (meterfs_ctx_t *)part->fsCtx);
				break;

			case mtDevCtl:
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				odevctl->err = meterfs_devctl(idevctl, odevctl, (meterfs_ctx_t *)part->fsCtx);
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);

		msgRespond(part->oid.port, &msg, rid);
	}
}


static int flashsrv_verifyRawIO(const flashsrv_memory_t *memory, msg_t *msg, size_t *size)
{
	if (msg->i.io.oid.id >= memory->pCnt) {
		msg->o.io.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->i.io.oid.id].pStatus == flashsrv_memory_inactive) {
		msg->o.io.err = -ENODEV;
		return -ENODEV;
	}

	if (memory->parts[msg->i.io.oid.id].pHeader->size <= msg->i.io.offs) {
		msg->o.io.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->i.io.oid.id].pHeader->size < (msg->i.io.offs + *size)) {
		*size = memory->parts[msg->i.io.oid.id].pHeader->size - msg->i.io.offs;
	}

	return EOK;
}


static void flashsrv_rawThread(void *arg)
{
	int res;
	msg_t msg;
	msg_rid_t rid;
	uint32_t beginAddr;

	flashsrv_memory_t *memory = (flashsrv_memory_t *)arg;

	for (;;) {
		if (msgRecv(memory->rawPort, &msg, &rid) < 0) {
			continue;
		}

		mutexLock(memory->lock);

		switch (msg.type) {
			case mtRead:
				res = flashsrv_verifyRawIO(memory, &msg, &msg.o.size);
				if (res == EOK) {
					beginAddr = memory->parts[msg.i.io.oid.id].pHeader->offset;
					msg.o.io.err = flashsrv_bufferedRead(memory->fOid.id, beginAddr + msg.i.io.offs, msg.o.data, msg.o.size);
				}
				else {
					msg.o.io.err = res;
				}
				break;

			case mtWrite:
				res = flashsrv_verifyRawIO(memory, &msg, &msg.i.size);
				if (res == EOK) {
					beginAddr = memory->parts[msg.i.io.oid.id].pHeader->offset;
					msg.o.io.err = flashsrv_bufferedWrite(memory->fOid.id, beginAddr + msg.i.io.offs, msg.i.data, msg.i.size);
				}
				else {
					msg.o.io.err = res;
				}
				break;

			case mtDevCtl:
				flashsrv_rawCtl(memory, &msg);
				break;

			case mtOpen:
				msg.o.io.err = EOK;
				break;

			case mtClose:
				(void)flashsrv_bufferedSync(memory->fOid.id);
				msg.o.io.err = EOK;
				break;

			case mtSync:
				msg.o.io.err = flashsrv_bufferedSync(memory->fOid.id);
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		mutexUnlock(memory->lock);

		msgRespond(memory->rawPort, &msg, rid);
	}
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;

	flashsrv_memory_t *memory = (flashsrv_memory_t *)arg;

	for (;;) {
		if (msgRecv(memory->fOid.port, &msg, &rid) < 0) {
			continue;
		}

		mutexLock(memory->lock);

		switch (msg.type) {
			case mtRead:
				msg.o.io.err = flashsrv_bufferedRead(memory->fOid.id, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = flashsrv_bufferedWrite(memory->fOid.id, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				flashsrv_devCtl(memory, &msg);
				break;

			case mtOpen:
			case mtClose:
				msg.o.io.err = EOK;
				break;

			case mtSync:
				msg.o.io.err = flashsrv_bufferedSync(memory->fOid.id);
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}

		mutexUnlock(memory->lock);

		msgRespond(memory->fOid.port, &msg, rid);
	}
}


/* Initialization functions */

static int flashsrv_initMeterfs(flashsrv_partition_t *part)
{
	meterfs_ctx_t *ctx;

	part->fsCtx = calloc(1, sizeof(meterfs_ctx_t));
	if (part->fsCtx == NULL) {
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
		ctx->eraseSector = flashsrv_fsEraseSectorf0;
	}
	else if (part->fID == 1) {
		ctx->read = flashsrv_fsReadf1;
		ctx->write = flashsrv_fsWritef1;
		ctx->eraseSector = flashsrv_fsEraseSectorf1;
	}

	if (meterfs_init(ctx) < 0) {
		LOG_ERROR("imxrt-flashsrv: init meterfs at flash: %u, partition: %u.", part->fID, part->oid.id);
		return -1;
	}

	return EOK;
}


static int flashsrv_mountPart(flashsrv_partition_t *part)
{
	void *mem;
	int res = EOK;
	char path[32];

	snprintf(path, sizeof(path), "/dev/flash%u.%s", part->fID, part->pHeader->name);

	switch (part->pHeader->type) {
		/* Raw partitions locate within single flash chip are handled by one thread */
		case ptable_raw:
			part->fsCtx = NULL;
			part->oid.port = flashsrv_common.flash_memories[part->fID].rawPort;

			res = create_dev(&part->oid, path);
			if (res < 0) {
				LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, res);
				return res;
			}

			if (!flashsrv_common.flash_memories[part->fID].rawActive) {
				flashsrv_common.flash_memories[part->fID].rawActive = 1;

				mem = malloc(THREAD_STACKSZ);
				if (mem == NULL) {
					LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
					return -ENOMEM;
				}

				beginthread(flashsrv_rawThread, IMXRT_FLASH_PRIO, mem, THREAD_STACKSZ, (void *)&flashsrv_common.flash_memories[part->fID]);
			}
			part->pStatus = flashsrv_memory_active;
			break;

		/* Each meterfs partiton is handled by separete thread */
		case ptable_meterfs:
			res = flashsrv_initMeterfs(part);
			if (res < 0) {
				return res;
			}

			portCreate(&part->oid.port);
			portRegister(part->oid.port, path, NULL);

			res = create_dev(&part->oid, path);
			if (res < 0) {
				LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, res);
				return res;
			}

			mem = malloc(METERFS_STACKSZ);
			if (mem == NULL) {
				LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
				return -ENOMEM;
			}

			beginthread(flashsrv_meterfsThread, IMXRT_FLASH_PRIO, mem, METERFS_STACKSZ, (void *)part);
			part->pStatus = flashsrv_memory_active;
			break;

		default:
			res = -EINVAL;
			break;
	}

	return res;
}


static ptable_t *flashsrv_ptableRead(flashsrv_memory_t *memory)
{
	ptable_t *ptable;
	uint32_t offs, count, size;
	uint8_t magic[sizeof(ptable_magic)];

	/* Read number of partitions */
	offs = memory->ctx.properties.size - memory->ctx.properties.sector_size;
	if (flash_directRead(&memory->ctx, offs, &count, sizeof(count)) != sizeof(count)) {
		return NULL;
	}
	count = le32toh(count);

	/* Verify partition table size */
	size = ptable_size(count);
	if (size > memory->ctx.properties.sector_size) {
		return NULL;
	}

	/* Read magic signature */
	if (flash_directRead(&memory->ctx, offs + size - sizeof(magic), magic, sizeof(magic)) != sizeof(magic)) {
		return NULL;
	}

	/* Verify magic signature */
	if (memcmp(magic, ptable_magic, sizeof(magic)) != 0) {
		return NULL;
	}

	ptable = malloc(size);
	if (ptable == NULL) {
		return NULL;
	}

	if (flash_directRead(&memory->ctx, offs, ptable, size) != size) {
		free(ptable);
		return NULL;
	}

	if (ptable_deserialize(ptable, memory->ctx.properties.size, memory->ctx.properties.sector_size) < 0) {
		free(ptable);
		return NULL;
	}

	return ptable;
}


static int flashsrv_partsInit(void)
{
	int res, i, j;

	flashsrv_memory_t *memory;
	ptable_t *ptable;

	for (i = 0; i < FLASH_MEMORIES_NO; ++i) {
		memory = flashsrv_common.flash_memories + i;
		if (memory->fStatus != flashsrv_memory_active) {
			continue;
		}

		/* Read partition table */
		ptable = flashsrv_ptableRead(memory);
		if (ptable == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot initialize partition table on flash %d, wrong attributes", i);
			continue;
		}

		memory->pCnt = ptable->count;
		memory->parts = calloc(memory->pCnt, sizeof(flashsrv_partition_t));
		if (memory->parts == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot allocate memory.");
			free(ptable);
			return -ENOMEM;
		}

		/* Initialize partitions */
		for (j = 0; j < memory->pCnt; ++j) {
			memory->parts[j].fID = i;
			memory->parts[j].oid.id = j;
			memory->parts[j].pHeader = ptable->parts + j;

			if (flashsrv_mountPart(&memory->parts[j]) < 0) {
				LOG_ERROR("imxrt-flashsrv: partition %s at flash %d is not mounted.", ptable->parts[j].name, i);
				memory->parts[j].pStatus = flashsrv_memory_inactive;
				free(memory->parts[j].fsCtx);
			}
		}

		res = flash_sync(&memory->ctx);
		if (res < 0) {
			return res;
		}
	}

	return EOK;
}


static int flashsrv_flashMemoriesInit(void)
{
	char path[32];
	int i, err = EOK;
	oid_t odir = { 0 };

	flashsrv_memory_t *memory;

	flashsrv_common.flexspi_addresses[0] = FLASH_EXT_DATA_ADDRESS;
	if (FLASH_MEMORIES_NO > 1) {
		flashsrv_common.flexspi_addresses[1] = FLASH_INTERNAL_DATA_ADDRESS;
	}

	/* Wait on root */
	while (lookup("/", NULL, &odir) < 0) {
		usleep(100000);
	}

	/* Initialize flash memories */
	for (i = 0; i < FLASH_MEMORIES_NO; ++i) {
		memory = flashsrv_common.flash_memories + i;

		if (mutexCreate(&memory->lock) != EOK) {
			return -ENOENT;
		}

		portCreate(&memory->fOid.port);
		portCreate(&memory->rawPort);

		memory->fOid.id = i;
		memory->rawActive = 0;
		memory->pCnt = 0;

		snprintf(path, sizeof(path), "/dev/flash%d", memory->fOid.id);

		err = create_dev(&memory->fOid, path);
		if (err < 0) {
			LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, err);
			return err;
		}

		memory->ctx.address = flashsrv_common.flexspi_addresses[i];

		err = flash_init(&memory->ctx);
		if (err == EOK) {
			memory->fStatus = flashsrv_memory_active;
		}
		else {
			memory->fStatus = flashsrv_memory_inactive;
			LOG_ERROR("imxrt-flashsrv: %s - has not been initialized correctly, err: %d.", path, err);
		}
	}

	return EOK;
}


static inline int isXIP(void *addr)
{
	uint32_t pc;
	__asm__ volatile("mov %0, pc"
					 : "=r"(pc));
	return pc >= (uint32_t)addr && pc < (uint32_t)addr + 0x10000000;
}


static int flashsrv_getProperties(uint8_t fID, flashsrv_properties_t *p)
{
	flashsrv_memory_t *mem;

	int res = flashsrv_getFlashMemory(fID, &mem);
	if (res == EOK) {
		mutexLock(mem->lock);
		p->size = mem->ctx.properties.size;
		p->psize = mem->ctx.properties.page_size;
		p->ssize = mem->ctx.properties.sector_size;
		p->offs = 0;
		mutexUnlock(mem->lock);
	}
	return res;
}


static ssize_t flashsrv_safeDirectWrite(uint8_t fID, size_t offset, const void *data, size_t size)
{
	ssize_t ret;
	flashsrv_memory_t *mem;

	ret = flashsrv_getFlashMemory(fID, &mem);
	if (ret < 0) {
		return ret;
	}

	mutexLock(mem->lock);
	ret = flashsrv_directWrite(fID, offset, data, size);
	mutexUnlock(mem->lock);
	return ret;
}


static ssize_t flashsrv_safeDirectRead(uint8_t fID, size_t offset, void *data, size_t size)
{
	ssize_t ret;
	flashsrv_memory_t *mem;

	ret = flashsrv_getFlashMemory(fID, &mem);
	if (ret < 0) {
		return ret;
	}

	mutexLock(mem->lock);
	ret = flashsrv_directRead(fID, offset, data, size);
	mutexUnlock(mem->lock);
	return ret;
}


static int flashsrv_safeSectorErase(uint8_t fID, size_t offs)
{
	ssize_t ret;
	flashsrv_memory_t *mem;

	ret = flashsrv_getFlashMemory(fID, &mem);
	if (ret < 0) {
		return ret;
	}

	mutexLock(mem->lock);
	ret = flashsrv_eraseSector(fID, offs);
	mutexUnlock(mem->lock);
	return ret;
}


__attribute__((weak)) int flashsrv_customIntInit(flashsrv_partitionOps_t *ops)
{
	return EOK;
};


static int flashsrv_customInit(void)
{
	flashsrv_common.ops.read = flashsrv_safeDirectRead;
	flashsrv_common.ops.write = flashsrv_safeDirectWrite;
	flashsrv_common.ops.eraseSector = flashsrv_safeSectorErase;
	flashsrv_common.ops.getProperties = flashsrv_getProperties;

	return flashsrv_customIntInit(&flashsrv_common.ops);
}


int main(void)
{
	void *mem;

	if (isXIP((void *)FLEXSPI1_AHB_ADDR) || (FLASH_MEMORIES_NO > 1 && isXIP((void *)FLEXSPI2_AHB_ADDR))) {
		LOG_ERROR("imxrt-flashsrv: require to run driver from ram.\n");
		return EXIT_FAILURE;
	}

	priority(IMXRT_FLASH_PRIO);

	if (flashsrv_flashMemoriesInit() != EOK) {
		LOG_ERROR("imxrt-flashsrv: flash memories were not initialized correctly.\n");
		return EXIT_FAILURE;
	}

	if (flashsrv_partsInit() != EOK) {
		LOG_ERROR("imxrt-flashsrv: partitions were not initialized correctly.\n");
		return EXIT_FAILURE;
	}

	if (flashsrv_customInit() < 0) {
		LOG_ERROR("imxrt-flashsrv: custom init failed.\n");
	}

	printf("imxrt-flashsrv: initialized.\n");

	/* flashsrv_devThread handles requests associated with the whole single flash chip */
	if (FLASH_MEMORIES_NO > 1 && flashsrv_common.flash_memories[1].fStatus == flashsrv_memory_active) {
		mem = malloc(THREAD_STACKSZ);
		if (mem == NULL) {
			LOG_ERROR("imxrt-flashsrv: cannot alloc memory.");
			return EXIT_FAILURE;
		}
		beginthread(flashsrv_devThread, IMXRT_FLASH_PRIO, mem, THREAD_STACKSZ, (void *)&flashsrv_common.flash_memories[0]);
		flashsrv_devThread(&flashsrv_common.flash_memories[1]);
	}
	else if (flashsrv_common.flash_memories[0].fStatus == flashsrv_memory_active) {
		flashsrv_devThread(&flashsrv_common.flash_memories[0]);
	}
	else {
		printf("imxrt-flashsrv: end\n");
	}


	return EXIT_SUCCESS;
}
