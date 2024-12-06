/*
 * Phoenix-RTOS
 *
 * GR716 Flash server
 *
 * Copyright 2023 Phoenix Systems
 * Author: Lukasz Leczkowski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <posix/utils.h>
#include <sys/msg.h>
#include <sys/file.h>
#include <sys/threads.h>

#include <meterfs.h>
#include <ptable.h>

#include "flashdrv.h"
#include "gr716-flashsrv.h"


#define FLASH_NO 1u

/* clang-format off */
#define LOG(fmt, ...) do { fprintf(stdout, "gr716-flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(fmt, ...) do { fprintf(stderr, "gr716-flashsrv: " fmt "\n", ##__VA_ARGS__); } while (0)
/* clang-format on */

#define METERFS_STACKSZ 2048
#define THREAD_STACKSZ  2048

#define GR716_FLASH_PRIO 3


typedef struct {
	ptable_part_t *pHeader;

	oid_t oid;
	uint8_t fID;

	void *fsCtx;
} flashsrv_partition_t;


typedef struct {
	flash_context_t ctx;
	flashsrv_partition_t *parts;

	uint32_t pCnt;
	uint8_t currPart;
	uint8_t rawActive;

	oid_t fOid;
	uint32_t rawPort;

	handle_t lock;
} flash_memory_t;


static struct {
	flash_memory_t flash_memories[FLASH_NO];
} flashsrv_common;


/* Callbacks to meterfs */


static ssize_t flashsrv_fsWritef0(struct _meterfs_devCtx_t *devCtx, off_t offs, const void *buff, size_t bufflen)
{
	(void)devCtx;

	flash_memory_t *flash_memory = flashsrv_common.flash_memories;

	return flash_directWrite(&flash_memory->ctx, offs, buff, bufflen);
}


static ssize_t flashsrv_fsReadf0(struct _meterfs_devCtx_t *devCtx, off_t offs, void *buff, size_t bufflen)
{
	(void)devCtx;

	flash_memory_t *flash_memory = flashsrv_common.flash_memories;

	return flash_readData(&flash_memory->ctx, offs, buff, bufflen);
}


static int flashsrv_fsEraseSectorf0(struct _meterfs_devCtx_t *devCtx, off_t offs)
{
	(void)devCtx;

	flash_memory_t *flash_memory = flashsrv_common.flash_memories;

	return flash_sectorErase(&flash_memory->ctx, offs);
}


/* Device control */


static void flashsrv_devCtl(flash_memory_t *memory, msg_t *msg)
{
	uint8_t fID;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	if (memory == NULL) {
		msg->o.err = -EINVAL;
		return;
	}

	fID = msg->oid.id;
	if (fID >= FLASH_NO) {
		msg->o.err = -EINVAL;
		return;
	}

	switch (idevctl->type) {
		case flashsrv_devctl_properties:
			odevctl->properties.size = memory->ctx.properties->totalSz;
			odevctl->properties.psize = memory->ctx.properties->pageSz;
			odevctl->properties.ssize = memory->ctx.properties->sectorSz;
			odevctl->properties.offs = 0;
			msg->o.err = EOK;
			break;

		case flashsrv_devctl_sync:
			msg->o.err = flash_sync(&memory->ctx);
			break;

		case flashsrv_devctl_eraseSector:
			if (idevctl->addr >= memory->ctx.properties->totalSz) {
				msg->o.err = -EINVAL;
				break;
			}
			msg->o.err = flash_sectorErase(&memory->ctx, idevctl->addr);
			break;

		case flashsrv_devctl_erasePartition:
			msg->o.err = flash_chipErase(&memory->ctx);
			break;

		case flashsrv_devctl_directWrite:
			if (idevctl->addr >= memory->ctx.properties->totalSz) {
				msg->o.err = -EINVAL;
				break;
			}
			msg->o.err = flash_directWrite(&memory->ctx, idevctl->addr, msg->i.data, msg->i.size);
			break;

		default:
			msg->o.err = -EINVAL;
			break;
	}
}


static void flashsrv_rawCtl(flash_memory_t *memory, msg_t *msg)
{
	int err;
	uint32_t sNb;
	uint8_t partID;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;


	if (memory == NULL) {
		msg->o.err = -EINVAL;
		return;
	}

	partID = msg->oid.id;
	if (partID >= memory->pCnt) {
		msg->o.err = -EINVAL;
		return;
	}

	switch (idevctl->type) {
		case flashsrv_devctl_properties:
			odevctl->properties.size = memory->parts[partID].pHeader->size;
			odevctl->properties.psize = memory->ctx.properties->pageSz;
			odevctl->properties.ssize = memory->ctx.properties->sectorSz;
			odevctl->properties.offs = memory->parts[partID].pHeader->offset;
			msg->o.err = EOK;
			break;

		case flashsrv_devctl_sync:
			msg->o.err = flash_sync(&memory->ctx);
			break;

		case flashsrv_devctl_eraseSector:
			if (idevctl->addr > memory->parts[partID].pHeader->size) {
				msg->o.err = -EINVAL;
				break;
			}

			msg->o.err = flash_sectorErase(&memory->ctx, memory->parts[partID].pHeader->offset + idevctl->addr);
			break;

		case flashsrv_devctl_erasePartition:
			sNb = memory->parts[partID].pHeader->size / memory->ctx.properties->sectorSz;

			for (int j = 0; j < sNb; j++) {
				err = flash_sectorErase(&memory->ctx, memory->parts[partID].pHeader->offset + j * memory->ctx.properties->sectorSz);
				if (err != EOK) {
					break;
				}
			}
			msg->o.err = err;
			break;

		case flashsrv_devctl_directWrite:
			if (idevctl->addr > memory->parts[partID].pHeader->size) {
				msg->o.err = -EINVAL;
				break;
			}

			msg->o.err = flash_directWrite(&memory->ctx, memory->parts[partID].pHeader->offset + idevctl->addr, msg->i.data, msg->i.size);
			break;


		default:
			msg->o.err = -EINVAL;
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
		while (msgRecv(part->oid.port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtRead:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_readFile(msg.oid.id, msg.i.io.offs, msg.o.data, msg.o.size, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtWrite:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_writeFile(msg.oid.id, msg.i.data, msg.i.size, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtLookup:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_lookup(msg.i.data, &msg.o.lookup.fil.id, (meterfs_ctx_t *)part->fsCtx);
				msg.o.lookup.fil.port = part->oid.port;
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				(void)memcpy(&msg.o.lookup.dev, &msg.o.lookup.fil, sizeof(oid_t));
				break;

			case mtOpen:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_open(msg.oid.id, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtClose:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_close(msg.oid.id, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			case mtDevCtl:
				mutexLock(flashsrv_common.flash_memories[part->fID].lock);
				flashsrv_common.flash_memories[part->fID].currPart = part->oid.id;
				msg.o.err = meterfs_devctl(idevctl, odevctl, (meterfs_ctx_t *)part->fsCtx);
				mutexUnlock(flashsrv_common.flash_memories[part->fID].lock);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}
		msgRespond(part->oid.port, &msg, rid);
	}
}


static int flashsrv_verifyRawIO(const flash_memory_t *memory, msg_t *msg, size_t *size)
{
	if (memory->parts[msg->oid.id].pHeader->size <= msg->i.io.offs) {
		msg->o.err = -EINVAL;
		return -EINVAL;
	}

	if (memory->parts[msg->oid.id].pHeader->size < (msg->i.io.offs + *size)) {
		*size = memory->parts[msg->oid.id].pHeader->size - msg->i.io.offs;
	}

	return EOK;
}


static void flashsrv_rawThread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	uint32_t startAddr;

	flash_memory_t *memory = (flash_memory_t *)arg;

	for (;;) {
		while (msgRecv(memory->rawPort, &msg, &rid) < 0) {
		}

		if (msg.oid.id >= memory->pCnt) {
			msg.o.err = -EINVAL;
		}
		else {
			switch (msg.type) {
				case mtRead:
					if (flashsrv_verifyRawIO(memory, &msg, &msg.o.size) < 0) {
						msg.o.err = -EINVAL;
						break;
					}

					startAddr = memory->parts[msg.oid.id].pHeader->offset;
					mutexLock(memory->lock);
					msg.o.err = flash_readData(&memory->ctx, startAddr + msg.i.io.offs, msg.o.data, msg.o.size);
					mutexUnlock(memory->lock);
					break;

				case mtWrite:
					if (flashsrv_verifyRawIO(memory, &msg, &msg.o.size) < 0) {
						msg.o.err = -EINVAL;
						break;
					}

					startAddr = memory->parts[msg.oid.id].pHeader->offset;
					mutexLock(memory->lock);
					msg.o.err = flash_bufferedWrite(&memory->ctx, startAddr + msg.i.io.offs, msg.i.data, msg.i.size);
					mutexUnlock(memory->lock);
					break;

				case mtDevCtl:
					mutexLock(memory->lock);
					flashsrv_rawCtl(memory, &msg);
					mutexUnlock(memory->lock);
					break;

				case mtOpen:
					msg.o.err = 0;
					break;

				case mtClose:
					mutexLock(memory->lock);
					(void)flash_sync(&memory->ctx);
					mutexUnlock(memory->lock);
					msg.o.err = 0;
					break;

				case mtSync:
					mutexLock(memory->lock);
					msg.o.err = flash_sync(&memory->ctx);
					mutexUnlock(memory->lock);
					break;

				case mtGetAttr:
					if (msg.i.attr.type == atSize) {
						msg.o.attr.val = memory->parts[msg.oid.id].pHeader->size;
						msg.o.err = 0;
					}
					else {
						msg.o.err = -ENOSYS;
					}
					break;

				default:
					msg.o.err = -ENOSYS;
					break;
			}
		}
		msgRespond(memory->rawPort, &msg, rid);
	}
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	msg_rid_t rid;
	flash_memory_t *memory = (flash_memory_t *)arg;

	for (;;) {
		while (msgRecv(memory->fOid.port, &msg, &rid) < 0) {
		}

		switch (msg.type) {
			case mtRead:
				mutexLock(memory->lock);
				msg.o.err = flash_readData(&memory->ctx, msg.i.io.offs, msg.o.data, msg.o.size);
				mutexUnlock(memory->lock);
				break;

			case mtWrite:
				mutexLock(memory->lock);
				msg.o.err = flash_bufferedWrite(&memory->ctx, msg.i.io.offs, msg.i.data, msg.i.size);
				mutexUnlock(memory->lock);
				break;

			case mtDevCtl:
				mutexLock(memory->lock);
				flashsrv_devCtl(memory, &msg);
				mutexUnlock(memory->lock);
				break;

			case mtOpen:
			case mtClose:
				msg.o.err = 0;
				break;

			case mtGetAttr:
				if (msg.i.attr.type == atSize) {
					msg.o.attr.val = memory->ctx.properties->totalSz;
					msg.o.err = 0;
				}
				else {
					msg.o.err = -ENOSYS;
				}
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}
		msgRespond(memory->fOid.port, &msg, rid);
	}
}


/* Initialization functions */


static int flashsrv_initMeterfs(flashsrv_partition_t *part)
{
	meterfs_ctx_t *ctx;

	part->fsCtx = calloc(1, sizeof(meterfs_ctx_t));
	if (part->fsCtx == NULL) {
		LOG_ERROR("failed to allocate memory");
		return -ENOMEM;
	}

	ctx = (meterfs_ctx_t *)part->fsCtx;

	ctx->sz = part->pHeader->size;
	ctx->offset = part->pHeader->offset;
	ctx->sectorsz = flashsrv_common.flash_memories[part->fID].ctx.properties->sectorSz;

	ctx->read = flashsrv_fsReadf0;
	ctx->write = flashsrv_fsWritef0;
	ctx->eraseSector = flashsrv_fsEraseSectorf0;

	ctx->keyInit = false;

	if (meterfs_init(ctx) < 0) {
		LOG_ERROR("failed to initialize meterfs at flash: %u, partition: %u", part->fID, part->oid.id);
		return -1;
	}

	return EOK;
}


static int flashsrv_mountPart(flashsrv_partition_t *part)
{
	void *mem;
	int res = EOK;
	char path[20];

	(void)snprintf(path, sizeof(path), "flash%u.%s", part->fID, part->pHeader->name);

	switch (part->pHeader->type) {
		/* Each meterfs partition is handled by separate thread */
		case ptable_meterfs:
			res = flashsrv_initMeterfs(part);
			if (res < 0) {
				return res;
			}

			portCreate(&part->oid.port);
			portRegister(part->oid.port, path, NULL);

			res = create_dev(&part->oid, path);
			if (res < 0) {
				LOG_ERROR("create %s - err: %d", path, res);
				return res;
			}

			mem = malloc(METERFS_STACKSZ);
			if (mem == NULL) {
				LOG_ERROR("cannot alloc memory.");
				return -ENOMEM;
			}

			beginthread(flashsrv_meterfsThread, GR716_FLASH_PRIO, mem, METERFS_STACKSZ, (void *)part);
			break;

		/* Raw partitions located within single flash chip are handled by one thread */
		case ptable_raw:
		/* Mount unknown partitions as raw for forward compatibility and allow FUSE-like usage */
		default:
			part->fsCtx = NULL;
			part->oid.port = flashsrv_common.flash_memories[part->fID].rawPort;

			res = create_dev(&part->oid, path);
			if (res < 0) {
				LOG_ERROR("create %s - err: %d", path, res);
				return res;
			}

			if (!flashsrv_common.flash_memories[part->fID].rawActive) {
				flashsrv_common.flash_memories[part->fID].rawActive = 1;

				mem = malloc(THREAD_STACKSZ);
				if (mem == NULL) {
					LOG_ERROR("cannot alloc memory.");
					return -ENOMEM;
				}

				beginthread(flashsrv_rawThread, GR716_FLASH_PRIO, mem, THREAD_STACKSZ, (void *)&flashsrv_common.flash_memories[part->fID]);
			}
			break;
	}

	return res;
}


static ptable_t *flashsrv_ptableRead(flash_memory_t *memory)
{
	ptable_t *ptable;
	uint32_t offs, count, size;
	uint8_t magic[sizeof(ptable_magic)];

	/* Read number of partitions */
	offs = memory->ctx.properties->totalSz - memory->ctx.properties->sectorSz;
	if (flash_readData(&memory->ctx, offs, &count, sizeof(count)) != sizeof(count)) {
		return NULL;
	}
	count = le32toh(count);

	/* Verify partition table size */
	size = ptable_size(count);
	if (size > memory->ctx.properties->sectorSz) {
		return NULL;
	}

	/* Read magic signature */
	if (flash_readData(&memory->ctx, offs + size - sizeof(magic), magic, sizeof(magic)) != sizeof(magic)) {
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

	if (flash_readData(&memory->ctx, offs, ptable, size) != size) {
		free(ptable);
		return NULL;
	}

	if (ptable_deserialize(ptable, memory->ctx.properties->totalSz, memory->ctx.properties->sectorSz) < 0) {
		free(ptable);
		return NULL;
	}

	return ptable;
}


static int flashsrv_partsInit(void)
{
	int res;
	flash_memory_t *memory;
	ptable_t *ptable;

	for (int i = 0; i < FLASH_NO; i++) {
		memory = &flashsrv_common.flash_memories[i];

		ptable = flashsrv_ptableRead(memory);
		if (ptable == NULL) {
			LOG_ERROR("cannot initialize partition table on flash %d, wrong attributes", i);
			continue;
		}

		memory->pCnt = ptable->count;
		memory->parts = calloc(memory->pCnt, sizeof(flashsrv_partition_t));
		if (memory->parts == NULL) {
			LOG_ERROR("cannot allocate memory.");
			free(ptable);
			return -ENOMEM;
		}

		/* Initialize partitions */
		for (int j = 0; j < memory->pCnt; ++j) {
			memory->parts[j].fID = i;
			memory->parts[j].oid.id = j;
			memory->parts[j].pHeader = ptable->parts + j;

			if (flashsrv_mountPart(&memory->parts[j]) < 0) {
				LOG_ERROR("partition %s at flash %d is not mounted.", ptable->parts[j].name, i);
				free(memory->parts[j].fsCtx);
				memory->parts[j].fsCtx = NULL;
			}
		}

		res = flash_sync(&memory->ctx);
		if (res < 0) {
			return res;
		}
	}

	return EOK;
}


static int flashsrv_memoryInit(void)
{
	int res;
	char path[8];
	flash_memory_t *memory;
	oid_t odir = { 0 };

	/* Wait for rootfs */
	while (lookup("/", NULL, &odir) < 0) {
		usleep(100000);
	}

	for (int i = 0; i < FLASH_NO; i++) {
		memory = &flashsrv_common.flash_memories[i];

		res = mutexCreate(&memory->lock);
		if (res != EOK) {
			LOG_ERROR("could not create mutex");
			return res;
		}

		res = portCreate(&memory->fOid.port);
		if (res != EOK) {
			(void)resourceDestroy(memory->lock);
			LOG_ERROR("could not create port");
			return res;
		}

		res = portCreate(&memory->rawPort);
		if (res != EOK) {
			portDestroy(memory->fOid.port);
			(void)resourceDestroy(memory->lock);
			LOG_ERROR("could not create port");
			return res;
		}

		memory->fOid.id = i;
		memory->rawActive = 0;
		memory->pCnt = 0;

		(void)snprintf(path, sizeof(path), "flash%d", i);

		res = create_dev(&memory->fOid, path);
		if (res != EOK) {
			portDestroy(memory->fOid.port);
			portDestroy(memory->rawPort);
			(void)resourceDestroy(memory->lock);
			LOG_ERROR("could not create device %s", path);
			return res;
		}

		res = flash_init(&memory->ctx, i);
		if (res != EOK) {
			portDestroy(memory->fOid.port);
			portDestroy(memory->rawPort);
			(void)resourceDestroy(memory->lock);
			LOG_ERROR("could not initialize flash memory %s", path);
			return res;
		}
	}

	return EOK;
}


int main(int argc, char *argv[])
{
	priority(GR716_FLASH_PRIO);

	if (flashsrv_memoryInit() != EOK) {
		LOG_ERROR("flash memories initialization failed");
		return EXIT_FAILURE;
	}

	if (flashsrv_partsInit() != EOK) {
		LOG_ERROR("partitions were not initialized correctly.\n");
		return EXIT_FAILURE;
	}

	LOG("initialized");

	flashsrv_devThread(&flashsrv_common.flash_memories[0]);

	return EXIT_SUCCESS;
}
