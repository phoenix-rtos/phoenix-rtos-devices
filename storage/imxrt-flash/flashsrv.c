/*
 * Phoenix-RTOS
 *
 * i.MX RT Flash server
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/file.h>

#include "posix/utils.h"

#include "flashdrv.h"
#include "flashsrv.h"

#define FLASHES_NO 2

#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)


enum { flash_memory_unactive = 0, flash_memory_active = 0xff };


typedef struct _flash_memory_t {
	flash_context_t context;
	oid_t oid;
	uint8_t status;
} flash_memory_t;


struct {
	flash_memory_t flash_memories[FLASHES_NO];
	uint32_t flexspi_adresses[FLASHES_NO];
} flashsrv_common;


static flash_memory_t *flashsrv_findFlash(oid_t oid)
{
	int i;
	for (i = 0; i < FLASHES_NO; ++i) {
		if (oid.id == flashsrv_common.flash_memories[i].oid.id)
			return flashsrv_common.flash_memories + i;
	}
	return NULL;
}


static void flashsrv_devCtl(msg_t *msg)
{
	flash_memory_t *flash_memory = NULL;
	flash_i_devctl_t *idevctl = (flash_i_devctl_t *)msg->i.raw;
	flash_o_devctl_t *odevctl = (flash_o_devctl_t *)msg->o.raw;

	switch (idevctl->type) {
		case flashsrv_devctl_properties:
			TRACE("imxrt-flashsrv: flashsrv_devctl_properties, id: %d, port: %d.", idevctl->oid.id, idevctl->oid.port);
			flash_memory = flashsrv_findFlash(idevctl->oid);
			if (flash_memory == NULL) {
				odevctl->err = -EINVAL;
				break;
			}

			odevctl->properties.fsize = flash_memory->context.properties.size;
			odevctl->properties.psize = flash_memory->context.properties.page_size;
			odevctl->properties.ssize = flash_memory->context.properties.sector_size;
			odevctl->err = EOK;
			break;

		case flashsrv_devctl_sync:
			TRACE("imxrt-flashsrv: flashsrv_devctl_sync, id: %d, port: %d.", idevctl->oid.id, idevctl->oid.port);
			flash_memory = flashsrv_findFlash(idevctl->oid);
			if (flash_memory == NULL) {
				odevctl->err = -EINVAL;
				break;
			}

			flash_sync(&flash_memory->context);
			odevctl->err = EOK;
			break;

		default:
			odevctl->err = -EINVAL;
			break;
	}
}


static int flashsrv_write(oid_t oid, size_t offset, const char *data, size_t size)
{
	flash_memory_t *flash_memory = flashsrv_findFlash(oid);

	if (flash_memory == NULL)
		return -EINVAL;

	if (flash_memory->status == flash_memory_unactive)
		return -EINVAL;

	return flash_writeDataPage(&flash_memory->context, offset, data, size);
}


static int flashsrv_read(oid_t oid, size_t offset, char *data, size_t size)
{
	flash_memory_t *flash_memory = flashsrv_findFlash(oid);

	if (flash_memory == NULL)
		return -EINVAL;

	if (flash_memory->status == flash_memory_unactive)
		return -EINVAL;

	return flash_readData(&flash_memory->context, offset, data, size);
}


static void flashsrv_devThread(void *arg)
{
	msg_t msg;
	unsigned rid;
	uint32_t port = (uint32_t)arg;

	while (1) {
		while (msgRecv(port, &msg, &rid) < 0)
			;

		switch (msg.type) {
			case mtRead:
				msg.o.io.err = flashsrv_read(msg.i.io.oid, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.io.err = flashsrv_write(msg.i.io.oid, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtDevCtl:
				flashsrv_devCtl(&msg);
				break;

			default:
				msg.o.io.err = -ENOSYS;
				break;
		}
		msgRespond(port, &msg, rid);
	}
}


static uint32_t flashsrv_init(uint32_t *port)
{
	char path[32];
	int i, err = EOK;
	oid_t oid = { 0 };
	oid_t odir = { 0 };

	flashsrv_common.flexspi_adresses[0] = FLEXSPI_DATA_ADDRESS;
	flashsrv_common.flexspi_adresses[1] = FLEXSPI2_DATA_ADDRESS;

	/* Wait on root */
	while (lookup("/", NULL, &odir) < 0)
		usleep(100000);

	portCreate(port);

	/* Initialize flash memories */
	for (i = 0; i < FLASHES_NO; ++i) {
		oid.id = i + 1;
		oid.port = *port;

		sprintf(path, "/dev/flash%d", oid.id);
		if ((err = create_dev(&oid, path)) < 0) {
			LOG_ERROR("imxrt-flashsrv: create %s - err: %d", path, err);
			return err;
		}

		flashsrv_common.flash_memories[i].oid = oid;
		flashsrv_common.flash_memories[i].context.address = flashsrv_common.flexspi_adresses[i];

		if ((err = flash_init(&flashsrv_common.flash_memories[i].context)) == EOK)
			flashsrv_common.flash_memories[i].status = flash_memory_active;
		else
			LOG_ERROR("imxrt-flashsrv: %s - has not been initialized correctly, err: %d.", path, err);
	}

	return EOK;
}


int main(void)
{
	uint32_t port;

	if (flashsrv_init(&port) == EOK) {
		printf("imxrt-flashsrv: initialized.\n");
	}
	else {
		printf("imxrt-flashsrv: uninitialized.\n");
		return -1;
	}

	flashsrv_devThread((void *)port);

	return EOK;
}
