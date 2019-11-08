/*
 * Phoenix-RTOS
 *
 * PC ATA server.
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include <sys/msg.h>
#include <sys/list.h>

#include <posix/idtree.h>
#include <posix/utils.h>
#include <phoenix/msg.h>
#include <phoenix/stat.h>

#include "atasrv.h"
#include "mbr.h"

#define LOG_ERROR(str, ...) do { fprintf(stderr, __FILE__  ":%d error: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)
#define TRACE(str, ...) do { if (0) fprintf(stderr, __FILE__  ":%d trace: " str "\n", __LINE__, ##__VA_ARGS__); } while (0)

#define ATASRV_DEV_NAME_BASE "/dev/hd"
#define ATASRV_DEV_NAME_CHAR 'a'

#define ATASRV_DEV_TYPE_HDD 0xFF
#define ATASRV_DEV_TYPE_PART 0x01


typedef struct _atasrv_partition atasrv_partition_t;


typedef struct {
	idnode_t node;
	uint8_t type;
	union {
		atasrv_partition_t *partition;
		ata_dev_t *ataDev;
	};
} atasrv_device_t;


typedef struct _atasrv_filesystem {
	struct _atasrv_filesystem *next;
	int (*handler)(void *, msg_t *);
	int (*mount)(id_t *, void **);
	int (*unmount)(void *);
	char name[16];
	uint8_t type;
} atasrv_filesystem_t;


struct _atasrv_partition {
	int portfd;
	uint32_t start;
	uint32_t size;
	uint32_t refs;
	uint8_t type;

	atasrv_device_t *dev;
	atasrv_filesystem_t *fs;

	void *fsData;
	char *stack;;
};


typedef struct {
	void *next, *prev;
	msg_t msg;
	int portfd;
	unsigned rid;
	atasrv_filesystem_t *fs;
	void *fsData;
	void *srvData;
} atasrv_request_t;


struct {
	idtree_t devices;
	atasrv_filesystem_t *filesystems;
	atasrv_request_t *queue;
	int portfd;
	int devices_cnt;
	handle_t lock, cond;

	char poolStacks[1][4 * 4096] __attribute__((aligned(8)));
} atasrv_common;


static atasrv_request_t *atasrv_newRequest(int portfd, atasrv_filesystem_t *fs, void *fsData, void *srvData)
{
	atasrv_request_t *req;

	if ((req = calloc(1, sizeof(*req))) != NULL) {
		req->fsData = fsData;
		req->fs = fs;
		req->srvData = srvData;
		req->portfd = portfd;
	}

	return req;
}


static void atasrv_freeRequest(atasrv_request_t *req)
{
	free(req);
}


static void atasrv_queueRequest(atasrv_request_t *req)
{
	LIST_ADD(&atasrv_common.queue, req);
}


static atasrv_request_t *atasrv_getRequest(void)
{
	atasrv_request_t *req;

	if ((req = atasrv_common.queue) != NULL)
		LIST_REMOVE(&atasrv_common.queue, req);

	return req;
}


static void atasrv_partThread(void *arg)
{
	atasrv_partition_t *p = (atasrv_partition_t *)arg;
	atasrv_request_t *req;

	for (;;) {
		req = atasrv_newRequest(p->portfd, p->fs, p->fsData, p);

		if (msgRecv(p->portfd, &req->msg, &req->rid) < 0)
			continue;

		/* TBD: handle umount here?
		if (req->msg.type == mtUmount) {

		}
		*/
		mutexLock(atasrv_common.lock);
		atasrv_queueRequest(req);
		mutexUnlock(atasrv_common.lock);

		condSignal(atasrv_common.cond);
	}

}


static void atasrv_poolThread(void *arg)
{
	atasrv_request_t *req;
	int err;

	for (;;) {
		mutexLock(atasrv_common.lock);
		while ((req = atasrv_getRequest()) == NULL)
			condWait(atasrv_common.cond, atasrv_common.lock, 0);
		mutexUnlock(atasrv_common.lock);

		err = req->fs->handler(req->fsData, &req->msg);

		msgRespond(req->portfd, err, &req->msg, req->rid);
		atasrv_freeRequest(req);
	}
}


int atasrv_registerDevice(void *data)
{
	atasrv_device_t *dev;
	char devName[16];
	int err = 0;
	ata_dev_t *ataDev = data;

	if (!ataDev)
		return -EINVAL;

	dev = malloc(sizeof(atasrv_device_t));

	if (!dev)
		return -ENOMEM;

	dev->ataDev = ataDev;
	dev->type = ATASRV_DEV_TYPE_HDD;

	idtree_alloc(&atasrv_common.devices, &dev->node);

	sprintf(devName, "%s%c", ATASRV_DEV_NAME_BASE, ATASRV_DEV_NAME_CHAR + idtree_id(&dev->node));
	err = create_dev(atasrv_common.portfd, idtree_id(&dev->node), devName, S_IFBLK);

	if (!err)
		atasrv_common.devices_cnt++;
	else
		idtree_remove(&atasrv_common.devices, &dev->node);

	return err;
}

static int atasrv_discoverPartitions(void)
{
	int j, i = 0, err = 0;
	atasrv_device_t *dev, *part;
	mbr_t *mbr;
	char devName[16];

	mbr = malloc(sizeof(mbr_t));
	if (!mbr)
		return -ENOMEM;

	for (j = 0; j < atasrv_common.devices_cnt; j++) {

		dev = lib_treeof(atasrv_device_t, node, idtree_find(&atasrv_common.devices, j));

		if (!dev)
			continue;

		memset(mbr, 0, sizeof(mbr_t));
		err = read_mbr(dev->ataDev, mbr);
		if (!err) {
			for (i = 0; i < 4; i++) {
				if (mbr->pent[i].type) {
					part = calloc(1, sizeof(atasrv_device_t));
					if (!part) {
						err = -ENOMEM;
						break;
					}

					part->type = ATASRV_DEV_TYPE_PART;
					part->partition = calloc(1, sizeof(atasrv_partition_t));
					if (!part->partition) {
						free(part);
						err = -ENOMEM;
						break;
					}

					part->partition->start = mbr->pent[i].first_sect_lba;
					part->partition->size = mbr->pent[i].sector_count;
					part->partition->type = mbr->pent[i].type;
					part->partition->dev = dev;

					idtree_alloc(&atasrv_common.devices, &part->node);
					sprintf(devName, "%s%c%d", ATASRV_DEV_NAME_BASE, ATASRV_DEV_NAME_CHAR + idtree_id(&dev->node), i + 1);
					err = create_dev(atasrv_common.portfd, idtree_id(&part->node), devName, S_IFBLK);
					if (err) {
						/* TODO: log error? */
						free(part->partition);
						free(part);
					}

				}
			}
		}
	}

	free(mbr);
	return err;
}

int atasrv_read(id_t *devId, offs_t offs, char *buff, size_t len, int *err)
{
	atasrv_device_t *dev;

	dev = lib_treeof(atasrv_device_t, node, idtree_find(&atasrv_common.devices, *devId));

	if (!dev) {
		*err = ENXIO;
		return 0;
	}

	switch(dev->type) {
		case ATASRV_DEV_TYPE_HDD:
			*err = atadrv_read(dev->ataDev, offs, buff, len);
		break;

		case ATASRV_DEV_TYPE_PART:
			*err = atadrv_read(dev->ataDev, dev->partition->start + offs, buff, len);
		break;

		default:
			*err = EINVAL;
			return -EINVAL;
	}

	if (*err == len) {
		*err = 0;
		return len;
	}

	return *err;
}


int atasrv_write(id_t *devId, offs_t offs, const char *buff, size_t len, int *err)
{
	atasrv_device_t *dev;

	dev = lib_treeof(atasrv_device_t, node, idtree_find(&atasrv_common.devices, *devId));

	if (!dev) {
		*err = ENXIO;
		return 0;
	}

	/*TODO: checks checks checks */

	switch(dev->type) {
		case ATASRV_DEV_TYPE_HDD:
			*err = atadrv_write(dev->ataDev, offs, buff, len);
			break;

		case ATASRV_DEV_TYPE_PART:
			*err = atadrv_write(dev->ataDev, dev->partition->start + offs, buff, len);
			break;

		default:
			*err = EINVAL;
			return -EINVAL;
	}

	if (*err == len) {
		*err = 0;
		return len;
	}

	return *err;
}


static void atasrv_msgLoop(void)
{
	msg_t msg;
	int err;
	unsigned rid, portfd = PORT_DESCRIPTOR;

	for (;;) {
		if (msgRecv(portfd, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtRead:
			msg.o.io = atasrv_read(&msg.object, msg.i.io.offs, msg.o.data, msg.o.size, &err);
			break;

		case mtWrite:
			msg.o.io = atasrv_write(&msg.object, msg.i.io.offs, msg.i.data, msg.i.size, &err);
			break;

	/*	TBD
		case mtMount:
			err = -ENOSYS;
			break;

		case mtSync:
			err = -ENOSYS;
			break;
	*/

		case mtGetAttr:
			err = -ENOSYS;
			break;

		case mtOpen:
			msg.o.open = msg.object;
			err = EOK;
			break;
		case mtClose:
			err = EOK;
			break;

		default:
			err = -EINVAL;
			break;
		}

		msgRespond(portfd, err, &msg, rid);
	}
}


static void atasrv_init(void)
{
	memset(&atasrv_common, 0, sizeof(atasrv_common));
	idtree_init(&atasrv_common.devices);
	atasrv_common.portfd = PORT_DESCRIPTOR;
}


int main(void)
{
	int pid, sid;

	pid = fork();
	if (pid < 0)
		exit(EXIT_FAILURE);

	if (pid > 0)
		exit(EXIT_SUCCESS);

	sid = setsid();
	if (sid < 0)
		exit(EXIT_FAILURE);

	atasrv_init();
	if (ata_init())
		return -ENXIO;

	atasrv_discoverPartitions();

	atasrv_msgLoop();

	return 0;
}
