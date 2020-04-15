/*
 * Phoenix-RTOS
 *
 * ATA server
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
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
#include <sys/threads.h>

#include <posix/idtree.h>
#include <posix/utils.h>
#include <phoenix/msg.h>
#include <phoenix/stat.h>

#include <libext2.h>

#include "ata.h"
#include "mbr.h"


#define DEV_NAME_BASE "/dev/hd"
#define DEV_NAME_CHAR 'a'

#define DEV_TYPE_HDD  0xFF
#define DEV_TYPE_PART 0x01

#define FS_NAME_SIZE  16
#define FS_REGISTER(NAME) do {                      \
	atasrv_fs_t *fs;                                \
	if ((fs = malloc(sizeof(atasrv_fs_t))) == NULL) \
		return -ENOMEM;                             \
	strcpy(fs->name, LIB##NAME##_NAME);             \
	fs->type    =  LIB##NAME##_TYPE;                \
	fs->handler = &LIB##NAME##_HANDLER;             \
	fs->mount   = &LIB##NAME##_MOUNT;               \
	fs->unmount = &LIB##NAME##_UNMOUNT;             \
	fs->next    = atasrv_common.filesystems;        \
	atasrv_common.filesystems = fs;                 \
} while (0)


extern int portGet(unsigned id);


typedef struct atasrv_part_t atasrv_part_t;
typedef struct atasrv_fs_t atasrv_fs_t;
typedef struct atasrv_req_t atasrv_req_t;


typedef struct {
	idnode_t node;
	uint8_t type;
	union {
		ata_dev_t *dev;
		atasrv_part_t *part;
	};
} atasrv_dev_t;


typedef ssize_t (*read_callback)(id_t *, offs_t, char *, size_t);
typedef ssize_t (*write_callback)(id_t *, offs_t, const char *, size_t);


struct atasrv_fs_t {
	uint8_t type;
	char name[FS_NAME_SIZE];
	atasrv_fs_t *next;

	int (*handler)(void *, msg_t *);
	int (*mount)(id_t *, void **, read_callback, write_callback);
	int (*unmount)(void *);
};


struct atasrv_part_t {
	int portfd;
	uint8_t type;
	uint32_t start;
	uint32_t size;
	uint32_t refs;

	atasrv_dev_t *dev;
	atasrv_fs_t *fs;

	void *data;
	char *stack;;
};


struct atasrv_req_t {
	int portfd;
	msg_t msg;
	unsigned rid;
	atasrv_fs_t *fs;
	void *data;
	atasrv_req_t *prev, *next;
};


struct {
	int portfd;
	unsigned int count;
	idtree_t devices;
	atasrv_fs_t *filesystems;
	atasrv_req_t *queue;
	handle_t lock, cond;

	char poolstack[4 * 4096] __attribute__((aligned(8)));
} atasrv_common;


static void atasrv_partthr(void *arg)
{
	atasrv_part_t *part = (atasrv_part_t *)arg;
	atasrv_req_t *req;

	for (;;) {
		if ((req = malloc(sizeof(atasrv_req_t))) == NULL)
			continue;

		req->portfd = part->portfd;
		req->fs = part->fs;
		req->data = part->data;

		if (msgRecv(part->portfd, &req->msg, &req->rid) < 0)
			continue;

		/* TBD: handle umount here?
		if (req->msg.type == mtUmount) {
		}
		*/

		mutexLock(atasrv_common.lock);
		LIST_ADD(&atasrv_common.queue, req);
		mutexUnlock(atasrv_common.lock);

		condSignal(atasrv_common.cond);
	}
}


static void atasrv_poolthr(void *arg)
{
	atasrv_req_t *req;
	int err;

	for (;;) {
		mutexLock(atasrv_common.lock);
		while (atasrv_common.queue == NULL)
			condWait(atasrv_common.cond, atasrv_common.lock, 0);
		req = atasrv_common.queue;
		LIST_REMOVE(&atasrv_common.queue, req);
		mutexUnlock(atasrv_common.lock);

		err = req->fs->handler(req->data, &req->msg);
		msgRespond(req->portfd, err, &req->msg, req->rid);
		free(req);
	}
}


static int atasrv_regdev(ata_dev_t *dev)
{
	atasrv_dev_t *srvdev;
	char name[16];
	int err;

	if ((srvdev = malloc(sizeof(atasrv_dev_t))) == NULL)
		return -ENOMEM;

	srvdev->dev = dev;
	srvdev->type = DEV_TYPE_HDD;

	idtree_alloc(&atasrv_common.devices, &srvdev->node);
	sprintf(name, "%s%c", DEV_NAME_BASE, DEV_NAME_CHAR + idtree_id(&srvdev->node));

	if ((err = create_dev(atasrv_common.portfd, idtree_id(&srvdev->node), name, S_IFBLK)))
		idtree_remove(&atasrv_common.devices, &srvdev->node);
	else
		atasrv_common.count++;

	return err;
}


static int atasrv_regpart(atasrv_dev_t *srvdev)
{
	atasrv_dev_t *srvpart;
	mbr_t *mbr;
	char name[16];
	int i, err = EOK;

	if ((mbr = malloc(sizeof(mbr_t))) == NULL)
		return -ENOMEM;

	if (!(err = read_mbr(srvdev->dev, mbr))) {
		for (i = 0; i < sizeof(mbr->pent) / sizeof(pentry_t); i++) {
			if (!(mbr->pent[i].type))
				continue;

			if ((srvpart = malloc(sizeof(atasrv_dev_t))) == NULL) {
				err = -ENOMEM;
				break;
			}

			if ((srvpart->part = malloc(sizeof(atasrv_part_t))) == NULL) {
				free(srvpart);
				err = -ENOMEM;
				break;
			}

			srvpart->type = DEV_TYPE_PART;
			srvpart->part->type = mbr->pent[i].type;
			srvpart->part->start = mbr->pent[i].start;
			srvpart->part->size = mbr->pent[i].sectors;
			srvpart->part->dev = srvdev;

			idtree_alloc(&atasrv_common.devices, &srvpart->node);
			sprintf(name, "%s%c%d", DEV_NAME_BASE, DEV_NAME_CHAR + idtree_id(&srvdev->node), i + 1);

			if ((err = create_dev(atasrv_common.portfd, idtree_id(&srvpart->node), name, S_IFBLK))) {
				free(srvpart->part);
				free(srvpart);
				break;
			}
		}
	}
	free(mbr);

	return err;
}


ssize_t atasrv_read(id_t *id, offs_t offs, char *buff, size_t len)
{
	atasrv_dev_t *srvdev;
	ata_dev_t *dev;

	if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, *id))) == NULL)
		return -EINVAL;

	switch(srvdev->type) {
		case DEV_TYPE_HDD:
			dev = srvdev->dev;
			break;

		case DEV_TYPE_PART:
			dev = srvdev->part->dev->dev;
			offs += srvdev->part->start * dev->secsize;
			break;

		default:
			return -EINVAL;
	}

	return ata_read(dev, offs, buff, len);
}


ssize_t atasrv_write(id_t *id, offs_t offs, const char *buff, size_t len)
{
	atasrv_dev_t *srvdev;
	ata_dev_t *dev;

	if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, *id))) == NULL)
		return -EINVAL;

	switch(srvdev->type) {
		case DEV_TYPE_HDD:
			dev = srvdev->dev;
			break;

		case DEV_TYPE_PART:
			dev = srvdev->part->dev->dev;
			offs += srvdev->part->start * dev->secsize;
			break;

		default:
			return -EINVAL;
	}

	return ata_write(dev, offs, buff, len);
}


static int atasrv_mount(id_t id, unsigned port, id_t *newid, mode_t *mode, const char *type, size_t len)
{
	atasrv_dev_t *srvdev;
	atasrv_fs_t *fs;
	int err;

	if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, id))) == NULL)
		return -EINVAL;

	if (len > FS_NAME_SIZE)
		len = FS_NAME_SIZE;

	for (fs = atasrv_common.filesystems; fs != NULL; fs = fs->next) {
		if (!strncmp(type, fs->name, len))
			break;
	}

	if ((srvdev->part->fs = fs) == NULL)
		return -EINVAL;

	if ((srvdev->part->portfd = portGet(port)) < 0)
		return srvdev->part->portfd;

	if ((srvdev->part->stack = malloc(2 * _PAGE_SIZE)) == NULL) {
		close(srvdev->part->portfd);
		return -ENOMEM;
	}

	if ((err = fs->mount(&id, &srvdev->part->data, atasrv_read, atasrv_write)) < 0) {
		close(srvdev->part->portfd);
		free(srvdev->part->stack);
		return err;
	}

	*newid = err;
	*mode = S_IFDIR;

	if ((err = beginthread(atasrv_partthr, 4, srvdev->part->stack, 2 * _PAGE_SIZE, srvdev->part)) < 0) {
		close(srvdev->part->portfd);
		free(srvdev->part->stack);
		fs->unmount(srvdev->part->data);
		return err;
	}

	return EOK;
}


static void atasrv_msgloop(void)
{
	msg_t msg;
	unsigned int rid, portfd = PORT_DESCRIPTOR;
	int err = EOK;

	for (;;) {
		if (msgRecv(portfd, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtRead:
			if ((msg.o.io = atasrv_read(&msg.object, msg.i.io.offs, msg.o.data, msg.o.size)) < 0) {
				err = (int)msg.o.io;
				msg.o.io = 0;
			}
			break;

		case mtWrite:
			if ((msg.o.io = atasrv_write(&msg.object, msg.i.io.offs, msg.i.data, msg.i.size)) < 0) {
				err = (int)msg.o.io;
				msg.o.io = 0;
			}
			break;

		case mtMount:
			err = atasrv_mount(msg.object, msg.i.mount.port, &msg.o.mount.id, &msg.o.mount.mode, msg.i.data, msg.i.size);
			break;

		/* TBD
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


static int atasrv_init(void)
{
	memset(&atasrv_common, 0, sizeof(atasrv_common));

	atasrv_common.portfd = PORT_DESCRIPTOR;
	idtree_init(&atasrv_common.devices);
	mutexCreate(&atasrv_common.lock);
	condCreate(&atasrv_common.cond);

	FS_REGISTER(EXT2);

	return EOK;
}


int main(void)
{
	atasrv_dev_t *srvdev;
	ata_dev_t *dev;
	int pid, sid, err, i;

	if ((err = atasrv_init()))
		return err;

	if ((err = ata_init()))
		return err;

	/* Register devices */
	dev = ata_common.devices;
	for (i = 0; i < ata_common.count; i++) {
		if (dev->type == ATA) {
			if ((err = atasrv_regdev(dev)))
				return err;
		}
		dev = dev->next;
	}

	/* Register partitions */
	for (i = 0; i < atasrv_common.count; i++) {
		if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, i))) == NULL)
			continue;

		if ((err = atasrv_regpart(srvdev)))
			return err;
	}

	pid = fork();

	if (pid < 0)
		exit(EXIT_FAILURE);

	if (pid > 0)
		exit(EXIT_SUCCESS);

	sid = setsid();

	if (sid < 0)
		exit(EXIT_FAILURE);

	beginthread(atasrv_poolthr, 4, atasrv_common.poolstack, sizeof(atasrv_common.poolstack), NULL);
	atasrv_msgloop();

	return EOK;
}
