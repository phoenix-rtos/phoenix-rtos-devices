/*
 * Phoenix-RTOS
 *
 * VirtIO block device server
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/rb.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <posix/idtree.h>
#include <posix/utils.h>

#include <libext2.h>

#include "mbr.h"
#include "virtio-blk.h"


/* Misc definitions */
#define HDD_BASE "/dev/hd" /* Base name for HDD devices */


/* VirtIO-blk server device types */
enum {
	DEV_BASE, /* VirtIO block device */
	DEV_PART  /* VirtIO block device partition */
};


/* VirtIO block device access callbacks */
static ssize_t virtio_blksrv_read(id_t id, offs_t offs, char *buff, size_t len);
static ssize_t virtio_blksrv_write(id_t id, offs_t offs, const char *buff, size_t len);


/* Filesystem callbacks types */
typedef int (*fs_handler)(void *, msg_t *);
typedef int (*fs_unmount)(void *);
typedef int (*fs_mount)(oid_t *, unsigned int, typeof(virtio_blksrv_read) *, typeof(virtio_blksrv_write) *, void **);


typedef struct {
	rbnode_t node;      /* RBTree node */
	char name[16];      /* Filesystem name */
	uint8_t type;       /* Compatible partition type */
	fs_handler handler; /* Message handler callback */
	fs_unmount unmount; /* Unmount callback */
	fs_mount mount;     /* Mount callback */
} virtio_blksrv_fs_t;


typedef struct _virtio_blksrv_dev_t  virtio_blksrv_dev_t;
typedef struct _virtio_blksrv_base_t virtio_blksrv_base_t;
typedef struct _virtio_blksrv_part_t virtio_blksrv_part_t;
typedef struct _virtio_blksrv_req_t  virtio_blksrv_req_t;


struct _virtio_blksrv_dev_t {
	idnode_t node;                     /* Device ID */
	uint8_t type;                      /* Device type, 0: DEV_BASE, 1: DEV_PART */
	union {
		virtio_blksrv_base_t *base;    /* VirtIO block device */
		virtio_blksrv_part_t *part;    /* VirtIO block device partition */
	};
	virtio_blksrv_dev_t *prev, *next;  /* Doubly linked list */
};


struct _virtio_blksrv_base_t {
	unsigned int number;        /* Device number */
	unsigned int npdevs;        /* Number of partitions within the device */
	virtio_blksrv_dev_t *pdevs; /* Device partitions */
	virtio_blk_t *dev;          /* Underlaying VirtIO block device */
};


struct _virtio_blksrv_part_t {
	/* Partition data */
	unsigned int port;          /* Partition port */
	unsigned int number;        /* Partition number */
	uint8_t type;               /* Partition type */
	uint32_t start;             /* Partition start (LBA) */
	uint32_t sectors;           /* Number of sectors */
	virtio_blksrv_dev_t *bdev;  /* VirtIO block device the partition is part of */

	/* Filesystem data */
	virtio_blksrv_fs_t *fs;     /* Mounted filesystem */
	void *fdata;                /* Mounted filesystem data */

	/* Partition filesystem thread stack */
	char pstack[4 * _PAGE_SIZE] __attribute__((aligned(8)));
};


struct _virtio_blksrv_req_t {
	msg_t msg;                        /* Request msg */
	unsigned long rid;                /* Request rid */
	virtio_blksrv_part_t *part;       /* Request receiver partition */
	virtio_blksrv_req_t *prev, *next; /* Doubly linked list */
};


struct {
	unsigned int port;           /* VirtIO-blk server port */
	unsigned int ndevs;          /* Number of registered VirtIO block devices */
	idtree_t sdevs;              /* Registered VirtIO-blk server devices */
	rbtree_t fss;                /* Registered filesystems */
	virtio_blksrv_req_t *rqueue; /* Requests FIFO queue */
	handle_t rlock, rcond;       /* Requests synchronization */

	/* Pool threads stacks */
	char pstacks[4][4 * _PAGE_SIZE] __attribute__((aligned(8)));
} virtio_blksrv_common;


static int virtio_blksrv_registerfs(const char *name, uint8_t type, fs_mount mount, fs_unmount unmount, fs_handler handler)
{
	virtio_blksrv_fs_t *fs;

	if ((fs = (virtio_blksrv_fs_t *)malloc(sizeof(virtio_blksrv_fs_t))) == NULL)
		return -ENOMEM;

	strcpy(fs->name, name);
	fs->type = type;
	fs->mount = mount;
	fs->unmount = unmount;
	fs->handler = handler;
	lib_rbInsert(&virtio_blksrv_common.fss, &fs->node);

	return EOK;
}


static int virtio_blksrv_cmpfs(rbnode_t *node1, rbnode_t *node2)
{
	virtio_blksrv_fs_t *fs1 = lib_treeof(virtio_blksrv_fs_t, node, node1);
	virtio_blksrv_fs_t *fs2 = lib_treeof(virtio_blksrv_fs_t, node, node2);

	return strcmp(fs1->name, fs2->name);
}


static virtio_blksrv_fs_t *virtio_blksrv_getfs(const char *name)
{
	virtio_blksrv_fs_t fs;

	strcpy(fs.name, name);

	return lib_treeof(virtio_blksrv_fs_t, node, lib_rbFind(&virtio_blksrv_common.fss, &fs.node));
}


static virtio_blksrv_fs_t *virtio_blksrv_findfs(uint8_t type)
{
	rbnode_t *node;
	virtio_blksrv_fs_t *fs;

	for (node = lib_rbMinimum(virtio_blksrv_common.fss.root); node != NULL; node = lib_rbNext(node)) {
		fs = lib_treeof(virtio_blksrv_fs_t, node, node);

		if (fs->type == type)
			return fs;
	}

	return NULL;
}


static void virtio_blksrv_fsthr(void *arg)
{
	virtio_blksrv_part_t *part = (virtio_blksrv_part_t *)arg;
	virtio_blksrv_req_t *req;
	int umount = 0;

	for (;;) {
		if ((req = (virtio_blksrv_req_t *)malloc(sizeof(virtio_blksrv_req_t))) == NULL)
			continue;

		req->part = part;
		while (msgRecv(req->part->port, &req->msg, &req->rid) < 0);

		if (req->msg.type == mtUmount)
			umount = 1;

		mutexLock(virtio_blksrv_common.rlock);

		LIST_ADD(&virtio_blksrv_common.rqueue, req);

		mutexUnlock(virtio_blksrv_common.rlock);
		condSignal(virtio_blksrv_common.rcond);

		if (umount)
			endthread();
	}
}


static void virtio_blksrv_poolthr(void *arg)
{
	virtio_blksrv_req_t *req;

	for (;;) {
		mutexLock(virtio_blksrv_common.rlock);

		while (virtio_blksrv_common.rqueue == NULL)
			condWait(virtio_blksrv_common.rcond, virtio_blksrv_common.rlock, 0);
		req = virtio_blksrv_common.rqueue->prev;
		LIST_REMOVE(&virtio_blksrv_common.rqueue, req);

		mutexUnlock(virtio_blksrv_common.rlock);

		if (req->msg.type == mtUmount) {
			req->part->fs->unmount(req->part->fdata);
			req->part->fs = NULL;
			req->part->fdata = NULL;
		}
		else {
			req->part->fs->handler(req->part->fdata, &req->msg);
		}

		msgRespond(req->part->port, &req->msg, req->rid);
		free(req);
	}
}


static int virtio_blksrv_initbase(virtio_blk_t *dev)
{
	virtio_blksrv_dev_t *sdev;

	if ((sdev = (virtio_blksrv_dev_t *)malloc(sizeof(virtio_blksrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((sdev->base = (virtio_blksrv_base_t *)malloc(sizeof(virtio_blksrv_base_t))) == NULL) {
		free(sdev);
		return -ENOMEM;
	}

	sdev->type = DEV_BASE;
	sdev->prev = NULL;
	sdev->next = NULL;
	sdev->base->npdevs = 0;
	sdev->base->pdevs = NULL;
	sdev->base->dev = dev;
	sdev->base->number = virtio_blksrv_common.ndevs++;
	idtree_alloc(&virtio_blksrv_common.sdevs, &sdev->node);

	return EOK;
}


static int virtio_blksrv_initpart(virtio_blksrv_dev_t *bdev, uint8_t type, uint32_t start, uint32_t sectors)
{
	virtio_blksrv_dev_t *pdev;
	int err;

	if ((bdev->type != DEV_BASE) || ((start + sectors) * SECTOR_SIZE > bdev->base->dev->size))
		return -EINVAL;

	if ((pdev = (virtio_blksrv_dev_t *)malloc(sizeof(virtio_blksrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((pdev->part = (virtio_blksrv_part_t *)malloc(sizeof(virtio_blksrv_part_t))) == NULL) {
		free(pdev);
		return -ENOMEM;
	}

	if ((err = portCreate(&pdev->part->port)) < 0) {
		free(pdev->part);
		free(pdev);
		return err;
	}

	pdev->type = DEV_PART;
	pdev->prev = NULL;
	pdev->next = NULL;
	pdev->part->type = type;
	pdev->part->start = start;
	pdev->part->sectors = sectors;
	pdev->part->fs = NULL;
	pdev->part->fdata = NULL;
	pdev->part->bdev = bdev;
	pdev->part->number = bdev->base->npdevs++;
	idtree_alloc(&virtio_blksrv_common.sdevs, &pdev->node);
	LIST_ADD(&bdev->base->pdevs, pdev);

	return EOK;
}


static int virtio_blksrv_read(id_t id, offs_t offs, char *buff, size_t len);
{
	virtio_blksrv_dev_t *sdev;
	virtio_blk_t *dev;

	if ((sdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, msg->i.io.oid.id))) == NULL)
		return -ENODEV;

	switch (sdev->type) {
	case DEV_BASE:
		dev = sdev->base->dev;
		if (msg->i.io.offs + msg->o.size > dev->size) {
			if (msg->i.io.offs > dev->size)
				return -EINVAL;
			msg->o.size = dev->size - msg->i.io.offs;
		}
		break;

	case DEV_PART:
		dev = sdev->part->bdev->base->dev;
		if (msg->i.io.offs + msg->o.size > sdev->part->sectors * dev->sectorsz) {
			if (msg->i.io.offs > sdev->part->sectors * dev->sectorsz)
				return -EINVAL;
			msg->o.size = sdev->part->sectors * dev->sectorsz - msg->i.io.offs;
		}
		msg->i.io.offs += sdev->part->start * dev->sectorsz;
		break;

	default:
		return -EFAULT;
	}

	return virtio_blk_read(dev, port, rid, msg);
}


static ssize_t virtio_blksrv_write(id_t id, offs_t offs, const char *buff, size_t len)
{
	virtio_blksrv_dev_t *sdev;
	virtio_blk_t *dev;

	if ((sdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	switch (sdev->type) {
	case DEV_BASE:
		dev = sdev->base->dev;
		break;

	case DEV_PART:
		dev = sdev->part->bdev->base->dev;
		offs += sdev->part->start * SECTOR_SIZE;
		break;

	default:
		return -1;
	}

	return virtio_blk_write(dev, offs, buff, len);
}


static int virtio_blksrv_mount(id_t id, const char *name, oid_t *oid)
{
	virtio_blksrv_dev_t *pdev;
	virtio_blksrv_fs_t *fs;
	int err;

	if ((pdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	if (pdev->type != DEV_PART)
		return -EINVAL;

	if (pdev->part->fs != NULL)
		return -EEXIST;

	if ((pdev->part->fs = fs = (name == NULL) ? virtio_blksrv_findfs(pdev->part->type) : virtio_blksrv_getfs(name)) == NULL)
		return -ENOENT;

	if (fs->type != pdev->part->type)
		return -EINVAL;

	oid->port = pdev->part->port;
	oid->id = id;

	if ((err = fs->mount(oid, SECTOR_SIZE, virtio_blksrv_read, virtio_blksrv_write, &pdev->part->fdata)) < 0)
		return err;

	oid->id = err;

	if ((err = beginthread(virtio_blksrv_fsthr, 4, pdev->part->pstack, sizeof(pdev->part->pstack), pdev->part)) < 0) {
		pdev->part->fs->unmount(pdev->part->fdata);
		pdev->part->fs = NULL;
		pdev->part->fdata = NULL;
		return err;
	}

	return EOK;
}


static int virtio_blksrv_getattr(id_t id, int type, int *attr)
{
	virtio_blksrv_dev_t *sdev;

	if ((sdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	switch (type) {
	case atSize:
		switch (sdev->type) {
		case DEV_BASE:
			*attr = sdev->base->dev->size;
			break;

		case DEV_PART:
			*attr = sdev->part->sectors * SECTOR_SIZE;
			break;
		}
		break;
	}

	return EOK;
}


static void virtio_blksrv_msgloop(void *arg)
{
	unsigned int port = *(unsigned int *)arg;
	unsigned long rid;
	mount_msg_t *mnt;
	msg_t msg;
	int err;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtMount:
			mnt = (mount_msg_t *)msg.i.raw;
			virtio_blksrv_mount(mnt->id, mnt->fstype, (oid_t *)msg.o.raw);
			break;

		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		case mtSync:
			msg.o.io.err = -ENOSYS;
			break;

		case mtRead:
			if (!(err = virtio_blksrv_read(port, rid, &msg)))
				continue;
			msg.o.io.err = err;
			break;

		case mtWrite:
			virtio_blksrv_write(msg.i.io.oid.id, msg.i.io.offs, msg.i.data, msg.i.size);
			continue;

		case mtGetAttr:
			virtio_blksrv_getattr(msg.i.attr.oid.id, msg.i.attr.type, &msg.o.attr.val);
			break;

		default:
			msg.o.io.err = -EINVAL;
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void virtio_blksrv_usage(const char *prog)
{
	printf("Usage: %s [options] or no args to mount first MBR partition as root\n", prog);
	printf("\t-p <id> <type> <start> <size> - registers partition\n");
	printf("\t\tid:    device id starting at 0\n");
	printf("\t\ttype:  partition type e.g. 0x83 for native Linux partition\n");
	printf("\t\tstart: partition start (LBA)\n");
	printf("\t\tsize:  partition size in sectors\n");
	printf("\t-r <id>                       - mounts root partition\n");
	printf("\t\tid:    partition id starting at 0\n");
	printf("\t-h                            - shows this help message\n");
}


static void virtio_blksrv_signalexit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	virtio_blksrv_dev_t *sdev, *bdev;
	virtio_blk_t *dev;
	rbnode_t *node;
	mbr_t *mbr;
	unsigned int i, j, type, start, sectors;
	int err, c, argn, id, pid, mroot = 0;
	oid_t oid;
	char path[32];

	/* Wait for console */
	while (write(STDOUT_FILENO, "", 0) < 0)
		usleep(10000);

	/* Detect VirtIO block devices */
	if ((err = virtio_blk_init()) < 0) {
		fprintf(stderr, "virtio-blk: failed to detect VirtIO block devices\n");
		return err;
	}

	/* Set parent exit handler */
	// signal(SIGUSR1, virtio_blksrv_signalexit);

	// /* Daemonize server */
	// if ((pid = fork()) < 0) {
	// 	fprintf(stderr, "virtio-blk: failed to daemonize server\n");
	// 	exit(EXIT_FAILURE);
	// }
	// else if (pid > 0) {
	// 	/* Parent waits to be killed by the child after finished server initialization */
	// 	sleep(10);
	// 	fprintf(stderr, "virtio-blk: failed to successfully initialize server\n");
	// 	exit(EXIT_FAILURE);
	// }

	// printf("ndevs: %u\n", virtio_blk_common.ndevs);
	// for (;;);

	/* Set child exit handler */
	// signal(SIGUSR1, virtio_blksrv_signalexit);

	// if (setsid() < 0) {
	// 	fprintf(stderr, "virtio-blk: failed to create new session\n");
	// 	exit(EXIT_FAILURE);
	// }

	/* Init common server data */
	if ((err = portCreate(&virtio_blksrv_common.port)) < 0) {
		fprintf(stderr, "virtio-blk: failed to create server port\n");
		return err;
	}

	if ((err = mutexCreate(&virtio_blksrv_common.rlock)) < 0) {
		fprintf(stderr, "virtio-blk: failed to create server requests mutex\n");
		return err;
	}

	if ((err = condCreate(&virtio_blksrv_common.rcond)) < 0) {
		fprintf(stderr, "virtio-blk: failed to create server requests condition variable\n");
		return err;
	}

	virtio_blksrv_common.ndevs = 0;
	virtio_blksrv_common.rqueue = NULL;
	idtree_init(&virtio_blksrv_common.sdevs);
	lib_rbInit(&virtio_blksrv_common.fss, virtio_blksrv_cmpfs, NULL);

	/* Register filesystems */
	if (virtio_blksrv_registerfs(LIBEXT2_NAME, LIBEXT2_TYPE, LIBEXT2_MOUNT, LIBEXT2_UNMOUNT, LIBEXT2_HANDLER) < 0)
		fprintf(stderr, "virtio-blk: failed to register ext2 filesystem\n");

	/* Init base VirtIO block devices - process the list in FIFO order */
	dev = virtio_blk_common.devs;
	for (i = 0; i < virtio_blk_common.ndevs; i++) {
		if ((err = virtio_blksrv_initbase((dev = dev->prev))) < 0) {
			fprintf(stderr, "virtio-blk: failed to initialize VirtIO block device %d\n", i);
			return err;
		}
	}

	if (argc > 1) {
		/* Process command line options */
		while ((c = getopt(argc, argv, "p:r:h")) != -1) {
			switch (c) {
			case 'p':
				if ((argn = optind - 1) > argc - 4) {
					fprintf(stderr, "virtio-blk: missing arg(s) for -p option\n");
					return -EINVAL;
				}

				id = strtoul(argv[argn++], NULL, 0);
				type = strtoul(argv[argn++], NULL, 0);
				start = strtoul(argv[argn++], NULL, 0);
				sectors = strtoul(argv[argn++], NULL, 0);
				optind += 3;

				if (((bdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, id))) == NULL) || (bdev->type != DEV_BASE)) {
					fprintf(stderr, "virtio-blk: invalid device id (%d) passed to -p option\n", id);
					return -EINVAL;
				}

				if ((err = virtio_blksrv_initpart(bdev, (uint8_t)type, (uint32_t)start, (uint32_t)sectors)) < 0) {
					fprintf(stderr, "virtio-blk: failed to register partition on device %d starting at LBA %u\n", id, start);
					return err;
				}
				break;

			case 'r':
				id = strtoul(optarg, NULL, 0);

				if (mroot) {
					fprintf(stderr, "virtio-blk: root partition is already mounted\n");
					return -EINVAL;
				}

				if ((err = virtio_blksrv_mount(virtio_blksrv_common.ndevs + id, NULL, &oid)) < 0) {
					fprintf(stderr, "virtio-blk: failed to mount root partition %d\n", id);
					return err;
				}

				mroot = 1;
				break;

			case 'h':
			default:
				virtio_blksrv_usage(argv[0]);
				return EOK;
			}
		}
	}
	else {
		/* Init partitions from MBR */
		if ((mbr = (mbr_t *)malloc(sizeof(mbr_t))) == NULL)
			return -ENOMEM;

		for (i = 0; i < virtio_blksrv_common.ndevs; i++) {
			if ((bdev = lib_treeof(virtio_blksrv_dev_t, node, idtree_find(&virtio_blksrv_common.sdevs, i))) == NULL)
				return -ENODEV;

			if ((err = mbr_read(bdev->base->dev, mbr)) < 0) {
				if (err == -ENOENT)
					continue;
				return err;
			}

			for (j = 0; j < sizeof(mbr->pent) / sizeof(mbr->pent[0]); j++) {
				if (!(mbr->pent[j].type))
					continue;
				if (virtio_blksrv_initpart(bdev, mbr->pent[j].type, mbr->pent[j].start, mbr->pent[j].sectors) < 0)
					fprintf(stderr, "virtio-blk: failed to register MBR partition %u from device %u\n", j, i);
			}
		}
		free(mbr);

		/* Mount first detected MBR partition as root */
		if (virtio_blksrv_mount(virtio_blksrv_common.ndevs, NULL, &oid) < 0)
			fprintf(stderr, "virtio-blk: failed to mount root MBR partition\n");
		else
			mroot = 1;
	}

	/* Register root */
	if (mroot && ((err = portRegister(oid.port, "/", &oid)) < 0)) {
		fprintf(stderr, "virtio-blk: failed to register root filesystem\n");
		return err;
	}
	else {
		/* Wait for root filesystem */
		while (lookup("/", NULL, &oid) < 0)
			usleep(10000);
	}

	/* Run pool threads */
	for (i = 0; i < sizeof(virtio_blksrv_common.pstacks) / sizeof(virtio_blksrv_common.pstacks[0]); i++) {
		if ((err = beginthread(virtio_blksrv_poolthr, 4, virtio_blksrv_common.pstacks[i], sizeof(virtio_blksrv_common.pstacks[i]), NULL)) < 0) {
			fprintf(stderr, "virtio-blk: failed to start pool thread %d\n", i);
			return err;
		}
	}

	/* Register devices */
	for (node = lib_rbMinimum(virtio_blksrv_common.sdevs.root); node != NULL; node = lib_rbNext(node)) {
		sdev = lib_treeof(virtio_blksrv_dev_t, node, lib_treeof(idnode_t, linkage, node));
		oid.port = virtio_blksrv_common.port;
		oid.id = idtree_id(&sdev->node);

		switch (sdev->type) {
		case DEV_BASE:
			sprintf(path, "%s%c", HDD_BASE, 'a' + sdev->base->number);
			break;

		case DEV_PART:
			sprintf(path, "%s%c%d", HDD_BASE, 'a' + sdev->part->bdev->base->number, sdev->part->number);
			break;
		}

		if (create_dev(&oid, path) < 0)
			fprintf(stderr, "virtio-blk: failed to register device %s\n", path);
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);

	virtio_blksrv_msgloop(&virtio_blksrv_common.port);

	return EOK;
}
