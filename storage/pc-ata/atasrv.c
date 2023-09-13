/*
 * Phoenix-RTOS
 *
 * Generic ATA devices server
 *
 * Copyright 2019, 2020 Phoenix Systems
 * Author: Kamil Amanowicz, Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <signal.h>
#include <stdint.h>
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

#include "ata.h"
#include "mbr.h"


/* Misc definitions */
#define HDD_BASE "/dev/hd" /* Base name for HDD devices */


/* ATA server device types */
enum {
	DEV_BASE, /* ATA device */
	DEV_PART  /* ATA device partition */
};


/* ATA device access callbacks */
static ssize_t atasrv_read(id_t id, offs_t offs, char *buff, size_t len);
static ssize_t atasrv_write(id_t id, offs_t offs, const char *buff, size_t len);


/* Filesystem callbacks types */
typedef int (*fs_handler)(void *, msg_t *);
typedef int (*fs_unmount)(void *);
typedef int (*fs_mount)(oid_t *, unsigned int, typeof(atasrv_read) *, typeof(atasrv_write) *, void **);


typedef struct {
	rbnode_t node;              /* RBTree node */
	char name[16];              /* Filesystem name */
	uint8_t type;               /* Compatible partition type */
	fs_handler handler;         /* Message handler callback */
	fs_unmount unmount;         /* Unmount callback */
	fs_mount mount;             /* Mount callback */
} atasrv_fs_t;


typedef struct _atasrv_dev_t  atasrv_dev_t;
typedef struct _atasrv_base_t atasrv_base_t;
typedef struct _atasrv_part_t atasrv_part_t;
typedef struct _atasrv_req_t  atasrv_req_t;


struct _atasrv_dev_t {
	idnode_t node;              /* Device ID */
	uint8_t type;               /* Device type, 0: DEV_BASE, 1: DEV_PART */
	union {
		atasrv_base_t *base;    /* ATA device */
		atasrv_part_t *part;    /* ATA device partition */
	};
	atasrv_dev_t *prev, *next;  /* Doubly linked list */
};


struct _atasrv_base_t {
	unsigned int idx;           /* Device index */
	unsigned int npdevs;        /* Number of partitions within the device */
	atasrv_dev_t *pdevs;        /* Device partitions */
	ata_dev_t *dev;             /* Underlaying ATA device */
};


struct _atasrv_part_t {
	/* Partition data */
	unsigned int idx;           /* Partition index */
	unsigned int port;          /* Partition port */
	uint8_t type;               /* Partition type */
	uint32_t start;             /* Partition start (LBA) */
	uint32_t sectors;           /* Number of sectors */
	atasrv_dev_t *bdev;         /* ATA device the partition is part of */

	/* Filesystem data */
	atasrv_fs_t *fs;            /* Mounted filesystem */
	void *fdata;                /* Mounted filesystem data */

	/* Partition filesystem thread stack */
	char pstack[4 * _PAGE_SIZE] __attribute__((aligned(8)));
};


struct _atasrv_req_t {
	msg_t msg;                  /* Request msg */
	msg_rid_t rid;              /* Request receiving context */
	atasrv_part_t *part;        /* Request receiver partition */
	atasrv_req_t *prev, *next;  /* Doubly linked list */
};


struct {
	unsigned int port;          /* ATA server port */
	unsigned int ndevs;         /* Number of registered ATA devices */
	idtree_t sdevs;             /* Registered ATA server devices */
	rbtree_t fss;               /* Registered filesystems */
	atasrv_req_t *rqueue;       /* Requests FIFO queue */
	handle_t rlock, rcond;      /* Requests synchronization */

	/* Pool threads stacks */
	char pstacks[4][4 * _PAGE_SIZE] __attribute__((aligned(8)));
} atasrv_common;


static int atasrv_registerfs(const char *name, uint8_t type, fs_mount mount, fs_unmount unmount, fs_handler handler)
{
	atasrv_fs_t *fs;

	if ((fs = (atasrv_fs_t *)malloc(sizeof(atasrv_fs_t))) == NULL)
		return -ENOMEM;

	strcpy(fs->name, name);
	fs->type = type;
	fs->mount = mount;
	fs->unmount = unmount;
	fs->handler = handler;
	lib_rbInsert(&atasrv_common.fss, &fs->node);

	return EOK;
}


static int atasrv_cmpfs(rbnode_t *node1, rbnode_t *node2)
{
	atasrv_fs_t *fs1 = lib_treeof(atasrv_fs_t, node, node1);
	atasrv_fs_t *fs2 = lib_treeof(atasrv_fs_t, node, node2);

	return strcmp(fs1->name, fs2->name);
}


static atasrv_fs_t *atasrv_getfs(const char *name)
{
	atasrv_fs_t fs;

	strcpy(fs.name, name);

	return lib_treeof(atasrv_fs_t, node, lib_rbFind(&atasrv_common.fss, &fs.node));
}


static atasrv_fs_t *atasrv_findfs(uint8_t type)
{
	rbnode_t *node;
	atasrv_fs_t *fs;

	for (node = lib_rbMinimum(atasrv_common.fss.root); node != NULL; node = lib_rbNext(node)) {
		fs = lib_treeof(atasrv_fs_t, node, node);

		if (fs->type == type)
			return fs;
	}

	return NULL;
}


static void atasrv_fsthr(void *arg)
{
	atasrv_part_t *part = (atasrv_part_t *)arg;
	atasrv_req_t *req;
	int umount = 0;

	for (;;) {
		if ((req = (atasrv_req_t *)malloc(sizeof(atasrv_req_t))) == NULL)
			continue;

		req->part = part;
		while (msgRecv(req->part->port, &req->msg, &req->rid) < 0);

		if (req->msg.type == mtUmount)
			umount = 1;

		mutexLock(atasrv_common.rlock);

		LIST_ADD(&atasrv_common.rqueue, req);

		mutexUnlock(atasrv_common.rlock);
		condSignal(atasrv_common.rcond);

		if (umount)
			endthread();
	}
}


static void atasrv_poolthr(void *arg)
{
	atasrv_req_t *req;

	for (;;) {
		mutexLock(atasrv_common.rlock);

		while (atasrv_common.rqueue == NULL)
			condWait(atasrv_common.rcond, atasrv_common.rlock, 0);
		req = atasrv_common.rqueue->prev;
		LIST_REMOVE(&atasrv_common.rqueue, req);

		mutexUnlock(atasrv_common.rlock);

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


static int atasrv_initbase(ata_dev_t *dev)
{
	atasrv_dev_t *sdev;

	if ((sdev = (atasrv_dev_t *)malloc(sizeof(atasrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((sdev->base = (atasrv_base_t *)malloc(sizeof(atasrv_base_t))) == NULL) {
		free(sdev);
		return -ENOMEM;
	}

	sdev->type = DEV_BASE;
	sdev->prev = NULL;
	sdev->next = NULL;
	sdev->base->npdevs = 0;
	sdev->base->pdevs = NULL;
	sdev->base->dev = dev;
	sdev->base->idx = atasrv_common.ndevs++;
	idtree_alloc(&atasrv_common.sdevs, &sdev->node);

	return EOK;
}


static int atasrv_initpart(atasrv_dev_t *bdev, uint8_t type, uint32_t start, uint32_t sectors)
{
	atasrv_dev_t *pdev;
	int err;

	if ((bdev->type != DEV_BASE) || ((start + sectors) * bdev->base->dev->sectorsz > bdev->base->dev->size))
		return -EINVAL;

	if ((pdev = (atasrv_dev_t *)malloc(sizeof(atasrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((pdev->part = (atasrv_part_t *)malloc(sizeof(atasrv_part_t))) == NULL) {
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
	pdev->part->idx = bdev->base->npdevs++;
	idtree_alloc(&atasrv_common.sdevs, &pdev->node);
	LIST_ADD(&bdev->base->pdevs, pdev);

	return EOK;
}


static ssize_t atasrv_read(id_t id, offs_t offs, char *buff, size_t len)
{
	atasrv_dev_t *sdev;
	ata_dev_t *dev;

	if ((sdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	switch (sdev->type) {
	case DEV_BASE:
		dev = sdev->base->dev;
		if (offs + len > dev->size) {
			if (offs > dev->size)
				return -EINVAL;
			len = dev->size - offs;
		}
		break;

	case DEV_PART:
		dev = sdev->part->bdev->base->dev;
		if (offs + len > sdev->part->sectors * dev->sectorsz) {
			if (offs > sdev->part->sectors * dev->sectorsz)
				return -EINVAL;
			len = sdev->part->sectors * dev->sectorsz - offs;
		}
		offs += sdev->part->start * dev->sectorsz;
		break;

	default:
		return -1;
	}

	return ata_read(dev, offs, buff, len);
}


static ssize_t atasrv_write(id_t id, offs_t offs, const char *buff, size_t len)
{
	atasrv_dev_t *sdev;
	ata_dev_t *dev;

	if ((sdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	switch (sdev->type) {
	case DEV_BASE:
		dev = sdev->base->dev;
		if (offs + len > dev->size) {
			if (offs > dev->size)
				return -EINVAL;
			len = dev->size - offs;
		}
		break;

	case DEV_PART:
		dev = sdev->part->bdev->base->dev;
		if (offs + len > sdev->part->sectors * dev->sectorsz) {
			if (offs > sdev->part->sectors * dev->sectorsz)
				return -EINVAL;
			len = sdev->part->sectors * dev->sectorsz - offs;
		}
		offs += sdev->part->start * dev->sectorsz;
		break;

	default:
		return -1;
	}

	return ata_write(dev, offs, buff, len);
}


static int atasrv_mount(id_t id, const char *name, oid_t *oid)
{
	atasrv_dev_t *pdev;
	atasrv_fs_t *fs;
	int err;

	if ((pdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	if (pdev->type != DEV_PART)
		return -EINVAL;

	if (pdev->part->fs != NULL)
		return -EEXIST;

	if ((pdev->part->fs = fs = (name == NULL) ? atasrv_findfs(pdev->part->type) : atasrv_getfs(name)) == NULL)
		return -ENOENT;

	if (fs->type != pdev->part->type)
		return -EINVAL;

	oid->port = pdev->part->port;
	oid->id = id;

	if ((err = fs->mount(oid, pdev->part->bdev->base->dev->sectorsz, atasrv_read, atasrv_write, &pdev->part->fdata)) < 0)
		return err;
	oid->id = err;

	if ((err = beginthread(atasrv_fsthr, 4, pdev->part->pstack, sizeof(pdev->part->pstack), pdev->part)) < 0) {
		pdev->part->fs->unmount(pdev->part->fdata);
		pdev->part->fs = NULL;
		pdev->part->fdata = NULL;
		return err;
	}

	return EOK;
}


static int atasrv_getattr(id_t id, int type, long long *attr)
{
	atasrv_dev_t *sdev;

	if ((sdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, id))) == NULL)
		return -ENODEV;

	switch (type) {
		case atSize:
			switch (sdev->type) {
				case DEV_BASE:
					*attr = (off_t)sdev->base->dev->size;
					break;

				case DEV_PART:
					*attr = (off_t)sdev->part->sectors * sdev->part->bdev->base->dev->sectorsz;
					break;
			}
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void atasrv_msgloop(void *arg)
{
	msg_rid_t rid;
	unsigned port = *(unsigned int *)arg;
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;
	msg_t msg;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtMount:
			imnt = (mount_i_msg_t *)msg.i.raw;
			omnt = (mount_o_msg_t *)msg.o.raw;
			omnt->err = atasrv_mount(imnt->dev.id, imnt->fstype, &omnt->oid);
			break;

		case mtUmount:
			/* TODO: add umount() support, return -ENOTSUP for now */
			msg.o.io.err = -ENOTSUP;
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg.o.raw;
			/* TODO: get mountpoint, return -ENOTSUP for now */
			omnt->err = -ENOTSUP;
			break;

		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		case mtSync:
			msg.o.io.err = -ENOSYS;
			break;

		case mtRead:
			msg.o.io.err = atasrv_read(msg.i.io.oid.id, msg.i.io.offs, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			msg.o.io.err = atasrv_write(msg.i.io.oid.id, msg.i.io.offs, msg.i.data, msg.i.size);
			break;

		case mtGetAttr:
			msg.o.attr.err = atasrv_getattr(msg.i.attr.oid.id, msg.i.attr.type, &msg.o.attr.val);
			break;

		default:
			msg.o.io.err = -EINVAL;
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void atasrv_usage(const char *prog)
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


static void atasrv_signalexit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	atasrv_dev_t *sdev, *bdev;
	ata_dev_t *dev;
	rbnode_t *node;
	mbr_t *mbr;
	unsigned int i, j, type, start, sectors;
	int err, c, argn, id, pid, mroot = 0;
	oid_t oid;
	char path[32];

	/* Wait for console */
	while (write(1, "", 0) < 0)
		usleep(50000);

	/* Detect ATA devices */
	if ((err = ata_init()) < 0) {
		fprintf(stderr, "pc-ata: failed to detect ATA devices\n");
		return err;
	}

	/* Set parent exit handler */
	signal(SIGUSR1, atasrv_signalexit);

	/* Daemonize server */
	if ((pid = fork()) < 0) {
		fprintf(stderr, "pc-ata: failed to daemonize server\n");
		exit(EXIT_FAILURE);
	}
	else if (pid > 0) {
		/* Parent waits to be killed by the child after finished server initialization */
		sleep(10);
		fprintf(stderr, "pc-ata: failed to successfully initialize server\n");
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, atasrv_signalexit);

	if (setsid() < 0) {
		fprintf(stderr, "pc-ata: failed to create new session\n");
		exit(EXIT_FAILURE);
	}

	/* Init common server data */
	if ((err = portCreate(&atasrv_common.port)) < 0) {
		fprintf(stderr, "pc-ata: failed to create server port\n");
		return err;
	}

	if ((err = mutexCreate(&atasrv_common.rlock)) < 0) {
		fprintf(stderr, "pc-ata: failed to create server requests mutex\n");
		return err;
	}

	if ((err = condCreate(&atasrv_common.rcond)) < 0) {
		fprintf(stderr, "pc-ata: failed to create server requests condition variable\n");
		return err;
	}

	atasrv_common.ndevs = 0;
	atasrv_common.rqueue = NULL;
	idtree_init(&atasrv_common.sdevs);
	lib_rbInit(&atasrv_common.fss, atasrv_cmpfs, NULL);

	/* Register filesystems */
	if (atasrv_registerfs(LIBEXT2_NAME, LIBEXT2_TYPE, LIBEXT2_MOUNT, LIBEXT2_UNMOUNT, LIBEXT2_HANDLER) < 0)
		fprintf(stderr, "pc-ata: failed to register ext2 filesystem\n");

	/* Init base ATA devices - process the list in FIFO order */
	dev = ata_common.devs;
	for (i = 0; i < ata_common.ndevs; i++) {
		if ((err = atasrv_initbase((dev = dev->prev))) < 0) {
			fprintf(stderr, "pc-ata: failed to initialize ATA device %d\n", i);
			return err;
		}
	}

	if (argc > 1) {
		/* Process command line options */
		while ((c = getopt(argc, argv, "p:r:h")) != -1) {
			switch (c) {
			case 'p':
				if ((argn = optind - 1) > argc - 4) {
					fprintf(stderr, "pc-ata: missing arg(s) for -p option\n");
					return -EINVAL;
				}

				id = strtoul(argv[argn++], NULL, 0);
				type = strtoul(argv[argn++], NULL, 0);
				start = strtoul(argv[argn++], NULL, 0);
				sectors = strtoul(argv[argn++], NULL, 0);
				optind += 3;

				if (((bdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, id))) == NULL) || (bdev->type != DEV_BASE)) {
					fprintf(stderr, "pc-ata: invalid device id (%d) passed to -p option\n", id);
					return -EINVAL;
				}

				if ((err = atasrv_initpart(bdev, (uint8_t)type, (uint32_t)start, (uint32_t)sectors)) < 0) {
					fprintf(stderr, "pc-ata: failed to register partition on device %d starting at LBA %u\n", id, start);
					return err;
				}
				break;

			case 'r':
				id = strtoul(optarg, NULL, 0);

				if (mroot) {
					fprintf(stderr, "pc-ata: root partition is already mounted\n");
					return -EINVAL;
				}

				if ((err = atasrv_mount(atasrv_common.ndevs + id, NULL, &oid)) < 0) {
					fprintf(stderr, "pc-ata: failed to mount root partition %d\n", id);
					return err;
				}

				mroot = 1;
				break;

			case 'h':
			default:
				atasrv_usage(argv[0]);
				return EOK;
			}
		}
	}
	else {
		/* Init partitions from MBR */
		if ((mbr = (mbr_t *)malloc(sizeof(mbr_t))) == NULL)
			return -ENOMEM;

		for (i = 0; i < atasrv_common.ndevs; i++) {
			if ((bdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.sdevs, i))) == NULL)
				return -ENODEV;

			if ((err = mbr_read(bdev->base->dev, mbr)) < 0) {
				if (err == -ENOENT)
					continue;
				return err;
			}

			for (j = 0; j < sizeof(mbr->pent) / sizeof(mbr->pent[0]); j++) {
				if (!(mbr->pent[j].type))
					continue;
				if (atasrv_initpart(bdev, mbr->pent[j].type, mbr->pent[j].start, mbr->pent[j].sectors) < 0)
					fprintf(stderr, "pc-ata: failed to register MBR partition %u from device %u\n", j, i);
			}
		}
		free(mbr);

		/* Mount first detected MBR partition as root */
		if (atasrv_mount(atasrv_common.ndevs, NULL, &oid) < 0)
			fprintf(stderr, "pc-ata: failed to mount root MBR partition\n");
		else
			mroot = 1;
	}

	/* Register root */
	if (mroot && ((err = portRegister(oid.port, "/", &oid)) < 0)) {
		fprintf(stderr, "pc-ata: failed to register root filesystem\n");
		return err;
	}
	else {
		/* Wait for root filesystem */
		while (lookup("/", NULL, &oid) < 0)
			usleep(10000);
	}

	/* Run pool threads */
	for (i = 0; i < sizeof(atasrv_common.pstacks) / sizeof(atasrv_common.pstacks[0]); i++) {
		if ((err = beginthread(atasrv_poolthr, 4, atasrv_common.pstacks[i], sizeof(atasrv_common.pstacks[i]), NULL)) < 0) {
			fprintf(stderr, "pc-ata: failed to start pool thread %d\n", i);
			return err;
		}
	}

	/* Register devices */
	for (node = lib_rbMinimum(atasrv_common.sdevs.root); node != NULL; node = lib_rbNext(node)) {
		sdev = lib_treeof(atasrv_dev_t, node, lib_treeof(idnode_t, linkage, node));
		oid.port = atasrv_common.port;
		oid.id = idtree_id(&sdev->node);

		switch (sdev->type) {
		case DEV_BASE:
			sprintf(path, "%s%c", HDD_BASE, 'a' + sdev->base->idx);
			break;

		case DEV_PART:
			sprintf(path, "%s%c%d", HDD_BASE, 'a' + sdev->part->bdev->base->idx, sdev->part->idx);
			break;
		}

		if (create_dev(&oid, path) < 0)
			fprintf(stderr, "pc-ata: failed to register device %s\n", path);
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);

	atasrv_msgloop(&atasrv_common.port);

	return EOK;
}
