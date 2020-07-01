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

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/file.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/rb.h>
#include <sys/threads.h>

#include <posix/idtree.h>
#include <posix/utils.h>

#include <libext2.h>

#include "ata.h"
#include "mbr.h"


/* ATA server device types */
enum {
	DEV_HDD  = 0xFF, /* Hard Disk Drive */
	DEV_PART = 0x01  /* HDD Partition */
};

/* ATA server devices name definitions */
#define HDD_NAME_BASE "/dev/hd"
#define HDD_NAME_CHAR 'a'

/* Filesystem registration macro */
#define FS_REGISTER(NAME) do {                           \
	atasrv_fs_t *fs;                                     \
	if ((fs = malloc(sizeof(atasrv_fs_t))) == NULL)      \
		return -ENOMEM;                                  \
	strcpy(fs->name, LIB##NAME##_NAME);                  \
	fs->type    =  LIB##NAME##_TYPE;                     \
	fs->mount   = &LIB##NAME##_MOUNT;                    \
	fs->unmount = &LIB##NAME##_UNMOUNT;                  \
	fs->handler = &LIB##NAME##_HANDLER;                  \
	lib_rbInsert(&atasrv_common.filesystems, &fs->node); \
} while (0)


/* Callbacks for ATA device access */
typedef ssize_t (*read_cb)(id_t, offs_t, char *, size_t);
typedef ssize_t (*write_cb)(id_t, offs_t, const char *, size_t);


typedef struct {
	rbnode_t node;                                     /* RB tree node */
	char name[16];                                     /* Filesystem name */
	uint8_t type;                                      /* Compatible partition type */
	int (*mount)(oid_t *, read_cb, write_cb, void **); /* Mount callback */
	int (*unmount)(void *);                            /* Unmount callback */
	int (*handler)(void *, msg_t *);                   /* Message handler callback */
} atasrv_fs_t;


typedef struct atasrv_hdd_t atasrv_hdd_t;
typedef struct atasrv_part_t atasrv_part_t;
typedef struct atasrv_dev_t atasrv_dev_t;
typedef struct atasrv_req_t atasrv_req_t;


struct atasrv_hdd_t {
	ata_dev_t *dev;             /* ATA device */
	unsigned int nparts;        /* Number of registered partitions */
	atasrv_part_t *parts;       /* Registered partitions */
};


struct atasrv_part_t {
	uint8_t type;               /* Partition type */
	uint32_t port;              /* Partition port */
	uint32_t start;             /* Partition start (LBA) */
	uint32_t sectors;           /* Number of sectors */

	atasrv_dev_t *srvhdd;       /* HDD device the partition belongs to */
	atasrv_fs_t *fs;            /* Mounted filesystem */

	void *data;                 /* Mounted filesystem data */
	char *stack;                /* Partition thread stack */
	atasrv_part_t *prev, *next; /* Doubly linked list */
};


struct atasrv_dev_t {
	idnode_t node;              /* Device ID */
	uint8_t type;               /* Device type */
	union {
		atasrv_hdd_t *hdd;      /* HDD device */
		atasrv_part_t *part;    /* Partition device */
	};
};


struct atasrv_req_t {
	msg_t msg;                  /* Request msg */
	unsigned int rid;           /* Request rid */

	atasrv_part_t *part;        /* Request receiver partition */
	atasrv_req_t *prev, *next;  /* Doubly linked list */
};


struct {
	uint32_t port;              /* ATA server port */
	unsigned int ndevices;      /* Number of HDD devices */
	idtree_t devices;           /* Registered devices */
	rbtree_t filesystems;       /* Registered filesystems */
	atasrv_req_t *queue;        /* Requests FIFO queue */
	handle_t lock, cond;        /* Requests synchronization */
	char poolstacks[1][4 * _PAGE_SIZE] __attribute__((aligned(8)));
} atasrv_common;


static int atasrv_fscmp(rbnode_t *node1, rbnode_t *node2)
{
	atasrv_fs_t *fs1 = lib_treeof(atasrv_fs_t, node, node1);
	atasrv_fs_t *fs2 = lib_treeof(atasrv_fs_t, node, node2);

	return strcmp(fs1->name, fs2->name);
}


static atasrv_fs_t *atasrv_fsfind(const char *name)
{
	rbnode_t *node = atasrv_common.filesystems.root;
	atasrv_fs_t *fs;
	int c;

	while (node != NULL) {
		fs = lib_treeof(atasrv_fs_t, node, node);
		c = strcmp(fs->name, name);
		if (c == 0)
			return fs;

		node = (c > 0) ? node->left : node->right;
	}

	return NULL;
}


static void atasrv_partthr(void *arg)
{
	atasrv_part_t *part = (atasrv_part_t *)arg;
	atasrv_req_t *req;

	for (;;) {
		if ((req = malloc(sizeof(atasrv_req_t))) == NULL)
			continue;

		req->part = part;
		while (msgRecv(req->part->port, &req->msg, &req->rid) < 0);

		/* TODO: handle umount here?
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

	for (;;) {
		mutexLock(atasrv_common.lock);
		while (atasrv_common.queue == NULL)
			condWait(atasrv_common.cond, atasrv_common.lock, 0);
		req = atasrv_common.queue;
		LIST_REMOVE(&atasrv_common.queue, req);
		mutexUnlock(atasrv_common.lock);

		req->part->fs->handler(req->part->data, &req->msg);
		msgRespond(req->part->port, &req->msg, req->rid);
		free(req);
	}
}


static int atasrv_inithdd(ata_dev_t *dev)
{
	atasrv_dev_t *srvhdd;

	if ((srvhdd = malloc(sizeof(atasrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((srvhdd->hdd = malloc(sizeof(atasrv_hdd_t))) == NULL) {
		free(srvhdd);
		return -ENOMEM;
	}

	srvhdd->type = DEV_HDD;
	srvhdd->hdd->dev = dev;
	srvhdd->hdd->nparts = 0;
	srvhdd->hdd->parts = NULL;
	idtree_alloc(&atasrv_common.devices, &srvhdd->node);
	atasrv_common.ndevices++;

	return EOK;
}


static int atasrv_initpart(atasrv_dev_t *srvhdd, uint8_t type, uint32_t start, uint32_t sectors)
{
	atasrv_dev_t *srvpart;
	int err;

	if ((srvpart = malloc(sizeof(atasrv_dev_t))) == NULL)
		return -ENOMEM;

	if ((srvpart->part = malloc(sizeof(atasrv_part_t))) == NULL) {
		free(srvpart);
		return -ENOMEM;
	}

	if ((srvpart->part->stack = malloc(2 * _PAGE_SIZE)) == NULL) {
		free(srvpart->part);
		free(srvpart);
		return -ENOMEM;
	}

	srvpart->type = DEV_PART;
	srvpart->part->type = type;
	srvpart->part->start = start;
	srvpart->part->sectors = sectors;
	srvpart->part->srvhdd = srvhdd;
	srvpart->part->fs = NULL;
	srvpart->part->data = NULL;
	idtree_alloc(&atasrv_common.devices, &srvpart->node);

	if ((err = portCreate(&srvpart->part->port)) < 0) {
		idtree_remove(&atasrv_common.devices, &srvpart->node);
		free(srvpart->part->stack);
		free(srvpart->part);
		free(srvpart);
		return err;
	}

	if ((err = beginthread(atasrv_partthr, 4, srvpart->part->stack, sizeof(srvpart->part->stack), srvpart->part)) < 0) {
		portDestroy(srvpart->part->port);
		idtree_remove(&atasrv_common.devices, &srvpart->node);
		free(srvpart->part->stack);
		free(srvpart->part);
		free(srvpart);
		return err;
	}

	return EOK;
}


static int atasrv_reghdd(atasrv_dev_t *srvhdd)
{
	char path[32];
	oid_t oid;
	int err;

	oid.port = atasrv_common.port;
	oid.id = idtree_id(&srvhdd->node);
	sprintf(path, "%s%c", HDD_NAME_BASE, HDD_NAME_CHAR + idtree_id(&srvhdd->node));

	if ((err = create_dev(&oid, path)) < 0)
		return err;

	return EOK;
}


static int atasrv_regpart(atasrv_dev_t *srvpart)
{
	char path[32];
	oid_t oid;
	int err;

	oid.port = atasrv_common.port;
	oid.id = idtree_id(&srvpart->node);
	sprintf(path, "%s%c%d", HDD_NAME_BASE, HDD_NAME_CHAR + idtree_id(&srvpart->part->srvhdd->node), srvpart->part->srvhdd->hdd->nparts);

	if ((err = create_dev(&oid, path)) < 0)
		return err;

	LIST_ADD(&srvpart->part->srvhdd->hdd->parts, srvpart->part);
	srvpart->part->srvhdd->hdd->nparts++;

	return EOK;
}


ssize_t atasrv_read(id_t id, offs_t offs, char *buff, size_t len)
{
	atasrv_dev_t *srvdev;
	ata_dev_t *dev;

	if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, id))) == NULL)
		return -ENODEV;

	switch (srvdev->type) {
		case DEV_HDD:
			dev = srvdev->hdd->dev;
			break;

		case DEV_PART:
			dev = srvdev->part->srvhdd->hdd->dev;
			offs += srvdev->part->start * dev->sectorsz;
			break;

		default:
			return -1;
	}

	return ata_read(dev, offs, buff, len);
}


ssize_t atasrv_write(id_t id, offs_t offs, const char *buff, size_t len)
{
	atasrv_dev_t *srvdev;
	ata_dev_t *dev;

	if ((srvdev = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, id))) == NULL)
		return -ENODEV;

	switch (srvdev->type) {
		case DEV_HDD:
			dev = srvdev->hdd->dev;
			break;

		case DEV_PART:
			dev = srvdev->part->srvhdd->hdd->dev;
			offs += srvdev->part->start * dev->sectorsz;
			break;

		default:
			return -1;
	}

	return ata_write(dev, offs, buff, len);
}


static int atasrv_mount(id_t id, const char *name, oid_t *oid)
{
	atasrv_dev_t *srvpart;
	atasrv_fs_t *fs;
	int err;

	if ((srvpart = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, id))) == NULL)
		return -ENODEV;

	if (srvpart->type != DEV_PART)
		return -EINVAL;

	if ((fs = atasrv_fsfind(name)) == NULL)
		return -EINVAL;

	if (srvpart->part->type != fs->type)
		return -EINVAL;

	oid->port = srvpart->part->port;
	oid->id = idtree_id(&srvpart->node);

	if ((err = fs->mount(oid, atasrv_read, atasrv_write, &srvpart->part->data)) < 0)
		return err;

	srvpart->part->fs = fs;

	return EOK;
}


static void atasrv_msgloop(void *arg)
{
	uint32_t port = (uint32_t)arg;
	unsigned int rid;
	mount_msg_t *mnt;
	oid_t *oid;
	msg_t msg;
	int err;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtRead:
			msg.o.io.err = atasrv_read(msg.i.io.oid.id, msg.i.io.offs, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			msg.o.io.err = atasrv_write(msg.i.io.oid.id, msg.i.io.offs, msg.i.data, msg.i.size);
			break;

		case mtMount:
			mnt = (mount_msg_t *)msg.i.raw;
			oid = (oid_t *)msg.o.raw;

			if ((err = atasrv_mount(mnt->id, mnt->fstype, oid)) < 0) {
				oid->port = -1;
				oid->id = 0;
			}
			break;

		/* TBD
		case mtSync:
			msg.o.io.err = -ENOSYS;
			break;
		*/

		case mtGetAttr:
			msg.o.io.err = -ENOSYS;
			break;

		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		default:
			msg.o.io.err = -EINVAL;
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


static void print_usage(const char* progname) {
	printf("Usage: %s [OPTIONS] or no args to mount first registered MBR partition as root\n", progname);
	printf("\t-p id type start size    Register HDD partition\n");
	printf("\t\tid:    HDD id starting at 0\n");
	printf("\t\ttype:  partition type e.g. 0x83 for native Linux partition\n");
	printf("\t\tstart: partition start (LBA)\n");
	printf("\t\tsize:  partition size in sectors\n");
	printf("\t-r id name               Mount root partition\n");
	printf("\t\tid:    partition id starting at 0\n");
	printf("\t\tname:  filesystem name e.g. ext2\n");
	printf("\t-d                       Daemonize server\n")
	printf("\t-h                       Show this help message\n");
}


int main(int argc, char **argv)
{
	atasrv_dev_t *srvdev, *srvhdd;
	ata_dev_t *dev;
	mbr_t *mbr;
	rbnode_t *node;
	id_t id;
	oid_t root = { -1, 0 };
	char *name;
	uint8_t type;
	uint32_t start, sectors;
	int i, j, c, argn, pid, sid, err;
	int daemonize = 0;

	if ((err = ata_init()))
		return err;

	if ((err = portCreate(&atasrv_common.port)) < 0)
		return err;

	if ((err = mutexCreate(&atasrv_common.lock)) < 0)
		return err;

	if ((err = condCreate(&atasrv_common.cond)) < 0)
		return err;

	atasrv_common.ndevices = 0;
	idtree_init(&atasrv_common.devices);
	lib_rbInit(&atasrv_common.filesystems, atasrv_fscmp, NULL);
	atasrv_common.queue = NULL;

	/* Register filesystems */
	FS_REGISTER(EXT2);

	/* Start thread pool */
	for (i = 0; i < sizeof(atasrv_common.poolstacks) / sizeof(atasrv_common.poolstacks[0]); i++)
		beginthread(atasrv_poolthr, 4, atasrv_common.poolstacks[i], sizeof(atasrv_common.poolstacks[i]), NULL);

	/* Init HDD devices */
	dev = ata_common.devices;
	for (i = 0; i < ata_common.ndevices; i++) {
		dev = dev->prev;
		if ((err = atasrv_inithdd(dev)))
			return err;
	}

	if (argc > 1) {
		while ((c = getopt(argc, argv, "dhp:r:")) != -1) {
			switch (c) {
			case 'd':
				daemonize = 1;

			case 'r':
				argn = optind - 1;
				if (argn + 1 < argc) {
					id = atoi(argv[argn++]);
					name = argv[argn];
					optind += 1;

					if ((err = atasrv_mount(atasrv_common.ndevices + id, name, &root)) < 0) {
						root.port = -1;
						root.id = 0;
					}
				}
				break;

			case 'p':
				argn = optind - 1;
				if (argn + 3 < argc) {
					id = atoi(argv[argn++]);
					type = (uint8_t)strtoul(argv[argn++], NULL, 0);
					start = (uint32_t)strtoul(argv[argn++], NULL, 0);
					sectors = (uint32_t)strtoul(argv[argn], NULL, 0);
					optind += 3;

					if ((srvhdd = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, id))) == NULL)
						return -ENODEV;

					if (srvhdd->type != DEV_HDD)
						return -EINVAL;

					if ((err = atasrv_initpart(srvhdd, type, start, sectors)) < 0)
						return err;
				}
				break;

			case 'h':
			default:
				print_usage(argv[0]);
				return EOK;
			}
		}
	}
	else {
		/* Init MBR partitions */
		if ((mbr = malloc(sizeof(mbr_t))) == NULL)
			return -ENOMEM;

		for (i = 0; i < atasrv_common.ndevices; i++) {
			if ((srvhdd = lib_treeof(atasrv_dev_t, node, idtree_find(&atasrv_common.devices, i))) == NULL)
				return -ENODEV;

			if ((err = read_mbr(srvhdd->hdd->dev, mbr)) < 0) {
				if (err == -ENOENT)
					continue;
				return err;
			}

			for (j = 0; j < sizeof(mbr->pent) / sizeof(mbr->pent[0]); j++) {
				if (!(mbr->pent[j].type))
					continue;
				if ((err = atasrv_initpart(srvhdd, mbr->pent[j].type, mbr->pent[j].start, mbr->pent[j].sectors)) < 0)
					return err;
			}
		}
		free(mbr);

		/* Try mounting first MBR partition with ext2 filesystem as root*/
		if ((err = atasrv_mount(atasrv_common.ndevices, "ext2", &root)) < 0) {
			root.port = -1;
			root.id = 0;
		}
	}

	if (root.port) {
		/* Register root */
		root.port = atasrv_common.ndevices;
		root.id = 2;
		if ((err = portRegister(root.port, "/", &root)) < 0)
			return err;

		atasrv_mount(atasrv_common.ndevices, "ext2", &root);
		/* Register devices */
		for (node = lib_rbMinimum(atasrv_common.devices.root); node != NULL; node = lib_rbNext(node)) {
			srvdev = lib_treeof(atasrv_dev_t, node, node);

			switch (srvdev->type) {
			case DEV_HDD:
				if ((err = atasrv_reghdd(srvdev)) < 0)
					return err;
				break;

			case DEV_PART:
				if ((err = atasrv_regpart(srvdev)) < 0)
					return err;
				break;

			default:
				return -1;
			}
		}
	}

	if (daemonize) {
		/* Daemonize */
		pid = fork();

		if (pid < 0)
			exit(EXIT_FAILURE);

		if (pid > 0)
			exit(EXIT_SUCCESS);

		sid = setsid();

		if (sid < 0)
			exit(EXIT_FAILURE);
	}

	atasrv_msgloop((void *)(atasrv_common.port));

	return EOK;
}
