/*
 * Phoenix-RTOS
 *
 * USB Mass Storage class driver
 *
 * Copyright 2021, 2024 Phoenix Systems
 * Author: Maciej Purski, Adam Greloch
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <arpa/inet.h>
#include <errno.h>
#include <signal.h>
#include <sys/list.h>
#include <sys/msg.h>
#include <sys/minmax.h>
#include <sys/types.h>
#include <sys/file.h>
#include <sys/threads.h>
#include <posix/utils.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

#include <posix/idtree.h>

#ifdef UMASS_MOUNT_EXT2
#include <libext2.h>
#endif

#include <usb.h>
#include <usbdriver.h>

#include "../pc-ata/mbr.h"

#define UMASS_N_MSG_THREADS  2
#define UMASS_N_POOL_THREADS 4

#define UMASS_SECTOR_SIZE 512

#define UMASS_WRITE 0
#define UMASS_READ  0x80

#define CBW_SIG 0x43425355
#define CSW_SIG 0x53425355

/* clang-format off */
#define LOG(str_, ...) do { printf("umass: " str_ "\n", ##__VA_ARGS__); } while (0)
#define LOG_ERROR(str_, ...) LOG(__FILE__ ":%d error: " str_, __LINE__, ##__VA_ARGS__)
#define TRACE(str_, ...) do { if (0) LOG(__FILE__ ":%d TRACE: " str_, __LINE__, ##__VA_ARGS__); } while (0)
/* clang-format on */


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dlen;
	uint8_t flags;
	uint8_t lun;
	uint8_t clen;
	uint8_t cmd[16];
} __attribute__((packed)) umass_cbw_t;


typedef struct {
	uint32_t sig;
	uint32_t tag;
	uint32_t dr;
	uint8_t status;
} __attribute__((packed)) umass_csw_t;


typedef struct {
	uint8_t opcode;
	uint8_t action : 5;
	uint8_t misc0 : 3;
	uint32_t lba;
	uint8_t misc1;
	uint16_t length;
	uint8_t control;
} __attribute__((packed)) scsi_cdb10_t;


/* Device access callbacks */
static ssize_t umass_read(id_t id, off_t offs, char *buf, size_t len);
static ssize_t umass_write(id_t id, off_t offs, const char *buf, size_t len);


/* Thread types */
static void umass_fsthr(void *arg);
static void umass_poolthr(void *arg);
static void umass_msgthr(void *arg);


/* Filesystem callbacks types */
typedef int (*fs_handler_t)(void *, msg_t *);
typedef int (*fs_unmount_t)(void *);
typedef int (*fs_mount_t)(oid_t *, unsigned int, typeof(umass_read) *, typeof(umass_write) *, void **);


typedef struct {
	rbnode_t node;      /* RBTree node */
	char name[16];      /* Filesystem name */
	uint8_t type;       /* Compatible partition type */
	fs_handler_t handler; /* Message handler callback */
	fs_unmount_t unmount; /* Unmount callback */
	fs_mount_t mount;     /* Mount callback */
} umass_fs_t;


typedef struct _umass_part_t {
	/* Partition data */
	unsigned int idx;  /* Partition index */
	unsigned int port; /* Partition port */
	uint32_t start;    /* Partition start */
	uint32_t sectors;  /* Number of sectors */

	/* Filesystem data */
	umass_fs_t *fs; /* Mounted filesystem */
	void *fdata;    /* Mounted filesystem data */

	/* Partition filesystem thread stack */
	char fsstack[4 * _PAGE_SIZE] __attribute__((aligned(8)));
} umass_part_t;


typedef struct _umass_req_t {
	msg_t msg;                        /* Request msg */
	msg_rid_t rid;                    /* Request receiving context */
	umass_part_t *part;               /* Request receiver partition */
	struct _umass_req_t *prev, *next; /* Doubly linked list */
} umass_req_t;


typedef struct umass_dev {
	idnode_t node; /* Device ID */

	char buffer[8 * UMASS_SECTOR_SIZE];
	usb_devinfo_t instance;
	char path[32];
	int pipeCtrl;
	int pipeIn;
	int pipeOut;
	int fileId;
	int tag;
	unsigned port;
	handle_t lock;

	umass_part_t part; /* TODO extend for more partitions */
} umass_dev_t;


static struct {
	idtree_t devices;
	umass_dev_t device;

	unsigned drvport;
	unsigned msgport;
	handle_t lock;
	rbtree_t fss; /* Registered filesystems */

	umass_req_t *rqueue;   /* Requests FIFO queue */
	handle_t rlock, rcond; /* Requests synchronization */

	bool mount_root;

	/* Message threads stacks */
	char mstacks[UMASS_N_MSG_THREADS][2 * _PAGE_SIZE] __attribute__((aligned(8)));

	/* Pool threads stacks */
	char pstacks[UMASS_N_POOL_THREADS][4 * _PAGE_SIZE] __attribute__((aligned(8)));
} umass_common;


static const usb_device_id_t filters[] = {
	/* USB Mass Storage class */
	{ USBDRV_ANY, USBDRV_ANY, USB_CLASS_MASS_STORAGE, USBDRV_ANY, USBDRV_ANY },
};


#ifdef UMASS_MOUNT_EXT2
static int umass_registerfs(const char *name, uint8_t type, fs_mount_t mount, fs_unmount_t unmount, fs_handler_t handler)
{
	umass_fs_t *fs;

	if ((fs = (umass_fs_t *)malloc(sizeof(umass_fs_t))) == NULL)
		return -ENOMEM;

	strcpy(fs->name, name);
	fs->type = type;
	fs->mount = mount;
	fs->unmount = unmount;
	fs->handler = handler;
	lib_rbInsert(&umass_common.fss, &fs->node);

	return EOK;
}
#endif


static int umass_cmpfs(rbnode_t *node1, rbnode_t *node2)
{
	umass_fs_t *fs1 = lib_treeof(umass_fs_t, node, node1);
	umass_fs_t *fs2 = lib_treeof(umass_fs_t, node, node2);

	return strcmp(fs1->name, fs2->name);
}


static int _umass_transmit(umass_dev_t *dev, void *cmd, size_t clen, char *data, size_t dlen, int dir)
{
	umass_cbw_t cbw = { 0 };
	umass_csw_t csw = { 0 };
	int ret = 0, bytes = 0;
	int dataPipe;

	if (clen > 16)
		return -1;

	dataPipe = (dir == usb_dir_out) ? dev->pipeOut : dev->pipeIn;
	cbw.sig = CBW_SIG;
	cbw.tag = dev->tag++;
	cbw.dlen = dlen;
	cbw.flags = (dir == usb_dir_out) ? UMASS_WRITE : UMASS_READ;
	cbw.lun = 0;
	cbw.clen = clen;
	memcpy(cbw.cmd, cmd, clen);

	if ((ret = usb_transferBulk(dev->pipeOut, &cbw, sizeof(cbw), usb_dir_out)) != sizeof(cbw)) {
		fprintf(stderr, "umass_transmit: usb_transferBulk OUT failed\n");
		return -EIO;
	}

	/* Optional data transfer */
	if (dlen > 0) {
		if ((ret = usb_transferBulk(dataPipe, data, dlen, dir)) < 0) {
			fprintf(stderr, "umass_transmit: umass_transmit data transfer failed\n");
			return ret;
		}
		bytes = ret;
	}

	if ((ret = usb_transferBulk(dev->pipeIn, &csw, sizeof(csw), usb_dir_in)) != sizeof(csw)) {
		fprintf(stderr, "umass_transmit: usb_transferBulk IN transfer failed\n");
		return -EIO;
	}

	/* Transfer finished, check transfer correctness */
	if (csw.sig != CSW_SIG || csw.tag != cbw.tag || csw.status != 0) {
		fprintf(stderr, "umass_transmit: transfer incorrect\n");
		return -EIO;
	}

	return bytes;
}


static int _umass_check(umass_dev_t *dev)
{
	scsi_cdb10_t readcmd = {
		.opcode = 0x28,
		.length = htons(0x1)
	};
	char testcmd[6] = { 0 };
	mbr_t *mbr;

	if (_umass_transmit(dev, testcmd, sizeof(testcmd), NULL, 0, usb_dir_in) < 0) {
		fprintf(stderr, "umass_transmit failed\n");
		return -1;
	}

	/* Read MBR */
	if (_umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, UMASS_SECTOR_SIZE, usb_dir_in) < 0) {
		fprintf(stderr, "umass_transmit 2 failed\n");
		return -1;
	}

	mbr = (mbr_t *)dev->buffer;
	if (mbr->magic != MBR_MAGIC)
		return -1;

	/* Read only the first partition */
	/* TODO read the rest of them*/
	dev->part.start = mbr->pent[0].start;
	dev->part.sectors = mbr->pent[0].sectors;
	dev->part.fs = NULL;
	dev->part.fdata = NULL;
	dev->part.idx = 0;

	return 0;
}


static umass_fs_t *umass_getfs(const char *name)
{
	umass_fs_t fs;

	strcpy(fs.name, name);

	return lib_treeof(umass_fs_t, node, lib_rbFind(&umass_common.fss, &fs.node));
}


/* TODO: handle mounting other filesystems than ext2 */
#if 0
static umass_fs_t *umass_findfs(uint8_t type)
{
	rbnode_t *node;
	umass_fs_t *fs;

	for (node = lib_rbMinimum(umass_common.fss.root); node != NULL; node = lib_rbNext(node)) {
		fs = lib_treeof(umass_fs_t, node, node);

		if (fs->type == type)
			return fs;
	}

	return NULL;
}
#endif


static int umass_readFromDev(umass_dev_t *dev, off_t offs, char *buf, size_t len)
{
	scsi_cdb10_t readcmd = { .opcode = 0x28 };
	int ret;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE)) {
		return -EINVAL;
	}

	if (offs + len > dev->part.sectors * UMASS_SECTOR_SIZE) {
		return -EINVAL;
	}

	len = min(len, sizeof(dev->buffer));

	readcmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->part.start);
	readcmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	mutexLock(dev->lock);
	ret = _umass_transmit(dev, &readcmd, sizeof(readcmd), dev->buffer, len, usb_dir_in);
	if (ret > 0)
		memcpy(buf, dev->buffer, len);
	else if (len > 0) {
		printf("read transmit failed for offs: %lld\n", offs);
	}
	mutexUnlock(dev->lock);

	return ret;
}


static int umass_writeToDev(umass_dev_t *dev, off_t offs, const char *buf, size_t len)
{
	scsi_cdb10_t writecmd = { .opcode = 0x2a };
	int ret;

	if ((offs % UMASS_SECTOR_SIZE) || (len % UMASS_SECTOR_SIZE))
		return -EINVAL;

	if (offs + len > dev->part.sectors * UMASS_SECTOR_SIZE)
		return -EINVAL;

	len = min(len, sizeof(dev->buffer));

	writecmd.lba = htonl(offs / UMASS_SECTOR_SIZE + dev->part.start);
	writecmd.length = htons((uint16_t)(len / UMASS_SECTOR_SIZE));

	mutexLock(dev->lock);
	memcpy(dev->buffer, buf, len);
	ret = _umass_transmit(dev, &writecmd, sizeof(writecmd), dev->buffer, len, usb_dir_out);
	mutexUnlock(dev->lock);
	if (ret < 0) {
		fprintf(stderr, "write transmit failed for offs: %lld\n", offs);
	}

	return ret;
}


static int umass_getattr(umass_dev_t *dev, int type, long long int *attr)
{
	if (type != atSize)
		return -EINVAL;

	*attr = dev->part.sectors * UMASS_SECTOR_SIZE;

	return EOK;
}


static umass_dev_t *_umass_devFind(id_t id)
{
	return lib_treeof(umass_dev_t, node, idtree_find(&umass_common.devices, id));
}


static ssize_t umass_read(id_t id, off_t offs, char *buf, size_t len)
{
	mutexLock(umass_common.lock);
	umass_dev_t *dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);
	if (dev == NULL) {
		return -ENODEV;
	}
	return umass_readFromDev(dev, offs, buf, len);
}


static ssize_t umass_write(id_t id, off_t offs, const char *buf, size_t len)
{
	mutexLock(umass_common.lock);
	umass_dev_t *dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);
	if (dev == NULL) {
		return -ENODEV;
	}
	return umass_writeToDev(dev, offs, buf, len);
}


static int umass_mountFromDev(umass_dev_t *dev, const char *name, oid_t *oid)
{
	umass_fs_t *fs;
	int err;

	if (dev == NULL) {
		return -ENODEV;
	}

	if (dev->part.fs != NULL)
		return -EEXIST;

	if ((dev->part.fs = fs = umass_getfs(name)) == NULL)
		return -ENOENT;

	if (portCreate(&dev->part.port) != 0) {
		fprintf(stderr, "umass: Can't create partition port!\n");
		return 1;
	}

	oid->port = dev->part.port;
	oid->id = dev->fileId;

	if ((err = fs->mount(oid, UMASS_SECTOR_SIZE, umass_read, umass_write, &dev->part.fdata)) < 0)
		return err;
	oid->id = err;

	if ((err = beginthread(umass_fsthr, 4, dev->part.fsstack, sizeof(dev->part.fsstack), &dev->part)) < 0) {
		dev->part.fs->unmount(dev->part.fdata);
		dev->part.fs = NULL;
		dev->part.fdata = NULL;
		return err;
	}

	return EOK;
}


static int umass_mount(id_t id, const char *name, oid_t *oid)
{
	umass_dev_t *dev;

	mutexLock(umass_common.lock);
	dev = _umass_devFind(id);
	mutexUnlock(umass_common.lock);

	return umass_mountFromDev(dev, name, oid);
}


static void umass_fsthr(void *arg)
{
	umass_part_t *part = (umass_part_t *)arg;
	umass_req_t *req;
	int umount = 0;

	for (;;) {
		if ((req = (umass_req_t *)malloc(sizeof(umass_req_t))) == NULL)
			continue;

		req->part = part;
		while (msgRecv(req->part->port, &req->msg, &req->rid) < 0)
			;

		if (req->msg.type == mtUmount)
			umount = 1;

		mutexLock(umass_common.rlock);

		LIST_ADD(&umass_common.rqueue, req);

		mutexUnlock(umass_common.rlock);
		condSignal(umass_common.rcond);

		if (umount)
			endthread();
	}
}


static void umass_poolthr(void *arg)
{
	umass_req_t *req;

	for (;;) {
		mutexLock(umass_common.rlock);

		while (umass_common.rqueue == NULL)
			condWait(umass_common.rcond, umass_common.rlock, 0);
		req = umass_common.rqueue->prev;
		LIST_REMOVE(&umass_common.rqueue, req);

		mutexUnlock(umass_common.rlock);

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


static void umass_msgthr(void *arg)
{
	umass_dev_t *dev;
	msg_rid_t rid;
	msg_t msg;
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	for (;;) {
		if (msgRecv(umass_common.msgport, &msg, &rid) < 0)
			break;

		/* Ignore this msg, as it might have been sent by us after deletion event */
		if (msg.type == mtUnlink) {
			msg.o.err = EOK;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		mutexLock(umass_common.lock);
		dev = _umass_devFind(msg.oid.id);
		mutexUnlock(umass_common.lock);

		if (dev == NULL) {
			msg.o.err = -ENOENT;
			msgRespond(umass_common.msgport, &msg, rid);
			continue;
		}

		switch (msg.type) {
			case mtOpen:
			case mtClose:
				msg.o.err = EOK;
				break;

			case mtMount:
				imnt = (mount_i_msg_t *)msg.i.raw;
				omnt = (mount_o_msg_t *)msg.o.raw;
				msg.o.err = umass_mount(msg.oid.id, imnt->fstype, &omnt->oid);
				break;

			case mtRead:
				msg.o.err = umass_readFromDev(dev, msg.i.io.offs, msg.o.data, msg.o.size);
				break;

			case mtWrite:
				msg.o.err = umass_writeToDev(dev, msg.i.io.offs, msg.i.data, msg.i.size);
				break;

			case mtGetAttr:
				msg.o.err = umass_getattr(dev, msg.i.attr.type, &msg.o.attr.val);
				break;

			default:
				msg.o.err = -ENOSYS;
				break;
		}

		msgRespond(umass_common.msgport, &msg, rid);
	}

	endthread();
}


static umass_dev_t *umass_devAlloc(void)
{
	umass_dev_t *dev;
	int rv;

	dev = malloc(sizeof(umass_dev_t));
	if (dev == NULL) {
		fprintf(stderr, "umass: Not enough memory\n");
		return NULL;
	}

	rv = mutexCreate(&dev->lock);
	if (rv < 0) {
		free(dev);
		return NULL;
	}

	idtree_alloc(&umass_common.devices, &dev->node);
	dev->fileId = idtree_id(&dev->node);

	rv = snprintf(dev->path, sizeof(dev->path), "/dev/umass%d", dev->fileId);
	if (rv < 0) {
		resourceDestroy(dev->lock);
		free(dev);
		return NULL;
	}

	return dev;
}


static int umass_mountRoot(umass_dev_t *dev)
{
#ifdef UMASS_MOUNT_EXT2
	int err;
	oid_t roid;

	err = umass_mountFromDev(dev, LIBEXT2_NAME, &roid);
	if (err < 0) {
		return err;
	}

	err = portRegister(roid.port, "/", &roid);
	if (err < 0) {
		return err;
	}

	umass_common.mount_root = false; /* don't try to mount root again */

	return EOK;
#else
	return -ENOSYS;
#endif
}


static int _umass_handleInsertion(usb_devinfo_t *insertion)
{
	int err;
	umass_dev_t *dev;
	oid_t oid;

	if ((dev = umass_devAlloc()) == NULL) {
		fprintf(stderr, "umass: devAlloc failed\n");
		return -ENOMEM;
	}

	dev->instance = *insertion;
	if ((dev->pipeCtrl = usb_open(insertion, usb_transfer_control, 0)) < 0) {
		free(dev);
		fprintf(stderr, "umass: usb_open failed\n");
		return -EINVAL;
	}

	if (usb_setConfiguration(dev->pipeCtrl, 1) != 0) {
		free(dev);
		fprintf(stderr, "umass: setConfiguration failed\n");
		return -EINVAL;
	}

	if ((dev->pipeIn = usb_open(insertion, usb_transfer_bulk, usb_dir_in)) < 0) {
		fprintf(stderr, "umass: pipe open failed \n");
		free(dev);
		return -EINVAL;
	}

	if ((dev->pipeOut = usb_open(insertion, usb_transfer_bulk, usb_dir_out)) < 0) {
		fprintf(stderr, "umass: pipe open failed\n");
		free(dev);
		return -EINVAL;
	}
	dev->tag = 0;

	if (_umass_check(dev)) {
		fprintf(stderr, "umass: umass_check failed\n");
		free(dev);
		return -EINVAL;
	}

	oid.port = umass_common.msgport;
	oid.id = dev->fileId;
	err = create_dev(&oid, dev->path);
	if (err != 0) {
		free(dev);
		fprintf(stderr, "usb: Can't create dev!\n");
		return -EINVAL;
	}

	printf("umass: New USB Mass Storage device: %s sectors: %d\n", dev->path, dev->part.sectors);

	return 0;
}


static int _umass_handleDeletion(usb_deletion_t *del)
{
	umass_dev_t *dev;
	rbnode_t *node, *next;

	node = lib_rbMinimum(umass_common.devices.root);
	while (node != NULL) {
		next = lib_rbNext(node);

		dev = lib_treeof(umass_dev_t, node, lib_treeof(idnode_t, linkage, node));
		if (dev->instance.bus == del->bus && dev->instance.dev == del->dev &&
				dev->instance.interface == del->interface) {
			resourceDestroy(dev->lock);
			remove(dev->path);
			fprintf(stderr, "umass: Device removed: %s\n", dev->path);
			free(dev);
		}

		node = next;
	}

	return 0;
}


static void umass_signalExit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char *argv[])
{
	rbnode_t *node;
	umass_dev_t *dev;
	int ret, i;
	pid_t pid;
	msg_t msg;
	usb_msg_t *umsg = (usb_msg_t *)msg.i.raw;
	char c;
	bool root_mount_pending = false;

	umass_common.mount_root = false;

	/* Wait for console */
	while (write(1, "", 0) < 0)
		usleep(50000);

	if (argc > 1) {
		/* Process command line options */
		while ((c = getopt(argc, argv, "r")) != -1) {
			switch (c) {
				case 'r':
					umass_common.mount_root = true;
					break;
				default:
					return EOK;
			}
		}
	}

	/* Set parent exit handler */
	signal(SIGUSR1, umass_signalExit);

	/* Daemonize server */
	if ((pid = fork()) < 0) {
		fprintf(stderr, "umass: failed to daemonize server\n");
		exit(EXIT_FAILURE);
	}
	else if (pid > 0) {
		/* Parent waits to be killed by the child after finished server initialization */
		sleep(10);
		fprintf(stderr, "umass: failed to successfully initialize server\n");
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, umass_signalExit);

	if (setsid() < 0) {
		fprintf(stderr, "umass: failed to create new session\n");
		exit(EXIT_FAILURE);
	}

	if ((ret = mutexCreate(&umass_common.rlock)) < 0) {
		fprintf(stderr, "umass: failed to create server requests mutex\n");
		return ret;
	}

	if ((ret = condCreate(&umass_common.rcond)) < 0) {
		fprintf(stderr, "umass: failed to create server requests condition variable\n");
		return ret;
	}

	umass_common.rqueue = NULL;
	idtree_init(&umass_common.devices);
	lib_rbInit(&umass_common.fss, umass_cmpfs, NULL);

#ifdef UMASS_MOUNT_EXT2
	/* Register filesystems */
	if (umass_registerfs(LIBEXT2_NAME, LIBEXT2_TYPE, LIBEXT2_MOUNT, LIBEXT2_UNMOUNT, LIBEXT2_HANDLER) < 0)
		fprintf(stderr, "umass: failed to register ext2 filesystem\n");
#endif

	/* Port for communication with the USB stack */
	if (portCreate(&umass_common.drvport) != 0) {
		fprintf(stderr, "umass: Can't create driver port!\n");
		return 1;
	}

	/* Port for communication with driver clients */
	if (portCreate(&umass_common.msgport) != 0) {
		fprintf(stderr, "umass: Can't create message port!\n");
		return 1;
	}

	if (mutexCreate(&umass_common.lock) < 0) {
		fprintf(stderr, "umass: Can't create mutex!\n");
		return 1;
	}

	/* Run message threads */
	for (i = 0; i < sizeof(umass_common.mstacks) / sizeof(umass_common.mstacks[0]); i++) {
		if ((ret = beginthread(umass_msgthr, 4, umass_common.mstacks[i], sizeof(umass_common.mstacks[i]), NULL)) != 0) {
			fprintf(stderr, "umass: fail to beginthread ret: %d\n", ret);
			return 1;
		}
	}

	/* Run pool threads */
	for (i = 0; i < sizeof(umass_common.pstacks) / sizeof(umass_common.pstacks[0]); i++) {
		if ((ret = beginthread(umass_poolthr, 4, umass_common.pstacks[i], sizeof(umass_common.pstacks[i]), NULL)) < 0) {
			fprintf(stderr, "umass: failed to start pool thread %d\n", i);
			return ret;
		}
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);

	if ((usb_connect(filters, sizeof(filters) / sizeof(filters[0]), umass_common.drvport)) < 0) {
		fprintf(stderr, "umass: Fail to connect to usb host!\n");
		return 1;
	}

	for (;;) {
		ret = usb_eventsWait(umass_common.drvport, &msg);
		if (ret != 0)
			return 1;
		mutexLock(umass_common.lock);
		switch (umsg->type) {
			case usb_msg_insertion:
				if (_umass_handleInsertion(&umsg->insertion) == 0) {
					if (umass_common.mount_root) {
						root_mount_pending = true;
						node = lib_rbMinimum(umass_common.devices.root);
						dev = lib_treeof(umass_dev_t, node, lib_treeof(idnode_t, linkage, node));
					}
				}
				else {
					fprintf(stderr, "umass: Failed to initialize device!\n");
				}
				break;
			case usb_msg_deletion:
				_umass_handleDeletion(&umsg->deletion);
				break;
			default:
				fprintf(stderr, "umass: Error when receiving event from host\n");
				break;
		}
		mutexUnlock(umass_common.lock);

		if (root_mount_pending) {
			ret = umass_mountRoot(dev);
			if (ret < 0) {
				fprintf(stderr, "umass: failed to mount root partition\n");
			}
			root_mount_pending = false;
		}
	}

	return 0;
}
