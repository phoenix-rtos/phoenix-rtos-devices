/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <posix/utils.h>
#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libvirtio.h>


/* VirtIO block device request types */
enum {
	REQ_READ       = 0x00,   /* Read request */
	REQ_WRITE      = 0x01,   /* Write request */
};


typedef struct {
	/* Header (device read-only) */
	uint32_t type;           /* Request type */
	uint32_t reserved;       /* Reserved field (previously request priority) */
	uint64_t sector;         /* Starting sector (512-byte offset) */

	/* Footer (device write-only) */
	volatile uint8_t status; /* Returned status */

	/* VirtIO request descriptors */
	virtio_seg_t header;     /* Header segment */
	virtio_seg_t data;       /* Data segment */
	virtio_seg_t footer;     /* Footer segment */
	virtio_req_t vreq;       /* VirtIO request */

	/* Custom helper fields */
	unsigned int len;        /* Number of bytes written to request buffers */
	handle_t lock;           /* Request mutex */
	handle_t cond;           /* Request condition variable */
} virtioblk_req_t;


typedef struct {
	/* Device data */
	virtio_dev_t vdev;         /* VirtIO device */
	virtqueue_t vq;            /* Device virtqueue */
	unsigned int sectorsz;     /* Device sector size */
	unsigned int size;         /* Device storage size */

	/* Interrupt handling */
	volatile unsigned int isr; /* Interrupt status */
	handle_t lock;             /* Interrupt mutex */
	handle_t cond;             /* Interrupt condition variable */
	handle_t inth;             /* Interrupt handle */
	char istack[2048] __attribute__((aligned(8)));

	/* Block device data */
	unsigned int port;         /* Device port */
	char pstack[4][2048] __attribute__((aligned(8)));
} virtioblk_dev_t;


/* VirtIO block devices descriptors */
static const virtio_devinfo_t info[] = {
	{ .type = vdevPCI, .id = 0x1001 },
	{ .type = vdevPCI, .id = 0x1042 },
#ifdef TARGET_RISCV64
	/* Direct VirtIO MMIO QEMU block device descriptors */
	{ .type = vdevMMIO, .id = 0x02, .irq = 8, .base = { (void *)0x10008000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 7, .base = { (void *)0x10007000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 6, .base = { (void *)0x10006000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 5, .base = { (void *)0x10005000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 4, .base = { (void *)0x10004000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 3, .base = { (void *)0x10003000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 2, .base = { (void *)0x10002000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x02, .irq = 1, .base = { (void *)0x10001000, 0x1000 } },
#endif
	{ .type = vdevNONE }
};


/* Interrupt handler */
static int virtioblk_int(unsigned int n, void *arg)
{
	virtioblk_dev_t *bdev = (virtioblk_dev_t *)arg;
	virtio_dev_t *vdev = &bdev->vdev;

	//virtqueue_disableIRQ(vdev, &bdev->vq);
	//bdev->isr = virtio_isr(vdev);
	*(volatile uint32_t *)((uintptr_t)vdev->info.base.addr + 0x64) = *(volatile uint32_t *)((uintptr_t)vdev->info.base.addr + 0x60);

	return bdev->cond;
}


/* Interrupt thread */
static void virtioblk_intthr(void *arg)
{
	virtioblk_dev_t *bdev = (virtioblk_dev_t *)arg;
	virtio_dev_t *vdev = &bdev->vdev;
	virtioblk_req_t *req;
	unsigned int len, isr;

	mutexLock(bdev->lock);
	//bdev->isr = 0;

	for (;;) {
		//while (!bdev->isr)
			condWait(bdev->cond, bdev->lock, 0);
		//isr = bdev->isr;
		//bdev->isr = 0;
		fprintf(stderr, "Got interrupt!\n");

		//virtio_isr(vdev);
		//isr = *(volatile uint32_t *)((uintptr_t)vdev->info.base.addr + 0x60);
		//*(volatile uint32_t *)((uintptr_t)vdev->info.base.addr + 0x64) = isr;

		// if (!(isr & 0x1))
		// 	continue;
		//if ((isr = virtio_read32(vdev, vdev->info.base.addr, 0x60)))
		
		//virtio_write32(vdev, vdev->info.base.addr, 0x64, isr);
		//while (!(virtio_isr(vdev) & 0x01))
		//while (!(bdev->isr & 0x01))
			//condWait(bdev->cond, bdev->lock, 0);

		//virtqueue_disableIRQ(vdev, &bdev->vq);
		//bdev->isr = 0;
		while ((req = (virtioblk_req_t *)virtqueue_dequeue(vdev, &bdev->vq, &len)) != NULL) {
			fprintf(stderr, "Got request!\n");
			req->len = len;
			condSignal(req->cond);
		}
		//virtqueue_enableIRQ(vdev, &bdev->vq);

		/* Handle requests that came after the last proceseed request and before enabling virtqueue interrupts */
		while ((req = (virtioblk_req_t *)virtqueue_dequeue(vdev, &bdev->vq, &len)) != NULL) {
			req->len = len;
			condSignal(req->cond);
		}
	}
}


/* Opens request context */
static int virtioblk_open(void *rctx)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;
	int err;

	if ((err = mutexCreate(&req->lock)) < 0)
		return err;

	if ((err = condCreate(&req->cond)) < 0) {
		resourceDestroy(req->lock);
		return err;
	}

	req->header.buff = &req->type;
	req->header.len = 0x10;
	req->header.prev = &req->footer;
	req->header.next = &req->data;
	req->data.prev = &req->header;
	req->data.next = &req->footer;
	req->footer.buff = (void *)&req->status;
	req->footer.len = 0x01;
	req->footer.prev = &req->data;
	req->footer.next = &req->header;
	req->vreq.segs = &req->header;

	return EOK;
}


/* Closes request context */
static void virtioblk_close(void *rctx)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;

	resourceDestroy(req->lock);
	resourceDestroy(req->cond);
}


/* Sends request to device */
static ssize_t virtioblk_sendRequest(virtioblk_dev_t *bdev, virtioblk_req_t *req)
{
	virtio_dev_t *vdev = &bdev->vdev;
	ssize_t ret;
	int err;

	mutexLock(req->lock);

	//req->status = 0xff;
	if ((err = virtqueue_enqueue(vdev, &bdev->vq, &req->vreq)) < 0) {
		fprintf(stderr, "Enqueue error!\n");
		mutexUnlock(req->lock);
		return err;
	}
	fprintf(stderr, "Before notify!\n");
	virtqueue_notify(vdev, &bdev->vq);
	fprintf(stderr, "After notify!\n");

	//while (req->status == 0xff)
		sleep(1);
		//condWait(req->cond, req->lock, 0);
	
	ret = (req->status) ? -EIO : req->len - 1;
	printf("Ret: %d\n", ret);
	mutexUnlock(req->lock);

	return ret;
}


/* Reads data from device */
static ssize_t virtioblk_read(virtioblk_dev_t *bdev, void *rctx, offs_t offs, char *buff, size_t len)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;

	if (offs + len > bdev->size) {
		if (offs > bdev->size)
			return -EINVAL;
		len = bdev->size - offs;
	}

	req->type = REQ_READ;
	req->sector = offs / bdev->sectorsz;
	req->data.buff = buff;
	req->data.len = len;
	req->vreq.rsegs = 1;
	req->vreq.wsegs = 2;

	return virtioblk_sendRequest(bdev, req);
}


/* Writes data to device */
static ssize_t virtioblk_write(virtioblk_dev_t *bdev, void *rctx, offs_t offs, const char *buff, size_t len)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;

	if (offs + len > bdev->size) {
		if (offs > bdev->size)
			return -EINVAL;
		len = bdev->size - offs;
	}

	req->type = REQ_WRITE;
	req->sector = offs / bdev->sectorsz;
	req->data.buff = (void *)buff;
	req->data.len = len;
	req->vreq.rsegs = 2;
	req->vreq.wsegs = 1;

	if (virtioblk_sendRequest(bdev, req))
		return -EIO;

	return len;
}


static int virtioblk_getattr(virtioblk_dev_t *bdev, int type, int *attr)
{
	printf("Attr type: %d\n", type);
	switch (type) {
	case atSize:
		printf("Disk size: %u\n", bdev->size);
		*attr = bdev->size;
		break;

	//default:
		//*attr = -EINVAL;
	}

	return EOK;
}


/* Destroys device */
static void virtioblk_destroyDev(virtioblk_dev_t *bdev)
{
	virtio_dev_t *vdev = &bdev->vdev;

	resourceDestroy(bdev->lock);
	resourceDestroy(bdev->cond);
	resourceDestroy(bdev->inth);
	virtqueue_destroy(vdev, &bdev->vq);
	virtio_destroyDev(vdev);
}


/* Initializes device */
static int virtioblk_initDev(virtioblk_dev_t *bdev)
{
	virtio_dev_t *vdev = &bdev->vdev;
	int err;

	if ((err = virtio_initDev(vdev)) < 0)
		return err;

	do {
		if ((err = virtio_writeFeatures(vdev, 0)) < 0)
			break;

		if ((err = virtqueue_init(vdev, &bdev->vq, 0, 128)) < 0)
			break;

		bdev->sectorsz = 512;
		bdev->size = bdev->sectorsz * virtio_readConfig64(vdev, 0x00);

		if ((err = mutexCreate(&bdev->lock)) < 0) {
			virtqueue_destroy(vdev, &bdev->vq);
			break;
		}

		if ((err = condCreate(&bdev->cond)) < 0) {
			resourceDestroy(bdev->lock);
			virtqueue_destroy(vdev, &bdev->vq);
			break;
		}

		if ((err = beginthread(virtioblk_intthr, 4, bdev->istack, sizeof(bdev->istack), bdev)) < 0) {
			resourceDestroy(bdev->cond);
			resourceDestroy(bdev->lock);
			virtqueue_destroy(vdev, &bdev->vq);
			break;
		}

		if ((err = interrupt(vdev->info.irq, virtioblk_int, bdev, bdev->cond, &bdev->inth)) < 0) {
			resourceDestroy(bdev->cond);
			resourceDestroy(bdev->lock);
			virtqueue_destroy(vdev, &bdev->vq);
			break;
		}

		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 2));

		return EOK;
	} while (0);

	virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
	virtio_destroyDev(vdev);

	return err;
}


/* Pool thread */
static void virtioblk_poolthr(void *arg)
{
	virtioblk_dev_t *bdev = (virtioblk_dev_t *)arg;
	unsigned char rctx[sizeof(virtioblk_req_t)];
	unsigned long rid;
	msg_t msg;

	if (virtioblk_open(rctx) < 0)
		return;

	for (;;) {
		if (msgRecv(bdev->port, &msg, &rid) < 0)
			continue;

		fprintf(stderr, "Msg type: %d\n", msg.type);
		switch (msg.type) {
		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		case mtRead:
			msg.o.io.err = virtioblk_read(bdev, rctx, msg.i.io.offs, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			msg.o.io.err = virtioblk_write(bdev, rctx, msg.i.io.offs, msg.i.data, msg.i.size);
			break;

		case mtGetAttr:
			virtioblk_getattr(bdev, msg.i.attr.type, &msg.o.attr.val);
			break;

		default:
			msg.o.io.err = -EINVAL;
		}
		fprintf(stderr, "Respond\n");
		msgRespond(bdev->port, &msg, rid);
	}

	virtioblk_close(rctx);
}


/* Registers device */
static int virtioblk_registerDev(const char *path, virtioblk_dev_t *bdev)
{
	oid_t oid;
	int i, err;

	if ((err = portCreate(&bdev->port)) < 0)
		return err;

	do {
		for (i = 0; i < sizeof(bdev->pstack) / sizeof(bdev->pstack[0]); i++) {
			if ((err = beginthread(virtioblk_poolthr, 4, bdev->pstack[i], sizeof(bdev->pstack[i]), bdev)) < 0)
				break;
		}

		if (err)
			break;

		/* TODO: detect and register MBR partitions */
		oid.port = bdev->port;
		oid.id = 0;
		if ((err = create_dev(&oid, path)) < 0)
			break;

		return EOK;
	} while (0);

	portDestroy(bdev->port);

	return err;
}


int main(void)
{
	virtioblk_dev_t *bdev;
	virtio_dev_t vdev;
	virtio_ctx_t vctx;
	char path[16];
	oid_t oid;
	int i, err, nbdevs = 0;

	/* Wait for console */
	while (write(STDOUT_FILENO, "", 0) < 0)
		usleep(10000);

	/* Wait for root filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	virtio_init();

	/* Detect and initialize VirtIO block devices */
	for (i = 0; info[i].type != vdevNONE; i++) {
		vctx.reset = 1;
		while ((err = virtio_find(&info[i], &vdev, &vctx)) != -ENODEV) {
			if (err < 0) {
				fprintf(stderr, "virtio-blk: failed to process VirtIO %s block device ", (info[i].type == vdevPCI) ? "PCI" : "MMIO");
				if (info[i].base.len)
					fprintf(stderr, "direct descriptor, base: %#x. ", info[i].base.addr);
				else
					fprintf(stderr, "descriptor, ID: %#x. ", info[i].id);
				fprintf(stderr, "Skipping...\n");
				continue;
			}

			if ((bdev = malloc(sizeof(virtioblk_dev_t))) == NULL) {
				fprintf(stderr, "virtio-blk: out of memory\n");
				return -ENOMEM;
			}
			bdev->vdev = vdev;

			if ((err = virtioblk_initDev(bdev)) < 0) {
				if (err != -ENODEV)
					fprintf(stderr, "virtio-blk: failed to init VirtIO block device, base: %#x. Skipping...\n", vdev.info.base.addr);
				free(bdev);
				continue;
			}
			// sleep(1);
			// unsigned char rctx[sizeof(virtioblk_req_t)];
			// char buff[512];
			// virtioblk_open(rctx);
			// //printf("TEST\n");
			// ssize_t ret = virtioblk_read(bdev, rctx, 0, buff, 512);
			// fprintf(stderr, "ret: %d\n", ret);
			// for (int i = 0; i < sizeof(buff); i += 16) {
			// 	for (int j = 0; j < 8; j++)
			// 		printf("%02x%02x ", buff[i + 2 * j + 1],  buff[i + 2 * j]);
			// 	printf("\n");
			// }
			// virtioblk_close(rctx);

			sprintf(path, "/dev/vblk%c", 'a' + nbdevs);
			if ((err = virtioblk_registerDev(path, bdev)) < 0) {
				fprintf(stderr, "virtio-blk: failed to register VirtIO block device, base: %#x. Skipping...\n", vdev.info.base.addr);
				virtioblk_destroyDev(bdev);
				free(bdev);
				continue;
			}
			nbdevs++;
		}
	}

	for (;;)
		sleep(10);

	virtio_done();

	return EOK;
}
