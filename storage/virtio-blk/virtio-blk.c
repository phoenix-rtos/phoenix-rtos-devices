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
#include <string.h>
#include <unistd.h>

#include <posix/utils.h>

#include <sys/file.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libvirtio.h>


/* Use polling instead of interrupts on RISCV64 (interrupt handler code triggers memory protection exception) */
#ifdef TARGET_RISCV64
#define USE_POLLING
#endif


typedef struct {
	/* Device data */
	virtio_dev_t vdev;           /* VirtIO device */
	virtqueue_t vq;              /* Device virtqueue */
	unsigned int sectorsz;       /* Device sector size */
	unsigned int size;           /* Device storage size */

	/* Interrupt handling */
	volatile unsigned int isr;   /* Interrupt status */
	handle_t lock;               /* Interrupt mutex */
	handle_t cond;               /* Interrupt condition variable */
	handle_t inth;               /* Interrupt handle */
	char istack[2048] __attribute__((aligned(8)));

	/* Block device data */
	unsigned int port;           /* Device port */
	char pstack[4][2048] __attribute__((aligned(8)));
} virtioblk_dev_t;


typedef struct {
	/* Request buffers (accessible by device) */
	struct {
		/* Request header (device read-only) */
		uint32_t type;           /* Request type */
		uint32_t reserved;       /* Reserved field (previously request priority) */
		uint64_t sector;         /* Starting sector (512-byte offset) */

		/* Footer (device write-only) */
		volatile uint8_t status; /* Returned status */
	} __attribute__((packed));

	/* VirtIO request segments */
	virtio_seg_t hdr;            /* Header segment */
	virtio_seg_t data;           /* Data segment */
	virtio_seg_t ftr;            /* Footer segment */
	virtio_req_t vreq;           /* VirtIO request */

	/* Custom helper fields */
	volatile unsigned int len;   /* Number of bytes written to request buffers */
	size_t buffsz;               /* Size of physciallly contiguous data buffer */
	void *buff;                  /* Physcially contiguous data buffer */
	handle_t lock;               /* Request mutex */
	handle_t cond;               /* Request condition variable */
} virtioblk_req_t;


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


#ifndef USE_POLLING
/* Interrupt handler */
static int virtioblk_int(unsigned int n, void *arg)
{
	virtioblk_dev_t *vblk = (virtioblk_dev_t *)arg;
	virtio_dev_t *vdev = &vblk->vdev;

	virtqueue_disableIRQ(vdev, &vblk->vq);
	vblk->isr = virtio_isr(vdev);

	return vblk->cond;
}
#endif


/* Interrupt thread */
static void virtioblk_intthr(void *arg)
{
	virtioblk_dev_t *vblk = (virtioblk_dev_t *)arg;
	virtio_dev_t *vdev = &vblk->vdev;
	virtioblk_req_t *req;
	unsigned int len;

	mutexLock(vblk->lock);
	vblk->isr = 0;

	for (;;) {
		while (!(vblk->isr & (1 << 0)))
			condWait(vblk->cond, vblk->lock, 0);
		vblk->isr = 0;

#ifdef USE_POLLING
		/* Poll for processed request (in polling mode requests are submitted and processed synchronously) */
		while ((req = virtqueue_dequeue(vdev, &vblk->vq, &len)) == NULL);

		mutexLock(req->lock);
		req->len = len;
		condSignal(req->cond);
		mutexUnlock(req->lock);
#else
		while ((req = virtqueue_dequeue(vdev, &vblk->vq, &len)) != NULL) {
			mutexLock(req->lock);
			req->len = len;
			condSignal(req->cond);
			mutexUnlock(req->lock);
		}
		virtqueue_enableIRQ(vdev, &vblk->vq);

		/* Get requests that might have come after last virtqueue_dequeue() and before virtqueue_enableIRQ() */
		while ((req = virtqueue_dequeue(vdev, &vblk->vq, &len)) != NULL) {
			mutexLock(req->lock);
			req->len = len;
			condSignal(req->cond);
			mutexUnlock(req->lock);
		}
#endif
	}

	endthread();
}


/* Sends request to device */
static int _virtioblk_send(virtioblk_dev_t *vblk, virtioblk_req_t *req)
{
	virtio_dev_t *vdev = &vblk->vdev;
	int err;

#ifdef USE_POLLING
	mutexLock(vblk->lock);
#endif
	req->len = 0;
	if ((err = virtqueue_enqueue(vdev, &vblk->vq, &req->vreq)) < 0)
		return err;
	virtqueue_notify(vdev, &vblk->vq);
#ifdef USE_POLLING
	vblk->isr |= (1 << 0);
	condSignal(vblk->cond);
	mutexUnlock(vblk->lock);
#endif

	while (!req->len)
		condWait(req->cond, req->lock, 0);

	if (req->status)
		return -EFAULT;

	return EOK;
}


/* Resizes physcially contiguous request data buffer */
static int _virtioblk_resizeBuff(virtioblk_req_t *req, size_t len)
{
	size_t buffsz;
	void *buff;

	if (len > req->buffsz) {
		buffsz = (len + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1);
		if ((buff = mmap(NULL, buffsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_CONTIGUOUS, 0)) == MAP_FAILED)
			return -ENOMEM;

		munmap(req->buff, req->buffsz);
		req->buffsz = buffsz;
		req->buff = buff;
	}

	return EOK;
}


/* Opens request context */
static void *virtioblk_open(void)
{
	virtioblk_req_t *req;

	if ((req = mmap(NULL, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1), PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_CONTIGUOUS, 0)) == MAP_FAILED)
		return NULL;

	req->buffsz = _PAGE_SIZE;
	if ((req->buff = mmap(NULL, req->buffsz, PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_CONTIGUOUS, 0)) == MAP_FAILED) {
		munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	if (mutexCreate(&req->lock) < 0) {
		munmap(req->buff, req->buffsz);
		munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	if (condCreate(&req->cond) < 0) {
		resourceDestroy(req->lock);
		munmap(req->buff, req->buffsz);
		munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	req->hdr.buff = &req->type;
	req->hdr.len = 0x10;
	req->hdr.prev = &req->ftr;
	req->hdr.next = &req->data;
	req->data.prev = &req->hdr;
	req->data.next = &req->ftr;
	req->ftr.buff = (void *)&req->status;
	req->ftr.len = 0x01;
	req->ftr.prev = &req->data;
	req->ftr.next = &req->hdr;
	req->vreq.segs = &req->hdr;

	return req;
}


/* Closes request context */
static void virtioblk_close(void *rctx)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;

	resourceDestroy(req->cond);
	resourceDestroy(req->lock);
	munmap(req->buff, req->buffsz);
	munmap(req, (sizeof(virtioblk_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
}


/* Reads data from device */
static ssize_t virtioblk_read(virtioblk_dev_t *vblk, void *rctx, offs_t offs, char *buff, size_t len)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;
	ssize_t ret;

	if (offs + len > vblk->size) {
		if (offs > vblk->size)
			return -EINVAL;
		len = vblk->size - offs;
	}

	if ((offs % 512) || (len % 512))
		return -EINVAL;

	mutexLock(req->lock);

	if ((ret = _virtioblk_resizeBuff(req, len)) < 0) {
		mutexUnlock(req->lock);
		return ret;
	}

	req->type = virtio_gtov32(&vblk->vdev, 0);
	req->sector = virtio_gtov64(&vblk->vdev, offs / 512);
	req->data.buff = req->buff;
	req->data.len = len;
	req->vreq.rsegs = 1;
	req->vreq.wsegs = 2;

	if (_virtioblk_send(vblk, req) < 0) {
		mutexUnlock(req->lock);
		return -EIO;
	}
	ret = req->len - 1;
	memcpy(buff, req->buff, ret);

	mutexUnlock(req->lock);

	return ret;
}


/* Writes data to device */
static ssize_t virtioblk_write(virtioblk_dev_t *vblk, void *rctx, offs_t offs, const char *buff, size_t len)
{
	virtioblk_req_t *req = (virtioblk_req_t *)rctx;
	ssize_t ret;

	if (offs + len > vblk->size) {
		if (offs > vblk->size)
			return -EINVAL;
		len = vblk->size - offs;
	}

	if ((offs % 512) || (len % 512))
		return -EINVAL;

	mutexLock(req->lock);

	if ((ret = _virtioblk_resizeBuff(req, len)) < 0) {
		mutexUnlock(req->lock);
		return ret;
	}

	req->type = virtio_gtov32(&vblk->vdev, 1);
	req->sector = virtio_gtov64(&vblk->vdev, offs / 512);
	req->data.buff = req->buff;
	req->data.len = len;
	req->vreq.rsegs = 2;
	req->vreq.wsegs = 1;
	memcpy(req->buff, buff, len);

	if (_virtioblk_send(vblk, req) < 0) {
		mutexUnlock(req->lock);
		return -EIO;
	}
	ret = len;

	mutexUnlock(req->lock);

	return ret;
}


/* Reads device attributes */
static int virtioblk_getattr(virtioblk_dev_t *vblk, int type, int *attr)
{
	switch (type) {
	case atSize:
		*attr = vblk->size;
		break;

	default:
		*attr = -EINVAL;
	}

	return EOK;
}


/* Destroys device */
static void virtioblk_destroyDev(virtioblk_dev_t *vblk)
{
	virtio_dev_t *vdev = &vblk->vdev;

	resourceDestroy(vblk->lock);
	resourceDestroy(vblk->cond);
	resourceDestroy(vblk->inth);
	virtqueue_destroy(vdev, &vblk->vq);
	virtio_destroyDev(vdev);
}


/* Initializes device */
static int virtioblk_initDev(virtioblk_dev_t *vblk)
{
	virtio_dev_t *vdev = &vblk->vdev;
	int err;

	if ((err = virtio_initDev(vdev)) < 0)
		return err;

	do {
		if ((err = virtio_writeFeatures(vdev, 0)) < 0)
			break;

		if ((err = virtqueue_init(vdev, &vblk->vq, 0, 128)) < 0)
			break;

		vblk->sectorsz = 512;
		vblk->size = 512 * virtio_readConfig64(vdev, 0x00);

		if ((err = mutexCreate(&vblk->lock)) < 0) {
			virtqueue_destroy(vdev, &vblk->vq);
			break;
		}

		if ((err = condCreate(&vblk->cond)) < 0) {
			resourceDestroy(vblk->lock);
			virtqueue_destroy(vdev, &vblk->vq);
			break;
		}

		if ((err = beginthread(virtioblk_intthr, 4, vblk->istack, sizeof(vblk->istack), vblk)) < 0) {
			resourceDestroy(vblk->cond);
			resourceDestroy(vblk->lock);
			virtqueue_destroy(vdev, &vblk->vq);
			break;
		}

#ifdef USE_POLLING
		virtqueue_disableIRQ(&vblk->vdev, &vblk->vq);
#else
		if ((err = interrupt(vdev->info.irq, virtioblk_int, vblk, vblk->cond, &vblk->inth)) < 0) {
			resourceDestroy(vblk->cond);
			resourceDestroy(vblk->lock);
			virtqueue_destroy(vdev, &vblk->vq);
			break;
		}
#endif
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
	virtioblk_dev_t *vblk = (virtioblk_dev_t *)arg;
	unsigned long rid;
	void *rctx;
	msg_t msg;

	if ((rctx = virtioblk_open()) == NULL)
		endthread();

	for (;;) {
		if (msgRecv(vblk->port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
		case mtClose:
			msg.o.io.err = EOK;
			break;

		case mtRead:
			msg.o.io.err = virtioblk_read(vblk, rctx, msg.i.io.offs, msg.o.data, msg.o.size);
			break;

		case mtWrite:
			msg.o.io.err = virtioblk_write(vblk, rctx, msg.i.io.offs, msg.i.data, msg.i.size);
			break;

		case mtGetAttr:
			virtioblk_getattr(vblk, msg.i.attr.type, &msg.o.attr.val);
			break;

		default:
			msg.o.io.err = -EINVAL;
		}

		msgRespond(vblk->port, &msg, rid);
	}

	virtioblk_close(rctx);
	endthread();
}


/* Registers device */
static int virtioblk_registerDev(const char *path, virtioblk_dev_t *vblk)
{
	int i, err;
	oid_t oid;

	if ((err = portCreate(&vblk->port)) < 0)
		return err;

	do {
		for (i = 0; i < sizeof(vblk->pstack) / sizeof(vblk->pstack[0]); i++) {
			if ((err = beginthread(virtioblk_poolthr, 4, vblk->pstack[i], sizeof(vblk->pstack[i]), vblk)) < 0)
				break;
		}

		if (err < 0)
			break;

		/* TODO: detect and register MBR partitions */
		oid.port = vblk->port;
		oid.id = 0;
		if ((err = create_dev(&oid, path)) < 0)
			break;

		return EOK;
	} while (0);

	portDestroy(vblk->port);

	return err;
}


int main(int argc, char **argv)
{
	virtioblk_dev_t *vblk;
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

			if ((vblk = malloc(sizeof(virtioblk_dev_t))) == NULL) {
				fprintf(stderr, "virtio-blk: out of memory\n");
				return -ENOMEM;
			}
			vblk->vdev = vdev;

			if ((err = virtioblk_initDev(vblk)) < 0) {
				if (err != -ENODEV)
					fprintf(stderr, "virtio-blk: failed to init VirtIO block device, base: %#x. Skipping...\n", vdev.info.base.addr);
				free(vblk);
				continue;
			}

			sprintf(path, "/dev/vblk%c", 'a' + nbdevs);
			if ((err = virtioblk_registerDev(path, vblk)) < 0) {
				fprintf(stderr, "virtio-blk: failed to register VirtIO block device, base: %#x. Skipping...\n", vdev.info.base.addr);
				virtioblk_destroyDev(vblk);
				free(vblk);
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
