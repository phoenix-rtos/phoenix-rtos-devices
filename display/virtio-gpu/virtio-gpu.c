/*
 * Phoenix-RTOS
 *
 * VirtIO GPU device driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski, Michal Slomczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <endian.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libvirtio.h>


typedef struct {
	/* Device data */
	virtio_dev_t vdev;              /* VirtIO device */
	virtqueue_t ctlq;               /* Control virtqueue */
	virtqueue_t curq;               /* Cursor virtqueue */
	void *displays[16];             /* Displays buffers */
	unsigned int ndisplays;         /* Number of connected displays */
	unsigned int rbmp;              /* Host resources bitmap */
	handle_t rlock;                 /* Host resources bitmap mutex */

	/* Interrupt handling */
	volatile unsigned int isr;      /* Interrupt status */
	handle_t lock;                  /* Interrupt mutex */
	handle_t cond;                  /* Interrupt condition variable */
	handle_t inth;                  /* Interrupt handle */
	char istack[2048] __attribute__((aligned(8)));
} virtiogpu_dev_t;


typedef struct {
	uint32_t x;                     /* Horizontal coordinate */
	uint32_t y;                     /* Vertical coordinate */
	uint32_t width;                 /* Rectangle width */
	uint32_t height;                /* Rectangle height */
} __attribute__((packed)) virtiogpu_rect_t;


typedef struct {
	struct {
		virtiogpu_rect_t r;         /* Display rectangle */
		uint32_t enabled;           /* Is enabled? */
		uint32_t flags;             /* Display flags */
	} pmodes[16];                   /* Displays */
} __attribute__((packed)) virtiogpu_display_t;


typedef struct {
	/* Request buffers (accessible by device) */
	struct {
		/* Request header (device readable/writable) */
		struct {
			uint32_t type;          /* Request/Response type */
			uint32_t flags;         /* Request flags */
			uint64_t fence;         /* Request fence ID */
			uint32_t ctx;           /* 3D rendering context */
			uint32_t pad;           /* Padding */
		} hdr;

		/* Request data (device access depends on request type) */
		union {
			/* Displays info request */
			volatile virtiogpu_display_t info;

			/* EDID request */
			volatile struct {
				uint32_t sid;       /* Scanout ID/data size */
				uint32_t pad;       /* Padding */
				uint8_t data[1024]; /* EDID data */
			} edid;

			/* Create resource 2D request */
			struct {
				uint32_t rid;       /* Resource ID */
				uint32_t format;    /* Resource format */
				uint32_t width;     /* Resource width */
				uint32_t height;    /* Resource height */
			} res2D;

			/* Unref resource request */
			struct {
				uint32_t rid;       /* Resource ID */
				uint32_t pad;       /* Padding */
			} unref;

			/* Set scanout request */
			struct {
				virtiogpu_rect_t r; /* Scanout rectangle */
				uint32_t sid;       /* Scanout ID */
				uint32_t rid;       /* Resource ID */
			} scanout;

			/* Flush scanout request */
			struct {
				virtiogpu_rect_t r; /* Scanout rectangle */
				uint32_t rid;       /* Resource ID */
				uint32_t pad;       /* Padding */
			} flush;

			/* Transfer resource 2D request */
			struct {
				virtiogpu_rect_t r; /* Buffer rectangle */
				uint64_t offset;    /* Resource offset */
				uint32_t rid;       /* Resource ID */
				uint32_t pad;       /* Padding */
			} trans2D;

			/* Attach resource request */
			struct {
				uint32_t rid;       /* Resource ID */
				uint32_t nbuffs;    /* Number of attached buffers (one buffer only) */
				uint64_t addr;      /* Buffer address */
				uint32_t len;       /* Buffer length */
				uint32_t pad;       /* Padding */
			} attach;

			/* Detach resource request */
			struct {
				uint32_t rid;       /* Resource ID */
				uint32_t pad;       /* Padding */
			} detach;

			/* Update cursor request */
			struct {
				struct {
					uint32_t sid;   /* Scanout ID */
					uint32_t x;     /* Horizontal coordinate */
					uint32_t y;     /* Vertical coordinate */
					uint32_t pad;   /* Padding */
				} pos;
				uint32_t rid;       /* Resource ID */
				uint32_t hx;        /* Hotspot x */
				uint32_t hy;        /* Hotspot y */
				uint32_t pad;       /* Padding */
			} cursor;
		};
	} __attribute__((packed));

	/* VirtIO request segments */
	virtio_seg_t rseg;              /* Device readable segment */
	virtio_seg_t wseg;              /* Device writeable segment */
	virtio_req_t vreq;              /* VirtIO request */

	/* Custom helper fields */
	volatile int done;              /* Indicates request completion */
	handle_t lock;                  /* Request mutex */
	handle_t cond;                  /* Request condition variable */
} virtiogpu_req_t;


/* VirtIO GPU device descriptors */
static const virtio_devinfo_t info[] = {
	{ .type = vdevPCI, .id = 0x1050 },
#ifdef TARGET_RISCV64
	/* Direct VirtIO MMIO QEMU GPU device descriptors */
	{ .type = vdevMMIO, .id = 0x10, .irq = 8, .base = { (void *)0x10008000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 7, .base = { (void *)0x10007000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 6, .base = { (void *)0x10006000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 5, .base = { (void *)0x10005000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 4, .base = { (void *)0x10004000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 3, .base = { (void *)0x10003000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 2, .base = { (void *)0x10002000, 0x1000 } },
	{ .type = vdevMMIO, .id = 0x10, .irq = 1, .base = { (void *)0x10001000, 0x1000 } },
#endif
	{ .type = vdevNONE }
};


/* Returns guest endianess */
static inline int virtiogpu_endian(void)
{
#if __BYTE_ORDER == __LITTLE_ENDIAN
	return 1;
#else
	return 0;
#endif
}


/* Interrupt handler */
static int virtiogpu_int(unsigned int n, void *arg)
{
	virtiogpu_dev_t *vgpu = (virtiogpu_dev_t *)arg;
	virtio_dev_t *vdev = &vgpu->vdev;

	virtqueue_disableIRQ(vdev, &vgpu->ctlq);
	virtqueue_disableIRQ(vdev, &vgpu->curq);
	vgpu->isr = virtio_isr(vdev);

	return vgpu->cond;
}


/* Interrupt thread */
static void virtiogpu_intthr(void *arg)
{
	virtiogpu_dev_t *vgpu = (virtiogpu_dev_t *)arg;
	virtio_dev_t *vdev = &vgpu->vdev;
	virtiogpu_req_t *req;

	mutexLock(vgpu->lock);
	vgpu->isr = 0;

	for (;;) {
		while (!vgpu->isr)
			condWait(vgpu->cond, vgpu->lock, 0);

		if (vgpu->isr & (1 << 0)) {
			while ((req = virtqueue_dequeue(vdev, &vgpu->ctlq, NULL)) != NULL) {
				mutexLock(req->lock);
				req->done = 1;
				condSignal(req->cond);
				mutexUnlock(req->lock);
			}

			while ((req = virtqueue_dequeue(vdev, &vgpu->curq, NULL)) != NULL) {
				mutexLock(req->lock);
				req->done = 1;
				condSignal(req->cond);
				mutexUnlock(req->lock);
			}
		}

		if (vgpu->isr & (1 << 1)) {
			/* Handle configuration change */
			printf("Configuration changed!\n");
		}

		vgpu->isr = 0;
		virtqueue_enableIRQ(vdev, &vgpu->ctlq);
		virtqueue_enableIRQ(vdev, &vgpu->curq);

		/* Get requests that might have come after last virtqueue_dequeue() and before virtqueue_enableIRQ() */
		while ((req = virtqueue_dequeue(vdev, &vgpu->ctlq, NULL)) != NULL) {
			mutexLock(req->lock);
			req->done = 1;
			condSignal(req->cond);
			mutexUnlock(req->lock);
		}

		while ((req = virtqueue_dequeue(vdev, &vgpu->curq, NULL)) != NULL) {
			mutexLock(req->lock);
			req->done = 1;
			condSignal(req->cond);
			mutexUnlock(req->lock);
		}
	}
}


/* Sends request to device */
static int _virtiogpu_send(virtiogpu_dev_t *vgpu, virtqueue_t *vq, virtiogpu_req_t *req, unsigned int resp)
{
	virtio_dev_t *vdev = &vgpu->vdev;
	int err;

	req->done = 0;
	if ((err = virtqueue_enqueue(vdev, vq, &req->vreq)) < 0)
		return err;
	virtqueue_notify(vdev, vq);

	while (!req->done)
		condWait(req->cond, req->lock, 0);

	if (le32toh(*(volatile uint32_t *)(&req->hdr.type)) != resp)
		return -EFAULT;

	return EOK;
}


/* Opens request context */
static virtiogpu_req_t *virtiogpu_open(void)
{
	virtiogpu_req_t *req;

	if ((req = mmap(NULL, (sizeof(virtiogpu_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1), PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_CONTIGUOUS, 0)) == MAP_FAILED)
		return NULL;

	if (mutexCreate(&req->lock) < 0) {
		munmap(req, (sizeof(virtiogpu_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	if (condCreate(&req->cond) < 0) {
		resourceDestroy(req->lock);
		munmap(req, (sizeof(virtiogpu_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
		return NULL;
	}

	req->rseg.prev = &req->wseg;
	req->rseg.next = &req->wseg;
	req->wseg.prev = &req->rseg;
	req->wseg.next = &req->rseg;
	req->vreq.segs = &req->rseg;
	req->vreq.rsegs = 1;
	req->vreq.wsegs = 1;

	return req;
}


/* Closes request context */
static void virtiogpu_close(virtiogpu_req_t *req)
{
	resourceDestroy(req->cond);
	resourceDestroy(req->lock);
	munmap(req, (sizeof(virtiogpu_req_t) + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1));
}


/* Retrieves display info */
static int virtiogpu_displayInfo(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, virtiogpu_display_t *info)
{
	int i, err;

	mutexLock(req->lock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr) + sizeof(req->info);

	req->hdr.type = htole32(0x100);
	req->hdr.flags = htole32(1 << 0);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1101)) < 0) {
		mutexUnlock(req->lock);
		return err;
	}

	for (i = 0; i < sizeof(info->pmodes) / sizeof(info->pmodes[0]); i++) {
		info->pmodes[i].r.x = le32toh(req->info.pmodes[i].r.x);
		info->pmodes[i].r.y = le32toh(req->info.pmodes[i].r.y);
		info->pmodes[i].r.width = le32toh(req->info.pmodes[i].r.width);
		info->pmodes[i].r.height = le32toh(req->info.pmodes[i].r.height);
		info->pmodes[i].enabled = le32toh(req->info.pmodes[i].enabled);
		info->pmodes[i].flags = le32toh(req->info.pmodes[i].flags);
	}

	mutexUnlock(req->lock);

	return EOK;
}


/* Retrieves display EDID */
static int virtiogpu_EDID(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int sid, void *edid, unsigned int *len)
{
	int i, err;

	if (!(virtio_readFeatures(&vgpu->vdev) & (1ULL << 1)))
		return -ENOTSUP;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->edid) - sizeof(req->edid.data);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr) + sizeof(req->edid);

	req->hdr.type = htole32(0x10a);
	req->hdr.flags = htole32(1 << 0);
	req->edid.sid = htole32(sid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1104)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	*len = le32toh(req->edid.sid);
	for (i = 0; i < *len; i++)
		*((uint8_t *)edid + i) = req->edid.data[i];

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Creates 2D resource */
static int virtiogpu_resource2D(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int format, unsigned int width, unsigned int height)
{
	int err, ret;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->res2D);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x101);
	req->hdr.flags = htole32(1 << 0);
	req->res2D.format = htole32(format);
	req->res2D.width = htole32(width);
	req->res2D.height = htole32(height);
	req->res2D.rid = htole32(1 << (__builtin_ffsl(vgpu->rbmp) - 1));

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}
	ret = le32toh(req->res2D.rid);
	vgpu->rbmp &= ~ret;

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return ret;
}


/* Destroys resource */
static int virtiogpu_unref(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->unref);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x102);
	req->hdr.flags = htole32(1 << 0);
	req->unref.rid = htole32(rid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}
	vgpu->rbmp |= rid;

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Sets scanout buffer for display */
static int virtiogpu_scanout(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int sid, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->scanout);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x103);
	req->hdr.flags = htole32(1 << 0);
	req->scanout.r.x = htole32(x);
	req->scanout.r.y = htole32(y);
	req->scanout.r.width = htole32(w);
	req->scanout.r.height = htole32(h);
	req->scanout.sid = htole32(sid);
	req->scanout.rid = htole32(rid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Flushes resource to display */
static int virtiogpu_flush(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->flush);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x104);
	req->hdr.flags = htole32(1 << 0);
	req->flush.r.x = htole32(x);
	req->flush.r.y = htole32(y);
	req->flush.r.width = htole32(w);
	req->flush.r.height = htole32(h);
	req->flush.rid = htole32(rid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Transfers data to host resource from underlying guest buffer */
static int virtiogpu_transfer2D(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int x, unsigned int y, unsigned int w, unsigned int h, unsigned int offs, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->trans2D);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x105);
	req->hdr.flags = htole32(1 << 0);
	req->trans2D.r.x = htole32(x);
	req->trans2D.r.y = htole32(y);
	req->trans2D.r.width = htole32(w);
	req->trans2D.r.height = htole32(h);
	req->trans2D.offset = htole64(offs);
	req->trans2D.rid = htole32(rid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Attaches guest buffer to host resource */
static int virtiogpu_attach(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int rid, void *buff, unsigned int len)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->attach);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x106);
	req->hdr.flags = htole32(1 << 0);
	req->attach.rid = htole32(rid);
	req->attach.nbuffs = htole32(1);
	req->attach.addr = htole64(va2pa(buff));
	req->attach.len = htole32(len);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Detaches guest buffer from resource */
static int virtiogpu_detach(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->detach);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x107);
	req->hdr.flags = htole32(1 << 0);
	req->detach.rid = htole32(rid);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq, req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Updates cursor icon and its position */
static int virtiogpu_updateCursor(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int x, unsigned int y, unsigned hx, unsigned hy, unsigned int sid, unsigned int rid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->cursor);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x300);
	req->hdr.flags = htole32(1 << 0);
	req->cursor.pos.sid = htole32(sid);
	req->cursor.pos.x = htole32(x);
	req->cursor.pos.y = htole32(y);
	req->cursor.rid = htole32(rid);
	req->cursor.hx = htole32(hx);
	req->cursor.hy = htole32(hy);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq ,req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Moves cursor */
static int virtiogpu_moveCursor(virtiogpu_dev_t *vgpu, virtiogpu_req_t *req, unsigned int x, unsigned int y, unsigned int sid)
{
	int err;

	mutexLock(req->lock);
	mutexLock(vgpu->rlock);

	req->rseg.buff = &req->hdr;
	req->rseg.len = sizeof(req->hdr) + sizeof(req->cursor);
	req->wseg.buff = &req->hdr;
	req->wseg.len = sizeof(req->hdr);

	req->hdr.type = htole32(0x301);
	req->hdr.flags = htole32(1 << 0);
	req->cursor.pos.sid = htole32(sid);
	req->cursor.pos.x = htole32(x);
	req->cursor.pos.y = htole32(y);

	if ((err = _virtiogpu_send(vgpu, &vgpu->ctlq ,req, 0x1100)) < 0) {
		mutexUnlock(vgpu->rlock);
		mutexUnlock(req->lock);
		return err;
	}

	mutexUnlock(vgpu->rlock);
	mutexUnlock(req->lock);

	return EOK;
}


/* Initializes device */
static int virtiogpu_initDev(virtiogpu_dev_t *vgpu)
{
	virtio_dev_t *vdev = &vgpu->vdev;
	virtiogpu_display_t info;
	virtiogpu_req_t *req;
	int i, rid, err;

	if ((err = virtio_initDev(vdev)) < 0)
		return err;

	do {
		/* Negotiate EDID support */
		if ((err = virtio_writeFeatures(vdev, (1 << 1))) < 0)
			break;

		/* Get number of connected displays */
		vgpu->ndisplays = virtio_readConfig32(vdev, 0x08);

		if ((err = virtqueue_init(vdev, &vgpu->ctlq, 0, 64)) < 0)
			break;

		if ((err = virtqueue_init(vdev, &vgpu->curq, 1, 16)) < 0)
			break;

		if ((err = mutexCreate(&vgpu->lock)) < 0) {
			virtqueue_destroy(vdev, &vgpu->ctlq);
			virtqueue_destroy(vdev, &vgpu->curq);
			break;
		}

		if ((err = condCreate(&vgpu->cond)) < 0) {
			resourceDestroy(vgpu->lock);
			virtqueue_destroy(vdev, &vgpu->ctlq);
			virtqueue_destroy(vdev, &vgpu->curq);
			break;
		}

		if ((err = beginthread(virtiogpu_intthr, 4, vgpu->istack, sizeof(vgpu->istack), vgpu)) < 0) {
			resourceDestroy(vgpu->cond);
			resourceDestroy(vgpu->lock);
			virtqueue_destroy(vdev, &vgpu->ctlq);
			virtqueue_destroy(vdev, &vgpu->curq);
			break;
		}

		/* TODO: fix race condition below */
		/* Added small delay for virtiogpu_intthr() to go to sleep before first intterupt */
		usleep(10);

		if ((err = interrupt(vdev->info.irq, virtiogpu_int, vgpu, vgpu->cond, &vgpu->inth)) < 0) {
			resourceDestroy(vgpu->cond);
			resourceDestroy(vgpu->lock);
			virtqueue_destroy(vdev, &vgpu->ctlq);
			virtqueue_destroy(vdev, &vgpu->curq);
			break;
		}

		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 2));

		/* Initialize resources bitmap */
		vgpu->rbmp = 0xffffffff;

		if ((req = virtiogpu_open()) == NULL) {
			err = -ENOMEM;
			break;
		}

		/* Initialize displays */
		for (i = 0; i < vgpu->ndisplays; i++) {
			if ((err = virtiogpu_displayInfo(vgpu, req, &info)) < 0)
				break;

			if ((rid = virtiogpu_resource2D(vgpu, req, virtiogpu_endian() ? 121 : 3, info.pmodes[0].r.width, info.pmodes[0].r.height)) < 0) {
				err = rid;
				break;
			}

			if ((vgpu->displays[i] = mmap(NULL, (4 * info.pmodes[0].r.width * info.pmodes[0].r.height + _PAGE_SIZE - 1) & ~(_PAGE_SIZE - 1), PROT_READ | PROT_WRITE, MAP_UNCACHED | MAP_ANONYMOUS, OID_CONTIGUOUS, 0)) == MAP_FAILED) {
				err = -ENOMEM;
				break;
			}

			if ((err = virtiogpu_attach(vgpu, req, rid, vgpu->displays[i], 4 * info.pmodes[0].r.width * info.pmodes[0].r.height)) < 0)
				break;

			if ((err = virtiogpu_scanout(vgpu, req, 0, 0, info.pmodes[0].r.width, info.pmodes[0].r.height, i, rid)) < 0)
				break;
		}

		if (err < 0)
			break;

		virtiogpu_close(req);

		return EOK;
	} while (0);

	virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
	virtio_destroyDev(vdev);

	return err;
}


int main(void)
{
	virtiogpu_dev_t *vgpu;
	virtio_dev_t vdev;
	virtio_ctx_t vctx;
	oid_t oid;
	int i, err, nvgpus = 0;

	/* Wait for console */
	while (write(STDOUT_FILENO, "", 0) < 0)
		usleep(10000);

	/* Wait for root filesystem */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	virtio_init();

	/* Detect and initialize VirtIO GPU devices */
	for (i = 0; info[i].type != vdevNONE; i++) {
		vctx.reset = 1;
		while ((err = virtio_find(&info[i], &vdev, &vctx)) != -ENODEV) {
			if (err < 0) {
				fprintf(stderr, "virtio-gpu: failed to process VirtIO %s GPU device ", (info[i].type == vdevPCI) ? "PCI" : "MMIO");
				if (info[i].base.len)
					fprintf(stderr, "direct descriptor, base: %p. ", info[i].base.addr);
				else
					fprintf(stderr, "descriptor, ID: %#x. ", info[i].id);
				fprintf(stderr, "Skipping...\n");
				continue;
			}

			if ((vgpu = malloc(sizeof(virtiogpu_dev_t))) == NULL) {
				fprintf(stderr, "virtio-gpu: out of memory\n");
				return -ENOMEM;
			}
			vgpu->vdev = vdev;

			if ((err = virtiogpu_initDev(vgpu)) < 0) {
				if (err != -ENODEV)
					fprintf(stderr, "virtio-gpu: failed to init VirtIO GPU device, base: %p. Skipping...\n", vdev.info.base.addr);
				free(vgpu);
				continue;
			}
			nvgpus++;
		}
	}

	for (;;)
		sleep(10);

	virtio_done();

	return EOK;
}
