/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Lukasz Kosinski, Michal Slomczynski
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

#include <sys/interrupt.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>

#include <libvirtio.h>


// /* Control virtqueue commands */
// enum {
// 	DISPLAY_INFO = 0x0100,
// 	RESOURCE_CREATE_2D,
// 	RESOURCE_UNREF,
// 	SET_SCANOUT,
// 	RESOURCE_FLUSH,
// 	TRANSFER_TO_HOST_2D,
// 	RESOURCE_ATTACH_BACKING,
// 	RESOURCE_DETACH_BACKING,
// 	GET_CAPSET_INFO,
// 	GET_CAPSET,
// 	GET_EDID,
// };


// enum {
// 	/* curqsor commands */
// 	UPDATE_curqSOR = 0x0300,
// 	MOVE_curqSOR,
// };

// enum {
// 	/* success responses */
// 	VIRTIO_GPU_RESP_OK_NODATA = 0x1100,
// 	VIRTIO_GPU_RESP_OK_DISPLAY_INFO,
// 	VIRTIO_GPU_RESP_OK_CAPSET_INFO,
// 	VIRTIO_GPU_RESP_OK_CAPSET,
// 	VIRTIO_GPU_RESP_OK_EDID,
// 	VIRTIO_GPU_RESP_ERR_UNSPEC = 0x1200,
// 	VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY,
// 	VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID,
// 	VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID,
// 	VIRTIO_GPU_RESP_ERR_INVALID_CONTEXT_ID,
// 	VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER
// }


typedef struct {
    virtio_dev_t vdev;
    virtqueue_t ctlq;
    virtqueue_t curq;

	/* Interrupt handling */
	volatile unsigned int isr; /* Interrupt status */
	handle_t lock;             /* Interrupt mutex */
	handle_t cond;             /* Interrupt condition variable */
	handle_t inth;             /* Interrupt handle */
	char istack[2048] __attribute__((aligned(8)));
} virtiogpu_dev_t;


typedef struct {
	uint32_t type;
	uint32_t flags;
	uint64_t fenceId;
	uint32_t ctxId;
	uint32_t padding;
} virtiogpu_hdr_t;


typedef struct {
	uint32_t x;
	uint32_t y;
	uint32_t width;
	uint32_t height;
} virtiogpu_rect_t;


typedef struct {
	virtiogpu_rect_t r;
	uint32_t enabled;
	uint32_t flags;
} virtiogpu_display_t;


typedef struct {
	uint32_t scanoutId;
	uint32_t x;
	uint32_t y;
	uint32_t padding;
} virtiogpu_curqpos_t;


typedef struct {
	virtiogpu_hdr_t hdr;
	virtiogpu_display_t pmodes[16];

	virtio_seg_t header;
	virtio_seg_t footer;
	virtio_req_t vreq;

	/* Custom helper fields */
	handle_t lock;             /* Request mutex */
	handle_t cond;             /* Request condition variable */
} virtiogpu_display_info_req_t;


// typedef struct {
// 	virtiogpu_hdr_t hdr;
// 	union {
// 		/* Display info */
// 		struct {
// 			virtiogpu_display_t pmodes[16];
// 		};
// 		/* EDID */
// 		struct {
// 			uint32_t size;
// 			uint32_t padding;
// 			uint8_t edid[1024];
// 		};
// 		/* Resource create 2D */
// 		struct {
// 			uint32_t resourceId;
// 			uint32_t format;
// 			uint32_t width;
// 			uint32_t height;
// 		};
// 		/* Resource unref */
// 		struct {
// 			uint32_t resourceId;
// 			uint32_t padding;
// 		};
// 		/* Set scanout */
// 		struct {
// 			virtiogpu_rect_t r;
// 			uint32_t scnaoutId;
// 			uint32_t resourceId;
// 		};
// 		/* Resource flush */
// 		struct {
// 			virtiogpu_rect_t r;
// 			uint32_t resourceId;
// 			uint32_t padding;
// 		};
// 		/* Transfer to host 2D */
// 		struct {
// 			virtiogpu_rect_t r;
// 			uint64_t offset;
// 			uint32_t resourceId;
// 			uint32_t padding;
// 		};
// 		/* Resource attach backing */
// 		struct {
// 			uint32_t resourceId;
// 			uint32_t nentries;
// 		};
// 		/* Resource deatch backing */
// 		struct {
// 			uint32_t resourceId;
// 			uint32_t padding;
// 		};
// 		/* Update curqsors */
// 		struct {
// 			virtiogpu_curqpos_t pos;
// 			uint32_t resourceId;
// 			uint32_t x;
// 			uint32_t y;
// 			uint32_t padding;
// 		};
// 	};

// 	virtio_seg_t header;
// 	virtio_seg_t footer;
// 	virtio_req_t vreq;

// 	/* Custom helper fields */
// 	unsigned int len;          /* Number of bytes written to request buffers */
// 	handle_t lock;             /* Request mutex */
// 	handle_t cond;             /* Request condition variable */
// } virtiogpu_req_t;


/* VirtIO GPU devices descriptors */
static const virtio_devinfo_t info[] = {
	{ .type = vdevPCI, .id = 0x1050 },
#ifdef TARGET_RISCV64
	/* Direct VirtIO MMIO QEMU block device descriptors */
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


static int virtiogpu_int(unsigned int n, void *arg)
{
	virtiogpu_dev_t *gpudev = (virtiogpu_dev_t *)arg;
	virtio_dev_t *vdev = &gpudev->vdev;

	virtqueue_disableIRQ(vdev, &gpudev->ctlq);
	virtqueue_disableIRQ(vdev, &gpudev->curq);
	gpudev->isr = virtio_isr(vdev);

	return gpudev->cond;
}

static void virtiogpu_intthr(void *arg)
{
	virtiogpu_dev_t *gpudev = (virtiogpu_dev_t *)arg;
	virtio_dev_t *vdev = &gpudev->vdev;
	//virtiogpu_req_t *req;

	mutexLock(gpudev->lock);
	gpudev->isr = 0;

	for (;;) {
		while (!gpudev->isr)
			condWait(gpudev->cond, gpudev->lock, 0);
		gpudev->isr = 0;

		printf("INT!\n");

		// if (gpudev->isr & 0x01) {
		// 	/* Get processed requests */

		// }
		// else if (gpudev->isr & 0x02) {
		// 	/* Handle configuration change */
		// }
		// gpudev->isr = 0;
		// virtqueue_enableIRQ(vdev, &gpudev->ctlq);
		// virtqueue_enableIRQ(vdev, &gpudev->curq);
	}
}

// static ssize_t virtiogpu_ctlqSendRequest(virtiogpu_dev_t *gpudev, virtiogpu_req_t *req)
// {
// 	virtio_dev_t *vdev = &gpudev->vdev;
// 	int err;

// 	mutexLock(req->lock);

// 	if ((err = virtqueue_enqueue(vdev, &gpudev->ctlq, &req->vreq)) < 0) {
// 		mutexUnlock(req->lock);
// 		return err;
// 	}
// 	virtqueue_notify(vdev, &gpudev->ctlq);

// 	while (req->hdr.type < 0x10b && req->hdr.type > 0x100)
// 		condWait(req->cond, req->lock, 0);
// }

// static ssize_t virtiogpu_curqSendRequest(virtiogpu_dev_t *gpudev, virtiogpu_req_t *req)
// {
// 	virtio_dev_t *vdev = &gpudev->vdev;
// 	int err;

// 	mutexLock(req->lock);

// 	if ((err = virtqueue_enqueue(vdev, &gpudev->curq)), virtqueue_e)
// }


static int virtiogpu_displayInfo(virtiogpu_dev_t *gpudev)
{
	virtio_dev_t *vdev = &gpudev->vdev;
	virtiogpu_display_info_req_t req;
	int err;

	if ((err = mutexCreate(&req.lock)) < 0)
		return err;

	if ((err = condCreate(&req.cond)) < 0) {
		resourceDestroy(req.lock);
		return err;
	}

	req.header.buff = &req;
	req.header.len = sizeof(req.hdr);
	req.header.prev = &req.footer;
	req.header.next = &req.footer;
	req.footer.buff = &req;
	req.footer.len = sizeof(req.hdr) + sizeof(req.pmodes);
	req.footer.prev = &req.header;
	req.footer.next = &req.header;
	req.vreq.segs = &req.header;
	req.vreq.rsegs = 1;
	req.vreq.wsegs = 1;

	mutexLock(req.lock);

	req.hdr.type = 0x100;
	req.hdr.fenceId = 0x1;

	virtqueue_enqueue(vdev, &gpudev->ctlq, &req.vreq);
	virtqueue_notify(vdev, &gpudev->ctlq);

	// while (req.hdr.type == 0x100)
	// 	condWait(req.cond, req.lock, 0);
	sleep(1);

	printf("Width: %u, Height: %u, Enabled: %u\n", req.pmodes[0].r.width, req.pmodes[0].r.height, req.pmodes[0].enabled);
	printf("Type: %#x\n", req.hdr.type);
	resourceDestroy(req.lock);
	resourceDestroy(req.cond);

	return 0;
}


/* Initializes device */
static int virtiogpu_initDev(virtiogpu_dev_t *gpudev)
{
	virtio_dev_t *vdev = &gpudev->vdev;
	int err;

	if ((err = virtio_initDev(vdev)) < 0)
		return err;

	do {
		if ((err = virtio_writeFeatures(vdev, 0)) < 0)
			break;

		if ((err = virtqueue_init(vdev, &gpudev->ctlq, 0, 128)) < 0)
			break;

        if ((err = virtqueue_init(vdev, &gpudev->curq, 1, 32)) < 0)
			break;

		if ((err = mutexCreate(&gpudev->lock)) < 0) {
			virtqueue_destroy(vdev, &gpudev->ctlq);
			virtqueue_destroy(vdev, &gpudev->curq);
			break;
		}

		if ((err = condCreate(&gpudev->cond)) < 0) {
			resourceDestroy(gpudev->lock);
			virtqueue_destroy(vdev, &gpudev->ctlq);
			virtqueue_destroy(vdev, &gpudev->curq);
			break;
		}

		if ((err = beginthread(virtiogpu_intthr, 4, gpudev->istack, sizeof(gpudev->istack), gpudev)) < 0) {
			resourceDestroy(gpudev->cond);
			resourceDestroy(gpudev->lock);
			virtqueue_destroy(vdev, &gpudev->ctlq);
			virtqueue_destroy(vdev, &gpudev->curq);
			break;
		}

		if ((err = interrupt(vdev->info.irq, virtiogpu_int, gpudev, gpudev->cond, &gpudev->inth)) < 0) {
			resourceDestroy(gpudev->cond);
			resourceDestroy(gpudev->lock);
			virtqueue_destroy(vdev, &gpudev->ctlq);
			virtqueue_destroy(vdev, &gpudev->curq);
			break;
		}

		virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 2));

		return EOK;
	} while (0);

	virtio_writeStatus(vdev, virtio_readStatus(vdev) | (1 << 7));
	virtio_destroyDev(vdev);

	return err;
}


int main(void)
{
    virtiogpu_dev_t *gpudev;
	virtio_dev_t vdev;
	virtio_ctx_t vctx;
	//char path[16];
	oid_t oid;
	int i, err, ngpudevs = 0;

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
					fprintf(stderr, "direct descriptor, base: %p. ", info[i].base.addr);
				else
					fprintf(stderr, "descriptor, ID: %#x. ", info[i].id);
				fprintf(stderr, "Skipping...\n");
				continue;
			}

			if ((gpudev = malloc(sizeof(virtiogpu_dev_t))) == NULL) {
				fprintf(stderr, "virtio-gpu: out of memory\n");
				return -ENOMEM;
			}
			gpudev->vdev = vdev;

			if ((err = virtiogpu_initDev(gpudev)) < 0) {
				if (err != -ENODEV)
					fprintf(stderr, "virtio-gpu: failed to init VirtIO GPU device, base: %p. Skipping...\n", vdev.info.base.addr);
				free(gpudev);
				continue;
			}
            printf("Intialized GPU %d\n", ngpudevs);
			virtiogpu_displayInfo(gpudev);
			// sprintf(path, "/dev/vblk%c", 'a' + nbdevs);
			// if ((err = virtioblk_registerDev(path, bdev)) < 0) {
			// 	fprintf(stderr, "virtio-blk: failed to register VirtIO block device, base: %#x. Skipping...\n", vdev.info.base.addr);
			// 	virtioblk_destroyDev(bdev);
			// 	free(bdev);
			// 	continue;
			// }
			ngpudevs++;
		}
	}

	for (;;)
		sleep(10);

	virtio_done();

	return EOK;
}