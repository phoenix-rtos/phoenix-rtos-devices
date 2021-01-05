/*
 * Phoenix-RTOS
 *
 * VirtIO block device driver
 *
 * Copyright 2020 Phoenix Systems
 * Author: Michal Slomczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/msg.h>

#include <libvirtio.h>


typedef struct {
    virtio_dev_t vdev;
    virtqueue_t control;
    virtqueue_t cursor;
} virtiogpu_dev_t;


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

		if ((err = virtqueue_init(vdev, &gpudev->control, 0, 128)) < 0)
			break;

        if ((err = virtqueue_init(vdev, &gpudev->cursor, 0, 32)) < 0)
			break;

		// if ((err = mutexCreate(&bdev->lock)) < 0) {
		// 	virtqueue_destroy(vdev, &bdev->vq);
		// 	break;
		// }

		// if ((err = condCreate(&bdev->cond)) < 0) {
		// 	resourceDestroy(bdev->lock);
		// 	virtqueue_destroy(vdev, &bdev->vq);
		// 	break;
		// }

		// if ((err = beginthread(virtioblk_intthr, 4, bdev->istack, sizeof(bdev->istack), bdev)) < 0) {
		// 	resourceDestroy(bdev->cond);
		// 	resourceDestroy(bdev->lock);
		// 	virtqueue_destroy(vdev, &bdev->vq);
		// 	break;
		// }

		// if ((err = interrupt(vdev->info.irq, virtioblk_int, bdev, bdev->cond, &bdev->inth)) < 0) {
		// 	resourceDestroy(bdev->cond);
		// 	resourceDestroy(bdev->lock);
		// 	virtqueue_destroy(vdev, &bdev->vq);
		// 	break;
		// }

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