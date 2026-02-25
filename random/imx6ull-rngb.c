/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL RNGB driver
 *
 * Copyright 2023 Phoenix Systems
 * Author: Dawid Szpejna
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <posix/utils.h>
#include <sys/debug.h>
#include <sys/platform.h>
#include <sys/interrupt.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/msg.h>


enum { rng_ver = 0,
	rng_cmd,
	rng_cr,
	rng_sr,
	rng_esr,
	rng_out };


#define RNGB_START_ADDRESS 0x02284000
#define RNGB_IRQ           (6 + 32)


#define SELF_TEST_DONE 1U
#define SEED_DONE      2U
#define ERROR_OCCURRED 4U
#define MMAP_SIZE      4096


/* intr_st indicates status of interrupts */
static struct {
	volatile uint32_t *base;
	volatile uint32_t intr_st;
	uint32_t port;

	handle_t lock;
	handle_t cond;
} common_rngb;


static int rngb_intr(unsigned int n, void *arg)
{
	uint32_t status = *(common_rngb.base + rng_sr);

	/* Self tests are done */
	if ((status & 0x10) != 0) {
		common_rngb.intr_st |= SELF_TEST_DONE;
	}

	/* Seeding is done*/
	if ((status & 0x20) != 0) {
		common_rngb.intr_st |= SEED_DONE;
	}

	/* An error occurred*/
	if ((status & (1 << 16)) != 0) {
		common_rngb.intr_st |= ERROR_OCCURRED;
	}

	/* Clear interrupt */
	*(common_rngb.base + rng_cmd) = 0x20;
	return 1;
}


static void systemCleanup(int level)
{
	if (level > 3) {
		resourceDestroy(common_rngb.cond);
	}

	if (level > 2) {
		resourceDestroy(common_rngb.lock);
	}

	if (level > 1) {
		portDestroy(common_rngb.port);
	}

	if (level > 0) {
		munmap((void *)common_rngb.base, MMAP_SIZE);
	}
}


static int systemInit(void)
{
	const char devpath[] = "random";
	int err;
	oid_t dev;

	common_rngb.base = mmap(NULL, MMAP_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE, OID_PHYSMEM, RNGB_START_ADDRESS);
	if (common_rngb.base == MAP_FAILED) {
		printf("rngb: could not map addr\n");
		return -1;
	}

	err = portCreate(&common_rngb.port);
	if (err != EOK) {
		printf("rngb: could not create port\n");
		systemCleanup(1);
		return -1;
	}

	dev.port = common_rngb.port;
	dev.id = 0;

	err = mutexCreate(&common_rngb.lock);
	if (err != EOK) {
		printf("rngb: could not create mutex for rngb\n");
		systemCleanup(2);
		return -1;
	}

	err = condCreate(&common_rngb.cond);
	if (err != EOK) {
		printf("rngb: could not create cond for rngb\n");
		systemCleanup(3);
		return -1;
	}

	err = create_dev(&dev, devpath);
	if (err != EOK) {
		printf("rngb: could not create port file %s (err %d)\n", devpath, err);
		systemCleanup(4);
		return -1;
	}

	return 0;
}


static int hardwareInit(void)
{
	int err;

	/* Reset and wait until rngb is sleeping */
	*(common_rngb.base + rng_cmd) = 0x40;
	do {
		err = *(common_rngb.base + rng_sr) & 0x4;
	} while (err == 0);

	interrupt(RNGB_IRQ, rngb_intr, NULL, common_rngb.cond, NULL);

	/* Run self test */
	*(common_rngb.base + rng_cmd) = 0x1;
	mutexLock(common_rngb.lock);
	while (common_rngb.intr_st == 0) {
		condWait(common_rngb.cond, common_rngb.lock, 3000);
	}
	mutexUnlock(common_rngb.lock);

	if ((common_rngb.intr_st & ERROR_OCCURRED) != 0) {
		printf("rngb: self-test failed\n");

		systemCleanup(4);
		return -1;
	}

	common_rngb.intr_st = 0;

	/* Run seeding */
	*(common_rngb.base + rng_cmd) = 0x2;
	mutexLock(common_rngb.lock);
	while (common_rngb.intr_st == 0) {
		condWait(common_rngb.cond, common_rngb.lock, 3000);
	}
	mutexUnlock(common_rngb.lock);

	if ((common_rngb.intr_st & ERROR_OCCURRED) != 0) {
		printf("rngb: error after generating seed\n");

		systemCleanup(4);
		return -1;
	}

	common_rngb.intr_st = 0;

	/* Enable automatic seeding */
	*(common_rngb.base + rng_cr) = 0x10;

	printf("imx6ull-rngb: initialized\n");

	return 0;
}


static int readRandoms(char *buff, size_t size, unsigned mode)
{
	union {
		uint32_t val32;
		char val8[sizeof(uint32_t)];
	} reg;
	unsigned int i;
	int status;

	if (size == 0) {
		return 0;
	}

	mutexLock(common_rngb.lock);
	i = 0;
	while (i < size) {
		if ((i % sizeof(uint32_t)) == 0) {
			status = *(common_rngb.base + rng_sr);
			if (((status >> 16) & 1U) == 1) {
				mutexUnlock(common_rngb.lock);
				return -EIO;
			}
			if (((status >> 8) & 0xf) == 0) {
				if ((mode & O_NONBLOCK) != 0) {
					break;
				}
				/* wait for new values in FIFO */
				continue;
			}
			reg.val32 = *(common_rngb.base + rng_out);
		}
		buff[i] = reg.val8[i % sizeof(uint32_t)];
		i++;
	}
	mutexUnlock(common_rngb.lock);

	return i == 0 ? -EAGAIN : i;
}


static void handleMsg(void *arg)
{
	msg_t msg;
	unsigned long int rid;
	int err;

	while (1) {
		err = msgRecv(common_rngb.port, &msg, &rid);
		if (err < 0) {
			if (err == -EINTR) {
				continue;
			}
			else {
				printf("rngb: msgRecv returned error: %s\n", strerror(-err));
				break;
			}
		}

		switch (msg.type) {
			case mtClose:
			case mtOpen:
				msg.o.io.err = msg.i.openclose.oid.id != 0 ? -ENOENT : EOK;
				break;

			case mtRead:
				if (msg.o.data != NULL) {
					msg.o.io.err = readRandoms(msg.o.data, msg.o.size > 512 ? 512 : msg.o.size, msg.i.io.mode);
				}
				else {
					msg.o.io.err = -EINVAL;
				}
				break;

			default:
				msg.o.io.err = -EINVAL;
				break;
		}

		msgRespond(common_rngb.port, &msg, rid);
	}
}


int main(int argc, char **argv)
{
	oid_t root;
	(void)argc;
	(void)argv;

	while (lookup("/", NULL, &root) < 0) {
		usleep(10000);
	}

	if (systemInit() || hardwareInit()) {
		return EIO;
	}

	handleMsg(NULL);

	return 0;
}
