/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL NOR flash server
 *
 * Copyright 2021 Phoenix Systems
 * Author: Lukasz Kosinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <getopt.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/file.h>

#include <posix/utils.h>

#include <meterfs.h>

#include "flashnor-ecspi.h"
#include "storage.h"


/* Default registered device files prefix and starting ID */
#define FLASHNOR_PREFIX "flashnor"
#define FLASHNOR_ID     0


static int meterfs_mount(void *ctx, void **fs, const char *data, unsigned long mode, oid_t *root)
{
	root->id = 0;
	*fs = ctx;
	return EOK;
}


static int meterfs_umount(void *fs)
{
	return EOK;
}


static void meterfs_handler(void *arg, msg_t *msg)
{
	meterfs_ctx_t *ctx = (meterfs_ctx_t *)arg;
	meterfs_i_devctl_t *idevctl = (meterfs_i_devctl_t *)msg->i.raw;
	meterfs_o_devctl_t *odevctl = (meterfs_o_devctl_t *)msg->o.raw;

	switch (msg->type) {
		case mtOpen:
			msg->o.io.err = meterfs_open(msg->i.openclose.oid.id, ctx);
			break;

		case mtClose:
			msg->o.io.err = meterfs_close(msg->i.openclose.oid.id, ctx);
			break;

		case mtRead:
			msg->o.io.err = meterfs_readFile(msg->i.io.oid.id, msg->i.io.offs, msg->o.data, msg->o.size, ctx);
			break;

		case mtWrite:
			msg->o.io.err = meterfs_writeFile(msg->i.io.oid.id, msg->i.data, msg->i.size, ctx);
			break;

		case mtLookup:
			msg->o.lookup.err = meterfs_lookup(msg->i.data, &msg->o.lookup.fil.id, ctx);
			break;

		case mtDevCtl:
			odevctl->err = meterfs_devctl(idevctl, odevctl, ctx);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void flashnor_msgloop(void *arg, msg_t *msg)
{
	mount_msg_t *mnt;

	switch (msg->type) {
		case mtMount:
			mnt = (mount_msg_t *)msg->i.raw;
			storage_mountfs(storage_get(mnt->id), mnt->fstype, NULL, 0, (oid_t *)msg->o.raw);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void flashnor_help(const char *prog)
{
	printf("Usage: %s [options] or no args to automatically detect and initialize all NOR flash devices\n", prog);
	printf("\t-e <n> | --ecspi <n>   - initialize ECSPI NOR flash device\n");
	printf("\t\tn:      ECSPI instance number\n");
	printf("\t-q <n> | --qspi <n>    - initialize QuadSPI NOR flash device\n");
	printf("\t\tn:      QuadSPI instance number\n");
	printf("\t-p <id> <start> <size> - register partition\n");
	printf("\t\tid:     device id starting at 0\n");
	printf("\t\tstart:  partition start in erase blocks\n");
	printf("\t\tsize:   partition size in erase blocks\n");
	printf("\t-r <id> <fs>           - mount root filesystem\n");
	printf("\t\tid:     device id starting at 0\n");
	printf("\t\tfs:     filesystem name\n");
	printf("\t-n <prefix> <id>       - use given prefix and starting id for device files registration\n");
	printf("\t\tprefix: device prefix\n");
	printf("\t\tid:     device starting id, e.g. -n flash 4 registers devices /dev/flash4, /dev/flash5, etc.\n");
	printf("\t-h | --help            - print this help message\n");
}


static void flashnor_exit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char *argv[])
{
	struct option longopts[] = {
		{ "ecspi", required_argument, NULL, 'e' },
		{ "qspi", required_argument, NULL, 'q' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	char path[32], *prefix = FLASHNOR_PREFIX;
	unsigned int port, id = FLASHNOR_ID;
	int err, arg, c;
	storage_t *dev, *parent;
	oid_t oid;
	pid_t pid;

	/* Wait for console */
	while (write(1, "", 0) < 0)
		usleep(10000);

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0)
		usleep(10000);

	/* Daemonize server */
	signal(SIGUSR1, flashnor_exit);

	if ((pid = fork()) < 0)
		exit(EXIT_FAILURE);

	if (pid > 0) {
		sleep(10);
		exit(EXIT_FAILURE);
	}

	signal(SIGUSR1, flashnor_exit);

	if (setsid() < 0)
		exit(EXIT_FAILURE);

	if ((err = storage_init(flashnor_msgloop, 16)) < 0) {
		fprintf(stderr, "imx6ull-flashnor: failed to initialize server, err: %d\n", err);
		return err;
	}
	port = err;

	if ((err = storage_registerfs("meterfs", meterfs_mount, meterfs_umount, meterfs_handler)) < 0) {
		fprintf(stderr, "imx6ull-flashnor: failed to register filesystem, err: %d\n", err);
		return err;
	}

	if (argc > 1) {
		while ((c = getopt_long(argc, argv, "e:q:p:r:n:h", longopts, NULL)) != -1) {
			switch (c) {
				case 'e':
					if ((dev = malloc(sizeof(storage_t))) == NULL) {
						err = -ENOMEM;
						fprintf(stderr, "imx6ull-flashnor: failed to allocate device, err: %d\n", err);
						return err;
					}

					if ((err = flashnor_ecspiInit(strtoul(optarg, NULL, 0), dev)) < 0) {
						fprintf(stderr, "imx6ull-flashnor: failed to initialize device, err: %d\n", err);
						return err;
					}

					if ((err = storage_add(dev)) < 0) {
						fprintf(stderr, "imx6ull-flashnor: failed to register device, err: %d\n", err);
						return err;
					}

					oid.port = port;
					oid.id = err;
					if (snprintf(path, sizeof(path), "/dev/%s%u", prefix, id++) >= sizeof(path)) {
						err = -ENAMETOOLONG;
						fprintf(stderr, "imx6ull-flashnor: failed to build device file path, err: %d\n", err);
						return err;
					}

					if ((err = create_dev(&oid, path)) < 0) {
						fprintf(stderr, "imx6ull-flashnor: failed to create device file, err: %d\n", err);
						return err;
					}
					break;

				case 'q':
					/* TODO: add QuadSPI NOR flash support */
					return -ENOTSUP;

				case 'p':
					if ((arg = optind - 1) + 3 > argc) {
						err = -EINVAL;
						fprintf(stderr, "imx6ull-flashnor: missing arg(s) for -p option, err: %d\n", err);
						return err;
					}

					if ((dev = malloc(sizeof(storage_t))) == NULL) {
						err = -ENOMEM;
						fprintf(stderr, "imx6ull-flashnor: failed to allocate device, err: %d\n", err);
						return err;
					}

					if ((parent = storage_get(strtoul(argv[arg++], NULL, 0))) == NULL) {
						err = -EINVAL;
						fprintf(stderr, "imx6ull-flashnor: failed to find parent device, err: %d\n", err);
						return err;
					}

					dev->start = strtoll(argv[arg++], NULL, 0);
					dev->size = strtoll(argv[arg++], NULL, 0);
					dev->ctx = parent->ctx;
					dev->parent = parent;
					optind += 2;

					if ((err = storage_add(dev)) < 0) {
						fprintf(stderr, "imx6ull-flashnor: failed to create partition, err: %d\n", err);
						return err;
					}

					oid.port = port;
					oid.id = err;
					if (snprintf(path, sizeof(path), "/dev/%s%u", prefix, id++) >= sizeof(path)) {
						err = -ENAMETOOLONG;
						fprintf(stderr, "imx6ull-flashnor: failed to build partition file path, err: %d\n", err);
						return err;
					}

					if ((err = create_dev(&oid, path)) < 0) {
						fprintf(stderr, "imx6ull-flashnor: failed to create partition file, err: %d\n", err);
						return err;
					}
					break;

				case 'r':
					/* TODO: add rootfs mount and register support */
					return -ENOTSUP;

				case 'n':
					if ((arg = optind - 1) + 2 > argc) {
						err = -EINVAL;
						fprintf(stderr, "imx6ull-flashnor: missing arg(s) for -n option, err: %d\n", err);
						return err;
					}

					prefix = argv[arg++];
					id = strtoul(argv[arg++], NULL, 0);
					optind++;
					break;

				case 'h':
				default:
					flashnor_help(argv[0]);
					return EOK;
			}
		}
	}
	else {
		/* TODO: detect and initialize all NOR flash devices */
	}

	kill(getppid(), SIGUSR1);
	storage_run(1, 4 * 4096);

	return EOK;
}
