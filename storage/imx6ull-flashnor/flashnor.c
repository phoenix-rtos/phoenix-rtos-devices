/*
 * Phoenix-RTOS
 *
 * i.MX 6ULL NOR flash server
 *
 * Copyright 2021, 2023 Phoenix Systems
 * Author: Lukasz Kosinski, Hubert Badocha
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
#include <string.h>
#include <unistd.h>

#include <sys/file.h>

#include <posix/utils.h>

#include <meterfs_storage.h>
#include <storage/storage.h>

#include "flashnor-ecspi.h"
#include "flashnor-qspi.h"
#include "flashnor-drv.h"


/* Default registered device files prefix and starting ID */
#define FLASHNOR_PREFIX "flashnor"
#define FLASHNOR_ID     0

static void flashnor_msgloop(void *arg, msg_t *msg)
{
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;
	(void)arg;

	switch (msg->type) {
		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			omnt->err = storage_mountfs(storage_get(imnt->dev.id), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.io.err = storage_umountfs(storage_get(((oid_t *)msg->i.data)->id));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			omnt->err = storage_mountpoint(storage_get(((oid_t *)msg->i.data)->id), &omnt->oid);
			break;

		default:
			msg->o.io.err = -EINVAL;
			break;
	}
}


static void flashnor_help(const char *prog)
{
	(void)printf("Usage: %s [options] or no args to automatically detect and initialize all NOR flash devices\n", prog);
	(void)printf("\t-e <n> | --ecspi <n>   - initialize ECSPI NOR flash device\n");
	(void)printf("\t\tn:      ECSPI instance number\n");
	(void)printf("\t-q <n> | --qspi <n>    - initialize QuadSPI NOR flash device\n");
	(void)printf("\t\tn:      QuadSPI instance number\n");
	(void)printf("\t-p <id> <start> <size> - register partition\n");
	(void)printf("\t\tid:     device id starting at 0\n");
	(void)printf("\t\tstart:  partition start in erase blocks\n");
	(void)printf("\t\tsize:   partition size in erase blocks\n");
	(void)printf("\t-r <id> <fs>           - mount root filesystem\n");
	(void)printf("\t\tid:     device id starting at 0\n");
	(void)printf("\t\tfs:     filesystem name\n");
	(void)printf("\t-n <prefix> <id>       - use given prefix and starting id for device files registration\n");
	(void)printf("\t\tprefix: device prefix\n");
	(void)printf("\t\tid:     device starting id, e.g. -n flash 4 registers devices /dev/flash4, /dev/flash5, etc.\n");
	(void)printf("\t-h | --help            - print this help message\n");
}


static void flashnor_exit(int sig)
{
	(void)sig;

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
	unsigned int id = FLASHNOR_ID;
	char *prefix = FLASHNOR_PREFIX;
	char path[32];
	int err, arg, c;
	storage_t *dev, *parent;
	pid_t pid;
	oid_t oid;
	flashnor_info_t info;
	int (*init)(int ndev, flashnor_info_t *info);

	/* Wait for console */
	while (write(1, "", 0) < 0) {
		usleep(10000);
	}

	/* Wait for rootfs */
	while (lookup("/", NULL, &oid) < 0) {
		usleep(10000);
	}

	/* Daemonize server */
	signal(SIGUSR1, flashnor_exit);

	pid = fork();
	if (pid < 0) {
		return EXIT_FAILURE;
	}

	if (pid > 0) {
		sleep(10);
		return EXIT_FAILURE;
	}

	signal(SIGUSR1, flashnor_exit);

	if (setsid() < 0) {
		return EXIT_FAILURE;
	}

	err = storage_init(flashnor_msgloop, 16);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to initialize server, err: %s\n", strerror(err));
		return EXIT_FAILURE;
	}

	err = storage_registerfs("meterfs", meterfs_mount, meterfs_umount);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to register filesystem, err: %s\n", strerror(err));
		return EXIT_FAILURE;
	}

	if (argc > 1) {
		while ((c = getopt_long(argc, argv, "e:q:p:r:n:h", longopts, NULL)) != -1) {
			switch (c) {
				case 'e':
				case 'q':
					init = (c == 'q') ? flashnor_qspiInit : flashnor_ecspiInit;
					dev = calloc(1, sizeof(storage_t));

					if (dev == NULL) {
						err = -ENOMEM;
						(void)printf("imx6ull-flashnor: failed to allocate device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					err = init(strtoul(optarg, NULL, 0), &info);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to initialize spi device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					err = flashnor_drvInit(&info, dev);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to initialize device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					err = storage_add(dev, &oid);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to register device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					if (snprintf(path, sizeof(path), "/dev/%s%u", prefix, id++) >= sizeof(path)) {
						err = -ENAMETOOLONG;
						(void)printf("imx6ull-flashnor: failed to build device file path, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					err = create_dev(&oid, path);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to create device file, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}
					break;

				case 'p':
					arg = optind - 1;
					if ((arg + 3) > argc) {
						err = -EINVAL;
						(void)printf("imx6ull-flashnor: missing arg(s) for -p option, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					dev = calloc(1, sizeof(storage_t));
					if (dev == NULL) {
						err = -ENOMEM;
						(void)printf("imx6ull-flashnor: failed to allocate device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}
					parent = storage_get(strtoul(argv[arg++], NULL, 0));
					if (parent == NULL) {
						err = -EINVAL;
						(void)printf("imx6ull-flashnor: failed to find parent device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					dev->start = strtoll(argv[arg++], NULL, 0) * parent->dev->mtd->erasesz;
					dev->size = strtoll(argv[arg++], NULL, 0) * parent->dev->mtd->erasesz;
					dev->parent = parent;

					/* Only storage parameter matters in case of partition. */
					err = flashnor_drvInit(NULL, dev);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to initialize device, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}
					optind += 2;

					err = storage_add(dev, &oid);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to create partition, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					if (snprintf(path, sizeof(path), "/dev/%s%u", prefix, id++) >= sizeof(path)) {
						err = -ENAMETOOLONG;
						(void)printf("imx6ull-flashnor: failed to build partition file path, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					err = create_dev(&oid, path);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to create partition file, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}
					break;

				case 'r':
					/* TODO: add rootfs mount and register support */
					return -ENOTSUP;

				case 'n':
					arg = optind - 1;
					if ((arg + 2) > argc) {
						err = -EINVAL;
						(void)printf("imx6ull-flashnor: missing arg(s) for -n option, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					prefix = argv[arg++];
					id = strtoul(argv[arg++], NULL, 0);
					optind++;
					break;

				case 'h':
				default:
					flashnor_help(argv[0]);
					return EXIT_SUCCESS;
			}
		}
	}
	else {
		/* TODO: detect and initialize all NOR flash devices */
	}

	kill(getppid(), SIGUSR1);
	(void)printf("imx6ull-flashnor: initialized\n");

	storage_run(1, 4 * 4096);

	return EXIT_SUCCESS;
}
