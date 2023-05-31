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

#include <libjffs2.h>
#include <meterfs_storage.h>
#include <storage/storage.h>

#include "imx6ull-flashnor-ecspi.h"
#include "imx6ull-flashnor-qspi.h"
#include "imx6ull-flashnor-drv.h"


/* Default registered device files prefix and starting ID */
#define FLASHNOR_PREFIX "flashnor"
#define FLASHNOR_ID     0


static char flashnor_path[32];
static char *flashnor_prefix = FLASHNOR_PREFIX;
static unsigned int flashnor_id = FLASHNOR_ID;


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
	(void)printf("\t-e <n> | --ecspi <n>         - initialize ECSPI NOR flash device\n");
	(void)printf("\t\tn:      ECSPI instance number\n");
	(void)printf("\t-q <n> | --qspi <n>          - initialize QuadSPI NOR flash device\n");
	(void)printf("\t\tn:      QuadSPI instance number\n");
	(void)printf("\t-p <id> <start> <size>       - register partition\n");
	(void)printf("\t\tid:     device id starting at 0\n");
	(void)printf("\t\tstart:  partition start in erase blocks\n");
	(void)printf("\t\tsize:   partition size in erase blocks\n");
	(void)printf("\t-r <id> <start> <size> <fs>  - mount root filesystem\n");
	(void)printf("\t\tid:     device id starting at 0\n");
	(void)printf("\t\tfs:     filesystem name\n");
	(void)printf("\t-n <prefix> <id>             - use given prefix and starting id for device files registration\n");
	(void)printf("\t\tprefix: device prefix\n");
	(void)printf("\t\tid:     device starting id, e.g. -n flash 4 registers devices /dev/flash4, /dev/flash5, etc.\n");
	(void)printf("\t-h | --help                  - print this help message\n");
}


static void flashnor_exit(int sig)
{
	(void)sig;

	exit(EXIT_SUCCESS);
}


static storage_t *flashnor_partAdd(int parentId, off_t start, size_t size)
{
	storage_t *dev = calloc(1, sizeof(storage_t)), *parent;
	int err;
	oid_t oid;

	if (dev == NULL) {
		err = -ENOMEM;
		(void)printf("imx6ull-flashnor: failed to allocate device, err: %s\n", strerror(err));
		return NULL;
	}
	parent = storage_get(parentId);
	if (parent == NULL) {
		err = -EINVAL;
		(void)printf("imx6ull-flashnor: failed to find parent device, err: %s\n", strerror(err));
		free(dev);
		return NULL;
	}

	dev->start = start;
	dev->size = size;
	dev->parent = parent;

	/* Only storage parameter matters in case of partition. */
	err = flashnor_drvInit(NULL, dev);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to initialize device, err: %s\n", strerror(err));
		free(dev);
		return NULL;
	}

	err = storage_add(dev, &oid);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to create partition, err: %s\n", strerror(err));
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}

	if (snprintf(flashnor_path, sizeof(flashnor_path), "/dev/%s%u", flashnor_prefix, flashnor_id++) >= sizeof(flashnor_path)) {
		err = -ENAMETOOLONG;
		(void)printf("imx6ull-flashnor: failed to build partition file flashnor_path, err: %s\n", strerror(err));
		(void)storage_remove(dev);
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}

	err = create_dev(&oid, flashnor_path);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to create partition file, err: %s\n", strerror(err));
		(void)storage_remove(dev);
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}
	return dev;
}


int main(int argc, char *argv[])
{
	struct option longopts[] = {
		{ "ecspi", required_argument, NULL, 'e' },
		{ "qspi", required_argument, NULL, 'q' },
		{ "help", no_argument, NULL, 'h' },
		{ NULL, 0, NULL, 0 }
	};
	int err, arg, c, parentId;
	storage_t *dev;
	pid_t pid;
	oid_t oid;
	off_t start;
	size_t size;
	flashnor_info_t info;
	int (*init)(int ndev, flashnor_info_t *info);
	char *fsName;

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
		(void)printf("imx6ull-flashnor: failed to register meterfs, err: %s\n", strerror(err));
		return EXIT_FAILURE;
	}

	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		(void)printf("imx6ull-flashnor: failed to register jffs2, err: %s\n", strerror(err));
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
						free(dev);
						return EXIT_FAILURE;
					}

					err = flashnor_drvInit(&info, dev);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to initialize device, err: %s\n", strerror(err));
						free(dev);
						return EXIT_FAILURE;
					}

					err = storage_add(dev, &oid);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to register device, err: %s\n", strerror(err));
						flashnor_drvDone(dev);
						free(dev);
						return EXIT_FAILURE;
					}

					if (snprintf(flashnor_path, sizeof(flashnor_path), "/dev/%s%u", flashnor_prefix, flashnor_id++) >= sizeof(flashnor_path)) {
						err = -ENAMETOOLONG;
						(void)printf("imx6ull-flashnor: failed to build device file flashnor_path, err: %s\n", strerror(err));
						(void)storage_remove(dev);
						flashnor_drvDone(dev);
						free(dev);
						return EXIT_FAILURE;
					}

					err = create_dev(&oid, flashnor_path);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to create device file, err: %s\n", strerror(err));
						(void)storage_remove(dev);
						flashnor_drvDone(dev);
						free(dev);
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

					parentId = strtoul(argv[arg], NULL, 0);
					start = strtoull(argv[arg + 1], NULL, 0);
					size = strtoull(argv[arg + 2], NULL, 0);

					dev = flashnor_partAdd(parentId, start, size);
					if (dev == NULL) {
						return EXIT_FAILURE;
					}
					optind += 2;
					break;

				case 'r':
					arg = optind - 1;
					if ((arg + 4) > argc) {
						err = -EINVAL;
						(void)printf("imx6ull-flashnor: missing arg(s) for -r option, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					parentId = strtoul(argv[arg], NULL, 0);
					start = strtoull(argv[arg + 1], NULL, 0);
					size = strtoull(argv[arg + 2], NULL, 0);
					fsName = argv[arg + 3];

					dev = flashnor_partAdd(parentId, start, size);
					if (dev == NULL) {
						return EXIT_FAILURE;
					}

					err = storage_mountfs(dev, fsName, NULL, 0, NULL, &oid);
					if (err < 0) {
						(void)printf("imx6ull-flashnor: failed to mount a filesystem - %s: %s\n", fsName, strerror(err));
						return EXIT_FAILURE;
					}

					portRegister(oid.port, "/", &oid);
					optind += 3;
					break;

				case 'n':
					arg = optind - 1;
					if ((arg + 2) > argc) {
						err = -EINVAL;
						(void)printf("imx6ull-flashnor: missing arg(s) for -n option, err: %s\n", strerror(err));
						return EXIT_FAILURE;
					}

					flashnor_prefix = argv[arg++];
					flashnor_id = strtoul(argv[arg++], NULL, 0);
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
