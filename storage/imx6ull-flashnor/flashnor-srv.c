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

#include <endian.h>
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
#include <ptable.h>
#include <storage/storage.h>

#include "imx6ull-flashnor-ecspi.h"
#include "imx6ull-flashnor-qspi.h"
#include "imx6ull-flashnor-drv.h"

#define LOG_TAG "imx6ull-flashnor: "

/* Default registered device files prefix and starting ID */
#define FLASHNOR_PREFIX "flashnor"
#define FLASHNOR_ID     0


static struct {
	char devicePath[32];
	char *devicePrefix;
	unsigned int deviceId;
} common = {
	.devicePrefix = FLASHNOR_PREFIX,
	.deviceId = FLASHNOR_ID,
};


static const storage_mtdops_t *getMtdOps(storage_t *storage)
{
	if ((storage == NULL) || (storage->dev == NULL) || (storage->dev->mtd == NULL)) {
		return NULL;
	}

	return storage->dev->mtd->ops;
}


static ssize_t doRead(storage_t *storage, off_t offs, void *dst, size_t len)
{
	if ((dst == NULL) || (offs < 0)) {
		return -EINVAL;
	}

	const storage_mtdops_t *ops = getMtdOps(storage);
	if ((ops == NULL) || (ops->read == NULL)) {
		return -EINVAL;
	}

	/* Read past the end of the device is not an error, report EOF */
	if ((size_t)offs >= storage->size) {
		return 0;
	}

	if (len > (storage->size - (size_t)offs)) {
		len = storage->size - (size_t)offs;
	}

	size_t retlen;
	const int err = ops->read(storage, storage->start + offs, dst, len, &retlen);
	if (err < 0) {
		return err;
	}

	return retlen;
}


static ssize_t flashnor_read(oid_t *oid, off_t offs, void *data, size_t len)
{
	return doRead(storage_get(oid->id), offs, data, len);
}


static ssize_t doWrite(storage_t *storage, off_t offs, const void *src, size_t len)
{
	if ((src == NULL) || (offs < 0)) {
		return -EINVAL;
	}

	const storage_mtdops_t *ops = getMtdOps(storage);
	if ((ops == NULL) || (ops->write == NULL) || (ops->erase == NULL)) {
		return -EINVAL;
	}

	const size_t blockSize = storage->dev->mtd->erasesz;
	if ((len != blockSize) || (offs % blockSize != 0)) {
		return -EOPNOTSUPP;
	}

	if (((size_t)offs >= storage->size) || (storage->size - (size_t)offs < len)) {
		return -ENOSPC;
	}

	offs += storage->start;
	int err = ops->erase(storage, offs, blockSize);
	if (err < 0) {
		return err;
	}

	size_t retlen;
	err = ops->write(storage, offs, src, len, &retlen);
	if (err < 0) {
		return err;
	}

	return retlen;
}


static ssize_t flashnor_write(oid_t *oid, off_t offs, const void *data, size_t len)
{
	return doWrite(storage_get(oid->id), offs, data, len);
}


static int flashnor_getAttr(oid_t *oid, int type, long long *attr)
{
	storage_t *strg = storage_get(oid->id);

	if (strg == NULL) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = (long long)strg->size;
			break;

		case atDev:
			*attr = (long long)strg->start;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static void flashnor_msgloop(void *arg, msg_t *msg)
{
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;
	(void)arg;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			msg->o.err = EOK;
			break;

		case mtRead:
			msg->o.err = flashnor_read(&msg->oid, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.err = flashnor_write(&msg->oid, msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtGetAttr:
			msg->o.err = flashnor_getAttr(&msg->oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountfs(storage_get(msg->oid.id), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.err = storage_umountfs(storage_get(msg->oid.id));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountpoint(storage_get(msg->oid.id), &omnt->oid);
			break;

		default:
			msg->o.err = -ENOSYS;
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
	(void)printf("\t\tstart:  partition start in bytes (erase block aligned)\n");
	(void)printf("\t\tsize:   partition size in bytes (erase block aligned)\n");
	(void)printf("\t-P <id>                      - read partition table from device\n");
	(void)printf("\t\tid:     device id\n");
	(void)printf("\t-r <dev> <fs>                - mount root filesystem\n");
	(void)printf("\t\tdev:    device node\n");
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


static storage_t *addPartition(storage_t *parent, ptable_part_t *part)
{
	storage_t *dev = calloc(1, sizeof(storage_t));
	int err;
	oid_t oid;

	(void)printf(LOG_TAG "adding partition: name = \"%.*s\", offset = 0x%08x, size = 0x%08x, type = 0x%02x (%s)\n",
			(int)sizeof(part->name), part->name, (unsigned int)part->offset, (unsigned int)part->size,
			(unsigned int)part->type, ptable_typeName(part->type));

	if (dev == NULL) {
		(void)printf(LOG_TAG "failed to allocate device, err: %s\n", strerror(ENOMEM));
		return NULL;
	}

	dev->start = part->offset;
	dev->size = part->size;
	dev->parent = parent;

	/* Only storage parameter matters in case of partition. */
	err = flashnor_drvInit(NULL, dev);
	if (err < 0) {
		(void)printf(LOG_TAG "failed to initialize device, err: %s\n", strerror(-err));
		free(dev);
		return NULL;
	}

	err = storage_add(dev, &oid);
	if (err < 0) {
		(void)printf(LOG_TAG "failed to create partition, err: %s\n", strerror(-err));
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}

	if (snprintf(common.devicePath, sizeof(common.devicePath), "/dev/%s%u.%.*s",
				common.devicePrefix, common.deviceId, (int)sizeof(part->name), part->name) >= sizeof(common.devicePath)) {
		(void)printf(LOG_TAG "failed to build partition file devicePath, err: %s\n", strerror(ENAMETOOLONG));
		(void)storage_remove(dev);
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}

	err = create_dev(&oid, common.devicePath);
	if (err < 0) {
		(void)printf(LOG_TAG "failed to create partition file, err: %s\n", strerror(-err));
		(void)storage_remove(dev);
		flashnor_drvDone(dev);
		free(dev);
		return NULL;
	}
	return dev;
}


static int flashnor_partsInit(int parentId)
{
	storage_t *parent = storage_get(parentId);
	if ((parent == NULL) || (parent->dev == NULL) || (parent->dev->mtd == NULL)) {
		(void)printf(LOG_TAG "failed to find parent device\n");
		return -1;
	}

	const size_t blockSize = parent->dev->mtd->erasesz;
	const off_t ptableOffset = parent->start + parent->size - blockSize;
	uint32_t partCount;
	if (doRead(parent, ptableOffset, &partCount, sizeof(partCount)) != sizeof(partCount)) {
		(void)printf(LOG_TAG "PTable: unable to read header from offset %zu\n", (size_t)ptableOffset);
		return -1;
	}
	partCount = le32toh(partCount);

	const uint32_t ptableSize = ptable_size(partCount);
	if (ptableSize > blockSize) {
		(void)printf(LOG_TAG "PTable: too large (%u, max: %zu)\n", (unsigned int)ptableSize, (size_t)blockSize);
		return -1;
	}

	{
		uint8_t magic[sizeof(ptable_magic)];
		const off_t magicOffset = ptableOffset + ptableSize - sizeof(magic);

		if (doRead(parent, magicOffset, magic, sizeof(magic)) != sizeof(magic)) {
			(void)printf(LOG_TAG "PTable: unable to read magic from offset %zu\n", (size_t)magicOffset);
			return -1;
		}

		if (memcmp(magic, ptable_magic, sizeof(magic)) != 0) {
			(void)printf(LOG_TAG "PTable: invalid magic number\n");
			return -1;
		}
	}


	ptable_t *ptable = malloc(ptableSize);
	if (ptable == NULL) {
		(void)printf(LOG_TAG "PTable: out of memory\n");
		return -1;
	}

	int ret = -1;
	if (doRead(parent, ptableOffset, ptable, ptableSize) != ptableSize) {
		(void)printf(LOG_TAG "PTable: unable to read contents\n");
		goto cleanup;
	}

	if (ptable_deserialize(ptable, parent->size, blockSize) < 0) {
		(void)printf(LOG_TAG "PTable: deserialization failed\n");
		goto cleanup;
	}

	(void)printf(LOG_TAG "detected partition table with %u entries\n", (unsigned int)partCount);
	for (uint32_t i = 0; i < partCount; ++i) {
		ptable_part_t *part = &ptable->parts[i];
		if (addPartition(parent, part) == NULL) {
			goto cleanup;
		}
	}

	ret = 0;
cleanup:
	free(ptable);
	return ret;
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
	flashnor_info_t info;
	int (*init)(int ndev, flashnor_info_t *info);
	const char *fsName;

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
		(void)printf(LOG_TAG "failed to initialize server, err: %s\n", strerror(-err));
		return EXIT_FAILURE;
	}

	err = storage_registerfs("meterfs", meterfs_mount, meterfs_umount);
	if (err < 0) {
		(void)printf(LOG_TAG "failed to register meterfs, err: %s\n", strerror(-err));
		return EXIT_FAILURE;
	}

	err = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (err < 0) {
		(void)printf(LOG_TAG "failed to register jffs2, err: %s\n", strerror(-err));
		return EXIT_FAILURE;
	}

	if (argc > 1) {
		while ((c = getopt_long(argc, argv, "e:q:P:r:n:h", longopts, NULL)) != -1) {
			switch (c) {
				case 'e':
				case 'q':
					init = (c == 'q') ? flashnor_qspiInit : flashnor_ecspiInit;
					dev = calloc(1, sizeof(storage_t));

					if (dev == NULL) {
						(void)printf(LOG_TAG "failed to allocate device, err: %s\n", strerror(ENOMEM));
						return EXIT_FAILURE;
					}

					err = init(strtoul(optarg, NULL, 0), &info);
					if (err < 0) {
						(void)printf(LOG_TAG "failed to initialize spi device, err: %s\n", strerror(-err));
						free(dev);
						return EXIT_FAILURE;
					}

					err = flashnor_drvInit(&info, dev);
					if (err < 0) {
						(void)printf(LOG_TAG "failed to initialize device, err: %s\n", strerror(-err));
						free(dev);
						return EXIT_FAILURE;
					}

					err = storage_add(dev, &oid);
					if (err < 0) {
						(void)printf(LOG_TAG "failed to register device, err: %s\n", strerror(-err));
						flashnor_drvDone(dev);
						free(dev);
						return EXIT_FAILURE;
					}

					++common.deviceId;
					if (snprintf(common.devicePath, sizeof(common.devicePath), "/dev/%s%u", common.devicePrefix, common.deviceId) >= sizeof(common.devicePath)) {
						(void)printf(LOG_TAG "failed to build device file devicePath, err: %s\n", strerror(ENAMETOOLONG));
						(void)storage_remove(dev);
						flashnor_drvDone(dev);
						free(dev);
						return EXIT_FAILURE;
					}

					err = create_dev(&oid, common.devicePath);
					if (err < 0) {
						(void)printf(LOG_TAG "failed to create device file, err: %s\n", strerror(-err));
						(void)storage_remove(dev);
						flashnor_drvDone(dev);
						free(dev);
						return EXIT_FAILURE;
					}
					break;

				case 'P':
					parentId = strtoul(optarg, NULL, 0);
					if (flashnor_partsInit(parentId) < 0) {
						return EXIT_FAILURE;
					}

					break;

				case 'r':
					arg = optind - 1;
					if ((arg + 2) > argc) {
						(void)printf(LOG_TAG "missing arg(s) for -r option, err: %s\n", strerror(EINVAL));
						return EXIT_FAILURE;
					}

					fsName = argv[arg + 1];
					err = lookup(optarg, NULL, &oid);
					if (err < 0) {
						(void)printf(LOG_TAG "lookup() failed for %s: %s\n", optarg, strerror(-err));
						return EXIT_FAILURE;
					}

					err = storage_mountfs(storage_get(oid.id), fsName, NULL, 0, NULL, &oid);
					if (err < 0) {
						(void)printf(LOG_TAG "failed to mount root filesystem, type %s: %s\n", fsName, strerror(-err));
						return EXIT_FAILURE;
					}

					portRegister(oid.port, "/", &oid);
					optind++;
					break;

				case 'n':
					arg = optind - 1;
					if ((arg + 2) > argc) {
						(void)printf(LOG_TAG "missing arg(s) for -n option, err: %s\n", strerror(EINVAL));
						return EXIT_FAILURE;
					}

					common.devicePrefix = argv[arg++];
					common.deviceId = strtoul(argv[arg++], NULL, 0);
					--common.deviceId;
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
	(void)printf(LOG_TAG "initialized\n");

	storage_run(1, 4 * 4096);

	return EXIT_SUCCESS;
}
