/*
 * Phoenix-RTOS
 *
 * Zynq-7000 NOR flash server
 *
 * Copyright 2022 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <sys/file.h>
#include <string.h>
#include <sys/stat.h>
#include <posix/utils.h>

#include <libjffs2.h>
#include <mtd/mtd.h>
#include <storage/storage.h>

#include "flashdrv.h"
#include "zynq-flash.h"

#define MTD_POS   (29)
#define MTD_MASK  (3 << MTD_POS)
#define MTD_CHAR  (1 << MTD_POS)
#define MTD_BLOCK (2 << MTD_POS)

#define GET_STORAGE_ID(id) (id & ~MTD_MASK)
#define GET_MTD_TYPE(id)   (id & MTD_MASK)

#define LIMITED_PORT_STACK_SIZE (1024U)

typedef struct {
	id_t targetId;
	struct timespec lastOp;
	size_t readbwBudget;
	size_t readmaxBudget;
	size_t writebwBudget;
	size_t writemaxBudget;
	size_t rateBudget;
	size_t maxRate;
} portlimit_data_t;


/* Operations on flash memory device */

static ssize_t flash_read(oid_t *oid, off_t offs, void *buff, size_t len)
{
	size_t retlen;
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL || (offs + len) > strg->size || buff == NULL) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if (strg->dev->mtd != NULL && strg->dev->mtd->ops != NULL && strg->dev->mtd->ops->read != NULL && GET_MTD_TYPE(oid->id) == MTD_CHAR) {
		res = strg->dev->mtd->ops->read(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->read != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK) {
		res = strg->dev->blk->ops->read(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static ssize_t flash_write(oid_t *oid, off_t offs, const void *buff, size_t len)
{
	size_t retlen;
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL || (offs + len) > strg->size) {
		res = -EINVAL;
	}
	else if (len == 0) {
		res = 0;
	}
	else if (strg->dev->mtd != NULL && strg->dev->mtd->ops != NULL && strg->dev->mtd->ops->write != NULL && GET_MTD_TYPE(oid->id) == MTD_CHAR && buff != NULL) {
		res = strg->dev->mtd->ops->write(strg, strg->start + offs, buff, len, &retlen);

		if (retlen > 0) {
			res = retlen;
		}
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->write != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK && buff != NULL) {
		res = strg->dev->blk->ops->write(strg, strg->start + offs, buff, len);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int flash_sync(oid_t *oid)
{
	ssize_t res = -ENOSYS;
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || strg->dev == NULL) {
		res = -EINVAL;
	}
	else if (GET_MTD_TYPE(oid->id) == MTD_CHAR) {
		res = -ENOSYS;
	}
	else if (strg->dev->blk != NULL && strg->dev->blk->ops != NULL && strg->dev->blk->ops->sync != NULL && GET_MTD_TYPE(oid->id) == MTD_BLOCK) {
		res = strg->dev->blk->ops->sync(strg);
	}
	else {
		res = -EINVAL;
	}

	return res;
}


static int flash_getAttr(oid_t *oid, int type, long long *attr)
{
	storage_t *strg = storage_get(GET_STORAGE_ID(oid->id));

	if (strg == NULL || attr == NULL) {
		return -EINVAL;
	}

	switch (type) {
		case atSize:
			*attr = strg->size;
			break;

		default:
			return -EINVAL;
	}

	return EOK;
}


static int flash_erase(storage_t *strg, off_t offs, size_t size)
{
	if (strg->dev->mtd->ops->erase == NULL) {
		return -EINVAL;
	}

	if (size == 0) {
		return EOK;
	}

	if (offs < 0 || (off_t)(offs + size) > (off_t)strg->size) {
		return -EINVAL;
	}

	return strg->dev->mtd->ops->erase(strg, strg->start + offs, size);
}


static int flash_info(storage_t *strg, flash_o_devctl_t *out)
{
	out->info.size = strg->size;
	out->info.type = strg->dev->mtd->type;
	out->info.erasesz = strg->dev->mtd->erasesz;
	out->info.writesz = strg->dev->mtd->writesz;
	out->info.erasesz = strg->dev->mtd->erasesz;
	out->info.writesz = strg->dev->mtd->writesz;
	out->info.writeBuffsz = strg->dev->mtd->writeBuffsz;
	out->info.metaSize = strg->dev->mtd->metaSize;
	out->info.oobSize = strg->dev->mtd->oobSize;
	out->info.oobAvail = strg->dev->mtd->oobAvail;

	return EOK;
}


static int flash_devCtl(msg_t *msg)
{
	const flash_i_devctl_t *idevctl = (const flash_i_devctl_t *)msg->i.raw;
	storage_t *strg = storage_get(GET_STORAGE_ID(msg->oid.id));

	if (strg == NULL || strg->dev == NULL || strg->dev->mtd == NULL || strg->dev->mtd->ops == NULL) {
		return -EINVAL;
	}

	switch (idevctl->type) {
		case flashsrv_devctl_erase:
			return flash_erase(strg, (off_t)idevctl->erase.address, idevctl->erase.size);

		case flashsrv_devctl_info:
			return flash_info(strg, (flash_o_devctl_t *)msg->o.raw);

		default:
			return -ENOSYS;
	}
}


static void flash_msgHandler(void *arg, msg_t *msg)
{
	storage_t *strg;
	mount_i_msg_t *imnt;
	mount_o_msg_t *omnt;

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			strg = storage_get(GET_STORAGE_ID(msg->oid.id));
			msg->o.err = (strg == NULL) ? -EINVAL : EOK;
			break;

		case mtRead:
			msg->o.err = flash_read(&msg->oid, msg->i.io.offs, msg->o.data, msg->o.size);
			break;

		case mtWrite:
			msg->o.err = flash_write(&msg->oid, msg->i.io.offs, msg->i.data, msg->i.size);
			break;

		case mtSync:
			msg->o.err = flash_sync(&msg->oid);
			break;

		case mtGetAttr:
			msg->o.err = flash_getAttr(&msg->oid, msg->i.attr.type, &msg->o.attr.val);
			break;

		case mtMount:
			imnt = (mount_i_msg_t *)msg->i.raw;
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountfs(storage_get(GET_STORAGE_ID(msg->oid.id)), imnt->fstype, msg->i.data, imnt->mode, &imnt->mnt, &omnt->oid);
			break;

		case mtUmount:
			msg->o.err = storage_umountfs(storage_get(GET_STORAGE_ID(msg->oid.id)));
			break;

		case mtMountPoint:
			omnt = (mount_o_msg_t *)msg->o.raw;
			msg->o.err = storage_mountpoint(storage_get(GET_STORAGE_ID(msg->oid.id)), &omnt->oid);
			break;

		case mtDevCtl:
			msg->o.err = flash_devCtl(msg);
			break;

		default:
			msg->o.err = -ENOSYS;
			break;
	}
}


static void flash_help(const char *prog)
{
	printf("Usage: %s [options] or no args to automatically detect and initialize a NOR flash device\n", prog);
	printf("\t-p <dev:start:size>          - register partition\n");
	printf("\t\tdev:    device path\n");
	printf("\t\tstart:  partition start in bytes\n");
	printf("\t\tsize:   partition size in bytes\n");
	printf("\t-r <dev:start:size:fs>       - mount root filesystem\n");
	printf("\t\tdev:    device name\n");
	printf("\t\tstart:  partition start in bytes\n");
	printf("\t\tsize:   partition size in bytes\n");
	printf("\t\tfs:     filesystem name\n");
	printf("\t-h                           - print this help message\n");
}


static int flash_limitShared(void *data, msg_t *msg)
{
	portlimit_data_t *limitData = (portlimit_data_t *)data;
	size_t opSize = msg->i.size + msg->o.size;
	struct timespec t1;
	time_t elapsedNs;
	size_t renewal;

	if (limitData->targetId != (id_t)-1) {
		if (msg->oid.id != 0U) {
			return -EPERM;
		}
		msg->oid.id = limitData->targetId;
	}

	if ((opSize > limitData->writemaxBudget && msg->type == mtWrite) ||
			(opSize > limitData->readmaxBudget && msg->type == mtRead)) {
		return -EFBIG;
	}

	do {
		clock_gettime(CLOCK_MONOTONIC, &t1);

		elapsedNs = (t1.tv_sec - limitData->lastOp.tv_sec) * 1000000000LL +
				(t1.tv_nsec - limitData->lastOp.tv_nsec);
		if (elapsedNs < 0) {
			elapsedNs = 0;
		}

		limitData->lastOp = t1;

		renewal = (size_t)(((time_t)limitData->readmaxBudget * elapsedNs) / 1000000000LL);
		limitData->readbwBudget += renewal;
		if (limitData->readbwBudget > limitData->readmaxBudget) {
			limitData->readbwBudget = limitData->readmaxBudget;
		}

		renewal = (size_t)(((time_t)limitData->writemaxBudget * elapsedNs) / 1000000000LL);
		limitData->writebwBudget += renewal;
		if (limitData->writebwBudget > limitData->writemaxBudget) {
			limitData->writebwBudget = limitData->writemaxBudget;
		}

		renewal = (size_t)(((time_t)limitData->maxRate * elapsedNs) / 1000000000LL);
		limitData->rateBudget += renewal;
		if (limitData->rateBudget > limitData->maxRate) {
			limitData->rateBudget = limitData->maxRate;
		}

		if (msg->type == mtRead && opSize > limitData->readbwBudget) {
			size_t need = opSize - limitData->readbwBudget;
			time_t sleepNs = (limitData->readmaxBudget != 0U) ?
					(((time_t)need * 1000000000LL) / (time_t)limitData->readmaxBudget) :
					0;
			struct timespec ts = { .tv_sec = sleepNs / 1000000000LL, .tv_nsec = sleepNs % 1000000000LL };
			nanosleep(&ts, NULL);
			continue;
		}

		if (msg->type == mtWrite && opSize > limitData->writebwBudget) {
			size_t need = opSize - limitData->writebwBudget;
			time_t sleepNs = (limitData->writemaxBudget != 0U) ?
					(((time_t)need * 1000000000LL) / (time_t)limitData->writemaxBudget) :
					0;
			struct timespec ts = { .tv_sec = sleepNs / 1000000000LL, .tv_nsec = sleepNs % 1000000000LL };
			nanosleep(&ts, NULL);
			continue;
		}

		if (limitData->rateBudget < 1U) {
			size_t need = 1U - limitData->rateBudget;
			time_t sleepNs = (limitData->maxRate != 0U) ?
					(((time_t)need * 1000000000LL) / (time_t)limitData->maxRate) :
					0;
			struct timespec ts = { .tv_sec = sleepNs / 1000000000LL, .tv_nsec = sleepNs % 1000000000LL };
			nanosleep(&ts, NULL);
			continue;
		}

	} while ((msg->type == mtRead && opSize > limitData->readbwBudget) ||
			(msg->type == mtWrite && opSize > limitData->writebwBudget) ||
			limitData->rateBudget < 1U);

	if (msg->type == mtRead) {
		limitData->readbwBudget -= opSize;
	}
	if (msg->type == mtWrite) {
		limitData->writebwBudget -= opSize;
	}
	limitData->rateBudget -= 1U;

	return EOK;
}


static int flash_oidResolve(const char *devPath, oid_t *oid)
{
	int res;
	char temp[32];

	if (strncmp("/dev/", devPath, 5) != 0) {
		return -EINVAL;
	}

	res = snprintf(temp, sizeof(temp), "devfs/%s", devPath + 5);
	if (res >= sizeof(temp)) {
		res = -ENAMETOOLONG;
		fprintf(stderr, "zynq-flash: failed to build file path, err: %d\n", res);
		return res;
	}

	return lookup(temp, NULL, oid);
}


static int flash_createMtdDev(const storage_t *strg, oid_t *oid)
{
	int res;
	char path[32];
	storage_t *part;
	unsigned int partID = 0;

	/* Find id for a new partition */
	if (strg->parent != NULL) {
		part = strg->parent->parts;
		if (part != NULL) {
			do {
				partID++;
				part = part->next;
			} while (part != strg->parent->parts);
		}
	}

	/* Add mtdchar device */
	if (strg->dev->mtd != NULL) {
		oid->id &= ~MTD_MASK;
		oid->id |= MTD_CHAR;

		res = snprintf(path, sizeof(path), "/dev/mtd%u", strg->dev->ctx->id);
		if (res >= sizeof(path)) {
			res = -ENAMETOOLONG;
			fprintf(stderr, "zynq-flash: failed to build file path, err: %d\n", res);
			return res;
		}

		if (strg->parent != NULL) {
			res += snprintf(path + res, sizeof(path) - res, "p%u", partID);
			if (res >= sizeof(path)) {
				res = -ENAMETOOLONG;
				fprintf(stderr, "zynq-flash: failed to build file path, err: %d\n", res);
				return res;
			}
		}

		res = create_dev(oid, path);
		if (res < 0) {
			fprintf(stderr, "zynq-flash: failed to create a device file, err: %d\n", res);
			return res;
		}
	}

	/* Add mtdblock device */
	if (strg->dev->blk != NULL) {
		oid->id &= ~MTD_MASK;
		oid->id |= MTD_BLOCK;

		res = snprintf(path, sizeof(path), "/dev/mtdblock%u", strg->dev->ctx->id);
		if (res >= sizeof(path)) {
			res = -ENAMETOOLONG;
			fprintf(stderr, "zynq-flash: failed to build file path, err: %d\n", res);
			return res;
		}

		if (strg->parent != NULL) {
			res += snprintf(path + res, sizeof(path) - res, "p%u", partID);
			if (res >= sizeof(path)) {
				res = -ENAMETOOLONG;
				fprintf(stderr, "zynq-flash: failed to build file path, err: %d\n", res);
				return res;
			}
		}

		res = create_dev(oid, path);
		if (res < 0) {
			fprintf(stderr, "zynq-flash: failed to create a device file, err: %d\n", res);
			return res;
		}
	}

	return EOK;
}


static int flash_partAdd(const char *parentPath, off_t start, size_t size)
{
	int err;
	oid_t poid, oid;
	storage_t *strg, *parent;

	err = flash_oidResolve(parentPath, &poid);
	if (err < 0) {
		fprintf(stderr, "zynq-flash: cannot resolve %s\n", parentPath);
		return err;
	}

	parent = storage_get(GET_STORAGE_ID(poid.id));
	if (parent == NULL) {
		err = -EINVAL;
		fprintf(stderr, "zynq-flash: failed to find a parent %s, err: %d\n", parentPath, err);
		return err;
	}

	strg = malloc(sizeof(storage_t));
	if (strg == NULL) {
		err = -ENOMEM;
		fprintf(stderr, "zynq-flash: failed to allocate a device, err: %d\n", err);
		return err;
	}

	strg->start = parent->start + start;
	strg->size = size;
	strg->parent = parent;
	strg->dev = parent->dev;
	strg->parts = NULL;

	err = storage_add(strg, &oid);
	if (err < 0) {
		free(strg);
		fprintf(stderr, "zynq-flash: failed to create a partition, err: %d\n", err);
		return err;
	}

	err = flash_createMtdDev(strg, &oid);
	if (err < 0) {
		storage_remove(strg);
		free(strg);
		return err;
	}

	return GET_STORAGE_ID(oid.id);
}


static int flash_sharedPortAdd(const char *portName, const char *devPath, size_t bwRead, size_t bwWrite, size_t rate, unsigned int reqthrpriority)
{
	oid_t port, strgOid;
	storage_t *strg = NULL;
	portlimit_data_t *limitData;
	int err;

	err = sys_namedResource(portName, strlen(portName), &port.port);
	if (err < 0) {
		fprintf(stderr, "zynq-flash: failed to find a named port %s, err: %d\n", portName, err);
		return err;
	}

	err = flash_oidResolve(devPath, &strgOid);
	if (err < 0) {
		fprintf(stderr, "zynq-flash: cannot resolve %s\n", devPath);
		return err;
	}

	strg = storage_get(GET_STORAGE_ID(strgOid.id));
	if (strg == NULL) {
		fprintf(stderr, "zynq-flash: failed to find storage %s, err: %d\n", devPath, err);
		return -EINVAL;
	}

	limitData = malloc(sizeof(portlimit_data_t));
	if (limitData == NULL) {
		fprintf(stderr, "zynq-flash: failed to allocate port limit data, err: %d\n", err);
		return -ENOMEM;
	}

	limitData->targetId = strgOid.id;
	limitData->readbwBudget = bwRead;
	limitData->readmaxBudget = bwRead;
	limitData->writebwBudget = bwWrite;
	limitData->writemaxBudget = bwWrite;
	limitData->rateBudget = rate;
	limitData->maxRate = rate;

	clock_gettime(CLOCK_MONOTONIC, &limitData->lastOp);

	err = storage_bindLimitedPort(port.port, flash_limitShared, limitData, reqthrpriority, LIMITED_PORT_STACK_SIZE);

	if (err < 0) {
		fprintf(stderr, "zynq-flash: failed to bind limited port, err: %d\n", err);
		free(limitData);
		return err;
	}

	return EOK;
}


static int flash_optsParse(int argc, char **argv)
{
	int err, c;
	unsigned int id;
	oid_t oid;
	off_t start;
	size_t size, rbw, wbw, rate;
	unsigned int prio;
	char *devPath, *arg, *fs, *portName, *part;

	while ((c = getopt(argc, argv, "p:r:n:h")) != -1) {
		err = -EINVAL;
		switch (c) {
			case 'p': /* <dev:start:size> */
				devPath = optarg;
				arg = strchr(optarg, ':');
				if (arg == NULL) {
					fprintf(stderr, "zynq-flash: missing a partition offset, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				start = strtol(arg, &arg, 0);
				if (*arg++ != ':') {
					fprintf(stderr, "zynq-flash: missing a partition size, err: %d\n", err);
					return err;
				}

				size = strtol(arg, &arg, 0);
				if (*arg != '\0') {
					fprintf(stderr, "zynq-flash: wrong partition size %s, err: %d\n", arg, err);
					return err;
				}

				err = flash_partAdd(devPath, start, size);
				if (err < 0) {
					fprintf(stderr, "zynq-flash: cannot add a partition %s: %d\n", arg, err);
					return err;
				}
				break;

			case 'r': /* <dev:start:size:fs> */
				devPath = optarg;
				arg = strchr(optarg, ':');
				if (arg == NULL) {
					fprintf(stderr, "zynq-flash: missing a partition offset, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				start = strtol(arg, &arg, 0);
				if (*arg++ != ':') {
					fprintf(stderr, "zynq-flash: missing a partition size, err: %d\n", err);
					return err;
				}

				size = strtol(arg, &arg, 0);
				if (*arg != ':') {
					fprintf(stderr, "zynq-flash: missing a filesystem name, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				fs = arg;

				err = flash_partAdd(devPath, start, size);
				if (err < 0) {
					fprintf(stderr, "zynq-flash: cannot add a partition %s: %d\n", arg, err);
					return err;
				}

				id = err;
				err = storage_mountfs(storage_get(id), fs, NULL, 0, NULL, &oid);
				if (err < 0) {
					fprintf(stderr, "zynq-flash: failed to mount a filesystem - %s: %d\n", fs, err);
					return err;
				}

				portRegister(oid.port, "/", &oid);
				break;

			case 'n': /* <portName:part:rbw:wbw:rate:prio> */
				portName = optarg;
				part = strchr(optarg, ':');
				if (part == NULL) {
					fprintf(stderr, "zynq-flash: missing a partition name, err: %d\n", err);
					return err;
				}

				*part++ = '\0';
				arg = strchr(part, ':');
				if (arg == NULL) {
					fprintf(stderr, "zynq-flash: missing a partition bandwidth limit, err: %d\n", err);
					return err;
				}

				*arg++ = '\0';
				rbw = strtol(arg, &arg, 0);
				if (*arg != ':') {
					fprintf(stderr, "zynq-flash: wrong bandwidth limit %s, err: %d\n", arg, err);
					return err;
				}

				*arg++ = '\0';
				wbw = strtol(arg, &arg, 0);
				if (*arg != ':') {
					fprintf(stderr, "zynq-flash: wrong bandwidth limit %s, err: %d\n", arg, err);
					return err;
				}

				*arg++ = '\0';
				rate = strtol(arg, &arg, 0);
				if (*arg != ':') {
					fprintf(stderr, "zynq-flash: wrong message rate limit %s, err: %d\n", arg, err);
					return err;
				}

				*arg++ = '\0';
				prio = strtol(arg, &arg, 0);
				if (*arg != '\0') {
					fprintf(stderr, "zynq-flash: wrong shared port thread priority %s, err: %d\n", arg, err);
					return err;
				}

				flash_sharedPortAdd(portName, part, rbw, wbw, rate, prio);

				break;


			case 'h':
				flash_help(argv[0]);
				return EOK;

			default:
				return -EINVAL;
		}
	}

	return EOK;
}


static int flash_drvInit(void)
{
	oid_t oid;
	storage_t *strg;
	int res, drvRes;

	do {
		strg = calloc(1, sizeof(storage_t));
		if (strg == NULL) {
			res = -ENOMEM;
			fprintf(stderr, "zynq-flash: failed to allocate storage, err: %d\n", res);
			return res;
		}

		drvRes = flashdrv_init(strg);
		if (drvRes < 0) {
			fprintf(stderr, "zynq-flash: failed to initialize flash memory driver, err: %d\n", drvRes);
			return drvRes;
		}

		res = storage_add(strg, &oid);
		if (res < 0) {
			fprintf(stderr, "zynq-flash: failed to add new storage, err: %d\n", res);
			return res;
		}

		res = flash_createMtdDev(strg, &oid);
		if (res < 0) {
			return res;
		}
	} while (drvRes > 0);

	return EOK;
}


static void flash_signalexit(int sig)
{
	exit(EXIT_SUCCESS);
}


int main(int argc, char **argv)
{
	int res;
	pid_t pid;

	/* Set parent exit handler */
	signal(SIGUSR1, flash_signalexit);

	/* Daemonize server */
	pid = fork();
	if (pid < 0) {
		fprintf(stderr, "zynq-flash: failed to daemonize server\n");
		exit(EXIT_FAILURE);
	}
	/* Parent waits to be killed by the child after finished server initialization */
	else if (pid > 0) {
		sleep(10);
		exit(EXIT_FAILURE);
	}

	/* Set child exit handler */
	signal(SIGUSR1, flash_signalexit);

	if (setsid() < 0) {
		fprintf(stderr, "zynq-flash: failed to create new session\n");
		exit(EXIT_FAILURE);
	}

	/* Initialize storage library */
	res = storage_init(flash_msgHandler, 16);
	if (res < 0) {
		fprintf(stderr, "zynq-flash: failed to initialize storage library, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Register file system related to NOR flash */
	res = storage_registerfs("jffs2", libjffs2_mount, libjffs2_umount);
	if (res < 0) {
		fprintf(stderr, "zynq-flash: failed to register jffs2 filesystem, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Initialize all flash devices and add them to the storage */
	res = flash_drvInit();
	if (res < 0) {
		fprintf(stderr, "zynq-flash: failed to initialize NOR flash memory driver, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Based on args, create new partitions and mount rootfs */
	res = flash_optsParse(argc, argv);
	if (res < 0) {
		fprintf(stderr, "zynq-flash: failed to parse arguments, err: %d\n", res);
		return EXIT_FAILURE;
	}

	/* Finished server initialization - kill parent */
	kill(getppid(), SIGUSR1);
	storage_run(2, 2 * _PAGE_SIZE);

	return EXIT_SUCCESS;
}
