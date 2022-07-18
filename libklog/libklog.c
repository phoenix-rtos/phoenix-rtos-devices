/*
 * Phoenix-RTOS
 *
 * Phoenix-RTOS klog driver
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <string.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <unistd.h>

#include <posix/utils.h>

#include "libklog.h"

static struct {
	char __attribute__((aligned(8))) stack[2048];
	libklog_write_t ttywrite;
	struct __errno_t e;
} libklog_common;


extern int sys_open(const char *filename, int oflag, ...);


static void pumpthr(void *arg)
{
	char buf[256];
	int fd, ret;
	oid_t dev;
	char *name;

	dev.id = 0;
	dev.port = 0;

	_errno_new(&libklog_common.e);

	if ((fd = open(_PATH_KLOG, O_RDONLY)) < 0) {
		/* On some architectures devFS might not be bound
		 * to /dev directory yet, which makes /dev/kmsg path not resolvable.
		 * To make devfs/kmsg resolvable, we need to register it first.
		 */
		strcpy(buf, "devfs");
		if ((name = strrchr(_PATH_KLOG, '/')) == NULL) {
			_errno_remove(&libklog_common.e);
			endthread();
		}

		strcat(buf, name);

		if (portRegister(0, buf, &dev) != 0) {
			_errno_remove(&libklog_common.e);
			endthread();
		}

		/* open() treats paths not starting with '/' slash as local */
		if ((fd = sys_open(buf, O_RDONLY, 0)) < 0) {
			_errno_remove(&libklog_common.e);
			endthread();
		}
	}

	while (1) {
		ret = read(fd, buf, sizeof(buf));
		if (ret <= 0) {
			if (errno == -EINTR || errno == -EPIPE)
				continue;
			else
				break;
		}
		libklog_common.ttywrite(buf, ret);
	}

	_errno_remove(&libklog_common.e);
	endthread();
}


int libklog_init(libklog_write_t clbk)
{
	oid_t dev;
	int err;

	libklog_common.ttywrite = clbk;

	/* kmsg device is handled inside kernel */
	dev.port = 0;
	dev.id = 0;
	if ((err = create_dev(&dev, _PATH_KLOG)) < 0)
		return err;

	/* Pump klog messages from kernel buffer to tty driver */
	if (beginthread(pumpthr, 4, libklog_common.stack, sizeof(libklog_common.stack), NULL) != 0)
		return -ENOMEM;

	return 0;
}
