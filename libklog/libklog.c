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
#include <stdint.h>
#include <string.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/types.h>
#include <unistd.h>
#include <poll.h>
#include <sys/file.h>

#include <posix/utils.h>

#include "libklog.h"

#define ERROR_MSG "libklog: Fatal error, exiting\n"

static struct {
	char __attribute__((aligned(8))) stack[2048];
	libklog_write_t ttywrite;
	struct __errno_t e;
	volatile int enabled;
	handle_t cond;
	handle_t lock;
	oid_t ctrl;
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
	fd = open(_PATH_KLOG, O_RDONLY);
	if (fd < 0) {
		/* On some architectures devFS might not be bound
		 * to /dev directory yet, which makes /dev/kmsg path not resolvable.
		 * To make devfs/kmsg resolvable, we need to register it first.
		 */
		strcpy(buf, "devfs");
		name = strrchr(_PATH_KLOG, '/');
		if (name == NULL) {
			_errno_remove(&libklog_common.e);
			endthread();
		}

		strcat(buf, name);

		if (portRegister(0, buf, &dev) != 0) {
			_errno_remove(&libklog_common.e);
			endthread();
		}

		/* open() treats paths not starting with '/' slash as local */
		fd = sys_open(buf, O_RDONLY, 0);
		if (fd < 0) {
			_errno_remove(&libklog_common.e);
			endthread();
		}
	}

	while (1) {
		ret = read(fd, buf, sizeof(buf));
		if (ret <= 0) {
			if ((ret == 0) || (errno == EINTR) || (errno == EPIPE)) {
				continue;
			}
			else {
				libklog_common.ttywrite(ERROR_MSG, sizeof(ERROR_MSG));
				break;
			}
		}

		/* Just stop here while dmesg on the console is disabled */
		if (libklog_common.enabled == 0) {
			mutexLock(libklog_common.lock);
			while (libklog_common.enabled == 0) {
				condWait(libklog_common.cond, libklog_common.lock, 0);
			}
			mutexUnlock(libklog_common.lock);
		}

		libklog_common.ttywrite(buf, ret);
	}

	close(fd);
	_errno_remove(&libklog_common.e);
	endthread();
}


int libklog_ctrlHandle(uint32_t port, msg_t *msg, msg_rid_t rid)
{
	int handled = 0;

	if (port != libklog_common.ctrl.port) {
		/* Failsafe - oid not registered */
		return -1;
	}

	switch (msg->type) {
		case mtOpen:
		case mtClose:
			if (msg->i.openclose.oid.id == libklog_common.ctrl.id) {
				msg->o.io.err = 0;
				handled = 1;
			}
			break;

		case mtRead:
			if (msg->i.io.oid.id == libklog_common.ctrl.id) {
				if ((msg->o.size != 0) && (msg->o.data != NULL)) {
					strncpy(msg->o.data, (libklog_common.enabled != 0) ? "1" : "0", msg->o.size);
					msg->o.io.err = (msg->o.size > 1) ? 2 : 1;
				}
				else {
					msg->o.io.err = -EINVAL;
				}
				handled = 1;
			}
			break;

		case mtWrite:
			if (msg->i.io.oid.id == libklog_common.ctrl.id) {
				if ((msg->i.size != 0) && (msg->i.data != NULL)) {
					switch (((char *)msg->i.data)[0]) {
						case '0':
							libklog_common.enabled = 0;
							break;

						case '1':
							libklog_common.enabled = 1;
							condSignal(libklog_common.cond);
							break;

						default:
							break;
					}
					msg->o.io.err = msg->i.size;
				}
				else {
					msg->o.io.err = -EINVAL;
				}
				handled = 1;
			}
			break;

		case mtGetAttr:
			if (msg->i.attr.oid.id == libklog_common.ctrl.id) {
				if (msg->i.attr.type == atPollStatus) {
					msg->o.attr.val = POLLIN | POLLRDNORM | POLLOUT | POLLWRNORM;
					msg->o.attr.err = 0;
				}
				else {
					msg->o.attr.err = -EINVAL;
				}
				handled = 1;
			}
			break;

		default:
			break;
	}

	if (handled != 0) {
		msgRespond(port, msg, rid);
	}

	return (handled != 0) ? 0 : -1;
}


int libklog_ctrlRegister(oid_t *oid)
{
	int err = create_dev(oid, "kmsgctrl");
	if (err >= 0) {
		libklog_common.ctrl = *oid;
	}
	return err;
}


int libklog_init(libklog_write_t clbk)
{
	oid_t dev;
	int err;

	libklog_common.ttywrite = clbk;
	libklog_common.enabled = 1;

	err = mutexCreate(&libklog_common.lock);
	if (err < 0) {
		return err;
	}

	err = condCreate(&libklog_common.cond);
	if (err < 0) {
		resourceDestroy(libklog_common.lock);
		return err;
	}

	/* kmsg device is handled inside kernel */
	dev.port = 0;
	dev.id = 0;
	err = create_dev(&dev, _PATH_KLOG);
	if (err < 0) {
		resourceDestroy(libklog_common.lock);
		resourceDestroy(libklog_common.cond);
		return err;
	}

	/* Pump klog messages from kernel buffer to tty driver */
	if (beginthread(pumpthr, 4, libklog_common.stack, sizeof(libklog_common.stack), NULL) != 0) {
		resourceDestroy(libklog_common.lock);
		resourceDestroy(libklog_common.cond);
		return -ENOMEM;
	}

	return 0;
}
