/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * Loop driver
 *
 * Copyright 2015 Phoenix Systems
 *
 * Author: Jakub Sejdak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _LOOP_PRIV_H_
#define _LOOP_PRIV_H_

#include <hal/if.h>
#include <main/if.h>
#include <fs/if.h>
#include <dev/if.h>
#include <vm/if.h>

#define LOOP_DEV_NAME_MAXLEN	256
#define LOOP_DEV_IDLE			0
#define LOOP_DEV_BUSY			1

typedef struct {
	file_ops_t loop_fops;
	mutex_t mutex;
} loop_control_t;

typedef struct {
	char name[LOOP_DEV_NAME_MAXLEN];
	char underlying_name[LOOP_DEV_NAME_MAXLEN];
	file_t *underlying_dev;
	unsigned long start;
	unsigned long length;
	int busy;
	mutex_t mutex;
} loop_device_t;

extern int _loop_init(const char *underlying_name, unsigned long start, unsigned long length);
extern void _loop_destroy(loop_device_t * loop_dev);

extern int loop_read(file_t* file, offs_t offs, char *buff, unsigned int len);
extern int loop_write(file_t* file, offs_t offs, char *buff, unsigned int len);
extern int loop_poll(file_t *file, ktime_t timeout, int op);
extern int loop_select_poll(file_t *file, unsigned *ready);
extern int loop_ioctl(file_t* file, unsigned int cmd, unsigned long arg);
extern int loop_fsync(file_t* file);
extern int loop_open(vnode_t *vnode, file_t* file);
extern int loop_close(vnode_t *vnode, file_t* file);
extern int loop_release(vnode_t *vnode);

#endif
