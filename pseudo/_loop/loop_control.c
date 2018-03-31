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

#include "loop_priv.h"
#include <include/loopctl.h>

static loop_control_t *loop_control = NULL;
static file_ops_t loop_control_fops;


int loop_minorFromName(const char *name)
{
	if (strncmp(name, "/dev/loop", 9) != 0)
		return -1;

	if (strlen(name) < 10)
		return -1;

	return main_atoi(&name[9]);
}


int loop_control_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
	vnode_t *vnode = file->vnode;
	if (vnode == NULL)
		return -EINVAL;

	proc_mutexLock(&loop_control->mutex);

	int i = 0;
	int ret = -EINVAL;
	int minor = -1;
	loop_info_t * loop_info = (loop_info_t *) arg;
	loop_device_t *loop_dev = NULL;

	switch (cmd) {
		case LOOP_DEV_CREATE:
		{
			ret = _loop_init(loop_info->underlying_name, loop_info->start, loop_info->length);
			break;
		}
		case LOOP_DEV_REMOVE:
		{
			minor = loop_minorFromName(loop_info->name);
			if (minor < 0)
				break;

			if (subdev_get(MAKEDEV(MAJOR_LOOP, minor), (void *) &loop_dev) != EOK)
				break;

			if (loop_dev->busy == LOOP_DEV_BUSY) {
				ret = -EBUSY;
				break;
			}

			_loop_destroy(loop_dev);
			subdev_unregister(MAKEDEV(MAJOR_LOOP, minor));

			ret = dev_rmnod(&loop_info->name[5]);	// skip "/dev/"
			break;
		}
		case LOOP_DEV_COUNT:
		{
			loop_info->count = 0;
			for (i = 0; i < MAX_MINOR; ++i) {
				if (subdev_get(MAKEDEV(MAJOR_LOOP, i), (void *) &loop_dev) == EOK)
					++loop_info->count;
			}

			ret = EOK;
			break;
		}
		case LOOP_DEV_STAT:
		{
			if (subdev_get(MAKEDEV(MAJOR_LOOP, loop_info->minor), (void *) &loop_dev) != EOK) {
				ret = -EINVAL;
				break;
			}

			strncpy(loop_info->name, loop_dev->name, LOOP_DEV_NAME_MAXLEN);
			loop_info->name[LOOP_DEV_NAME_MAXLEN] = 0;
			strncpy(loop_info->underlying_name, loop_dev->underlying_name, LOOP_DEV_NAME_MAXLEN);
			loop_info->underlying_name[LOOP_DEV_NAME_MAXLEN] = 0;
			loop_info->start = loop_dev->start;
			loop_info->length = loop_dev->length;
			loop_info->status = loop_dev->busy;

			ret = EOK;
			break;
		}
		default:
			ret = -EPERM;
			break;
	}

	proc_mutexUnlock(&loop_control->mutex);
	return ret;
}


int loop_control_open(vnode_t *vnode, file_t* file)
{
	return 0;
}


int loop_control_close(vnode_t *vnode, file_t* file)
{
	return 0;
}


void _loop_controlInit(void)
{
	assert(loop_control == NULL);

	loop_control_fops.ioctl = loop_control_ioctl;
	loop_control_fops.open  = loop_control_open;
	loop_control_fops.close = loop_control_close;

	if (dev_register(MAKEDEV(MAJOR_LOOP_CONTROL, 0), &loop_control_fops) < 0) {
		main_printf(ATTR_ERROR, "loop-control: Failed to register /dev/loop-control device\n");
		return;
	}

	loop_control = (loop_control_t *) vm_kmalloc(sizeof(loop_control_t));
	if (loop_control == NULL) {
		main_printf(ATTR_ERROR, "loop-control: No memory availabe for /dev/loop-control device\n");
		return;
	}

	loop_control->loop_fops.read        = loop_read;
	loop_control->loop_fops.write       = loop_write;
	loop_control->loop_fops.poll        = loop_poll;
	loop_control->loop_fops.select_poll = loop_select_poll;
	loop_control->loop_fops.ioctl       = loop_ioctl;
	loop_control->loop_fops.fsync       = loop_fsync;
	loop_control->loop_fops.open        = loop_open;
	loop_control->loop_fops.close       = loop_close;
	loop_control->loop_fops.release     = loop_release;
	proc_mutexCreate(&loop_control->mutex);

	if (subdev_register(MAKEDEV(MAJOR_LOOP_CONTROL, 0), loop_control) != EOK) {
		main_printf(ATTR_ERROR, "loop-control: Failed to register /dev/loop-control subdevice\n");
		vm_kfree(loop_control);
		loop_control = NULL;
		dev_unregister(MAKEDEV(MAJOR_LOOP_CONTROL, 0));
		return;
	}

	assert(dev_mknod(MAKEDEV(MAJOR_LOOP_CONTROL, 0), "loop-control") == EOK);

	if (dev_register(MAKEDEV(MAJOR_LOOP, 0), &loop_control->loop_fops) < 0) {
		main_printf(ATTR_ERROR, "loop: Failed to register loop device\n");
		proc_mutexUnlock(&loop_control->mutex);
		return;
	}

	main_printf(ATTR_INFO, "loop-control: Created /dev/loop-control device\n");
}
