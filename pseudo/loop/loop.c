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

int loop_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	offs_t new_offs = offs + loop_dev->start;
	int ret = underlying_dev->vnode->fops->read(underlying_dev, new_offs, buff, len);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	offs_t new_offs = offs + loop_dev->start;
	int ret = underlying_dev->vnode->fops->write(underlying_dev, new_offs, buff, len);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_poll(file_t *file, ktime_t timeout, int op)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->poll(underlying_dev, timeout, op);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_select_poll(file_t *file, unsigned *ready)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->select_poll(underlying_dev, ready);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->ioctl(underlying_dev, cmd, arg);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_fsync(file_t* file)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->fsync(underlying_dev);

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_open(vnode_t *vnode, file_t* file)
{
	if (subdev_get(vnode->dev, &vnode->file_priv) != EOK) {
		vnode->file_priv = NULL;
		return -1;
	}

	loop_device_t *loop_dev = (loop_device_t *) vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	int ret = fs_open(loop_dev->underlying_name, O_RDWR, 0, &loop_dev->underlying_dev);
	if (ret == EOK)
		loop_dev->busy = LOOP_DEV_BUSY;

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_close(vnode_t *vnode, file_t* file)
{
	loop_device_t *loop_dev = (loop_device_t *) file->vnode->file_priv;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->close(underlying_dev->vnode, underlying_dev);
	if (ret == EOK)
		loop_dev->busy = LOOP_DEV_IDLE;

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int loop_release(vnode_t *vnode)
{
	loop_device_t *loop_dev = NULL;
	if (subdev_get(vnode->dev, (void *) &loop_dev) != EOK)
		return -1;

	proc_mutexLock(&loop_dev->mutex);

	file_t *underlying_dev = loop_dev->underlying_dev;
	int ret = underlying_dev->vnode->fops->release(underlying_dev->vnode);
	if (ret == EOK)
		loop_dev->busy = LOOP_DEV_IDLE;

	loop_dev->underlying_dev = NULL;

	proc_mutexUnlock(&loop_dev->mutex);

	return ret;
}


int _loop_init(const char *underlying_name, unsigned long start, unsigned long length)
{
	int minor = dev_minorAlloc(MAKEDEV(MAJOR_LOOP, 0));
	if (minor < 0) {
		main_printf(ATTR_ERROR, "loop: No minor available for loop device\n");
		return -EINVAL;
	}

	loop_device_t *loop_dev = (loop_device_t *) vm_kmalloc(sizeof(loop_device_t));
	if (loop_dev == NULL) {
		main_printf(ATTR_ERROR, "loop: No memory availabe!\n");
		return -ENOMEM;
	}
	strncpy(loop_dev->underlying_name, underlying_name, LOOP_DEV_NAME_MAXLEN);
	loop_dev->underlying_name[LOOP_DEV_NAME_MAXLEN - 1] = 0;
	loop_dev->underlying_dev = NULL;
	loop_dev->start = start;
	loop_dev->length = length;
	loop_dev->busy = LOOP_DEV_IDLE;
	proc_mutexCreate(&loop_dev->mutex);

	if (subdev_register(MAKEDEV(MAJOR_LOOP, minor), loop_dev) != EOK) {
		main_printf(ATTR_ERROR, "loop: Failed to register loop subdevice\n");
		vm_kfree(loop_dev);
		return -EINVAL;
	}

	char name[] = "loopXXX";
	int name_size = strlen(name);
	if (minor < 10) {
		name[name_size - 3] = '0' + minor;
		name[name_size - 2] = 0;
	}
	else if (minor < 100) {
		name[name_size - 3] = '0' + minor / 10;
		name[name_size - 2] = '0' + minor % 10;
		name[name_size - 1] = 0;
	}
	else {
		name[name_size - 3] = '0' + minor / 100;
		name[name_size - 2] = '0' + (minor % 100) / 10;
		name[name_size - 1] = '0' + minor % 10;
	}

	strcpy(loop_dev->name, "/dev/");
	strcpy(loop_dev->name + 5, name);
	assert(dev_mknod(MAKEDEV(MAJOR_LOOP, minor), name) == EOK);

	main_printf(ATTR_INFO, "loop: Created loop device %s -> %s (%lu : %lu))\n", loop_dev->name,
																				loop_dev->underlying_name,
																				start,
																				start + length);
	return EOK;
}

void _loop_destroy(loop_device_t * loop_dev)
{
	assert(loop_dev != NULL);

	main_printf(ATTR_INFO, "loop: Destroyed loop device %s\n", loop_dev->name);

	proc_mutexTerminate(&loop_dev->mutex);
	vm_kfree(loop_dev);
}
