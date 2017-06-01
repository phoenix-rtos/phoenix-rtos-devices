/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * stub implementation of /dev/tty
 * 
 * Copyright 2012-2015 Phoenix Systems
 * Author: Janusz Gralak, Pawel Kolodziej, Jacek Popko
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <proc/if.h>
#include <fs/if.h>

#include <dev/if.h>
#include <dev/tty/pty/pty.h>


static pty_t pty_common;


int pty_open(vnode_t *vnode, file_t* file)
{

	int i = MINOR(vnode->dev) / 2;

	if (i >= SIZE_PTY)
		return -ENOENT;

	proc_semaphoreDown(&pty_common.mutex);
	if (pty_common.virtuals[i] == NULL && MINOR(vnode->dev) % 2 == 0) {

		if ((pty_common.page[i] = vm_pageAlloc(sizeof(pty_con)/ SIZE_PAGE +1 , vm_pageAlloc)) == NULL) {
			proc_semaphoreUp(&pty_common.mutex);
			return -ENOMEM;

		}
		if (vm_kmap(pty_common.page[i], PGHD_PRESENT | PGHD_WRITE,(void **) &(pty_common.virtuals[i])) < 0){
			vm_pageFree(pty_common.page[i]);
			proc_semaphoreUp(&pty_common.mutex);
			return -ENOMEM;
		}

		pty_common.virtuals[i]->ws.ws_row = 24;
		pty_common.virtuals[i]->ws.ws_col = 80;
		pty_common.virtuals[i]->m_echo = 0;
		pty_common.virtuals[i]->status = OPEN_MASTER;

		proc_semaphoreCreate(&pty_common.virtuals[i]->rw.mutex, 1);
		proc_thqCreate(&pty_common.virtuals[i]->rw.waitq);
		pty_common.virtuals[i]->rw.rp = 0;
		pty_common.virtuals[i]->rw.rb = 0;
		pty_common.virtuals[i]->rw.rbuffsz = SIZE_PTY_BUFF;

		proc_semaphoreCreate(&pty_common.virtuals[i]->wr.mutex, 1);
		proc_thqCreate(&pty_common.virtuals[i]->wr.waitq);
		pty_common.virtuals[i]->wr.rp = 0;
		pty_common.virtuals[i]->wr.rb = 0;
		pty_common.virtuals[i]->wr.rbuffsz = SIZE_PTY_BUFF;
		
		vnode->flags |= VNODE_TTY;
		proc_semaphoreUp(&pty_common.mutex);
		return EOK;
	}

	if (pty_common.virtuals[i] && pty_common.virtuals[i]->status == OPEN_MASTER && MINOR(vnode->dev) % 2 == 1) {
		pty_common.virtuals[i]->status |= OPEN_SLAVE;
		proc_semaphoreUp(&pty_common.mutex);
		return EOK;
	}

	proc_semaphoreUp(&pty_common.mutex);
	return -EACCES;
}


int pty_read(file_t* file, offs_t offs, char *buff, unsigned int len)
{
  vnode_t* vnode = file->vnode;
	pty_buf *cur;
	int i;

	i = MINOR(vnode->dev) / 2;

	proc_semaphoreDown(&pty_common.mutex);

	if (i >= SIZE_PTY) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EIO;
	}

	if (pty_common.virtuals[i]->status & CLOSE_MASTER || pty_common.virtuals[i]->status & CLOSE_SLAVE) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EIO;
	}

	if (MINOR(vnode->dev) % 2 == 0)
		cur = &pty_common.virtuals[i]->rw;
	else
		cur = &pty_common.virtuals[i]->wr;

	proc_semaphoreUp(&pty_common.mutex);

	return pty_sget(cur, buff, len, MINOR(vnode->dev) % 2);
}


int pty_sget(pty_buf *cur, char *buff, unsigned int len, int minor)
{
	int err;
	unsigned int l, cnt;

	proc_semaphoreDown(&cur->mutex);

	while ((cur->rb == cur->rp))
		if ((err = proc_condWait(&cur->waitq, &cur->mutex, 0)) < 0)
			return err;


	if (cur->rp > cur->rb)
		l = min(cur->rp - cur->rb, len);
	else
		l = min(cur->rbuffsz - cur->rb, len);

	hal_memcpy(buff, cur->pbuff+cur->rb, l);

	cnt = l;
	if ((len > l) && (cur->rp < cur->rb)) {
		hal_memcpy(buff + l, cur->pbuff, min(len - l, cur->rp));
		cnt += min(len - l, cur->rp);
	}

	cur->rb = ((cur->rb + cnt) % cur->rbuffsz);

	proc_semaphoreUp(&cur->mutex);

	return cnt;
}


int pty_write(file_t* file, offs_t offs, char *buff, unsigned int len)
{
    vnode_t* vnode = file->vnode;
	pty_buf *cur;
	int i;

	proc_semaphoreDown(&pty_common.mutex);

	i = MINOR(vnode->dev) / 2;

	if (i >= SIZE_PTY) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EIO;
	}
	if (pty_common.virtuals[i]->status & CLOSE_MASTER || pty_common.virtuals[i]->status & CLOSE_SLAVE) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EIO;
	}

	if (MINOR(vnode->dev) % 2 == 1)
		cur = &pty_common.virtuals[i]->rw;
	else
		cur = &pty_common.virtuals[i]->wr;

	proc_semaphoreUp(&pty_common.mutex);

	return pty_sput(cur, buff, len, MINOR(vnode->dev) % 2 );
}


int pty_sput(pty_buf *cur,char *buff, unsigned int len, int minor)
{
	unsigned int l, i, bonus = 0;
	if (len == 0)
		return 0;

	proc_semaphoreDown(&cur->mutex);

	if (cur->rp >= cur->rb)
		l = cur->rbuffsz - (cur->rp - cur->rb);
	else
		l = cur->rb - cur->rp;

	if (l < len) {
		proc_semaphoreUp(&cur->mutex);
		return -ENOMEM;
	}

	for (i = 0; i < len; i++) {

		if ((buff[i] == 0x0a) && (minor == 1)){
			cur->pbuff[cur->rp] ='\r';
			cur->rp = ( (cur->rp + 1) % cur->rbuffsz);
			bonus++;
		}
		cur->pbuff[cur->rp] = *(buff + i);
		cur->rp = ((cur->rp + 1) % cur->rbuffsz);

	}

	proc_threadCondSignal(&cur->waitq);
	proc_semaphoreUp(&cur->mutex);

	return len;
}


int pty_poll(file_t* file, ktime_t timeout, int op)
{
	return EOK;
}


int pty_select_poll(file_t* file, unsigned *ready)
{
    vnode_t* vnode = file->vnode;
	pty_buf *cbuf;
	int i;
	proc_semaphoreDown(&pty_common.mutex);

	i = MINOR(vnode->dev) / 2;

	if (i >= SIZE_PTY) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EINVAL;
	}
	if (MINOR(vnode->dev) % 2 == 0)
		cbuf = &pty_common.virtuals[i]->rw;
	else
		cbuf = &pty_common.virtuals[i]->wr;

	proc_semaphoreUp(&pty_common.mutex);

	*ready = 0;

	proc_semaphoreDown(&cbuf->mutex);

	if ((cbuf->rp != cbuf->rb))
		*ready |= FS_READY_READ;

	if ((pty_common.virtuals[i]->status & CLOSE_MASTER) || (pty_common.virtuals[i]->status & CLOSE_SLAVE))
		*ready |= FS_READY_HUP;
	else
		*ready |= FS_READY_WRITE;

	proc_semaphoreUp(&cbuf->mutex);

	return EOK;
}


int pty_ioctl(file_t* file, unsigned int cmd, unsigned long arg)
{
  vnode_t* vnode = file->vnode;
	struct termios *termios_p = (struct termios *)arg;
	struct winsize *ws = (struct winsize*)arg;
	pty_con *cur;
	int ret = EOK;
	int  i;
	proc_semaphoreDown(&pty_common.mutex);

	i = MINOR(vnode->dev) / 2;

	if ((i >= SIZE_PTY) || (cur = pty_common.virtuals[i]) == NULL) {
		proc_semaphoreUp(&pty_common.mutex);
		return -EINVAL;
	}

	switch(cmd){
		case TIOCGWINSZ:
			ws->ws_row = cur->ws.ws_row;
			ws->ws_col = cur->ws.ws_col;
			break;
		case TIOCSWINSZ:
			cur->ws.ws_row = ws->ws_row;
			cur->ws.ws_col = ws->ws_col;
			break;
		case TCSETS:
			cur->m_echo = ((termios_p->c_lflag & ECHO) == ECHO);
		   	break;
		case TCGETS:
			termios_p->c_lflag = cur->m_echo ? ECHO | ECHOE : 0;
			break;
		default:
			ret = -EINVAL;
			break;
	}

	proc_semaphoreUp(&pty_common.mutex);

	return ret;
}


int pty_release(vnode_t *vnode)
{
	int i;

	proc_semaphoreDown(&pty_common.mutex);

	i = MINOR(vnode->dev) / 2;

	if ((i < SIZE_PTY) && pty_common.virtuals[i] &&  pty_common.virtuals[i]->status != 0) {

		pty_common.virtuals[i]->status |= (MINOR(vnode->dev) % 2 == 0) ? CLOSE_MASTER : CLOSE_SLAVE;
		if ((pty_common.virtuals[i]->status & CLOSE_MASTER) && (pty_common.virtuals[i]->status & CLOSE_SLAVE)) {

			proc_semaphoreTerminate(&pty_common.virtuals[i]->wr.mutex);
			proc_semaphoreTerminate(&pty_common.virtuals[i]->rw.mutex);

			vm_kunmap(pty_common.virtuals[i]);
			vm_pageFree(pty_common.page[i]);
			pty_common.virtuals[i] = NULL;
			pty_common.page[i] = NULL;

			proc_semaphoreUp(&pty_common.mutex);

			return -ENOENT;
		}
		if (pty_common.virtuals[i]->status == (OPEN_MASTER|CLOSE_MASTER)){

			proc_semaphoreTerminate(&pty_common.virtuals[i]->wr.mutex);
			proc_semaphoreTerminate(&pty_common.virtuals[i]->rw.mutex);

			vm_kunmap(pty_common.virtuals[i]);
			vm_pageFree(pty_common.page[i]);
			pty_common.virtuals[i] = NULL;
			pty_common.page[i] = NULL;

			proc_semaphoreUp(&pty_common.mutex);

			return -ENOENT;
		}
	}
	proc_semaphoreUp(&pty_common.mutex);
	return 0;
}


int pty_fsync(file_t* file)
{
	return EOK;
}


/* Function initializes pty */
void _pty_init(void)
{
	static const file_ops_t pty_ops = {
		.open = pty_open,
		.read = pty_read,
		.write = pty_write,
		.poll = pty_poll,
		.ioctl = pty_ioctl,
		.select_poll = pty_select_poll,
		.fsync = pty_fsync,
		.release = pty_release,
	};
	char line[] = "ptyp0";
	int i;

	proc_semaphoreCreate(&pty_common.mutex, 1);

	if (dev_register(MAKEDEV(MAJOR_PTY, 0), &pty_ops) < 0) {
		main_printf(ATTR_ERROR, "dev[pty]: Can't register pty!\n" );
		return;
	}

	for (i = 0; i < SIZE_PTY; i++){
		pty_common.virtuals[i] = NULL;
		pty_common.page[i] = NULL;

		line[4] = i < 10 ? i + '0' : i - 10 + 'a';
		line[0] = 'p';
		assert(EOK == dev_mknod(MAKEDEV(MAJOR_PTY, 2 * i), line));
		line[0] = 't';
		assert(EOK == dev_mknod(MAKEDEV(MAJOR_PTY, 2 * i + 1), line));
	}
}
