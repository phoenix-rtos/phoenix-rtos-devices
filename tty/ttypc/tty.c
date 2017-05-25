/**
 * stub implementation of /dev/tty
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file tty.c
 * 
 * @Copyright 2012 - 2014 Phoenix Systems
 * 
 * @author Pawel Kolodziej <pawel.kolodziej@phoesys.com>
 * @author Janusz Gralak <janusz.gralak@phoesys.com>
 * @author Marcin Stragowski <marcin.stragowski@phoesys.com>
 * @author Jacek Popko <jacek.popko@phoesys.com>
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
#include <include/ioctl.h>


static int tty_open(vnode_t *vnode, file_t* file)
{
	assert(vnode != NULL);
	vnode->flags |= VNODE_TTY;
	return 0;
}


static int tty_read(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	return 0;
}


static int tty_write(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	return 0;
}

static int tty_poll(file_t *file, ktime_t timeout, int op)
{
	return EOK;
}

static int tty_select_poll(file_t *file, unsigned *ready)
{
	*ready = 0;
	return EOK;
}

int tty_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	struct winsize *ws = (struct winsize*)arg;
	switch(cmd){
		case TIOCGWINSZ:
			ws->ws_row = 24;
			ws->ws_col = 80;
			break;
		default:
			return -EINVAL;
	}
	  
	return 0;
}


/* Function initializes tty */
void _tty_init(void)
{
	static const file_ops_t tty_ops = {
		.open = tty_open,
		.read = tty_read,
		.write = tty_write,
		.poll = tty_poll,
		.ioctl = tty_ioctl,
		.select_poll = tty_select_poll
	};

	if (dev_register(MAKEDEV(MAJOR_TTY, 0), &tty_ops) < 0) {
		main_printf(ATTR_ERROR, "tty: Can't register device for /dev/tty!\n" );
		return;
	}

	return;
}
