/*
 * Phoenix-RTOS
 *
 * ttypc - VT220 terminal emulator based on VGA and 101-key US keyboard
 *
 * Driver initialization
 *
 * Copyright 2012, 2017 Phoenix Systems
 * Copyright 2006, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej, Janusz Gralak
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <libphoenix.h>

#include "ttypc_virt.h"


#define SIZE_TTYPC_RBUFF  128


#define VRAM_MONO     (void *)0xb0000    /* VRAM address of mono 80x25 mode */
#define VRAM_COLOR    (void *)0xb8000    /* VRAM address of color 80x25 mode */

#define CRTC_CURSORH  0x0e               /* cursor address mid */
#define CRTC_CURSORL  0x0f               /* cursor address low */


struct {
	ttypc_virt_t virtuals[4];
	ttypc_virt_t *cv;

	int color;
	unsigned int inp_irq;
	void *inp_base;
	void *out_base;
	void *out_crtc;
	
	unsigned char extended;
	unsigned int lockst;
	unsigned int shiftst;
	
	unsigned char **rbuff;
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;
} ttypc_common;


#if 0

static int ttypc_read(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	unsigned int minor;
	int err = 0;

	if ((minor = MINOR(vnode->dev)) >= SIZE_VIRTUALS)
		return -EINVAL;

	err = ttypc_virt_sget(&ttypc_common.virtuals[minor], buff, len);
	return err;
}


static int ttypc_write(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	unsigned int minor;

	if ((minor = MINOR(vnode->dev)) >= SIZE_VIRTUALS)
		return -EINVAL;

	ttypc_virt_sput(&ttypc_common.virtuals[minor], (u8 *)buff, len);
	return len;
}


static int ttypc_poll(file_t *file, ktime_t timeout, int op)
{
	return EOK;
}

static int ttypc_select_poll(file_t *file, unsigned *ready)
{
	vnode_t *vnode = file->vnode;
	unsigned int minor;
	*ready = 0;

	if ((minor = MINOR(vnode->dev)) >= SIZE_VIRTUALS)
		return -EINVAL;
	if(ttypc_common.virtuals[minor].rp != ttypc_common.virtuals[minor].rb)
		*ready |= FS_READY_READ;
	*ready |= FS_READY_WRITE;
	return EOK;
}


static int ttypc_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	vnode_t *vnode = file->vnode;
	unsigned int minor;
	struct termios *termios_p = (struct termios *)arg;

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


	
	if ((minor = MINOR(vnode->dev)) >= SIZE_VIRTUALS)
		return -EINVAL;

	switch (cmd) {
	   	case TCSETS:
   			ttypc_common.virtuals[minor].m_echo = ((termios_p->c_lflag & ECHO) == ECHO);
	   		break;
		case TCGETS:
			termios_p->c_lflag = ttypc_common.virtuals[minor].m_echo ? ECHO : 0;
			break;
	   	default:
			/* main_printf(ATTR_DEBUG, "%s: unsupported cmd %d", __FUNCTION__, cmd); */
			break;
	}

	if ((minor = MINOR(vnode->dev)) >= SIZE_VIRTUALS)
		return -EINVAL;

	return 0;
}


static int ttypc_open(vnode_t *vnode, file_t* file)
{
	assert(vnode != NULL);
	vnode->flags |= VNODE_TTY;
	return 0;
}
#endif


int main(int argc, char *argv[])
{
	unsigned int i;

	ph_printf("ttypc: Initializing VGA VT220 terminal emulator\n");

	/* Test monitor type */
	ttypc_common.color = (inb((void *)0x3cc) & 0x01);
	
	ttypc_common.inp_irq = 1;
	ttypc_common.inp_base = (void *)0x60;

	ttypc_common.out_base = ttypc_common.color ? VRAM_COLOR : VRAM_MONO);
	ttypc_common.out_crtc = ttypc_common.color ? (void *)0x3d4 : (void *)0x3b4;

	/* Initialize virutal terminals and register devices */
	for (i = 0; i < sizeof(ttypc_common.virtuals) / sizeof(ttypc_virt_t); i++) {
		if (_ttypc_virt_init(&ttypc_common, &ttypc_common.virtuals[i]) < 0) {
			ph_printf("ttypc: Can't initialize virtual terminal %d!\n", i);
			return -1;
		}
	}

	ttypc_common.cv = &ttypc_common.virtuals[0];
	ttypc_common.cv->vram = ttypc_common.out_base;

	_ttypc_vga_cursor(ttypc_common.cv);

	/* Initialize keyboard */
	_ttypc_kbd_init(&ttypc_common);

	ph_register("/dev/ttypc", oid);

	return 0;
}
