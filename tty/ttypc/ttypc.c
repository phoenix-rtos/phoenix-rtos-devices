/**
 * ttypc - VT220 terminal emulator based on VGA and 101-key US keyboard.
 *
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * @file ttypc.c
 * 
 * Copyright 2012 Phoenix Systems
 * Copyright 2006, 2008 Pawel Pisarczyk
 * 
 * Author Pawel Pisarczyk <pawel.pisarczyk@phoesys.com>
 * @author Pawel Kolodziej <pawel.kolodziej@phoesys.com>
 * @author Janusz Gralak <janusz.gralak@phoesys.com>
 * @author Marcin Stragowski <marcin.stragowski@phoesys.com>
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


ttypc_t ttypc_common;


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


/* Function initializes ttypc */
void _ttypc_init(void)
{
	unsigned int i;

	ph_printf("ttypc: Initializing virtual terminals\n");

	/* Test monitor type */
	ttypc_common.color = (hal_inb((void *)MAIN_MISCIN) & 0x01);
	
	ttypc_common.inp_irq = 1;
	ttypc_common.inp_base = (void *)0x60;

	ttypc_common.out_base = VADDR_KERNEL + (ttypc_common.color ? VRAM_COLOR : VRAM_MONO);
	ttypc_common.out_crtc = ttypc_common.color ? (void *)CRTR_COLOR : (void *)CRTR_MONO;
	
	/* Initialize virutal terminals and register devices */
	for (i = 0; i < SIZE_VIRTUALS; i++) {
		if (_ttypc_virt_init(&ttypc_common, &ttypc_common.virtuals[i]) < 0) {
			main_printf(ATTR_ERROR, "dev[ttypc]: Can't initialize virtual terminal %d!\n", i);
			return;
		}
	}

	ttypc_common.cv = &ttypc_common.virtuals[0];
	ttypc_common.cv->vram = ttypc_common.out_base;

	proc_semaphoreCreate(&ttypc_common.mutex, 1);

	_ttypc_vga_cursor(ttypc_common.cv);

	/* Initialize keyboard */
	_ttypc_kbd_init(&ttypc_common);

	/* Register driver */
	if (dev_register(MAKEDEV(MAJOR_TTYPC, 0), &ttypc_ops) < 0) {
		main_printf(ATTR_ERROR, "dev/ttypc: Can't register device for virtual terminal %d!\n", i);
		return;
	}

	return;
}
