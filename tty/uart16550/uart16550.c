/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * UART 16550 driver
 *
 * Copyright 2012-2015 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
 *
 * %LICENSE%
 */

#include <hal/if.h>
#include <main/if.h>
#include <vm/if.h>
#include <proc/if.h>
#include <fs/if.h>
#include <dev/if.h>

#include <dev/serial/uart16550/uart16550.h>


typedef struct {
	void *base;
	unsigned int irq;

	u8 *rbuff;
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;

	u8 *sbuff;
	unsigned int sbuffsz;
	unsigned int sp;
	unsigned int se;

	semaphore_t mutex;
	spinlock_t spinlock;
	thq_t waitq;
} uart16550_t;


static uart16550_t *serials[SIZE_SERIALS];


static int uart16550_interrupt(unsigned int n, cpu_context_t *ctx, void *arg)
{
	uart16550_t *serial = (uart16550_t *)arg;
	u8 iir;

	if ((iir = hal_inb(serial->base + REG_IIR)) & IIR_IRQPEND)
		return IHRES_IGNORE;

	proc_spinlockSet(&serial->spinlock);

	/* Receive */
	if ((iir & IIR_DR) == IIR_DR) {
		u8 lsr;
		
		while (1) {
			lsr = hal_inb(serial->base + REG_LSR);
			
			/*if (lsr & 2)
				over = 1;*/

			if ((lsr & 1) == 0)
				break;
				
			serial->rbuff[serial->rp] = hal_inb(serial->base + REG_RBR);
			serial->rp = ( (serial->rp+1) % serial->rbuffsz);

			if (serial->rp == serial->rb)
				serial->rb = ( (serial->rb+1) % serial->rbuffsz);
		}

		proc_threadCondSignal(&serial->waitq);
	}

	/* Transmit */
	if ((iir & IIR_THRE) == IIR_THRE) {
		serial->sp = ( (serial->sp+1) % serial->sbuffsz);
		if (serial->sp != serial->se) {
			hal_outb(serial->base + REG_THR, serial->sbuff[serial->sp]);
		}
	}

	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return IHRES_HANDLED;
}


int uart16550_readchunk(uart16550_t *serial, char *buff, unsigned int len, int *repfl)
{
	unsigned int l, cnt, bytes;
	int err;

	/* Wait for data or for timeout */
	proc_spinlockSet(&serial->spinlock);

	while (serial->rp == serial->rb) {
		if ((err = proc_threadCondWait(&serial->waitq, &serial->spinlock, 0)) < 0)
			return err;
	}

	if (serial->rp > serial->rb) {
		l = min(serial->rp - serial->rb, len);
		bytes = serial->rp - serial->rb;
	}
	else {
		l = min(serial->rbuffsz - serial->rb, len);
		bytes = serial->rbuffsz - serial->rb + serial->rp;
	}

	hal_memcpy(buff, &serial->rbuff[serial->rb], l);
	
	cnt = l;
	if ((len > l) && (serial->rp < serial->rb)) {
		hal_memcpy(buff + l, &serial->rbuff[0], min(len - l, serial->rp));
		cnt += min(len - l, serial->rp);
	}
	serial->rb = ((serial->rb + cnt) % serial->rbuffsz);

	/* Suggest repetition */
	if (bytes - cnt)
		*repfl = 1;
	else
		*repfl = 0;

	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	
	return cnt;
}


static int uart16550_read(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	uart16550_t *serial;
	unsigned int l;
	int err, repfl;

	if (MINOR(vnode->dev) >= SIZE_SERIALS)
		return -EINVAL;
	
	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -ENOENT;

	proc_semaphoreDown(&serial->mutex);
	
	l = 0;
	repfl = 1;

	while ((l < len) && repfl) {
		if ((err = uart16550_readchunk(serial, buff+l, min(SIZE_SERIAL_CHUNK, len), &repfl)) < 0) {
			proc_semaphoreUp(&serial->mutex);
			return err;
		}
		l += err;
	}

	proc_semaphoreUp(&serial->mutex);
	return l;
}


static int uart16550_write(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t *vnode = file->vnode;
	uart16550_t *serial;
	unsigned int sp, se;
	unsigned int l;
	unsigned int cnt;

	if (MINOR(vnode->dev) >= SIZE_SERIALS)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	proc_semaphoreDown(&serial->mutex);

	proc_spinlockSet(&serial->spinlock);
	sp = serial->sp;
	se = serial->se;	
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	if (sp > se)
		l = min(sp - se, len);
	else
		l = min(serial->sbuffsz - se, len);

	/* It is assumed that send buffer and its size are constant after initialization */
	hal_memcpy(&serial->sbuff[se], buff, l);

	cnt = l;
	if ((len > l) && (se >= sp)) {
		hal_memcpy(serial->sbuff, buff + l, min(len - l, sp));
		cnt += min(len - l, sp);
	}

	/* Initialize sending process */
	proc_spinlockSet(&serial->spinlock);
	if (serial->se == serial->sp)
		hal_outb(serial->base, serial->sbuff[serial->sp]);
	
	serial->se = ((serial->se + cnt) % serial->sbuffsz);
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	proc_semaphoreUp(&serial->mutex);

	return cnt;
}


static int uart16550_poll(file_t *file, ktime_t timeout, int op)
{
	vnode_t *vnode = file->vnode;
	uart16550_t *serial;
	int err = EOK;

	if (MINOR(vnode->dev) >= SIZE_SERIALS)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	/* Wait for data or for timeout */
	if (op == POLL_READ) {
		proc_spinlockSet(&serial->spinlock);

		while (serial->rp == serial->rb) {
			err = proc_threadCondWait(&serial->waitq, &serial->spinlock, timeout);
			if (err == -ETIME)
				break;
			else if (err < 0)
				return err;
		}
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
	}
	
	/* Wait for end of sending process or for timeout */
	else if (op == POLL_WRITE) {
		proc_spinlockSet(&serial->spinlock);

		while ((serial->se + 1) % serial->sbuffsz == serial->sp) {
			err = proc_threadCondWait(&serial->waitq, &serial->spinlock, timeout);
			if (err == -ETIME)
				break;
			else if (err < 0)
				return err;
		}
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
	}

	return err;
}


static int uart16550_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	return -ENOENT;
}


/* (MOD) modify memory allocation - kmalloc should be used instead of pageAlloc */
int _uart16550_detect(void *base, unsigned int irq, unsigned int speed, uart16550_t **serial)
{
	page_t *page;
	void *vaddr;
	
	/* Test if device exist */
	if (hal_inb(base + REG_IIR) == 0xff)
		return ERR_DEV_DETECT;

	main_printf(ATTR_DEV, "dev: [uart ] Detected interface on 0x%x irq=%d\n", (u32)base, irq);

	/* Allocate and map memory for driver structures */
	if ((*serial = vm_kmalloc(sizeof(uart16550_t))) == NULL)
		return -ENOMEM;	

	if ((page = vm_pageAlloc(1, vm_pageAlloc)) == NULL) {
		vm_kfree(*serial);
		return -ENOMEM;
	}

	if (vm_kmap(page, PGHD_WRITE | PGHD_PRESENT, &vaddr) < 0) {
		vm_kfree(*serial);
		vm_pageFree(page);
		return -ENOMEM;
	}

	(*serial)->rbuff = (u8 *)vaddr;
	(*serial)->base = base;
	(*serial)->irq = irq;
	(*serial)->rbuffsz = SIZE_PAGE / 2;
	(*serial)->rb = 0;
	(*serial)->rp = 0;
	
	(*serial)->sbuff = (*serial)->rbuff + (*serial)->rbuffsz;
	(*serial)->sbuffsz = SIZE_PAGE - (*serial)->rbuffsz;	
	(*serial)->sp = (unsigned int)-1;
	(*serial)->se = 0;
	
	proc_thqCreate(&(*serial)->waitq);

	proc_spinlockCreate(&((*serial)->spinlock), "serial.spinlock");
	hal_interruptsSetHandler(irq, uart16550_interrupt, (*serial));

	proc_semaphoreCreate(&(*serial)->mutex, 1);

	/* Set speed (MOD) */
	hal_outb(base + REG_LCR, LCR_DLAB);
	hal_outb(base + REG_LSB, speed);
	hal_outb(base + REG_MSB, 0);

	/* Set data format (MOD) */
	hal_outb(base + REG_LCR, LCR_D8N1);

	/* Enable FIFO - this is required for Transmeta Crusoe (MOD) */
	hal_outb(base + 2, 0x01);

	/* Enable hardware interrupts */
	hal_outb(base + REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	hal_outb(base + REG_IMR, IMR_THRE | IMR_DR);
	
	return EOK;
}


void _uart16550_init(unsigned int speed)
{
	static const file_ops_t uart16550_ops = {
		.read = uart16550_read,
		.write = uart16550_write,
		.poll = uart16550_poll,
		.ioctl = uart16550_ioctl,
	};

	_uart16550_detect((void *)0x3f8, 4, speed, &serials[0]);
	_uart16550_detect((void *)0x2f8, 3, speed, &serials[1]);

	if (dev_register(MAKEDEV(MAJOR_SERIAL, 0), &uart16550_ops) < 0) {
		main_printf(ATTR_ERROR, "dev[uart16550]: Can't register serial device!\n");
		return;
	}

	return;
}
