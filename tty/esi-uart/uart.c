/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * eSi-UART serial port driver
 *
 * Copyright 2012-2013 Phoenix Systems
 *
 * Author: Jacek Popko
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
#include <dev/dev.h>
#include <lib/assert.h>
#include <esirisc/device.h>
#include <esirisc/uart.h>
#include <esirisc/config.h>
#include "if.h"

/** queue impl. flavour: head == tail >>> queue is empty; (tail+1)%bufsize == head >>>  queue is full */
typedef struct {
	void *base;
	unsigned int irq;

	u8 *rbuff;
	unsigned rbuffsz;
	unsigned rb;		/**< head of the rx queue = first char waiting to be processed*/
	unsigned rp;		/**< tail of the rx queue = free space where the next rx'ed char will land*/

	u8 *sbuff;
	unsigned sbuffsz;
	unsigned sp;		/**< head of the tx queue*/
	unsigned se;		/**< tail of the tx queue*/

	mutex_t mutex;
	spinlock_t spinlock;
	thq_t waitq;
	unsigned overflow;
	unsigned framing;
	unsigned minor;
	char name[8];
} serial_t;


static serial_t *serials[SIZE_SERIALS];

/* the semantics are:
 * RX: store on a local buffer while possible, then just stop rx; any possibly lost data is unnoticed. When the user calls the rx func, pass her as many chars as requested,
 * up to the contents of the local buffer.
 * XXX Note that this means holding on to old data and missing newer data, with no notice of when this could have happened. Advantage: no work if no one calls the read func.
 * TX: store as much as possible of the data passed by the user into a local buffer,
 * that will be then pumped out char by char by the ISR. No action when txor is idle.
 */

static int uart_isr(unsigned int n, cpu_context_t *ctx, void *arg)
{
	serial_t *serial = (serial_t *)arg;
	esi_uart_t *uart_dev;
	unsigned short status;

	assert(serial != NULL);
	uart_dev = (esi_uart_t *)serial->base;
	assert(uart_dev != NULL);

	proc_spinlockSet(&serial->spinlock);
	status = uart_dev->status;

	/* Not our interrupt */
	if ((status & (ESI_UART_RX_FULL | ESI_UART_TX_EMPTY |
			ESI_UART_FRAMING_ERROR | ESI_UART_RX_OVERFLOW)) == 0) {
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
		return IHRES_IGNORE;
	}

	if (status & ESI_UART_RX_OVERFLOW) {
		serial->overflow++;
		GPIO_ON(0x80);
	}
	
	if (status & ESI_UART_FRAMING_ERROR)
		serial->framing++;

	/* Clear status bits once processed */
	uart_dev->status = ESI_UART_RX_OVERFLOW | ESI_UART_FRAMING_ERROR;
	

	/* Receive */
	while (!(uart_dev->status & ESI_UART_RX_EMPTY))
		if ((serial->rp == serial->rb - 1)   ||	/*strange avoidance of %rbuffsz*/
			((serial->rp == serial->rbuffsz - 1) && (serial->rb == 0)))	{

			/* No space left in the receive buffer, disable RX interrupt */
			uart_dev->control &= ~ESI_UART_RX_INT_ENABLE;
			break;
		}
		else { /*store the just-received char*/
			serial->rbuff[serial->rp] = uart_dev->rx_data;
			serial->rp = (serial->rp + 1) % serial->rbuffsz;
			proc_threadCondSignal(&serial->waitq);
		}

	/* Transmit */
	/*XXX There is no threadCondSignal, so the poll function for write won't wake up*/
	while (!(uart_dev->status & ESI_UART_TX_FULL))
		if (serial->sp != serial->se) {
			uart_dev->tx_data = serial->sbuff[serial->sp];
			serial->sp = (serial->sp + 1) % serial->sbuffsz;
		}
		else {
			/* Disable the interrupt raised on empty TX buffer if we have nothing to send */
			uart_dev->control &= ~ESI_UART_TX_INT_ENABLE;
			break;
		}

	proc_spinlockClear(&serial->spinlock, sopGetCycles);
	return IHRES_HANDLED;
}


/** Put up to <len> received chars into the given buffer
 *
 * Locks (minimally) only on the producer side of the queue, so:
 * 	- the producer can keep producing the rest of the time
 * 	- needs external locking against other consumers.
 * Gets a local copy of the queue tail index at the time of function start, so
 * won't consume past that point, so can not return more chars than rxbuffer's size.
 *
 * @param serial
 * @param buff
 * @param len			number of chars requested
 * @param repfl[OUT]	are chars still available?
 * @return				how many chars were put into the buffer, or error (negative)
 */
static int _uart_readChunk(serial_t *serial, char *buff, unsigned int len, int *repfl)
{
	unsigned int l; /*qty of linear (non-wrapping) chars to read*/
	unsigned int cnt;
	unsigned int bytes; /*qty of chars available to read*/
	unsigned int rp;
	esi_uart_t *uart_dev = (esi_uart_t *)serial->base;
	int err;

	/* Wait for data or for timeout */ /*XXX no timeout!*/
	proc_spinlockSet(&serial->spinlock);

	while (serial->rp == serial->rb) /*empty*/
		if ((err = proc_threadCondWait(&serial->waitq, &serial->spinlock, 0)) < 0)
			return err;

	rp = serial->rp;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	/* access to serial_t fields out of the spinlock - but OK because we only touch
	 * our side of the queue (head, consumer). Need then to guarantee
	 * only one consumer: needs external locking, so function is named _*
	 */
	if (rp > serial->rb) {	/*circular buffer is not wrapped around*/
		l = min(rp - serial->rb, len);
		bytes = rp - serial->rb;
	}
	else {
		l = min(serial->rbuffsz - serial->rb, len);
		bytes = serial->rbuffsz - serial->rb + rp;
	}

	hal_memcpy(buff, &serial->rbuff[serial->rb], l);

	cnt = l;
	if ((len > l) && (rp < serial->rb)) {
		hal_memcpy(buff + l, &serial->rbuff[0], min(len - l, rp));
		cnt += min(len - l, rp);
	}

	proc_spinlockSet(&serial->spinlock);
	serial->rb = ((serial->rb + cnt) % serial->rbuffsz);
	uart_dev->control |= ESI_UART_RX_INT_ENABLE;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	/* Suggest repetition */
	if (bytes - cnt)	/*is data still available?*/
		*repfl = 1;
	else
		*repfl = 0;		/*XXX if new data appeared since we got the tail, we are not noticing it*/

	return cnt;
}


/** Put some received chars into the given buffer
 *
 * Blocks if no chars are available.
 * Might return less than the asked-for quantity.
 * Reads repeatedly the rx queue while new data appears in it, so might return more chars than rxbuffer's size.
 *
 * @param vnode
 * @param offs	UNUSED
 * @param buff
 * @param len	max number of chars to read
 * @return		number of read chars, or error (negative)
 */
static int uart_read(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t* vnode = file->vnode;
	serial_t *serial;
	unsigned int l;
	int err, repfl;

	if (MINOR(vnode->dev) >= SIZE_SERIALS)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -ENOENT;

	proc_mutexLock(&serial->mutex);

	l = 0;
	repfl = 1;

	while ((l < len) && repfl) {	/*still didn't manage to read <len> chars AND repetition is suggested*/
		if ((err = _uart_readChunk(serial, buff + l, min(SIZE_SERIAL_CHUNK, len), &repfl)) < 0) {
			proc_mutexUnlock(&serial->mutex);
			return err;
		}
		l += err;
	}

	proc_mutexUnlock(&serial->mutex);
	return l;
}


/** Queue chars for sending
 *
 * Queues up to <len> chars and starts transmission
 *
 * @param vnode
 * @param offs	UNUSED
 * @param buff
 * @param len
 * @return the number of chars that could be queued
 */
static int uart_write(file_t *file, offs_t offs, char *buff, unsigned int len)
{
	vnode_t* vnode = file->vnode;
	serial_t *serial;
	unsigned int sp, se;
	unsigned int l;
	unsigned int cnt;
	esi_uart_t *uart_dev;

	if (MINOR(vnode->dev) >= SIZE_SERIALS)
		return -EINVAL;

	if ((serial = serials[MINOR(vnode->dev)]) == NULL)
		return -EINVAL;

	uart_dev = (esi_uart_t *)serial->base;

	proc_mutexLock(&serial->mutex);

	proc_spinlockSet(&serial->spinlock);
	sp = serial->sp;
	se = serial->se;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	/* calculate free space available; could be wrapped around the buffer*/
	if (sp > se)	/* circular buffer's contents are wrapped around*/
		l = min(sp - se, len);	 /*so free space isn't wrapped. XXX It should be sp-se-1 to avoid filling the buffer up to sp == se*/
	else
		l = min(serial->sbuffsz - se, len); /*XXX causes overrun if head == 0*/

	/* It is assumed that send buffer and its size are constant after initialization */
	hal_memcpy(&serial->sbuff[se], buff, l);

	cnt = l;
	if ((len > l) && (se >= sp)) { /*there are still bytes to write, and the buffer was not wrapped, so the free space was wrapped. */
		/*XXX it should be sp-1, to avoid filling the buffer up to sp == se*/
		hal_memcpy(serial->sbuff, buff + l, min(len - l, sp));
		cnt += min(len - l, sp);
	}

	/* Initialize sending process */
	proc_spinlockSet(&serial->spinlock);
	serial->se = ((serial->se + cnt) % serial->sbuffsz);
	uart_dev->control |= ESI_UART_TX_INT_ENABLE;
	proc_spinlockClear(&serial->spinlock, sopGetCycles);

	proc_mutexUnlock(&serial->mutex);

	return cnt;
}


/* semantics:
 * op POLL_READ: wait until at least one char can be read
 * op POLL_WRITE: wait until at least one char can be written
 */

static int uart_poll(file_t *file, ktime_t timeout, int op)
{
	vnode_t* vnode = file->vnode;
	serial_t *serial;
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
				return err; /*XXX return without releasing the spinlock (though threadCondWait releases anyway)*/
		}
		proc_spinlockClear(&serial->spinlock, sopGetCycles);
	}

	/* Wait for end of sending process or for timeout */
	/*XXX not true! while-condition is "sending queue is full"*/
	else if (op == POLL_WRITE) {
		proc_spinlockSet(&serial->spinlock);

		while ((serial->se + 1) % serial->sbuffsz == serial->sp) { /*while the sending queue is full*/
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


static int uart_ioctl(file_t *file, unsigned int cmd, unsigned long arg)
{
	return -ENOENT;
}


/* (MOD) modify memory allocation - kmalloc should be used instead of pageAlloc */
static int uart_init_one(esi_device_info_t *device, unsigned int speed, serial_t **serial, const int minor)
{
	esi_uart_t *uart_dev;
	page_t *page;
	void *vaddr;
	int status;


	/* Allocate and map memory for driver structures */
	if ((*serial = vm_kmalloc(sizeof(serial_t))) == NULL)
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
	main_memset(vaddr, 0, SIZE_PAGE);
	main_memset(*serial, 0, sizeof(serial_t));

	status = vm_iomap((addr_t)device->base_address, device->size, PGHD_KERNEL_RW, &(*serial)->base); /*XXX flags should be PGHD_DEV_RW*/
	assert(status == EOK);

	(*serial)->irq = device->irq;
	(*serial)->rbuff = (u8 *)vaddr;
	(*serial)->rbuffsz = SIZE_PAGE / 2;
	(*serial)->sbuff = (*serial)->rbuff + (*serial)->rbuffsz;
	(*serial)->sbuffsz = SIZE_PAGE - (*serial)->rbuffsz;
	(*serial)->minor = minor;
	strcpy((*serial)->name, "UARTx");
	(*serial)->name[4] = minor + '0';

	proc_spinlockCreate(&((*serial)->spinlock), (*serial)->name);
	proc_thqCreate(&(*serial)->waitq);
	proc_mutexCreate(&(*serial)->mutex);

	uart_dev = (esi_uart_t *)(*serial)->base;
	esi_uart_set_baud_rate(uart_dev, esi_get_frequency(), speed);
	/* Clear pending interrupts */
	uart_dev->status = ESI_UART_RX_FULL | ESI_UART_TX_EMPTY;
	/* Enable RX interrupt (TX one is enabled every time data are put into TX buffer) */
	uart_dev->control = ESI_UART_ENABLE | ESI_UART_RX_INT_ENABLE;	/* XXX IRQ enabled before the handler was set*/

	hal_interruptsSetHandler((*serial)->irq, uart_isr, (*serial));

	return EOK;
}


int _serial_init(unsigned int speed)
{
	static const file_ops_t serial_ops = {
		.read = uart_read,
		.write = uart_write,
		.poll = uart_poll,
		.ioctl = uart_ioctl,
	};
	esi_device_info_t *device;
	int result = 0, count = 0;

	/* 0 in size field indicates end of device list. */
	for (device = esi_device_get_list(); (device != NULL) && (device->size != 0); device++) {
		if (device->device_id == ESI_DID_ENSILICA_APB_UART) {
			main_printf(ATTR_INFO, "dev: eSi-UART rev=%d, base=0x%p, cfg=0x%x, irq=%d\n",
				device->revision, device->base_address, device->config, device->irq);

			if (uart_init_one(device, speed, &serials[count], count) != 0) {
				main_printf(ATTR_ERROR, "eSi-UART initialization failed\n");
				result++;
			}
			else
				count++;
		}
	}

	if (dev_register(MAKEDEV(MAJOR_SERIAL, 0), &serial_ops) < 0) {
		main_printf(ATTR_ERROR, "serial: Can't register serial device!\n");
		return -1;
	}

	return -result;
}
