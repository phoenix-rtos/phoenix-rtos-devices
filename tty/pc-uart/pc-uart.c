/* 
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * UART 16550 driver for PC
 *
 * Copyright 2012-2015 Phoenix Systems
 * Copyright 2001, 2008 Pawel Pisarczyk
 * Author: Pawel Pisarczyk, Pawel Kolodziej
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <unistd.h>
#include <sys/msg.h>

#include "pc-uart.h"


typedef struct {
	void *base;
	unsigned int irq;

	u8 rbuff[256];
	unsigned int rbuffsz;
	unsigned int rb;
	unsigned int rp;
	handle_t rcond;

	u8 sbuff[256];
	unsigned int sbuffsz;
	unsigned int sp;
	unsigned int se;

	handle_t mutex;
	handle_t intcond;
} uart_t;


static uart_t *uarts[4];

#if 0
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

#endif


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
	u8 iir;

//	if ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
//		return 0;

	return uart->intcond;
}


void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	u8 iir, lsr;

	for (;;) {
		mutexLock(uart->mutex);

		while ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
			condWait(uart->intcond, uart->mutex, 0);

		/* Receive */
		if ((iir & IIR_DR) == IIR_DR) {
printf("DR %d\n", uart->rp);
			while (1) {
				lsr = inb(uart->base + REG_LSR);
				/*if (lsr & 2)
					over = 1;*/

				if ((lsr & 1) == 0)
					break;
				
				uart->rbuff[uart->rp] = inb(uart->base + REG_RBR);
				uart->rp = ( (uart->rp + 1) % uart->rbuffsz);

				if (uart->rp == uart->rb)
					uart->rb = ((uart->rb+1) % uart->rbuffsz);
			}

			condSignal(uart->rcond);
		}

		/* Transmit */
		if ((iir & IIR_THRE) == IIR_THRE) {
			uart->sp = ((uart->sp+1) % uart->sbuffsz);
			if (uart->sp != uart->se) {
				outb(uart->base + REG_THR, uart->sbuff[uart->sp]);
			}
		}

		mutexUnlock(uart->mutex);
	}

	return;
}


int _uart_init(void *base, unsigned int irq, unsigned int speed, uart_t **uart)
{
	/* Test if device exist */
	if (inb(base + REG_IIR) == 0xff)
		return -ENOENT;

	printf("pc-uart: Detected interface on 0x%x irq=%d\n", (u32)base, irq);

	/* Allocate and map memory for driver structures */
	if ((*uart = malloc(sizeof(uart_t))) == NULL)
		return -ENOMEM;	

	(*uart)->base = base;
	(*uart)->irq = irq;

	(*uart)->rbuffsz = sizeof((*uart)->rbuff);
	(*uart)->rb = (*uart)->rp = 0;

	(*uart)->sbuffsz = sizeof((*uart)->sbuff);
	(*uart)->sp = (unsigned int)-1;
	(*uart)->se = 0;

	condCreate(&(*uart)->intcond);
	mutexCreate(&(*uart)->mutex);

	interrupt(irq, uart_interrupt, (*uart), (*uart)->intcond);

	u8 *stack;
	stack = (u8 *)malloc(2048);

	condCreate(&(*uart)->rcond);

	beginthread(uart_intthr, 1, stack, 2048, (void *)*uart);

	/* Set speed (MOD) */
	outb(base + REG_LCR, LCR_DLAB);
	outb(base + REG_LSB, speed);
	outb(base + REG_MSB, 0);

	/* Set data format (MOD) */
	outb(base + REG_LCR, LCR_D8N1);

	/* Enable FIFO - this is required for Transmeta Crusoe (MOD) */
	outb(base + 2, 0x01);

	/* Enable hardware interrupts */
	outb(base + REG_MCR, MCR_OUT2);

	/* Set interrupt mask */
	outb(base + REG_IMR, IMR_THRE | IMR_DR);
	
	return EOK;
}


#if 0
static int uart_write(offs_t offs, char *buff, unsigned int len)
{
	uart_t *serial;
	unsigned int sp, se;
	unsigned int l;
	unsigned int cnt;

	mutexLock(
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
#endif


int main(void)
{
	void *base = (void *)0x3f8;
	unsigned int n = 4;
	u32 port;
//	msghdr_t hdr;

	printf("pc-uart: Initializing UART 16550 driver %s\n", "");

	_uart_init(base, n, BPS_115200, &uarts[0]);

	portCreate(&port);
	if (portRegister(port, "/dev/ttyS0") < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}

	for (;;) {
//		recv(port, data, size, &hdr, 0);

//sys/msg.h:extern int respond(u32 port, int err, void *data, size_t size);

		usleep(100);
	}

	return 0;
}
