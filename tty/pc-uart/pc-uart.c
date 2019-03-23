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

#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <sys/file.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>

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
	handle_t scond;

	handle_t mutex;
	handle_t intcond;
	handle_t inth;

	oid_t oid;

	int ready;
} uart_t;


static uart_t *uarts[4];


static int uart_interrupt(unsigned int n, void *arg)
{
	uart_t *uart = (uart_t *)arg;
/*
	u8 iir;
	if ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
		return 0;
*/
	return uart->intcond;
}


//int uart_send(char c)
//{



void uart_intthr(void *arg)
{
	uart_t *uart = (uart_t *)arg;
	u8 iir, lsr;
	char c;

	for (;;) {
		mutexLock(uart->mutex);

		while ((iir = inb(uart->base + REG_IIR)) & IIR_IRQPEND)
			condWait(uart->intcond, uart->mutex, 0);

		/* Receive */
		if ((iir & IIR_DR) == IIR_DR) {
			while (1) {
				lsr = inb(uart->base + REG_LSR);
				/*if (lsr & 2)
					over = 1;*/

				if ((lsr & 1) == 0)
					break;

				c = inb(uart->base + REG_RBR);
				if (c == 0xd) {
					c = 0xa;
					uart->ready = 1;
				}

				if (c ==  0x7f) {
					c = 0;

					if (uart->rp != uart->rb) {
						uart->rp = ((uart->rp - 1) % uart->rbuffsz);
						c = '\b';
					}

				}
				else {
					uart->rbuff[uart->rp] = c;
					uart->rp = ((uart->rp + 1) % uart->rbuffsz);

					if (uart->rp == uart->rb)
						uart->rb = ((uart->rb + 1) % uart->rbuffsz);
				}

				/* echo */
				if (c) {
					if (uart->se == uart->sp)
						outb(uart->base, c);
					else
						uart->sbuff[uart->se] = c;
					uart->se = ((uart->se + 1) % uart->sbuffsz);

					if (c == '\b') {
						if (uart->se == uart->sp)
							outb(uart->base, ' ');
						else
							uart->sbuff[uart->se] = ' ';
						uart->se = ((uart->se + 1) % uart->sbuffsz);

						if (uart->se == uart->sp)
							outb(uart->base, '\b');
						else
							uart->sbuff[uart->se] = '\b';
						uart->se = ((uart->se + 1) % uart->sbuffsz);
					}
				}
			}

			condSignal(uart->rcond);
		}

		/* Transmit */
		if ((iir & IIR_THRE) == IIR_THRE) {
			uart->sp = ((uart->sp + 1) % uart->sbuffsz);
			if (uart->sp != uart->se) {
//printf("T %c\n", uart->sbuff[uart->sp]);
//for (;;);
				outb(uart->base + REG_THR, uart->sbuff[uart->sp]);
			}
			else
				condSignal(uart->scond);
		}

		mutexUnlock(uart->mutex);
	}

	return;
}





#if 0
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


static int uart_write(u8 d, size_t len, char *buff)
{
	uart_t *serial;
	unsigned int sp, se;
	unsigned int l;
	unsigned int cnt;
	int err;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return -EINVAL;

	if ((serial = uarts[d]) == NULL)
		return -ENOENT;

	if (!len)
		return 0;

	mutexLock(serial->mutex);

	/* Wait for transmitter */
	while (serial->sp != serial->se)
		if ((err = condWait(serial->scond, serial->mutex, 0)) < 0)
			return err;

	sp = serial->sp;
	se = serial->se;

	if (sp > se)
		l = min(sp - se, len);
	else
		l = min(serial->sbuffsz - se, len);

	/* It is assumed that send buffer and its size are constant after initialization */
	memcpy(&serial->sbuff[se], buff, l);

	cnt = l;
	if ((len > l) && (se >= sp)) {
		memcpy(serial->sbuff, buff + l, min(len - l, sp));
		cnt += min(len - l, sp);
	}

	/* Initialize sending process */

	serial->se = ((se + cnt) % serial->sbuffsz);

	if (se == sp)
		outb(serial->base, serial->sbuff[sp]);

	mutexUnlock(serial->mutex);

	return cnt;
}


int uart_readchunk(uart_t *serial, char *buff, unsigned int len, int *repfl)
{
	unsigned int l, cnt, bytes;
	int err;

	/* Wait for data or for timeout */
	mutexLock(serial->mutex);

	while ((serial->rp == serial->rb) || (!serial->ready)) {
		if ((err = condWait(serial->rcond, serial->mutex, 0)) < 0) {
			mutexUnlock(serial->mutex);
			return err;
		}
	}

	if (serial->rp > serial->rb) {
		l = min(serial->rp - serial->rb, len);
		bytes = serial->rp - serial->rb;
	}
	else {
		l = min(serial->rbuffsz - serial->rb, len);
		bytes = serial->rbuffsz - serial->rb + serial->rp;
	}

	memcpy(buff, &serial->rbuff[serial->rb], l);

	cnt = l;
	if ((len > l) && (serial->rp < serial->rb)) {
		memcpy(buff + l, &serial->rbuff[0], min(len - l, serial->rp));
		cnt += min(len - l, serial->rp);
	}
	serial->rb = ((serial->rb + cnt) % serial->rbuffsz);

	/* Suggest repetition */
	if (bytes - cnt)
		*repfl = 1;
	else
		*repfl = 0;


	mutexUnlock(serial->mutex);

	return cnt;
}


static int uart_read(u8 d, size_t len, char *buff)
{
	uart_t *serial;
	unsigned int l;
	int err, repfl;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return -EINVAL;

	if ((serial = uarts[d]) == NULL)
		return -ENOENT;

	l = 0;
	repfl = 1;

	while ((l < len) && repfl) {
		if ((err = uart_readchunk(serial, buff + l, min(SIZE_SERIAL_CHUNK, len), &repfl)) < 0) {
			if (!l)
				return err;
			break;
		}
		l += err;
	}

	if (repfl == 0)
		serial->ready = 0;

	return l;
}


static int uart_poll_status(u8 d)
{
	uart_t *serial;
	int revents = 0;

	if (d >= sizeof(uarts) / sizeof(uart_t *))
		return POLLNVAL;

	if ((serial = uarts[d]) == NULL)
		return POLLNVAL;

	mutexLock(serial->mutex);
	if ((serial->rp != serial->rb) && serial->ready)
		revents |= POLLIN|POLLRDNORM;
	if (serial->sp == serial->se)
		revents |= POLLOUT|POLLWRNORM;
	mutexUnlock(serial->mutex);

	return revents;
}


u8 uart_get(oid_t *oid)
{
	unsigned int i;

	for (i = 0; i < sizeof(uarts) / sizeof(uart_t); i++) {
		if ((uarts[i]->oid.id == oid->id) && (uarts[i]->oid.port == oid->port))
			return i;
	}
	return 0;
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

	memset((*uart), 0, sizeof(uart_t));

	(*uart)->base = base;
	(*uart)->irq = irq;

	(*uart)->rbuffsz = sizeof((*uart)->rbuff);
	(*uart)->rb = (*uart)->rp = 0;

	(*uart)->ready = 0;

	(*uart)->sbuffsz = sizeof((*uart)->sbuff);
	(*uart)->sp = (unsigned int)-1;
	(*uart)->se = 0;

	condCreate(&(*uart)->intcond);
	mutexCreate(&(*uart)->mutex);

	interrupt(irq, uart_interrupt, (*uart), (*uart)->intcond, &(*uart)->inth);

	u8 *stack;
	stack = (u8 *)malloc(4096);

//stack = mmap((void *)0, 4096 * 2, 0, 0, NULL, 0);
//stack += 0x10;

	condCreate(&(*uart)->rcond);
	condCreate(&(*uart)->scond);

//printf("user stack=%p\n", stack);

//for (;;);

	beginthread(uart_intthr, 1, stack, 4096, (void *)*uart);

//for (;;);

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


void poolthr(void *arg)
{
	u32 port = (u32)arg;
	msg_t msg;
	unsigned int rid;

	for (;;) {
		if (msgRecv(port, &msg, &rid) < 0)
			continue;

		switch (msg.type) {
		case mtOpen:
			break;
		case mtWrite:
			msg.o.io.err = uart_write(uart_get(&msg.i.io.oid), msg.i.size, msg.i.data);
			break;
		case mtRead:
			msg.o.io.err = uart_read(uart_get(&msg.i.io.oid), msg.o.size, msg.o.data);
			break;
		case mtClose:
			break;
		case mtGetAttr:
			if (msg.i.attr.type == atPollStatus)
				msg.o.attr.val = uart_poll_status(uart_get(&msg.i.io.oid));
			else
				msg.o.attr.val = -EINVAL;
			break;
		}

		msgRespond(port, &msg, rid);
	}
}


int main(void)
{
	void *base = (void *)0x3f8;
	unsigned int n = 4;
	u32 port;

	printf("pc-uart: Initializing UART 16550 driver %s\n", "");

	_uart_init(base, n, BPS_115200, &uarts[0]);

	portCreate(&port);
	if (portRegister(port, "/dev/ttyS0", &uarts[0]->oid) < 0) {
		printf("Can't register port %d\n", port);
//for (;;);
		return -1;
	}
	/*if (portRegister(port, "/dev/ttyS1", &uarts[1]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}
	if (portRegister(port, "/dev/ttyS2", &uarts[2]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}
	if (portRegister(port, "/dev/ttyS3", &uarts[3]->oid) < 0) {
		printf("Can't register port %d\n", port);
		return -1;
	}*/

	void *stack = malloc(2048);

	/* Run threads */
	beginthread(poolthr, 1, stack, 2048, (void *)port);
	poolthr((void *)port);

	return 0;
}
