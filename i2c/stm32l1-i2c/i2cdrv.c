/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * STM32L1 I2C driver
 *
 * Copyright 2017 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include HAL
#include "i2cdrv.h"
#include "gpiodrv.h"
#include "proc/threads.h"
#include "proc/msg.h"


/* Temporary solution */
unsigned int i2cdrv_id;


struct {
	volatile unsigned int *base;
	volatile unsigned int *rcc;

	char busy;

	thread_t *idleEv;
	thread_t *irqEv;

	spinlock_t idleSp;
	spinlock_t irqSp;

	intr_handler_t irqHandler;
} i2cdrv_common;


enum { _I2C_READ = 0, _I2C_WRITE };


enum { i2c_cr1 = 0, i2c_cr2, i2c_oar1, i2c_oar2, i2c_dr, i2c_sr1, i2c_sr2, i2c_ccr, i2c_trise };


enum { rcc_cr = 0, rcc_icscr, rcc_cfgr, rcc_cir, rcc_ahbrstr, rcc_apb2rstr, rcc_apb1rstr,
	rcc_ahbenr, rcc_apb2enr, rcc_apb1enr, rcc_ahblpenr, rcc_apb2lpenr, rcc_apb1lpenr, rcc_csr };


static int i2cdrv_irqHandler(unsigned int n, cpu_context_t *context, void *arg)
{
	/* Event irq disable */
	*(i2cdrv_common.base + i2c_cr2) &= ~(3 << 9);
	proc_threadWakeup(&i2cdrv_common.irqEv);

	return 0;
}


static inline void i2cdv_waitForIrq(void)
{
	hal_spinlockSet(&i2cdrv_common.irqSp);
	*(i2cdrv_common.base + i2c_cr2) |= (3 << 9);
	proc_threadWait(&i2cdrv_common.irqEv, &i2cdrv_common.irqSp, 0);
	hal_spinlockClear(&i2cdrv_common.irqSp);
}


static unsigned int i2cdrv_transaction(char op, char addr, char reg, void *buff, unsigned int count)
{
	int i;

	if (count < 1)
		return 0;

	hal_spinlockSet(&i2cdrv_common.idleSp);
	/* Wait for i2c idle */
	while (i2cdrv_common.busy)
		proc_threadWait(&i2cdrv_common.idleEv, &i2cdrv_common.idleSp, 0);

	i2cdrv_common.busy = 1;
	hal_cpuSetDevBusy(1);
	*(i2cdrv_common.base + i2c_cr2) |= (3 << 9);

	/* Start condition generation */
	*(i2cdrv_common.base + i2c_cr1) |= (1 << 8);
	hal_spinlockClear(&i2cdrv_common.idleSp);

	while (!(*(i2cdrv_common.base + i2c_sr1) & 1))
		i2cdv_waitForIrq();

	*(i2cdrv_common.base + i2c_dr) = addr << 1;

	while (!((*(i2cdrv_common.base + i2c_sr1) & 2) && (*(i2cdrv_common.base + i2c_sr2) & 1)))
		i2cdv_waitForIrq();

	while (!(*(i2cdrv_common.base + i2c_sr1) & (1 << 7)))
		i2cdv_waitForIrq();

	*(i2cdrv_common.base + i2c_dr) = reg;

	if (op == _I2C_READ) {
		*(i2cdrv_common.base + i2c_cr1) |= (1 << 8);
		if (count > 1)
			*(i2cdrv_common.base + i2c_cr1) |= (1 << 10);

		while (!(*(i2cdrv_common.base + i2c_sr1) & 1))
			i2cdv_waitForIrq();

		*(i2cdrv_common.base + i2c_dr) = (addr << 1) | 1;

		while (!((*(i2cdrv_common.base + i2c_sr1) & 2) && (*(i2cdrv_common.base + i2c_sr2) & 1)))
			i2cdv_waitForIrq();

		for (i = 0; i < count; ++i) {
			while (!(*(i2cdrv_common.base + i2c_sr1) & (1 << 6)))
				i2cdv_waitForIrq();

			if ((i + 2) >= count)
				*(i2cdrv_common.base + i2c_cr1) &= ~(1 << 10);

			((char *)buff)[i] = *(i2cdrv_common.base + i2c_dr);
		}
	}
	else {
		for (i = 0; i < count; ++i) {
			while (!(*(i2cdrv_common.base + i2c_sr1) & (1 << 7)))
				i2cdv_waitForIrq();

			*(i2cdrv_common.base + i2c_dr) = ((char *)buff)[i];
		}

		while (!(*(i2cdrv_common.base + i2c_sr1) & (1 << 7)))
			i2cdv_waitForIrq();
	}

	*(i2cdrv_common.base + i2c_cr1) |= (1 << 9);

	hal_spinlockSet(&i2cdrv_common.idleSp);
	i2cdrv_common.busy = 0;
	hal_cpuSetDevBusy(0);
	proc_threadWakeup(&i2cdrv_common.idleEv);
	hal_spinlockClear(&i2cdrv_common.idleSp);

	return i;
}


static void i2cdrv_thread(void *arg)
{
	char buff[64];
	unsigned int msgsz, t;
	msghdr_t hdr;
	i2cdrv_devctl_t *devctl = (i2cdrv_devctl_t *)buff;

	for (;;) {
		msgsz = proc_recv(i2cdrv_id, buff, sizeof(buff), &hdr);

		switch (hdr.op) {
		case MSG_READ:
			/* Not implemented yet */
			if (hdr.type == MSG_NORMAL)
				proc_respond(i2cdrv_id, EINVAL, NULL, 0);

			break;

		case MSG_WRITE:
			/* Not implemented yet */
			if (hdr.type == MSG_NORMAL)
				proc_respond(i2cdrv_id, EINVAL, NULL, 0);

			break;

		case MSG_DEVCTL: {
			switch (devctl->type) {
			case I2CDRV_DEF:
				/* Not implemented yet */
				if (hdr.type == MSG_NORMAL)
					proc_respond(i2cdrv_id, EINVAL, NULL, 0);

				break;

			case I2CDRV_GET:
				t = i2cdrv_transaction(_I2C_READ, devctl->addr, devctl->reg, buff, min(hdr.rsize, sizeof(buff)));
				if (hdr.type == MSG_NORMAL)
					proc_respond(i2cdrv_id, EOK, &buff, t);

				break;

			case I2CDRV_SET:
				t = i2cdrv_transaction(_I2C_WRITE, devctl->addr, devctl->reg, devctl->buff, msgsz - (unsigned int)(&(((i2cdrv_devctl_t *)0)->buff)));
				if (hdr.type == MSG_NORMAL)
					proc_respond(i2cdrv_id, EOK, &t, sizeof(t));

				break;

			default:
				if (hdr.type == MSG_NORMAL)
					proc_respond(i2cdrv_id, EINVAL, NULL, 0);

				break;
			}
			break;
		}

		default:
			if (hdr.type == MSG_NORMAL)
				proc_respond(i2cdrv_id, EINVAL, NULL, 0);
			break;
		}
	}

}


void i2cdrv_init(void)
{
	unsigned int t;

	i2cdrv_common.base = (void *)0x40005800;
	i2cdrv_common.rcc = (void *)0x40023800;

	*(i2cdrv_common.rcc + rcc_apb1enr) |= (1 << 22);

	hal_cpuDataBarrier();

	/* Disable I2C periph */
	*(i2cdrv_common.base + i2c_cr1) &= ~1;

	/* TODO: Use gpiodrv message interface */
	gpiodrv_configPin(GPIOB, 10, 2, 4, 1, 0, 0);
	gpiodrv_configPin(GPIOB, 11, 2, 4, 1, 0, 0);

	/* Enable ACK after each byte */
	*(i2cdrv_common.base + i2c_cr2) |= 1 << 10;

	/* Peripheral clock = 2 MHz */
	t = *(i2cdrv_common.base + i2c_cr2) & ~0x1ff;
	*(i2cdrv_common.base + i2c_cr2) = t | (1 << 2);

	/* 95,325 kHz SCK */
	t = *(i2cdrv_common.base + i2c_ccr) & ~((1 << 14) | 0x7ff);
	*(i2cdrv_common.base + i2c_ccr) = t | 0xb;

	/* 500 ns SCL rise time */
	t = *(i2cdrv_common.base + i2c_trise) & ~0x1ff;
	*(i2cdrv_common.base + i2c_trise) = t | 3;

	/* Enable I2C periph */
	*(i2cdrv_common.base + i2c_cr1) |= 1;

	i2cdrv_common.idleEv = NULL;
	i2cdrv_common.irqEv = NULL;

	i2cdrv_common.busy = 0;

	hal_spinlockCreate(&i2cdrv_common.idleSp, "i2c idle spinlock");
	hal_spinlockCreate(&i2cdrv_common.irqSp, "i2c irq spinlock");

	i2cdrv_common.irqHandler.next = NULL;
	i2cdrv_common.irqHandler.prev = NULL;
	i2cdrv_common.irqHandler.f = i2cdrv_irqHandler;
	i2cdrv_common.irqHandler.data = NULL;
	i2cdrv_common.irqHandler.pmap = NULL;

	hal_interruptsSetHandler(49, &i2cdrv_common.irqHandler);

	proc_portCreate(&i2cdrv_id);
	proc_portRegister(i2cdrv_id, "/i2cdrv");

	proc_threadCreate(0, i2cdrv_thread, 2, 512, 0, 0);
}
