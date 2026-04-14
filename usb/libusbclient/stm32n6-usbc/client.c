/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * Device-side CDC ACM driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdatomic.h>
#include "phy.h"
#include "client.h"

#define THREADS_PRIORITY 4
#define STACKSZ          384

#define USB_REG(x) stm_common.dc.base[x]


static struct {
	usb_dc_t dc;
	usb_common_data_t data;
	char stack[STACKSZ] __attribute__((aligned(8)));
} stm_common;


void *usbclient_allocBuff(uint32_t size)
{
	return mmap(NULL, size, PROT_WRITE | PROT_READ, MAP_UNCACHED | MAP_ANONYMOUS, -1, 0);
}


void usbclient_buffDestory(void *addrs, uint32_t size)
{
	munmap(addrs, size);
}


static int usbclient_intr(unsigned int intr, void *data)
{
	uint32_t gintsts = USB_REG(GINTSTS);
	uint32_t gintmsk = USB_REG(GINTMSK);
	uint32_t irqStatus = gintsts & gintmsk;

	ctrl_hifiq_handler(&irqStatus);

	if (irqStatus == 0UL) {
		return -1;
	}

	stm_common.dc.pending_event |= irqStatus;

	return EOK;
}


static void usbclient_irqThread(void *arg)
{
	mutexLock(stm_common.dc.irqLock);
	while (stm_common.dc.runIrqThread == 1U) {
		condWait(stm_common.dc.irqCond, stm_common.dc.irqLock, 0);

		uint32_t pending = atomic_exchange(&stm_common.dc.pending_event, 0U);

		/* low-speed IRQ handling */
		ctrl_lifiq_handler(pending);

		if ((stm_common.dc.currEvent != stm_common.dc.prevEvent) && (stm_common.dc.cbEvent != NULL)) {
			stm_common.dc.cbEvent(stm_common.dc.currEvent, stm_common.dc.ctxUser);
			stm_common.dc.prevEvent = stm_common.dc.currEvent;
		}
	}
	mutexUnlock(stm_common.dc.irqLock);

	endthread();
}


int usbclient_send(int endpt, const void *data, unsigned int len)
{
	stm32n6_endpt_t *ep;
	int res = (int)len;

	if (endpt >= ENDPOINTS_NUMBER || endpt < 0) {
		return -EINVAL;
	}

	if (stm_common.dc.isPortOpened == 0U) {
		return -EIO;
	}

	ep = &stm_common.data.endpts[(uint8_t)endpt];

	if (ep->in.type == USB_EP_TYPE_BULK) {
		clbc_epTransmit((uint8_t)endpt, (uint8_t *)data, len);

		while (ep->in.xfer_active == 1U) {
			semaphoreDown(&stm_common.dc.semBulkTx, 0);
		}

		if (ep->in.xfer_failed != 0U) {
			res = -EIO;
		}
		else {
			res = res - (USB_REG(DIEPTSIZ0 + endpt * EP_STRIDE) & 0x7FFFFUL);
		}

		semaphoreUp(&stm_common.dc.semBulkTx);
	}

	return (int)res;
}


int usbclient_receive(int endpt, void *data, unsigned int len)
{
	stm32n6_endpt_t *ep;
	int res = (int)len;

	if (endpt >= ENDPOINTS_NUMBER || endpt < 0) {
		return -EINVAL;
	}

	ep = &stm_common.data.endpts[(uint8_t)endpt];

	if (ep->out.type == USB_EP_TYPE_BULK) {
		clbc_epReceive((uint8_t)endpt, (uint8_t *)data, len);

		while (ep->out.xfer_active == 1U) {
			semaphoreDown(&stm_common.dc.semBulkRx, 0);
		}

		if (ep->out.xfer_failed != 0U) {
			res = -EIO;
		}
		else {
			res = res - (USB_REG(DOEPTSIZ0 + endpt * EP_STRIDE) & 0x7FFFFUL);
		}

		semaphoreUp(&stm_common.dc.semBulkRx);
	}

	return (int)res;
}


static void usbclient_freeResorces(void)
{
	(void)resourceDestroy(stm_common.dc.irqLock);
	(void)resourceDestroy(stm_common.dc.irqCond);
	(void)resourceDestroy(stm_common.dc.endp0Lock);
	(void)resourceDestroy(stm_common.dc.endp0Cond);
	(void)resourceDestroy(stm_common.dc.inth);
	(void)semaphoreDone(&stm_common.dc.semBulkTx);
	(void)semaphoreDone(&stm_common.dc.semBulkRx);

	/* unmap registers */
	phy_unmapRegs();

	/* unmap descriptors memory */
	usbclient_buffDestory((void *)stm_common.data.setupMem, USB_BUFFER_SIZE);
}


int usbclient_destroy(void)
{
	stm_common.dc.runIrqThread = 0;
	(void)condSignal(stm_common.dc.irqCond);

	(void)threadJoin(-1, 0);

	usbclient_freeResorces();

	return EOK;
}


int usbclient_init(usb_desc_list_t *desList)
{
	int res = phy_mapRegs();
	if (res < 0) {
		return res;
	}

	memset((uint8_t *)&stm_common + offsetof(usb_dc_t, base), 0, sizeof(stm_common) - offsetof(usb_dc_t, base));

	stm_common.dc.base = phy_getOtgBase();

	if (stm_common.dc.base == MAP_FAILED) {
		return -ENOMEM;
	}
	stm_common.dc.runIrqThread = 1U;
	stm_common.data.setupMem = usbclient_allocBuff(USB_BUFFER_SIZE);
	stm_common.dc.currEvent = USBCLIENT_EV_DISCONNECT;

	if (stm_common.dc.cbEvent != NULL) {
		stm_common.dc.cbEvent(stm_common.dc.currEvent, stm_common.dc.ctxUser);
	}

	if (desc_init(desList, &stm_common.data, &stm_common.dc) < 0) {
		usbclient_freeResorces();
		return -ENOMEM;
	}

	if (mutexCreate(&stm_common.dc.irqLock) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	if (condCreate(&stm_common.dc.irqCond) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	if (mutexCreate(&stm_common.dc.endp0Lock) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	if (condCreate(&stm_common.dc.endp0Cond) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	if (semaphoreCreate(&stm_common.dc.semBulkTx, 1U) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	if (semaphoreCreate(&stm_common.dc.semBulkRx, 1U) != EOK) {
		usbclient_freeResorces();
		return -ENOENT;
	}

	clbc_init(&stm_common.data, &stm_common.dc);
	ctrl_init(&stm_common.data, &stm_common.dc);

	/* initialize low-level drivers */
	res = phy_clk_reset();
	if (res < 0) {
		fprintf(stderr, "[USB CLIENT]: clk reset failed\n");
		return -EIO;
	}

	res = phy_usbss_init();
	if (res < 0) {
		fprintf(stderr, "[USB CLIENT]: OTG reset failed\n");
		return -EIO;
	}

	res = phy_clear_config();
	if (res < 0) {
		fprintf(stderr, "[USB CLIENT]: OTG clear failed\n");
		return -EIO;
	}

	stm_common.dc.threadNum = beginthread(usbclient_irqThread, THREADS_PRIORITY, stm_common.stack, STACKSZ, NULL);
	if (stm_common.dc.threadNum < 0) {
		fprintf(stderr, "[USB CLIENT] failed to create usb thread\n");
		usbclient_freeResorces();
		return -EIO;
	}
	interrupt(phy_getIrq(), usbclient_intr, NULL, stm_common.dc.irqCond, &stm_common.dc.inth);

	/* Configure interrupts */
	phy_config(&stm_common.dc);

	return EOK;
}


void usbclient_setUserContext(void *ctxUser)
{
	stm_common.dc.ctxUser = ctxUser;
}


void usbclient_setEventCallback(void (*cbEvent)(int, void *))
{
	stm_common.dc.cbEvent = cbEvent;
}


void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *))
{
	stm_common.dc.cbClassSetup = cbClassSetup;
}
