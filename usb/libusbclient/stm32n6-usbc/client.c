/*
 * Phoenix-RTOS
 * client - usb client for STM32N6 (DWC2 Logic)
 * SLAVE MODE + SEQUENTIAL STATUS STAGE FIX
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski
 */

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <errno.h>

#include "phy.h"
#include "client.h"


#define THREADS_PRIORITY 3
#define STACKSZ          4096


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
	ctrl_hfIrq();

	return 0;
}

static void usbclient_irqThread(void *arg)
{
	unsigned i = 0;
	int event;
	mutexLock(stm_common.dc.irqLock);
	while (stm_common.dc.runIrqThread) {
		condWait(stm_common.dc.irqCond, stm_common.dc.irqLock, 0);

		/* after interrupt - longer operations */
		ctrl_lfIrq();
		// if ((stm_common.dc.setupstat & 0x1) && stm_common.dc.runIrqThread)
		// 	desc_classSetup(&stm_common.dc.setup);

		// ctrl_hfIrq();

		/* Initialize endpoints */
		// if (stm_common.dc.op == DC_OP_INIT) {
		// 	stm_common.dc.endptFailed = 0;
		// 	event = USBCLIENT_EV_INIT;

		// 	ctrl_initQtd();

		// 	for (i = 1; i < ENDPOINTS_NUMBER; ++i) {
		// 		if (ctrl_endptInit(i, &stm_common.data.endpts[i]) < 0) {
		// 			stm_common.dc.endptFailed = i;
		// 			event = USBCLIENT_EV_FAULT;
		// 			break;
		// 		}
		// 	}

		// 	if (stm_common.dc.cbEvent) {
		// 		stm_common.dc.cbEvent(event, stm_common.dc.ctxUser);
		// 	}

		// 	stm_common.dc.op = DC_OP_NONE;
		// }
	}
	mutexUnlock(stm_common.dc.irqLock);

	endthread();
}

int usbclient_send(int endpt, const void *data, unsigned int len)
{
	return -1;
}


int usbclient_receive(int endpt, void *data, unsigned int len)
{
	return EOK;
}


void usbclient_cleanData(void)
{
	usbclient_buffDestory((void *)stm_common.dc.base, FIFO_SIZE);
	usbclient_buffDestory((void *)stm_common.data.setupMem, FIFO_SIZE);
}


int usbclient_destroy(void)
{
	stm_common.dc.runIrqThread = 0;
	condSignal(stm_common.dc.irqCond);

	threadJoin(-1, 0);

	resourceDestroy(stm_common.dc.inth);
	resourceDestroy(stm_common.dc.irqLock);
	resourceDestroy(stm_common.dc.irqCond);
	resourceDestroy(stm_common.dc.endp0Lock);
	resourceDestroy(stm_common.dc.endp0Cond);

	usbclient_cleanData();

	return 0;
}


int usbclient_init(usb_desc_list_t *desList)
{
	int res = phy_init();

	if (res != 0) {
		return res;
	}

	stm_common.dc.base = phy_getOtgBase();

	if (stm_common.dc.base == MAP_FAILED) {
		return ENOMEM;
	}

	stm_common.dc.irqLock = 0;
	stm_common.dc.irqCond = 0;
	stm_common.dc.endp0Lock = 0;
	stm_common.dc.endp0Cond = 0;
	stm_common.dc.endptFailed = 0;
	stm_common.dc.runIrqThread = 1;

	stm_common.dc.connected = 0;
	stm_common.dc.dev_addr = 0;

	stm_common.data.setupMem = usbclient_allocBuff(USB_BUFFER_SIZE);


	if (desc_init(desList, &stm_common.data, &stm_common.dc) < 0) {
		usbclient_cleanData();
		return -ENOMEM;
	}

	if (mutexCreate(&stm_common.dc.irqLock) != EOK) {
		usbclient_cleanData();
		return -ENOENT;
	}

	if (condCreate(&stm_common.dc.irqCond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(stm_common.dc.irqLock);
		return -ENOENT;
	}

	if (mutexCreate(&stm_common.dc.endp0Lock) != EOK) {
		usbclient_cleanData();
		resourceDestroy(stm_common.dc.irqLock);
		resourceDestroy(stm_common.dc.irqCond);
		return -ENOENT;
	}

	if (condCreate(&stm_common.dc.endp0Cond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(stm_common.dc.irqLock);
		resourceDestroy(stm_common.dc.irqCond);
		resourceDestroy(stm_common.dc.endp0Lock);
		return -ENOENT;
	}

	if (clbc_init(&stm_common.data, &stm_common.dc) < 0) {
		usbclient_cleanData();
		resourceDestroy(stm_common.dc.irqLock);
		resourceDestroy(stm_common.dc.irqCond);
		resourceDestroy(stm_common.dc.endp0Lock);
		resourceDestroy(stm_common.dc.endp0Cond);
		return -ENOENT;
	}

	if (ctrl_init(&stm_common.data, &stm_common.dc) < 0) {
		usbclient_cleanData();
		resourceDestroy(stm_common.dc.irqLock);
		resourceDestroy(stm_common.dc.irqCond);
		resourceDestroy(stm_common.dc.endp0Lock);
		resourceDestroy(stm_common.dc.endp0Cond);
		return -ENOENT;
	}

	beginthread(usbclient_irqThread, THREADS_PRIORITY, stm_common.stack, STACKSZ, NULL);
	interrupt(phy_getIrq(), usbclient_intr, NULL, stm_common.dc.irqCond, &stm_common.dc.inth);

	/* Wait 500ms and catch endpoint initialization error */
	// for (int i = 0; i < 10; i++) {
	// 	if (stm_common.dc.endptFailed > 0) {
	// 		usbclient_destroy();
	// 		return -ENOMEM;
	// 	}

	// 	usleep(50 * 1000);
	// }
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
