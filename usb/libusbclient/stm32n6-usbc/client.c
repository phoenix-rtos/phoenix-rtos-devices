/*
 * Phoenix-RTOS
 *
 * client - usb client
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
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
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>

#include "phy.h"
#include "client.h"

#define THREADS_PRIORITY 2
#define STACKSZ 512


struct {
	usb_dc_t dc;
	usb_common_data_t data;

	char stack[STACKSZ] __attribute__ ((aligned(8)));
} usb_common;


// TODO: Olaf:
int usbclient_send(int endpt, const void *data, unsigned int len)
{
	dtd_t *res;

	if (len > USB_BUFFER_SIZE)
		return -1;

	if (!usb_common.data.endpts[endpt].caps[USB_ENDPT_DIR_IN].init)
		return -1;

	memcpy(usb_common.data.endpts[endpt].buf[USB_ENDPT_DIR_IN].vBuffer, data, len);
	res = ctrl_execTransfer(endpt, usb_common.data.endpts[endpt].buf[USB_ENDPT_DIR_IN].pBuffer, len, USB_ENDPT_DIR_IN);

	if (res == NULL || DTD_ERROR(res))
		return -1;

	return len - DTD_SIZE(res);
}

// TODO: Olaf:
static int usbclient_rcvEndp0(void *data, unsigned int len)
{
	int res = -1;

	(void)len; /* FIXME: unused */

	mutexLock(usb_common.dc.endp0Lock);
	while ((usb_common.dc.op != DC_OP_RCV_ENDP0) && (usb_common.dc.op != DC_OP_RCV_ERR))
		condWait(usb_common.dc.endp0Cond, usb_common.dc.endp0Lock, 0);
	mutexUnlock(usb_common.dc.endp0Lock);

	if (usb_common.dc.op != DC_OP_RCV_ERR) {
		res = usb_common.data.endpts[0].buf[USB_ENDPT_DIR_OUT].len;
		memcpy(data, (const char *)usb_common.data.endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer, res); /* copy data to buffer */
		usb_common.dc.op = DC_OP_NONE;

		ctrl_execTransfer(0, usb_common.data.endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN); /* ACK */
	}

	return res;
}

// TODO: Olaf:
int usbclient_receive(int endpt, void *data, unsigned int len)
{
	dtd_t *dtd;
	int res;

	if (len > USB_BUFFER_SIZE)
		return -1;

	if (!usb_common.data.endpts[endpt].caps[USB_ENDPT_DIR_OUT].init)
		return -1;

	if (endpt == 0)
		return usbclient_rcvEndp0(data, len);

	dtd = ctrl_execTransfer(endpt, usb_common.data.endpts[endpt].buf[USB_ENDPT_DIR_OUT].pBuffer, USB_BUFFER_SIZE, USB_ENDPT_DIR_OUT);

	if (dtd == NULL || DTD_ERROR(dtd))
		return -1;

	res = USB_BUFFER_SIZE - DTD_SIZE(dtd);
	if (res > len)
		res = len;

	memcpy(data, (const char *)usb_common.data.endpts[endpt].buf[USB_ENDPT_DIR_OUT].vBuffer, res);

	return res;
}

// TODO: Olaf:
static int usbclient_intr(unsigned int intr, void *data)
{
	ctrl_hfIrq();

	return 0;
}

// TODO: Olaf:
static void usbclient_irqThread(void *arg)
{
	unsigned i;
	int event;

	mutexLock(usb_common.dc.irqLock);
	while (usb_common.dc.runIrqThread) {
		condWait(usb_common.dc.irqCond, usb_common.dc.irqLock, 0);

		/* Low frequency interrupts, handle for OUT control endpoint */
		if ((usb_common.dc.setupstat & 0x1) && usb_common.dc.runIrqThread)
			desc_classSetup(&usb_common.dc.setup);

		ctrl_lfIrq();

		/* Initialize endpoints */
		if (usb_common.dc.op == DC_OP_INIT) {
			usb_common.dc.endptFailed = 0;
			event = USBCLIENT_EV_INIT;

			ctrl_initQtd();

			for (i = 1; i < ENDPOINTS_NUMBER; ++i) {
				if (ctrl_endptInit(i, &usb_common.data.endpts[i]) < 0) {
					usb_common.dc.endptFailed = i;
					event = USBCLIENT_EV_FAULT;
					break;
				}
			}

			if (usb_common.dc.cbEvent) {
				usb_common.dc.cbEvent(event, usb_common.dc.ctxUser);
			}

			usb_common.dc.op = DC_OP_NONE;
		}
	}
	mutexUnlock(usb_common.dc.irqLock);

	endthread();
}

static void usbclient_cleanData(void)
{
	int i;

	usbclient_buffDestroy((void *)usb_common.dc.base, USB_BUFFER_SIZE);
	usbclient_buffDestroy((void *)usb_common.data.setupMem, USB_BUFFER_SIZE);

	usbclient_buffDestroy((void *)usb_common.dc.dtdMem, USB_BUFFER_SIZE);
	usbclient_buffDestroy((void *)usb_common.dc.endptqh, USB_BUFFER_SIZE);

	for (i = 0; i < ENDPOINTS_NUMBER; ++i) {
		if (usb_common.data.endpts[i].caps[USB_ENDPT_DIR_IN].init) {
			usbclient_buffDestroy((void *)usb_common.data.endpts[i].buf[USB_ENDPT_DIR_IN].vBuffer, USB_BUFFER_SIZE);
			usb_common.data.endpts[i].buf[USB_ENDPT_DIR_IN].vBuffer = NULL;
		}

		if (usb_common.data.endpts[i].caps[USB_ENDPT_DIR_OUT].init) {
			usbclient_buffDestroy((void *)usb_common.data.endpts[i].buf[USB_ENDPT_DIR_OUT].vBuffer, USB_BUFFER_SIZE);
			usb_common.data.endpts[i].buf[USB_ENDPT_DIR_OUT].vBuffer = NULL;
		}
	}
}

void usbclient_setUserContext(void *ctxUser)
{
	usb_common.dc.ctxUser = ctxUser;
}

void usbclient_setEventCallback(void (*cbEvent)(int, void *))
{
	usb_common.dc.cbEvent = cbEvent;
}

void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *))
{
	usb_common.dc.cbClassSetup = cbClassSetup;
}

// TODO: Olaf:
int usbclient_init(usb_desc_list_t *desList)
{
	int i;

	phy_init();

	usb_common.dc.irqLock = 0;
	usb_common.dc.irqCond = 0;
	usb_common.dc.endp0Lock = 0;
	usb_common.dc.endp0Cond = 0;
	usb_common.dc.endptFailed = 0;
	usb_common.dc.runIrqThread = 1;

	usb_common.dc.connected = 0;
	usb_common.dc.dev_addr = 0;
	usb_common.dc.base = phy_getBase(USB_BUFFER_SIZE);

	if (usb_common.dc.base == MAP_FAILED)
		return -ENOMEM;

	usb_common.data.setupMem = usbclient_allocBuff(USB_BUFFER_SIZE);

	if (usb_common.data.setupMem == MAP_FAILED){
		usbclient_cleanData();
		return -ENOMEM;
	}

	for (i = 1; i < ENDPOINTS_NUMBER; ++i) {
		usb_common.data.endpts[i].caps[USB_ENDPT_DIR_OUT].init = 0;
		usb_common.data.endpts[i].caps[USB_ENDPT_DIR_IN].init = 0;
	}

	if (desc_init(desList, &usb_common.data, &usb_common.dc) < 0) {
		usbclient_cleanData();
		return -ENOMEM;
	}

	if (mutexCreate(&usb_common.dc.irqLock) != EOK) {
		usbclient_cleanData();
		return -ENOENT;
	}

	if (condCreate(&usb_common.dc.irqCond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(usb_common.dc.irqLock);
		return -ENOENT;
	}

	if (mutexCreate(&usb_common.dc.endp0Lock) != EOK) {
		usbclient_cleanData();
		resourceDestroy(usb_common.dc.irqLock);
		resourceDestroy(usb_common.dc.irqCond);
		return -ENOENT;
	}

	if (condCreate(&usb_common.dc.endp0Cond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(usb_common.dc.irqLock);
		resourceDestroy(usb_common.dc.irqCond);
		resourceDestroy(usb_common.dc.endp0Lock);
		return -ENOENT;
	}

	if (ctrl_init(&usb_common.data, &usb_common.dc) < 0) {
		usbclient_cleanData();
		resourceDestroy(usb_common.dc.irqLock);
		resourceDestroy(usb_common.dc.irqCond);
		resourceDestroy(usb_common.dc.endp0Lock);
		resourceDestroy(usb_common.dc.endp0Cond);
		return -ENOENT;
	}

	beginthread(usbclient_irqThread, THREADS_PRIORITY, usb_common.stack, STACKSZ, NULL);
	interrupt(phy_getIrq(), usbclient_intr, NULL, usb_common.dc.irqCond, &usb_common.dc.inth);

	/* Wait 500ms and catch endpoint initialization error */
	for (i = 0; i < 10; i++) {
		if (usb_common.dc.endptFailed > 0) {
			usbclient_destroy();
			return -ENOMEM;
		}

		usleep(50 * 1000);
	}

	return EOK;
}

// TODO: Olaf:
int usbclient_destroy(void)
{
	usb_common.dc.runIrqThread = 0;
	condSignal(usb_common.dc.irqCond);

	ctrl_reset();

	threadJoin(-1, 0);

	resourceDestroy(usb_common.dc.inth);
	resourceDestroy(usb_common.dc.irqLock);
	resourceDestroy(usb_common.dc.irqCond);
	resourceDestroy(usb_common.dc.endp0Lock);
	resourceDestroy(usb_common.dc.endp0Cond);

	usbclient_cleanData();

	return 0;
}
