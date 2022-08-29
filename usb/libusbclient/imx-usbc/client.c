/*
 * Phoenix-RTOS
 *
 * client - usb client
 *
 * Copyright 2019-2021 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski, Gerard Swiderski
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
#include <phoenix/arch/imxrt.h>

#include "phy.h"
#include "client.h"

#define THREADS_PRIORITY 2
#define STACKSZ 512


struct {
	usb_dc_t dc;
	usb_common_data_t data;

	char stack[STACKSZ] __attribute__ ((aligned(8)));
} imx_common;


int usbclient_send(int endpt, const void *data, unsigned int len)
{
	dtd_t *res;

	if (len > USB_BUFFER_SIZE)
		return -1;

	if (!imx_common.data.endpts[endpt].caps[USB_ENDPT_DIR_IN].init)
		return -1;

	memcpy(imx_common.data.endpts[endpt].buf[USB_ENDPT_DIR_IN].vBuffer, data, len);
	res = ctrl_execTransfer(endpt, imx_common.data.endpts[endpt].buf[USB_ENDPT_DIR_IN].pBuffer, len, USB_ENDPT_DIR_IN);

	if (res == NULL || DTD_ERROR(res))
		return -1;

	return len - DTD_SIZE(res);
}


static int usbclient_rcvEndp0(void *data, unsigned int len)
{
	int res = -1;

	(void)len; /* FIXME: unused */

	mutexLock(imx_common.dc.endp0Lock);
	while ((imx_common.dc.op != DC_OP_RCV_ENDP0) && (imx_common.dc.op != DC_OP_RCV_ERR))
		condWait(imx_common.dc.endp0Cond, imx_common.dc.endp0Lock, 0);
	mutexUnlock(imx_common.dc.endp0Lock);

	if (imx_common.dc.op != DC_OP_RCV_ERR) {
		res = imx_common.data.endpts[0].buf[USB_ENDPT_DIR_OUT].len;
		memcpy(data, (const char *)imx_common.data.endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer, res); /* copy data to buffer */
		imx_common.dc.op = DC_OP_NONE;

		ctrl_execTransfer(0, imx_common.data.endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN); /* ACK */
	}

	return res;
}


int usbclient_receive(int endpt, void *data, unsigned int len)
{
	dtd_t *dtd;
	int res;

	if (len > USB_BUFFER_SIZE)
		return -1;

	if (!imx_common.data.endpts[endpt].caps[USB_ENDPT_DIR_OUT].init)
		return -1;

	if (endpt == 0)
		return usbclient_rcvEndp0(data, len);

	dtd = ctrl_execTransfer(endpt, imx_common.data.endpts[endpt].buf[USB_ENDPT_DIR_OUT].pBuffer, USB_BUFFER_SIZE, USB_ENDPT_DIR_OUT);

	if (dtd == NULL || DTD_ERROR(dtd))
		return -1;

	res = USB_BUFFER_SIZE - DTD_SIZE(dtd);
	if (res > len)
		res = len;

	memcpy(data, (const char *)imx_common.data.endpts[endpt].buf[USB_ENDPT_DIR_OUT].vBuffer, res);

	return res;
}


static int usbclient_intr(unsigned int intr, void *data)
{
	ctrl_hfIrq();

	return 0;
}


static void usbclient_irqThread(void *arg)
{
	unsigned i;
	int event;

	mutexLock(imx_common.dc.irqLock);
	while (imx_common.dc.runIrqThread) {
		condWait(imx_common.dc.irqCond, imx_common.dc.irqLock, 0);

		/* Low frequency interrupts, handle for OUT control endpoint */
		if ((imx_common.dc.setupstat & 0x1) && imx_common.dc.runIrqThread)
			desc_classSetup(&imx_common.dc.setup);

		ctrl_lfIrq();

		/* Initialize endpoints */
		if (imx_common.dc.op == DC_OP_INIT) {
			imx_common.dc.endptFailed = 0;
			event = USBCLIENT_EV_INIT;

			ctrl_initQtd();

			for (i = 1; i < ENDPOINTS_NUMBER; ++i) {
				if (ctrl_endptInit(i, &imx_common.data.endpts[i]) < 0) {
					imx_common.dc.endptFailed = i;
					event = USBCLIENT_EV_FAULT;
					break;
				}
			}

			if (imx_common.dc.cbEvent) {
				imx_common.dc.cbEvent(event, imx_common.dc.ctxUser);
			}

			imx_common.dc.op = DC_OP_NONE;
		}
	}
	mutexUnlock(imx_common.dc.irqLock);

	endthread();
}


static void usbclient_cleanData(void)
{
	int i;

	usbclient_buffDestory((void *)imx_common.dc.base, USB_BUFFER_SIZE);
	usbclient_buffDestory((void *)imx_common.data.setupMem, USB_BUFFER_SIZE);

	usbclient_buffDestory((void *)imx_common.dc.dtdMem, USB_BUFFER_SIZE);
	usbclient_buffDestory((void *)imx_common.dc.endptqh, USB_BUFFER_SIZE);

	for (i = 0; i < ENDPOINTS_NUMBER; ++i) {
		if (imx_common.data.endpts[i].caps[USB_ENDPT_DIR_IN].init) {
			usbclient_buffDestory((void *)imx_common.data.endpts[i].buf[USB_ENDPT_DIR_IN].vBuffer, USB_BUFFER_SIZE);
			imx_common.data.endpts[i].buf[USB_ENDPT_DIR_IN].vBuffer = NULL;
		}

		if (imx_common.data.endpts[i].caps[USB_ENDPT_DIR_OUT].init) {
			usbclient_buffDestory((void *)imx_common.data.endpts[i].buf[USB_ENDPT_DIR_OUT].vBuffer, USB_BUFFER_SIZE);
			imx_common.data.endpts[i].buf[USB_ENDPT_DIR_OUT].vBuffer = NULL;
		}
	}
}


void usbclient_setUserContext(void *ctxUser)
{
	imx_common.dc.ctxUser = ctxUser;
}


void usbclient_setEventCallback(void (*cbEvent)(int, void *))
{
	imx_common.dc.cbEvent = cbEvent;
}


void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *))
{
	imx_common.dc.cbClassSetup = cbClassSetup;
}


int usbclient_init(usb_desc_list_t *desList)
{
	int i;

	phy_init();

	imx_common.dc.irqLock = 0;
	imx_common.dc.irqCond = 0;
	imx_common.dc.endp0Lock = 0;
	imx_common.dc.endp0Cond = 0;
	imx_common.dc.endptFailed = 0;
	imx_common.dc.runIrqThread = 1;

	imx_common.dc.connected = 0;
	imx_common.dc.dev_addr = 0;
	imx_common.dc.base = phy_getBase(USB_BUFFER_SIZE);

	if (imx_common.dc.base == MAP_FAILED)
		return -ENOMEM;

	imx_common.data.setupMem = usbclient_allocBuff(USB_BUFFER_SIZE);

	if (imx_common.data.setupMem == MAP_FAILED){
		usbclient_cleanData();
		return -ENOMEM;
	}

	for (i = 1; i < ENDPOINTS_NUMBER; ++i) {
		imx_common.data.endpts[i].caps[USB_ENDPT_DIR_OUT].init = 0;
		imx_common.data.endpts[i].caps[USB_ENDPT_DIR_IN].init = 0;
	}

	if (desc_init(desList, &imx_common.data, &imx_common.dc) < 0) {
		usbclient_cleanData();
		return -ENOMEM;
	}

	if (mutexCreate(&imx_common.dc.irqLock) != EOK) {
		usbclient_cleanData();
		return -ENOENT;
	}

	if (condCreate(&imx_common.dc.irqCond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(imx_common.dc.irqLock);
		return -ENOENT;
	}

	if (mutexCreate(&imx_common.dc.endp0Lock) != EOK) {
		usbclient_cleanData();
		resourceDestroy(imx_common.dc.irqLock);
		resourceDestroy(imx_common.dc.irqCond);
		return -ENOENT;
	}

	if (condCreate(&imx_common.dc.endp0Cond) != EOK) {
		usbclient_cleanData();
		resourceDestroy(imx_common.dc.irqLock);
		resourceDestroy(imx_common.dc.irqCond);
		resourceDestroy(imx_common.dc.endp0Lock);
		return -ENOENT;
	}

	if (ctrl_init(&imx_common.data, &imx_common.dc) < 0) {
		usbclient_cleanData();
		resourceDestroy(imx_common.dc.irqLock);
		resourceDestroy(imx_common.dc.irqCond);
		resourceDestroy(imx_common.dc.endp0Lock);
		resourceDestroy(imx_common.dc.endp0Cond);
		return -ENOENT;
	}

	beginthread(usbclient_irqThread, THREADS_PRIORITY, imx_common.stack, STACKSZ, NULL);
	interrupt(phy_getIrq(), usbclient_intr, NULL, imx_common.dc.irqCond, &imx_common.dc.inth);

	/* Wait 500ms and catch endpoint initialization error */
	for (i = 0; i < 10; i++) {
		if (imx_common.dc.endptFailed > 0) {
			usbclient_destroy();
			return -ENOMEM;
		}

		usleep(50 * 1000);
	}

	return EOK;
}


int usbclient_destroy(void)
{
	imx_common.dc.runIrqThread = 0;
	condSignal(imx_common.dc.irqCond);

	ctrl_reset();

	threadJoin(-1, 0);

	resourceDestroy(imx_common.dc.inth);
	resourceDestroy(imx_common.dc.irqLock);
	resourceDestroy(imx_common.dc.irqCond);
	resourceDestroy(imx_common.dc.endp0Lock);
	resourceDestroy(imx_common.dc.endp0Cond);

	usbclient_cleanData();

	return 0;
}
