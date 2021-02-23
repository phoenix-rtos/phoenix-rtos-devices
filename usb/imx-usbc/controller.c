/*
 * Phoenix-RTOS
 *
 * imx-controller - usb device controller driver
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
#include <sys/mman.h>

#include "client.h"
#include "phy.h"


struct {
	size_t qtdOffs;

	usb_dc_t *dc;
	usb_common_data_t *data;
} ctrl_common;


/* device cotroller register offsets */
enum {
	/* identification regs */
	id = 0x0, hwgeneral, hwhost, hwdevice, hwtxbuf, hwrxbuf,

	/* operational regs */
	gptimer0ld	= 0x20, gptimer0ctrl, gptimer1ld, gptimer1ctrl, sbuscfg,

	/* capability regs */
	caplength = 0x40, hciversion = 0x40, hcsparams, hccparams,
	dciversion = 0x48, dccparams,

	/* operational regs cont. */
	usbcmd = 0x50, usbsts, usbintr, frindex,
	periodiclistbase = 0x55, deviceaddr = 0x55, asynclistaddr = 0x56,
	endpointlistaddr = 0x56, burstsize = 0x58, txfilltunning, endptnak = 0x5E,
	endptnaken, configflag, portsc1, otgsc = 0x69, usbmode, endptsetupstat,
	endptprime, endptflush, endptstat, endptcomplete, endptctrl0, endptctrl1,
	endptctrl2, endptctrl3, endptctrl4, endptctrl5, endptctrl6, endptctrl7
};


static int ctrl_allocBuff(int endpt, int dir)
{
	/* Allocate buffer for the first time (initially vBuffer is NULL) */
	if (ctrl_common.data->endpts[endpt].buf[dir].vBuffer == NULL)
		ctrl_common.data->endpts[endpt].buf[dir].vBuffer = usbclient_allocBuff(USB_BUFFER_SIZE);

	if (ctrl_common.data->endpts[endpt].buf[dir].vBuffer == MAP_FAILED)
		return -ENOMEM;

	ctrl_common.data->endpts[endpt].buf[dir].pBuffer = VM_2_PHYM((void *)ctrl_common.data->endpts[endpt].buf[dir].vBuffer);
	ctrl_common.data->endpts[endpt].buf[dir].len = 0;

	return EOK;
}


void ctrl_initQtd(void)
{
	/*
	 * Reset qtd offset for non control endpoints everytime device is re/connected to
	 * host, at any time when desc_setup(REQ_SET_ADDRESS) is initiated
	 */

	ctrl_common.qtdOffs = 0;
}


static void *ctrl_allocQtdMem(void)
{
	if (!ctrl_common.qtdOffs) {
		/* Allocate buffer for the first time (initially dtdMem is NULL) */
		if (ctrl_common.dc->dtdMem == NULL)
			ctrl_common.dc->dtdMem = usbclient_allocBuff(USB_BUFFER_SIZE);

		if (ctrl_common.dc->dtdMem == MAP_FAILED)
			return MAP_FAILED;

		memset(ctrl_common.dc->dtdMem, 0, USB_BUFFER_SIZE);
	}

	return  (void *)(ctrl_common.dc->dtdMem + 0x40 * (ctrl_common.qtdOffs++));
}


static int ctrl_dtdInit(int endpt, int inQH, int outQh)
{
	dtd_t *dtd;
	int qh = endpt * 2;

	if (endpt == 0)
		return -EINVAL;

	/* initialize dtd for selected endpoints */
	if (outQh) {
		dtd = ctrl_allocQtdMem();

		if (dtd == MAP_FAILED)
			return -ENOMEM;

		ctrl_common.dc->endptqh[qh].base = (((uint32_t)va2pa(dtd)) & ~0xfff);
		ctrl_common.dc->endptqh[qh].size = 0x40;
		ctrl_common.dc->endptqh[qh].head = dtd;
		ctrl_common.dc->endptqh[qh].tail = dtd;
	}

	if (inQH) {
		dtd = ctrl_allocQtdMem();

		if (dtd == MAP_FAILED)
			return -ENOMEM;

		ctrl_common.dc->endptqh[++qh].base = (((uint32_t)va2pa(dtd)) & ~0xfff) + (64 * sizeof(dtd_t));
		ctrl_common.dc->endptqh[qh].size = 0x40;
		ctrl_common.dc->endptqh[qh].head = dtd + 64;
		ctrl_common.dc->endptqh[qh].tail = dtd + 64;
	}

	return EOK;
}


static int ctrl_initEndptQh(int endpt, int dir, endpt_data_t *endpt_init)
{
	uint32_t setup = 0;
	int qh = endpt * 2 + dir;

	if (ctrl_allocBuff(endpt, dir) < 0)
		return -ENOMEM;

	ctrl_common.dc->endptqh[qh].caps =  endpt_init->caps[dir].max_pkt_len << 16;
	ctrl_common.dc->endptqh[qh].caps |= endpt_init->caps[dir].ios << 15;
	ctrl_common.dc->endptqh[qh].caps |= endpt_init->caps[dir].zlt << 29;
	ctrl_common.dc->endptqh[qh].caps |= endpt_init->caps[dir].mult << 30;
	ctrl_common.dc->endptqh[qh].dtd_next = 1;

	setup |= endpt_init->ctrl[dir].data_toggle << (6 + dir * 16);
	setup |= endpt_init->ctrl[dir].type << (2 + dir * 16);
	setup |= endpt_init->ctrl[dir].stall << (0 + dir * 16);

	return setup;
}


int ctrl_endptInit(int endpt, endpt_data_t *endpt_init)
{
	int8_t i;
	int res = EOK;
	uint32_t setup = 0;

	if (endpt == 0)
		return -EINVAL;

	if ((res = ctrl_dtdInit(endpt, endpt_init->caps[USB_ENDPT_DIR_IN].init, endpt_init->caps[USB_ENDPT_DIR_OUT].init)) != EOK)
		return res;

	for(i = 0; i < ENDPOINTS_DIR_NB; ++i) {
		if (endpt_init->caps[i].init)
			setup |= ctrl_initEndptQh(endpt, i, endpt_init);
	}

	*(ctrl_common.dc->base + endptctrl0 + endpt) = setup;

	for(i = 0; i < ENDPOINTS_DIR_NB; ++i) {
		if (endpt_init->caps[i].init)
			*(ctrl_common.dc->base + endptctrl0 + endpt) |= 1 << (7 + i * 16);
	}

	return res;
}


int ctrl_endpt0Init(void)
{
	uint32_t qh_addr;

	/* allocate ENDPT0_IN once, if already allocated reuse */
	if (ctrl_allocBuff(0, USB_ENDPT_DIR_IN) < 0)
		return -ENOMEM;

	/* allocate ENDPT0_OUT once, if already allocated reuse */
	if (ctrl_allocBuff(0, USB_ENDPT_DIR_OUT) < 0)
		return -ENOMEM;

	ctrl_common.data->endpts[0].caps[USB_ENDPT_DIR_IN].init = 1;
	ctrl_common.data->endpts[0].caps[USB_ENDPT_DIR_OUT].init = 1;

	/* allocate queue head list for the first time (initially endptqh is NULL) */
	if (ctrl_common.dc->endptqh == NULL)
		ctrl_common.dc->endptqh = usbclient_allocBuff(USB_BUFFER_SIZE);

	if (ctrl_common.dc->endptqh == MAP_FAILED)
		return -ENOMEM;

	memset((void *)ctrl_common.dc->endptqh, 0, USB_BUFFER_SIZE);

	qh_addr = ((uint32_t)va2pa((void *)ctrl_common.dc->endptqh)) & ~0xfff;

	ctrl_common.dc->endptqh[0].caps =  0x40 << 16; /* max 64 bytes */
	ctrl_common.dc->endptqh[0].caps |= 0x1 << 29;
	ctrl_common.dc->endptqh[0].caps |=  0x1 << 15; /* ios */
	ctrl_common.dc->endptqh[0].dtd_next = 0x1; /* invalid */

	ctrl_common.dc->endptqh[1].caps =  0x40 << 16;
	ctrl_common.dc->endptqh[1].caps |= 0x1 << 29;
	ctrl_common.dc->endptqh[1].caps |=  0x1 << 15;
	ctrl_common.dc->endptqh[1].dtd_next = 1;

	ctrl_common.dc->endptqh[0].base = qh_addr + (32 * sizeof(dqh_t));
	ctrl_common.dc->endptqh[0].size = 0x40;
	ctrl_common.dc->endptqh[0].head = (dtd_t *)(ctrl_common.dc->endptqh + 32);
	ctrl_common.dc->endptqh[0].tail = (dtd_t *)(ctrl_common.dc->endptqh + 32);

	ctrl_common.dc->endptqh[1].base = qh_addr + (48 * sizeof(dqh_t));
	ctrl_common.dc->endptqh[1].size = 0x10;
	ctrl_common.dc->endptqh[1].head = (dtd_t *)(ctrl_common.dc->endptqh + 48);
	ctrl_common.dc->endptqh[1].tail = (dtd_t *)(ctrl_common.dc->endptqh + 48);

	*(ctrl_common.dc->base + endpointlistaddr) = qh_addr;
	*(ctrl_common.dc->base + endptprime) |= 1;
	*(ctrl_common.dc->base + endptprime) |= 1 << 16;

	return EOK;
}


static dtd_t *ctrl_getDtd(int endpt, int dir)
{
	int qh = endpt * 2 + dir;
	uint32_t base_addr;
	dtd_t *ret;

	base_addr = ((uint32_t)ctrl_common.dc->endptqh[qh].head & ~((ctrl_common.dc->endptqh[qh].size * sizeof(dtd_t)) - 1));

	ret = ctrl_common.dc->endptqh[qh].tail++;
	ctrl_common.dc->endptqh[qh].tail = (dtd_t *)(base_addr | ((uint32_t)ctrl_common.dc->endptqh[qh].tail & (((ctrl_common.dc->endptqh[qh].size) * sizeof(dtd_t)) - 1)));

	return ret;
}


static int ctrl_buildDtd(dtd_t *dtd, uint32_t paddr, uint32_t size)
{
	int i = 0;
	int tempSize = size;

	if (size > USB_BUFFER_SIZE)
		return -EINVAL;

	dtd->dtd_next = 1;
	dtd->dtd_token = size << 16;
	dtd->dtd_token |= 1 << 7;

	/* TODO: allow to use additional dtd within the same dQH */
	while (tempSize > 0) {
		dtd->buff_ptr[i++] = paddr;
		tempSize -= 0x1000;
		paddr += 0x1000;
	}

	return EOK;
}


/* In case of non dTD error function may return NULL */
dtd_t *ctrl_execTransfer(int endpt, uint32_t paddr, uint32_t sz, int dir)
{
	int shift;
	uint32_t offs;
	volatile dtd_t *dtd;

	int qh = (endpt << 1) + dir;

	if (ctrl_common.dc->connected) {
		dtd = ctrl_getDtd(endpt, dir);

		ctrl_buildDtd((dtd_t *)dtd, paddr, sz);

		shift = endpt + ((qh & 1) ? 16 : 0);
		offs = (uint32_t)dtd & (((ctrl_common.dc->endptqh[qh].size) * sizeof(dtd_t)) - 1);

		ctrl_common.dc->endptqh[qh].dtd_next = (ctrl_common.dc->endptqh[qh].base + offs) & ~1;
		ctrl_common.dc->endptqh[qh].dtd_token &= ~(1 << 6);
		ctrl_common.dc->endptqh[qh].dtd_token &= ~(1 << 7);

		while ((*(ctrl_common.dc->base + endptprime) & (1 << shift)))
			;

		*(ctrl_common.dc->base + endptprime) |= 1 << shift;

		/* prime the endpoint and wait for it to prime */
		while ((*(ctrl_common.dc->base + endptprime) & (1 << shift)))
			;
		*(ctrl_common.dc->base + endptprime) |= 1 << shift;
		while (!(*(ctrl_common.dc->base + endptprime) & (1 << shift)) && (*(ctrl_common.dc->base + endptstat) & (1 << shift)) && (*(ctrl_common.dc->base + portsc1) & 1))
			;

		/* wait to finish transaction while device is attached to the host */
		while (DTD_ACTIVE(dtd) && !DTD_ERROR(dtd) && (*(ctrl_common.dc->base + portsc1) & 1))
			;

		/* check if device is attached */
		if (*(ctrl_common.dc->base + portsc1) & 1)
			return (dtd_t *)dtd;
	}

	return NULL;
}


int ctrl_hfIrq(void)
{
	int endpt = 0;

	if ((ctrl_common.dc->setupstat = *(ctrl_common.dc->base + endptsetupstat)) & 0x1) {
		/* Find setup transaction endpoint (0-15) */
		while (!((ctrl_common.dc->setupstat >> endpt) & 1)) {
			if (++endpt > 15) {
				/* Clear USB interrupt */
				*(ctrl_common.dc->base + usbsts) |= 1;
				return 0;
			}
		}

		/* Trip wire: ensure that the setup data payload is extracted from a QH by the DCD without being corrupted */
		do {
			*(ctrl_common.dc->base + usbcmd) |= 1 << 13;
			memcpy(&ctrl_common.dc->setup, (void *)ctrl_common.dc->endptqh[endpt].setup_buff, sizeof(usb_setup_packet_t));
		} while (!(*(ctrl_common.dc->base + usbcmd) & 1 << 13));

		/* Acknowledge setup transfer */
		*(ctrl_common.dc->base + endptsetupstat) |= 1 << endpt;

		/* Trip wire semaphore clear */
		*(ctrl_common.dc->base + usbcmd) &= ~(1 << 13);

		/* Clear USB interrupt */
		*(ctrl_common.dc->base + usbsts) |= 1;

		ctrl_common.dc->endptqh[0].head = ctrl_common.dc->endptqh[0].tail;
		ctrl_common.dc->endptqh[1].head = ctrl_common.dc->endptqh[1].tail;

		while (*(ctrl_common.dc->base + endptsetupstat) & 1)
			;

		desc_setup(&ctrl_common.dc->setup);
	}
	else {
		/* Clear USB interrupt */
		*(ctrl_common.dc->base + usbsts) |= 1;
	}

	return 1;
}


int ctrl_lfIrq(void)
{
	int status = ctrl_common.dc->connected;

	ctrl_common.dc->connected = (*(ctrl_common.dc->base + portsc1) & 1) && (*(ctrl_common.dc->base + otgsc) & 1 << 9);

	/* Reset received */
	if ((*(ctrl_common.dc->base + usbsts) & 1 << 6)) {

		*(ctrl_common.dc->base + endptsetupstat) = *(ctrl_common.dc->base + endptsetupstat);
		*(ctrl_common.dc->base + endptcomplete) = *(ctrl_common.dc->base + endptcomplete);

		while (*(ctrl_common.dc->base + endptprime))
			;

		*(ctrl_common.dc->base + endptflush) = 0xffffffff;

		while (*(ctrl_common.dc->base + portsc1) & 1 << 8)
			;

		*(ctrl_common.dc->base + usbsts) |= 1 << 6;
		ctrl_common.dc->status = DC_DEFAULT;

		if (ctrl_common.dc->cbEvent)
			ctrl_common.dc->cbEvent(USBCLIENT_EV_RESET, ctrl_common.dc->ctxUser); /* URI - RESET received */
	}

	/* Connect state has changed */
	if (ctrl_common.dc->connected != status) {

		while (*(ctrl_common.dc->base + endptprime))
			;

		*(ctrl_common.dc->base + endptflush) = 0xffffffff;

		if (ctrl_common.dc->cbEvent) {
			status = ctrl_common.dc->connected ? USBCLIENT_EV_CONNECT : USBCLIENT_EV_DISCONNECT;
			ctrl_common.dc->cbEvent(status, ctrl_common.dc->ctxUser);
		}
	}

	return 1;
}


static void ctrl_devInit(void)
{
	*(ctrl_common.dc->base + endptflush) = 0xffffffff;
	/* Run/Stop bit */
	*(ctrl_common.dc->base + usbcmd) &= ~1;
	/* Controller resets its internal pipelines, timers etc. */
	*(ctrl_common.dc->base + usbcmd) |= 1 << 1;
	ctrl_common.dc->status = DC_POWERED;

	/* Run/Stop register is set to 0 when the reset process is complete. */
	while (*(ctrl_common.dc->base + usbcmd) & (1 << 1))
		;

	/* set usb mode to device */
	*(ctrl_common.dc->base + usbmode) |= 2;
	/* trip wire mode (setup lockout mode disabled) */
	*(ctrl_common.dc->base + usbmode) |= 1 << 3;

	*(ctrl_common.dc->base + usbintr) |= 0x57;
	*(ctrl_common.dc->base + usbintr) |= 3 << 18;
	*(ctrl_common.dc->base + usbintr) |= 1 << 6;

	*(ctrl_common.dc->base + usbsts) |= 1;

	ctrl_common.dc->status = DC_ATTACHED;
	*(ctrl_common.dc->base + usbcmd) |= 1;
}


int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	ctrl_common.dc = dc_in;
	ctrl_common.data = usb_data_in;

	ctrl_initQtd();

	ctrl_devInit();

	return ctrl_endpt0Init();
}


void ctrl_setAddress(uint32_t addr)
{
	*(ctrl_common.dc->base + deviceaddr) = addr;
}


void ctrl_reset(void)
{
	/* stop controller */
	*(ctrl_common.dc->base + endptflush) = 0xffffffff;
	*(ctrl_common.dc->base + usbintr) = 0;

	/* reset controller */
	*(ctrl_common.dc->base + usbcmd) &= ~1;
	*(ctrl_common.dc->base + usbcmd) |= 2;
}
