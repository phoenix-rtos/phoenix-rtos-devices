/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include "../imx-usbc/imxdevice.h"

#include <sys/interrupt.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <sys/debug.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <string.h>
#include <errno.h>

#include <phoenix/arch/imxrt.h>

#define USB_BASE_ADDR       0x402E0000
#define OCRAM2_BASE_ADRR    0x20200000

static usb_common_data_t usb_data;
static usb_dc_t dc = { 0 };
static uint8_t counter_OCRAM2 = 0;


static int ctrlqh_init(void)
{
	uint32_t qh_addr;

	/* map queue head list */
	dc.endptqh = (void *)(OCRAM2_BASE_ADRR + USB_BUFFER_SIZE * counter_OCRAM2++);

	if (dc.endptqh == MAP_FAILED)
		return -ENOMEM;

	memset((void *)dc.endptqh, 0, USB_BUFFER_SIZE);

	qh_addr = (uint32_t)dc.endptqh;

	dc.endptqh[0].caps =  0x40 << 16; /* max 64 bytes */
	dc.endptqh[0].caps |= 0x1 << 29;
	dc.endptqh[0].caps |=  0x1 << 15; /* ios */
	dc.endptqh[0].dtd_next = 0x1; /* invalid */

	dc.endptqh[1].caps =  0x40 << 16;
	dc.endptqh[1].caps |= 0x1 << 29;
	dc.endptqh[1].caps |=  0x1 << 15;
	dc.endptqh[1].dtd_next = 1;

	dc.endptqh[0].base = qh_addr + (32 * sizeof(dqh_t));
	dc.endptqh[0].size = 0x10;
	dc.endptqh[0].head = (dtd_t *)(dc.endptqh + 32);
	dc.endptqh[0].tail = (dtd_t *)(dc.endptqh + 32);

	dc.endptqh[1].base = qh_addr + (48 * sizeof(dqh_t));
	dc.endptqh[1].size = 0x10;
	dc.endptqh[1].head = (dtd_t *)(dc.endptqh + 48);
	dc.endptqh[1].tail = (dtd_t *)(dc.endptqh + 48);

	*(dc.base + endpointlistaddr) = qh_addr;
	*(dc.base + endptprime) |= 1;
	*(dc.base + endptprime) |= 1 << 16;

	return EOK;
}


static int dtd_init(int endpt)
{
	dtd_t *buff;
	int qh = endpt * 2;

	if (!endpt)
		return -EINVAL;

	buff = (void *)(OCRAM2_BASE_ADRR + USB_BUFFER_SIZE * counter_OCRAM2++);

	if (buff == MAP_FAILED)
		return -ENOMEM;

	memset(buff, 0, USB_BUFFER_SIZE);

	dc.endptqh[qh].base = (((uint32_t)va2pa(buff)) & ~0xfff);
	dc.endptqh[qh].size = 0x40;
	dc.endptqh[qh].head = buff;
	dc.endptqh[qh].tail = buff;

	dc.endptqh[++qh].base = (((uint32_t)va2pa(buff)) & ~0xfff) + (64 * sizeof(dtd_t));
	dc.endptqh[qh].size = 0x40;
	dc.endptqh[qh].head = buff + 64;
	dc.endptqh[qh].tail = buff + 64;

	return EOK;
}


static int endpt_init(int endpt, endpt_init_t *endpt_init)
{
	uint32_t setup = 0;
	int res;
	int qh_rx = endpt * 2 + USB_ENDPT_DIR_OUT;
	int qh_tx = endpt * 2 + USB_ENDPT_DIR_IN;

	if (endpt == 0)
		return -EINVAL;

	if ((res = dtd_init(endpt)) != EOK)
		return res;

	dc.endptqh[qh_rx].caps =  endpt_init->rx_caps.max_pkt_len << 16;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.ios << 15;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.zlt << 29;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.mult << 30;
	dc.endptqh[qh_rx].dtd_next = 1;

	dc.endptqh[qh_tx].caps =  endpt_init->tx_caps.max_pkt_len << 16;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.ios << 15;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.zlt << 29;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.mult << 30;
	dc.endptqh[qh_tx].dtd_next = 1;

	setup |= endpt_init->rx_ctrl.type << 2;
	setup |= endpt_init->tx_ctrl.type << 18;
	setup |= endpt_init->rx_ctrl.data_toggle << 6;
	setup |= endpt_init->tx_ctrl.data_toggle << 22;

	*(dc.base + endptctrl0 + endpt) = setup;
	*(dc.base + endptctrl0 + endpt) |= 1 << 7;
	*(dc.base + endptctrl0 + endpt) |= 1 << 23;

	return EOK;
}


static int dc_intr(unsigned int intr, void *data)
{
	dc_hf_intr();
	dc_lf_intr();

	return 0;
}


static int setClock(int dev, unsigned int state)
{
	platformctl_t pctl;

	pctl.action = pctl_set;
	pctl.type = pctl_devclock;
	pctl.devclock.dev = dev;
	pctl.devclock.state = state;

	return platformctl(&pctl);
}


int usbclient_init(usb_conf_t *conf)
{
	int i = 1;
	int res = 0;

	setClock(pctl_clk_usboh3, clk_state_run);

	usb_data.endNb = 0;
	usb_data.read_buffer.data = (void *)(OCRAM2_BASE_ADRR + USB_BUFFER_SIZE * counter_OCRAM2++);
	usb_data.pread_buffer = (uint32_t)usb_data.read_buffer.data;
	usb_data.write_buffer.data = (void *)(OCRAM2_BASE_ADRR + USB_BUFFER_SIZE * counter_OCRAM2++);
	usb_data.pwrite_buffer = (uint32_t)usb_data.write_buffer.data;

	usb_data.local_conf = (void *)(OCRAM2_BASE_ADRR + USB_BUFFER_SIZE * counter_OCRAM2++);

	if (init_desc(conf, &usb_data, &dc) < 0)
		return -ENOMEM;

	dc.base = (void *)USB_BASE_ADDR;
	dc.lock = 0;
	dc.cond = 0;
	dc.dev_addr = 0;

	if (mutexCreate(&dc.lock) != EOK)
		return -ENOENT;

	if (condCreate(&dc.cond) != EOK) {
		resourceDestroy(dc.lock);
		return -ENOENT;
	}

	*(dc.base + endptflush) = 0xffffffff;
	/* Run/Stop bit */
	*(dc.base + usbcmd) &= ~1;
	/* Controller resets its internal pipelines, timers etc. */
	*(dc.base + usbcmd) |= 1 << 1;
	dc.status = DC_POWERED;

	/* Run/Stop register is set to 0 when the reset process is complete. */
	while (*(dc.base + usbcmd) & (1 << 1));

	/* set usb mode to device */
	*(dc.base + usbmode) |= 2;
	/* trip wire mode (setup lockout mode disabled) */
	*(dc.base + usbmode) |= 1 << 3;

	*(dc.base + usbintr) |= 0x57;

	dc.status = DC_ATTACHED;
	*(dc.base + usbcmd) |= 1;

	/* map queue heads list and init control endpoint */
	if ((res = ctrlqh_init()) != EOK) {
		resourceDestroy(dc.lock);
		resourceDestroy(dc.cond);
		return res;
	}

	interrupt(usb_otg1_irq, dc_intr, NULL, dc.cond, &dc.inth);

	while (dc.op != DC_OP_EXIT) {
		if (dc.op == DC_OP_INIT) {
			while (i <= usb_data.endNb) {
				if ((res = endpt_init(i, &usb_data.in_endpt[i-1])) != EOK)
					return res;
				i++;
			}
			return res;
		}
	}

	return EOK;
}


int usbclient_destroy(void)
{
	/* stopping device controller */
	*(dc.base + usbintr) = 0;
	*(dc.base + usbcmd) &= ~1;

	resourceDestroy(dc.lock);
	resourceDestroy(dc.cond);

	return 0;
}
