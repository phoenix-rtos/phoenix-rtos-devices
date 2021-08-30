/*
 * Phoenix-RTOS
 *
 * ehci root hub implementation
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <hcd.h>
#include <hub.h>
#include "ehci.h"


void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable);

static void ehci_resetPort(hcd_t *hcd, int port)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	volatile int *reg = (hcd->base + portsc1) + (port - 1);

	*reg &= ~0x0000002AU;
	*reg |= PORTSC_PR;
	/*
	 * This is imx deviation. According to ehci documentation
	 * it is up to software to set the PR bit 0 after waiting 20ms
	 */
	while (*reg & PORTSC_PR);
	usleep(20 * 1000);

	*reg |= PORTSC_PP;
	ehci->portResetChange = 1 << port;

	if ((*reg & PORTSC_PSPD) == PORTSC_PSPD_HS)
		phy_enableHighSpeedDisconnect(hcd, 1);
}


static int ehci_getPortStatus(usb_dev_t *hub, int port, usb_port_status_t *status)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *)hcd->priv;
	int val;

	if (port > hub->nports)
		return -1;

	status->wPortChange = 0;
	status->wPortStatus = 0;

	val = *(hcd->base + portsc1 + port - 1);
	if (val & PORTSC_CCS)
		status->wPortStatus |= USB_PORT_STAT_CONNECTION;

	if (val & PORTSC_CSC)
		status->wPortChange |= USB_PORT_STAT_C_CONNECTION;

	if (val & PORTSC_ENA)
		status->wPortStatus |= USB_PORT_STAT_ENABLE;

	if (val & PORTSC_PEC)
		status->wPortChange |= USB_PORT_STAT_C_ENABLE;

	if (val & PORTSC_OCA)
		status->wPortStatus |= USB_PORT_STAT_OVERCURRENT;

	if (val & PORTSC_OCC)
		status->wPortChange |= USB_PORT_STAT_C_OVERCURRENT;

	if (val & PORTSC_SUSP)
		status->wPortStatus |= USB_PORT_STAT_SUSPEND;

	if (val & PORTSC_PR)
		status->wPortStatus |= USB_PORT_STAT_RESET;

	if (val & PORTSC_PP)
		status->wPortStatus |= USB_PORT_STAT_POWER;

	if (val & PORTSC_PTC)
		status->wPortStatus |= USB_PORT_STAT_TEST;

	if (ehci->portResetChange & (1 << port))
		status->wPortChange |= USB_PORT_STAT_C_RESET;

	if ((val & PORTSC_PSPD) >> 26 == 1)
		status->wPortStatus |= USB_PORT_STAT_LOW_SPEED;
	else if ((val & PORTSC_PSPD) >> 26 == 2)
		status->wPortStatus |= USB_PORT_STAT_HIGH_SPEED;

	/* TODO: set indicator */

	return 0;
}


static int ehci_setPortFeature(usb_dev_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;

	switch (wValue) {
		case USB_PORT_FEAT_RESET:
			ehci_resetPort(hcd, port);
			break;
		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_POWER:
		case USB_PORT_FEAT_TEST:
		case USB_PORT_FEAT_INDICATOR:
			/* TODO */
			break;
		default:
			return -1;
	}

	return 0;
}


static int ehci_clearPortFeature(usb_dev_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *) hcd->priv;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;
	switch (wValue) {
		/* For 'change' features, ack the change */
		case USB_PORT_FEAT_C_CONNECTION:
			*portsc = val | PORTSC_CSC;
			break;
		case USB_PORT_FEAT_C_ENABLE:
			*portsc = val | PORTSC_PEC;
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			*portsc = val | PORTSC_OCC;
			break;
		case USB_PORT_FEAT_C_RESET:
			ehci->portResetChange &= ~(1 << port);
			break;
		case USB_PORT_FEAT_ENABLE:
			/* Disable port */
			*portsc = val & ~PORTSC_ENA;
			break;
		case USB_PORT_FEAT_POWER:
		case USB_PORT_FEAT_INDICATOR:
		case USB_PORT_FEAT_SUSPEND:
		case USB_PORT_FEAT_C_SUSPEND:
			/* TODO */
			break;
		default:
			return -1;
	}

	return 0;
}


static int ehci_getDesc(usb_dev_t *hub, char *buf, size_t len)
{
	hcd_t *hcd = hub->hcd;
	usb_hub_desc_t *desc;

	if (len < 9)
		return -1;

	desc = (usb_hub_desc_t *)buf;
	desc->bDescLength = 9;
	desc->bDescriptorType = USB_DESC_TYPE_HUB;
	desc->wHubCharacteristics = 0x1;
	desc->bPwrOn2PwrGood = 10;
	desc->bHubContrCurrent = 10;
	desc->bNbrPorts = *(hcd->base + hcsparams) & 0xf;
	desc->variable[0] = 0; /* Device not removable */
	desc->variable[1] = 0xff; /* PortPwrCtrlMask */

	return 0;
}


uint32_t ehci_getHubStatus(usb_dev_t *hub)
{
	hcd_t *hcd = hub->hcd;
	volatile int *portsc;
	uint32_t status = 0;
	int i;

	for (i = 0; i < hub->nports; i++) {
		portsc = hcd->base + portsc1 + i;
		if (*portsc & PORTSC_CSC)
			status |= 1 << (i + 1);
		if ((*portsc & PORTSC_CSC) && (*portsc & PORTSC_CCS) == 0)
			phy_enableHighSpeedDisconnect(hcd, 0);
	}

	return status;
}


int ehci_roothubReq(usb_transfer_t *t)
{
	usb_dev_t *hub = t->ep->device;
	usb_setup_packet_t *setup = t->setup;
	int ret = 0;

	switch (setup->bRequest) {
		case REQ_GET_STATUS:
			ret = ehci_getPortStatus(hub, setup->wIndex, (usb_port_status_t *)t->buffer);
			break;
		case REQ_SET_FEATURE:
			ret = ehci_setPortFeature(hub, setup->wIndex, setup->wValue);
			break;
		case REQ_CLEAR_FEATURE:
			ret = ehci_clearPortFeature(hub, setup->wIndex, setup->wValue);
			break;
		case REQ_GET_DESCRIPTOR:
			ret = ehci_getDesc(hub, t->buffer, t->size);
		case REQ_SET_CONFIGURATION:
			break;
	}

	t->finished = 1;
	if (ret != 0) {
		t->error = 1;
		t->transferred = 0;
	}
	else {
		t->error = 0;
		t->transferred = t->size;
	}

	return 0;
}
