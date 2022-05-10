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

#include <sys/minmax.h>

#include <hcd.h>
#include <hub.h>
#include "ehci.h"


static const struct {
	usb_device_desc_t dev;
	usb_configuration_desc_t cfg;
	usb_interface_desc_t iface;
	usb_endpoint_desc_t ep;
	const char langID[2];
	const char product[16];
	const char manufacturer[16];
} __attribute__((packed)) ehci_descs = {
	{
		.bLength = 18,
		.bcdUSB = 0x0200,
		.bDeviceClass = USB_CLASS_HUB,
		.bDeviceSubClass = 0,
		.bDeviceProtocol = 1, /* Single TT */
		.bMaxPacketSize0 = 64,
		.idVendor = 0x0,
		.idProduct = 0x0,
		.bcdDevice = 0x0,
		.iManufacturer = 2,
		.iProduct = 1,
		.iSerialNumber = 0,
		.bNumConfigurations = 1,
	},
	{
		.bLength = 9,
		.bDescriptorType = USB_DESC_CONFIG,
		.wTotalLength = 40,
		.bNumInterfaces = 1,
		.bConfigurationValue = 1,
		.iConfiguration = 0,
		.bmAttributes = 0,
		.bMaxPower = 0,
	},
	{
		.bLength = 9,
		.bDescriptorType = USB_DESC_CONFIG,
		.bInterfaceNumber = 0,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_HUB,
		.bInterfaceSubClass = 0,
		.bInterfaceProtocol = 0,
		.iInterface = 0,
	},
	{
		.bLength = 7,
		.bDescriptorType = USB_DESC_ENDPOINT,
		.bEndpointAddress = USB_ENDPT_DIR_IN | (1 << 7),
		.bmAttributes = USB_ENDPT_TYPE_INTR,
		.wMaxPacketSize = 4,
		.bInterval = 0xFF,
	},
	{ 0x04, 0x09 }, /* US English */
	"2.0 root hub",
	"Phoenix Systems"
};


static void ehci_resetPort(hcd_t *hcd, int port)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	volatile int *reg = (hcd->base + portsc1) + (port - 1);

	*reg &= ~PORTSC_ENA;
	*reg |= PORTSC_PR;
	/*
	 * This is imx deviation. According to ehci documentation
	 * it is up to software to set the PR bit 0 after waiting 20ms
	 */
	while (*reg & PORTSC_PR)
		;
	usleep(20 * 1000);

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

	if (port > hub->nports)
		return -1;

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
	ehci_t *ehci = (ehci_t *)hcd->priv;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;
	switch (wValue) {
		/* For 'change' features, ack the change */
		case USB_PORT_FEAT_C_CONNECTION:
			*portsc = val | PORTSC_CSC;
			if ((val & PORTSC_CCS) == 0)
				phy_enableHighSpeedDisconnect(hcd, 0);
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


static int ehci_getStringDesc(usb_dev_t *hub, int index, char *buf, size_t size)
{
	usb_string_desc_t *desc;
	size_t len = 0;
	const char *src;
	int i;

	switch (index) {
		case 0:
			/* LangId string */
			len = 4;
			src = ehci_descs.langID;
			break;
		case 1:
			/* Product string */
			len = strlen(ehci_descs.product) * 2 + 3;
			src = ehci_descs.product;
			break;
		case 2:
			/* Manufacturer string */
			len = strlen(ehci_descs.manufacturer) * 2 + 3;
			src = ehci_descs.manufacturer;
			break;
		default:
			return 0;
	}

	if (size < len)
		return -1;

	desc = (usb_string_desc_t *)buf;
	desc->bDescriptorType = USB_DESC_STRING;
	desc->bLength = len;

	if (index == 0) {
		memcpy(buf, src, len - 2);
	}
	else {
		/* Unicode encode */
		memset(desc->wData, 0, len - 2);

		for (i = 0; src[i] != '\0'; i++)
			desc->wData[i * 2] = src[i];
	}

	return len;
}


static int ehci_getDesc(usb_dev_t *hub, int type, int index, char *buf, size_t size)
{
	hcd_t *hcd = hub->hcd;
	usb_hub_desc_t *hdesc;
	int bytes = 0;

	switch (type) {
		case USB_DESC_DEVICE:
			bytes = min(size, sizeof(ehci_descs.dev));
			memcpy(buf, &ehci_descs.dev, bytes);
			break;
		case USB_DESC_CONFIG:
			bytes = min(size, sizeof(ehci_descs.dev) + sizeof(ehci_descs.iface) + sizeof(ehci_descs.ep));
			memcpy(buf, &ehci_descs.cfg, bytes);
			break;
		case USB_DESC_STRING:
			bytes = ehci_getStringDesc(hub, index, buf, size);
			break;
		case USB_DESC_TYPE_HUB:
			if (size < sizeof(usb_hub_desc_t) + 2)
				break;
			hdesc = (usb_hub_desc_t *)buf;
			hdesc->bDescLength = 9;
			hdesc->bDescriptorType = USB_DESC_TYPE_HUB;
			hdesc->wHubCharacteristics = 0x1;
			hdesc->bPwrOn2PwrGood = 10;
			hdesc->bHubContrCurrent = 10;
			hdesc->bNbrPorts = *(hcd->base + hcsparams) & 0xf;
			hdesc->variable[0] = 0;    /* Device not removable */
			hdesc->variable[1] = 0xff; /* PortPwrCtrlMask */
			break;
	}

	return 0;
}


uint32_t ehci_getHubStatus(usb_dev_t *hub)
{
	hcd_t *hcd = hub->hcd;
	uint32_t status = 0;
	int i, val;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	for (i = 0; i < hub->nports; i++) {
		val = ehci->portsc;
		if (val & (PORTSC_CSC | PORTSC_PEC | PORTSC_OCC))
			status |= 1 << (i + 1);
	}

	return status;
}


int ehci_roothubReq(usb_dev_t *hub, usb_transfer_t *t)
{
	usb_setup_packet_t *setup = t->setup;
	int ret;

	/* It will be finished, when a port status changes */
	if (t->type == usb_transfer_interrupt) {
		/* Enable Port Status Changed interrupt if this is a first call */
		if ((*(hub->hcd->base + usbintr) & USBSTS_PCI) == 0)
			*(hub->hcd->base + usbintr) |= USBSTS_PCI;
		return 0;
	}

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
			ret = ehci_getDesc(hub, setup->wValue >> 8, setup->wValue & 0xff, t->buffer, t->size);
			break;
		case REQ_SET_ADDRESS:
		case REQ_SET_CONFIGURATION:
			/* Pass on */
			ret = 0;
			break;
		default:
			/* Not implemented */
			ret = -1;
	}

	usb_transferFinished(t, ret);

	return 0;
}
