#include <string.h>
#include <stdio.h>

#include <hcd.h>
#include <hub.h>
#include "ehci.h"

void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable);

static void hub_resetPort(hcd_t *hcd, int port)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	volatile int *reg = (hcd->base + portsc1) + (port - 1);

	printf("resetting port: %d\n", port);
	*reg |= PORTSC_PR;
	*reg &= ~PORTSC_ENA;
	/* This is imx deviation. According to ehci documentation
	 * it is up to software to set the PR bit 0 after waiting 20ms */
	while (*reg & PORTSC_PR);

	*reg |= PORTSC_ENA;

	ehci->portResetChange = 1 << port;
	/* Turn port power on */
	*reg |= PORTSC_PP;

	if ((*reg & PORTSC_PSPD) == PORTSC_PSPD_HS)
		phy_enableHighSpeedDisconnect(hcd, 1);

	printf("PORT RESET SUCCESSFUL: PORTSC: %x\n", *reg);
}


static int hub_statusChanged(usb_hub_t *hub, uint32_t *status)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *)hcd->priv;
	volatile int *portsc;
	int i;

	*status = 0;

	/* Status not changed */
	if ((ehci->status & USBSTS_PCI) == 0)
		return 0;

	ehci->status &= ~USBSTS_PCI;

	for (i = 0; i < hub->nports; i++) {
		portsc = hcd->base + portsc1 + i;
		if (*portsc & PORTSC_CSC)
			*status |= 1 << (i + 1);
		if ((*portsc & PORTSC_CSC) && (*portsc & PORTSC_CCS) == 0)
			phy_enableHighSpeedDisconnect(hcd, 0);
	}

	return 0;
}


static int hub_getPortStatus(usb_hub_t *hub, int port, usb_port_status_t *status)
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
	printf("PORT STATUS PSPD: %x\n", (val & PORTSC_PSPD) >> 26);

	/* TODO: set indicator */
	printf("port status: %x change: %x\n", status->wPortStatus, status->wPortChange);

	return 0;
}


static int hub_setPortFeature(usb_hub_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;

	switch (wValue) {
	case USB_PORT_FEAT_RESET:
		hub_resetPort(hcd, port);
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


static int hub_clearPortFeature(usb_hub_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *) hcd->priv;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	printf("hub: clearPortFeature: %x\n", wValue);
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


void ehci_rootHubInit(usb_hub_t *hub, int nports)
{
	hub->statusChanged = hub_statusChanged;
	hub->getPortStatus = hub_getPortStatus;
	hub->clearPortFeature = hub_clearPortFeature;
	hub->setPortFeature = hub_setPortFeature;
	/* TODO: set root hub device descriptors */
	hub->nports = nports;
	strncpy(hub->dev->name, "USB 2.0 root hub", sizeof(hub->dev->name));
}