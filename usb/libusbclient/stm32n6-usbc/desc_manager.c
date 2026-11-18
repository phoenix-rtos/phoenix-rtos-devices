/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * Descriptor manager
 *
 * Copyright 2026 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
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
#include <sys/types.h>
#include <phoenix/arch/armv8m/stm32/n6/stm32n6.h>
#include <cdc.h>

#include "client.h"

#define USB_REG(x)             *(desc_common.dc->base + x)
#define CONF_DESC_LENGTH(addr) (((usb_configuration_desc_t *)(addr))->wTotalLength)


typedef struct {
	addr_t pmAddr;
	usb_string_desc_t *vmStruct;
} addr_string_desc_t;


static struct {
	uint8_t *dev;
	uint8_t *cfg;
	addr_string_desc_t str0;
	addr_string_desc_t strMan;
	addr_string_desc_t strProd;
	addr_t hidReports;
	usb_dc_t *dc;
	usb_common_data_t *data;
} __attribute__((aligned(4))) desc_common;


static void desc_endptInit(usb_endpoint_desc_t *endpt)
{
	uint8_t epNum = endpt->bEndpointAddress & 0x7U;
	stm32n6_endpt_t *ep = &desc_common.data->endpts[epNum];
	uint8_t dir = ((endpt->bEndpointAddress & (1U << 7)) > 0U) ? 1U : 0U;

	if (dir == 1U) {
		ep->in.maxpacket = endpt->wMaxPacketSize;
		ep->in.type = endpt->bmAttributes & 0x03;
		ep->in.is_in = 1U;
		ep->in.epNum = epNum;

		/*
		 * Descriptors are configured for HS device, right now
		 * we are FS device so overwriting max packet value.
		 */
		if (ep->in.type == 2U) {
			ep->in.maxpacket = 0x40UL;
		}
	}
	else {
		ep->out.maxpacket = endpt->wMaxPacketSize;
		ep->out.type = endpt->bmAttributes & 0x03;
		ep->out.is_in = 0U;
		ep->out.epNum = epNum;

		if (ep->out.type == 2U) {
			ep->out.maxpacket = 0x40UL;
		}
	}
}


static void desc_strInit(usb_desc_list_t *desList, size_t *localOffset, int strOrder)
{
	void *target = (void *)(desc_common.data->setupMem + *localOffset);

	switch (strOrder) {
		case 0:
			desc_common.str0.vmStruct = (usb_string_desc_t *)target;
			desc_common.str0.pmAddr = (addr_t)target;
			memcpy(desc_common.str0.vmStruct, desList->descriptor, sizeof(usb_string_desc_t));
			*localOffset += desList->descriptor->bFunctionLength;
			break;
		case 1:
			desc_common.strMan.vmStruct = (usb_string_desc_t *)target;
			desc_common.strMan.pmAddr = (addr_t)target;
			memcpy(desc_common.strMan.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;
		case 2:
			desc_common.strProd.vmStruct = (usb_string_desc_t *)target;
			desc_common.strProd.pmAddr = (addr_t)target;
			memcpy(desc_common.strProd.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;
	}
}


static void desc_devAdjust(usb_device_desc_t *desc)
{
	/* Microsoft Windows requires these classes to identify the device as a
	 * virtual COM port */
	desc->bDeviceClass = USB_CLASS_CDC;
	desc->bDeviceSubClass = USB_SUBCLASS_ACM;
}


static void desc_epAdjust(usb_endpoint_desc_t *desc)
{
	/* Full-Speed handles max packet size of at most 64 bytes */
	desc->wMaxPacketSize = (desc->wMaxPacketSize > 64U) ? 64U : desc->wMaxPacketSize;
}


static void desc_acmAdjust(usb_desc_cdc_acm_t *desc)
{
	/* only advertise SET_LINE_CODING, GET_LINE_CODING, and SET_CONTROL_LINE_STATE */
	desc->bmCapabilities = 0x2U;
}


static void desc_cfgAdjust(usb_configuration_desc_t *desc)
{
	/*
	 * bmAttributes:
	 *   bit 7 - reserved, must be 1 (usb 1.0 bus powered)
	 *   bit 6 - self powered
	 *   bit 5 - remote wakeup
	 *   bits 0 to 4 - reserved, must be 0
	 * advertise as bus powered without remote wakeup
	 */
	desc->bmAttributes = (1U << 7U);
}


static void desc_callAdjust(usb_desc_cdc_call_t *desc)
{
	/*
	 * bmCapabilities:
	 * bits 2 to 7 - reserved (must be 0)
	 * bit 1 - device can send/receive call management information over a
	 *         data class interface.
	 * bit 0 - device handles call management itself
	 */
	desc->bmCapabilities = 0x00U;
}


int desc_init(usb_desc_list_t *desList, usb_common_data_t *usbDataIn, usb_dc_t *dcIn)
{
	size_t localOffset = 0;
	uint32_t stringDescCnt = 0, len;
	char *vrtAddr;

	desc_common.dc = dcIn;
	desc_common.data = usbDataIn;

	memset(desc_common.data->setupMem, 0, USB_BUFFER_SIZE);

	/* Extract mandatory descriptors to mapped memory */
	for (; desList != NULL; desList = desList->next) {

		vrtAddr = desc_common.data->setupMem + localOffset;
		len = desList->descriptor->bFunctionLength;
		if (localOffset + len > USB_BUFFER_SIZE) {
			return -ENOMEM;
		}

		switch (desList->descriptor->bDescriptorType) {
			case USB_DESC_DEVICE:
				desc_common.dev = (uint8_t *)vrtAddr;
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_device_desc_t));
				desc_devAdjust((usb_device_desc_t *)vrtAddr);
				localOffset += len;

				break;

			case USB_DESC_CONFIG:
				desc_common.cfg = (uint8_t *)vrtAddr;
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_configuration_desc_t));
				desc_cfgAdjust((usb_configuration_desc_t *)(desList->descriptor));
				localOffset += len;
				break;

			case USB_DESC_INTERFACE:
				memcpy(vrtAddr, desList->descriptor, len);
				localOffset += len;
				break;

			case USB_DESC_ENDPOINT:
				desc_epAdjust((usb_endpoint_desc_t *)(desList->descriptor));
				memcpy(vrtAddr, desList->descriptor, len);
				localOffset += len;
				desc_endptInit((usb_endpoint_desc_t *)desList->descriptor);
				break;

			case USB_DESC_TYPE_HID:
				memcpy(vrtAddr, desList->descriptor, len);
				localOffset += len;
				break;

			case USB_DESC_TYPE_CDC_CS_INTERFACE:
				memcpy(vrtAddr, desList->descriptor, len);
				if (desList->descriptor->bDescriptorSubtype == 0x02) {
					desc_acmAdjust((usb_desc_cdc_acm_t *)vrtAddr);
				}
				else if (desList->descriptor->bDescriptorSubtype == 0x01) {
					desc_callAdjust((usb_desc_cdc_call_t *)desList->descriptor);
				}
				localOffset += len;
				break;

			case USB_DESC_TYPE_HID_REPORT:
				desc_common.hidReports = (addr_t)vrtAddr;
				memcpy(vrtAddr, &desList->descriptor->bDescriptorSubtype, len - 2);
				localOffset += len - 2;
				break;

			case USB_DESC_STRING:
				desc_strInit(desList, &localOffset, stringDescCnt++);
				break;

			case USB_DESC_TYPE_DEV_QUAL:
			case USB_DESC_TYPE_OTH_SPD_CFG:
			case USB_DESC_TYPE_INTF_PWR:
				/* Not implemented yet */
				break;

			default:
				break;
		}
	}

	return EOK;
}


static void desc_ReqSetAddress(const usb_setup_packet_t *setup)
{
	USB_REG(otg_dcfg) &= ~(0x7FUL << DCFG_DAD);
	USB_REG(otg_dcfg) |= ((setup->wValue & 0x7FUL) << DCFG_DAD);

	desc_common.dc->devAddr = setup->wValue;
	desc_common.dc->deviceState = USBD_STATE_ADDRESSED;
	desc_common.dc->currEvent = USBCLIENT_EV_INIT;

	clbc_epTransmit(0, NULL, 0);
}


static void desc_ReqSetConfig(const usb_setup_packet_t *setup)
{
	stm32n6_endpt_data_t *ep = NULL;
	uint8_t cnt, setupCfg;

	setupCfg = setup->wValue;

	if (desc_common.dc->deviceState == USBD_STATE_ADDRESSED) {
		if (setupCfg == ((usb_configuration_desc_t *)desc_common.cfg)->bConfigurationValue) {
			desc_common.dc->deviceState = USBD_STATE_CONFIGURED;
			desc_common.dc->currEvent = USBCLIENT_EV_CONFIGURED;

			/*
			 * iterate through all non-control endpoints and activate them
			 * according to configuration descriptor
			 */
			for (cnt = 1; cnt < ENDPOINTS_NUMBER; cnt++) {

				ep = &desc_common.data->endpts[cnt].in;
				if (ep->type != 0U) {
					desc_epDeactivation(ep);
					desc_epActivation(ep);
				}
				ep = &desc_common.data->endpts[cnt].out;

				if (ep->type != 0U) {
					desc_epDeactivation(ep);
					desc_epActivation(ep);
				}
			}

			/* send Status IN packet */
			clbc_epTransmit(0, NULL, 0);
		}
		else if (setupCfg != 0U) {
			/* if configuration value is nor 0 neither our configuration value, send error */
			clbc_epStall(&desc_common.data->endpts[0]);
			desc_common.dc->ep0State = USBD_EP0_STALL;
		}
	}
	else if (desc_common.dc->deviceState == USBD_STATE_CONFIGURED) {
		if (setupCfg == 0U) {
			desc_common.dc->deviceState = USBD_STATE_ADDRESSED;
			clbc_epTransmit(0, NULL, 0);
		}
		else if (setupCfg != ((usb_configuration_desc_t *)desc_common.cfg)->bConfigurationValue) {
			clbc_epStall(&desc_common.data->endpts[0]);
			desc_common.dc->ep0State = USBD_EP0_STALL;
		}
		else {
			/* No action needed */
		}
	}
	else {
		/* No action needed */
	}
}


static void desc_ReqGetConfig(const usb_setup_packet_t *setup)
{
	if (desc_common.dc->deviceState == USBD_STATE_CONFIGURED) {
		desc_common.dc->ep0State = USBD_EP0_DATA_IN;
		desc_common.dc->ep0buffTX[0] = ((usb_configuration_desc_t *)desc_common.cfg)->bConfigurationValue;
		clbc_epTransmit(0, desc_common.dc->ep0buffTX, 1);
	}
	else if (desc_common.dc->deviceState == USBD_STATE_ADDRESSED) {
		desc_common.dc->ep0State = USBD_EP0_DATA_IN;
		desc_common.dc->ep0buffTX[0] = 0U;
		clbc_epTransmit(0, desc_common.dc->ep0buffTX, 1);
	}
	else {
		clbc_epStall(&desc_common.data->endpts[0]);
	}
}


static void desc_ReqGetDescriptor(const usb_setup_packet_t *setup)
{
	uint32_t len;

	if ((setup->wValue >> 8) == USB_DESC_DEVICE) {
		len = (uint32_t)MIN(sizeof(usb_device_desc_t), setup->wLength);
		clbc_epTransmit(0, desc_common.dev, len);
	}
	else if ((setup->wValue >> 8) == USB_DESC_CONFIG) {
		len = (uint32_t)MIN(CONF_DESC_LENGTH(desc_common.cfg), setup->wLength);
		clbc_epTransmit(0, desc_common.cfg, len);
	}
	else if ((setup->wValue >> 8) == USB_DESC_STRING) {
		if ((setup->wValue & 0xff) == 0U) {
			clbc_epTransmit(0, (uint8_t *)desc_common.str0.vmStruct, MIN(desc_common.str0.vmStruct->bLength, setup->wLength));
		}
		else if ((setup->wValue & 0xff) == 1U) {
			clbc_epTransmit(0, (uint8_t *)desc_common.strMan.vmStruct, MIN(desc_common.strMan.vmStruct->bLength, setup->wLength));
		}
		else if (((setup->wValue & 0xff) == 2U) || ((setup->wValue & 0xffU) == 4U)) {
			clbc_epTransmit(0, (uint8_t *)desc_common.strProd.vmStruct, MIN(desc_common.strProd.vmStruct->bLength, setup->wLength));
		}
		else {
			clbc_epStall(&desc_common.data->endpts[0]);
			desc_common.dc->ep0State = USBD_EP0_STALL;
		}
	}
	else if ((setup->wValue >> 8) == USB_DESC_TYPE_HID_REPORT) {
		clbc_epTransmit(0, (uint8_t *)desc_common.hidReports, 76);
	}
}


int desc_setup(const usb_setup_packet_t *setup)
{
	static uint8_t getStatusResp[] = { 0x00, 0x00 };
	static uint8_t getIfaceResp[] = { 0x00 };
	int res = EOK;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) == REQUEST_TYPE_STANDARD) {
		switch (setup->bRequest) {
			case REQ_SET_ADDRESS:
				desc_common.dc->ep0State = USBD_EP0_STATUS_IN;
				desc_ReqSetAddress(setup);
				break;

			case REQ_SET_CONFIGURATION:
				desc_common.dc->ep0State = USBD_EP0_STATUS_IN;
				desc_ReqSetConfig(setup);
				break;

			case REQ_GET_DESCRIPTOR:
				desc_common.dc->ep0State = USBD_EP0_DATA_IN;
				desc_ReqGetDescriptor(setup);
				break;

			case REQ_GET_STATUS:
				clbc_epTransmit(0, getStatusResp, sizeof(getStatusResp));
				break;

			case REQ_GET_INTERFACE:
				clbc_epTransmit(0, getIfaceResp, sizeof(getIfaceResp));
				break;

			case REQ_SET_INTERFACE:
				if (setup->wValue == 0U) {
					/* send ZLP to ack */
					clbc_epTransmit(0U, NULL, 0U);
				}
				else {
					/* stall unknown interfaces */
					clbc_epStall(&desc_common.data->endpts[0]);
				}
				break;

			case REQ_GET_CONFIGURATION:
				desc_common.dc->ep0State = USBD_EP0_DATA_IN;
				desc_ReqGetConfig(setup);
				break;

			case REQ_CLEAR_FEATURE:
			case REQ_SET_FEATURE:
			case REQ_SET_DESCRIPTOR:
			case REQ_SYNCH_FRAME:
			default:
				res = -1;
				break;
		}
	}
	else if (EXTRACT_REQ_TYPE(setup->bmRequestType) == REQUEST_TYPE_CLASS) {
		switch (setup->bRequest) {
			case CLASS_REQ_SET_LINE_CODING:
			case CLASS_REQ_GET_LINE_CODING:
			case CLASS_REQ_SET_CONTROL_LINE_STATE:
				(void)desc_classSetup(setup);
				break;

			default:
				res = -1;
				break;
		}
	}
	else {
		res = -1;
	}

	/* stall on failures */
	if (res < 0) {
		desc_common.dc->ep0State = USBD_EP0_STALL;
		clbc_epStall(&desc_common.data->endpts[0]);
	}

	return res;
}


int desc_classSetup(const usb_setup_packet_t *setup)
{
	int res;
	usb_xfer_desc_t desc;

	res = CLASS_SETUP_ACK;

	switch (setup->bRequest) {
		case CLASS_REQ_SET_LINE_CODING:
			desc_common.dc->ep0State = USBD_EP0_DATA_OUT;
			desc.len = setup->wLength;
			res = clbc_ep0Receive(&desc);
			if (res != 0) {
				return -EIO;
			}

			if (desc_common.dc->cbClassSetup != NULL) {
				res = desc_common.dc->cbClassSetup(setup, (void *)desc.head, (unsigned int)setup->wLength, desc_common.dc->ctxUser);
			}
			break;
		case CLASS_REQ_GET_LINE_CODING:
			desc_common.dc->ep0State = USBD_EP0_DATA_IN;
			res = desc_common.dc->cbClassSetup(setup, desc_common.dc->ep0buffTX, 0, desc_common.dc->ctxUser);

			if (res > 0) {
				clbc_epTransmit(0, desc_common.dc->ep0buffTX, res);
				return EOK;
			}
			else {
				return -ENOENT;
			}
			break;
		case CLASS_REQ_SET_CONTROL_LINE_STATE:
			desc_common.dc->ep0State = USBD_EP0_STATUS_IN;
			res = desc_common.dc->cbClassSetup(setup, NULL, 0, desc_common.dc->ctxUser);
			if (res == CLASS_SETUP_ACK) {
				if (setup->wValue == 0x3U) {
					desc_common.dc->isPortOpened = 1U;
				}
				else {
					desc_common.dc->isPortOpened = 0U;
				}
			}
			break;
		default:
			break;
	}

	if (res == CLASS_SETUP_ACK) {
		/* Send STATUS IN packet (ZLP)*/
		clbc_epTransmit(0, NULL, 0);
	}

	return EOK;
}


void desc_epActivation(stm32n6_endpt_data_t *ep)
{
	uint32_t mask = 0;

	if (ep->is_in == 1U) {
		/* Program DIEPCTLx register of this EP */
		mask |= (0x7FFUL & ep->maxpacket);
		mask |= (1UL << DIEPCTL_SD0PID);
		mask |= ((0x3U & ep->type) << DIEPCTL_EPTYP);
		mask |= ((0xFU & ep->epNum) << DIEPCTL_TXFNUM);
		mask |= (1UL << DIEPCTL_USBAEP);

		USB_REG(otg_diepctl0 + ep->epNum * EP_STRIDE) = mask;

		/* Unmask otg_daint register for given EP */
		USB_REG(otg_daintmsk) |= DAINTMSK_IN(ep->epNum);
	}
	else if (ep->is_in == 0U) {
		/* Program DOEPCTLx register of this EP */
		mask |= (0x7FFUL & ep->maxpacket);
		mask |= (1UL << DOEPCTL_SD0PID);
		mask |= ((0x3U & ep->type) << DOEPCTL_EPTYP);
		mask |= (1UL << DOEPCTL_USBAEP);

		USB_REG(otg_doepctl0 + ep->epNum * EP_STRIDE) = mask;

		/* Unmask otg_daint register for given EP */
		USB_REG(otg_daintmsk) |= DAINTMSK_OUT(ep->epNum);
	}
	else {
		/* No action needed */
	}
}


void desc_epDeactivation(stm32n6_endpt_data_t *ep)
{
	if (ep->is_in == 1U) {

		/* clear the USB active endpoint bit */
		USB_REG(otg_diepctl0 + ep->epNum * EP_STRIDE) &= ~(1UL << DIEPCTL_EPENA);

		/* mask otg_daint register for given EP */
		USB_REG(otg_daintmsk) &= ~(DAINTMSK_IN(ep->epNum));
	}
	else if (ep->is_in == 0U) {
		USB_REG(otg_doepctl0 + ep->epNum * EP_STRIDE) &= ~(1UL << DOEPCTL_EPENA);

		USB_REG(otg_daintmsk) &= ~(DAINTMSK_OUT(ep->epNum));
	}
	else {
		/* No action needed */
	}
}
