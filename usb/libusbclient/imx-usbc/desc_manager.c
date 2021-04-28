/*
 * Phoenix-RTOS
 *
 * descriptor manager
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

#include <phoenix/arch/imxrt.h>

#include "client.h"


/* Container used in USB interrupt handler where va2pa() translation cannot be used */
typedef struct {
	addr_t pmAddr;
	usb_string_desc_t *vmStruct;
} addr_string_desc_t;


struct {
	addr_t dev;
	addr_t cfg;

	addr_string_desc_t str0;
	addr_string_desc_t strMan;
	addr_string_desc_t strProd;

	addr_t hidReports;

	usb_dc_t *dc;
	usb_common_data_t *data;
} desc_common;


static void desc_endptInit(usb_endpoint_desc_t *endpt)
{
	int endNb = endpt->bEndpointAddress & 0x7;
	int dir = (endpt->bEndpointAddress & (1 << 7)) ? 1 : 0;

	desc_common.data->endpts[endNb].caps[dir].mult = 0;
	desc_common.data->endpts[endNb].caps[dir].zlt = 1;
	desc_common.data->endpts[endNb].caps[dir].max_pkt_len = endpt->wMaxPacketSize;
	desc_common.data->endpts[endNb].caps[dir].ios = 1;
	desc_common.data->endpts[endNb].caps[dir].init = 1;

	desc_common.data->endpts[endNb].ctrl[dir].type = endpt->bmAttributes & 0x03;
	desc_common.data->endpts[endNb].ctrl[dir].data_toggle = 1;
	desc_common.data->endpts[endNb].ctrl[dir].data_inhibit = 0;
	desc_common.data->endpts[endNb].ctrl[dir].stall = 0;
}


static void desc_strInit(usb_desc_list_t *desList, int *localOffset, int strOrder)
{
	switch (strOrder) {
		case 0:
			desc_common.str0.vmStruct = (usb_string_desc_t *)(desc_common.data->setupMem + *localOffset);
			desc_common.str0.pmAddr = VM_2_PHYM(desc_common.str0.vmStruct);
			memcpy(desc_common.str0.vmStruct, desList->descriptor, sizeof(usb_string_desc_t));
			*localOffset += desList->descriptor->bFunctionLength;
			break;

		case 1:
			desc_common.strMan.vmStruct = (usb_string_desc_t *)(desc_common.data->setupMem + *localOffset);
			desc_common.strMan.pmAddr = VM_2_PHYM(desc_common.strMan.vmStruct);
			memcpy(desc_common.str0.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;

		case 2:
			desc_common.strProd.vmStruct = (usb_string_desc_t *)(desc_common.data->setupMem + *localOffset);
			desc_common.strProd.pmAddr = VM_2_PHYM(desc_common.strProd.vmStruct);
			memcpy(desc_common.strProd.vmStruct, desList->descriptor, desList->descriptor->bFunctionLength);
			*localOffset += desList->descriptor->bFunctionLength;
			break;

		default:
			break;
	}
}


int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	int localOffset = 0;
	uint32_t string_desc_count = 0;
	char *vrtAddr;

	desc_common.dc = dc_in;
	desc_common.data = usb_data_in;

	memset(desc_common.data->setupMem, 0, USB_BUFFER_SIZE);

	/* Extract mandatory descriptors to mapped memory */
	for (; desList != NULL; desList = desList->next) {

		if (localOffset > USB_BUFFER_SIZE)
			return -ENOMEM	;

		switch (desList->descriptor->bDescriptorType) {
			case USB_DESC_DEVICE:
				vrtAddr = desc_common.data->setupMem + localOffset;
				desc_common.dev = VM_2_PHYM(vrtAddr);
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_device_desc_t));
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_CONFIG:
				vrtAddr = desc_common.data->setupMem + localOffset;
				desc_common.cfg = VM_2_PHYM(vrtAddr);
				memcpy(vrtAddr, desList->descriptor, sizeof(usb_configuration_desc_t));
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_INTERFACE:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_ENDPOINT:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				desc_endptInit((usb_endpoint_desc_t *)desList->descriptor);
				break;

			case USB_DESC_TYPE_HID:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_TYPE_CDC_CS_INTERFACE:
				memcpy(desc_common.data->setupMem + localOffset, desList->descriptor, desList->descriptor->bFunctionLength);
				localOffset += desList->descriptor->bFunctionLength;
				break;

			case USB_DESC_TYPE_HID_REPORT:
				vrtAddr = desc_common.data->setupMem + localOffset;
				desc_common.hidReports = VM_2_PHYM(vrtAddr);
				memcpy(vrtAddr, &desList->descriptor->bDescriptorSubtype, desList->descriptor->bFunctionLength - 2);
				localOffset += desList->descriptor->bFunctionLength - 2;
				break;

			case USB_DESC_STRING:
				desc_strInit(desList, &localOffset, string_desc_count++);
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
	if (setup->wValue) {
		desc_common.dc->status = DC_ADDRESS;

		desc_common.dc->dev_addr = setup->wValue << 25;
		desc_common.dc->dev_addr |= 1 << 24;

		ctrl_setAddress(desc_common.dc->dev_addr);

		desc_common.dc->op = DC_OP_INIT;
		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN);
	}
	else if (desc_common.dc->status != DC_CONFIGURED) {
		desc_common.dc->status = DC_DEFAULT;
	}
}


static void desc_ReqSetConfig(void)
{
	if (desc_common.dc->status == DC_ADDRESS) {
		desc_common.dc->status = DC_CONFIGURED;
		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN);

		if (desc_common.dc->cbEvent)
			desc_common.dc->cbEvent(USBCLIENT_EV_CONFIGURED, desc_common.dc->ctxUser);
	}
}


static int desc_ReqGetConfig(const usb_setup_packet_t *setup)
{
	if (setup->wValue != 0 || setup->wIndex != 0 || setup->wLength != 1)
		return EOK;

	if (desc_common.dc->status != DC_CONFIGURED)
		desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer[0] = 0;
	else
		desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer[1] = 1;

	ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, setup->wLength, USB_ENDPT_DIR_OUT);

	return EOK;
}


static void desc_ReqGetDescriptor(const usb_setup_packet_t *setup)
{
	if (setup->wValue >> 8 == USB_DESC_DEVICE) {
		ctrl_execTransfer(0, desc_common.dev, sizeof(usb_device_desc_t), USB_ENDPT_DIR_IN);
	}
	else if (setup->wValue >> 8 == USB_DESC_CONFIG) {
		ctrl_execTransfer(0, desc_common.cfg, setup->wLength, USB_ENDPT_DIR_IN);
	}
	else if (setup->wValue >> 8 == USB_DESC_STRING) {
		if ((setup->wValue & 0xff) == 0) {
			ctrl_execTransfer(0, desc_common.str0.pmAddr, MIN(desc_common.str0.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
		}
		else if ((setup->wValue & 0xff) == 1) {
			ctrl_execTransfer(0, desc_common.strMan.pmAddr, MIN(desc_common.strMan.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
		}
		else if ((setup->wValue & 0xff) == 2) {
			ctrl_execTransfer(0, desc_common.strProd.pmAddr, MIN(desc_common.strProd.vmStruct->bLength, setup->wLength), USB_ENDPT_DIR_IN);
		}
		else if ((setup->wValue & 0xff) == 4) {
			ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN);
			ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, 71, USB_ENDPT_DIR_OUT);
			return;
		}
	}
	else if (setup->wValue >> 8 == USB_DESC_TYPE_HID_REPORT) {
		ctrl_execTransfer(0, desc_common.hidReports, 76, USB_ENDPT_DIR_IN);
	}

	ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, 0x40, USB_ENDPT_DIR_OUT);
}


static void desc_defaultSetup(const usb_setup_packet_t *setup)
{
	if (*(uint32_t *)setup == 0xdeadc0de) {
		desc_common.dc->op = DC_OP_EXIT;
	}
	else {
		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, setup->wLength, USB_ENDPT_DIR_OUT);
		ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, 0, USB_ENDPT_DIR_IN);
		desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer[0] = 0;

		ctrl_execTransfer(1, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, 0x80, USB_ENDPT_DIR_OUT);
		desc_common.dc->op = DC_OP_RCV_ENDP0;
	}
}


int desc_setup(const usb_setup_packet_t *setup)
{
	int res = EOK;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) != REQUEST_TYPE_STANDARD)
		return EOK;

	switch (setup->bRequest) {
		case REQ_SET_ADDRESS:
			desc_ReqSetAddress(setup);
			break;

		case REQ_SET_CONFIGURATION:
			desc_ReqSetConfig();
			break;

		case REQ_GET_DESCRIPTOR:
			desc_ReqGetDescriptor(setup);
			break;

		case REQ_CLEAR_FEATURE:
		case REQ_GET_STATUS:
		case REQ_GET_INTERFACE:
		case REQ_SET_INTERFACE:
		case REQ_SET_FEATURE:
		case REQ_SET_DESCRIPTOR:
		case REQ_SYNCH_FRAME:
			break;

		case REQ_GET_CONFIGURATION:
			desc_ReqGetConfig(setup);
			break;

		default:
			desc_defaultSetup(setup);
			break;
	}

	return res;
}


int desc_classSetup(const usb_setup_packet_t *setup)
{
	int res;
	dtd_t *dtd;
	void *buf;
	int32_t sz = 0;

	if (!desc_common.dc->cbClassSetup)
		return EOK;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) != REQUEST_TYPE_CLASS)
		return EOK;

	if (setup->wLength > 0) {
		sz = 64 + setup->wLength;               /* read data to buffer with URB struct */
		dtd = ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].pBuffer, sz, USB_ENDPT_DIR_OUT);

		/* Wakeup ENDP0 on error (HID case) */
		if (dtd == NULL || DTD_ERROR(dtd)) {
			desc_common.dc->op = DC_OP_RCV_ERR; /* mark that data is uncomplete */
			desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].len = -1;

			condSignal(desc_common.dc->endp0Cond);

			return -1;
		}

		sz -= DTD_SIZE(dtd);
		if (sz < 0)
			sz = 0;
	}

	buf = desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].vBuffer;

	/*
	 * `buf` on entry to cbClassSetup contains the input data from host,
	 * `buf` on exit from cbClassSetub (if res>0) is reused as output data to host
	 */
	res = desc_common.dc->cbClassSetup(setup, buf, sz, desc_common.dc->ctxUser);

	/* Wakeup end point 0 listener (HID case) */
	if (res == CLASS_SETUP_ENDP0) {
		desc_common.dc->op = DC_OP_RCV_ENDP0;  /* mark that data is ready */
		desc_common.data->endpts[0].buf[USB_ENDPT_DIR_OUT].len = sz;

		condSignal(desc_common.dc->endp0Cond);

		return EOK;
	}
	/* No action - when device doesn't support particular setup request */
	else if (res == CLASS_SETUP_NOACTION) {
		return EOK;
	}
	else if (res <= CLASS_SETUP_ERROR) {
		return -1;
	}
	/* Send back data to host using ENDPT_DIR_IN (reuse buffer) */
	else if (res > CLASS_SETUP_ACK) {
		memcpy(desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].vBuffer, buf, res);
	}

	/*
	 * if res>0: send back data to host
	 * if res=0: CLASS_SETUP_ACK, send an ACK
	 */
	ctrl_execTransfer(0, desc_common.data->endpts[0].buf[USB_ENDPT_DIR_IN].pBuffer, res, USB_ENDPT_DIR_IN);

	return EOK;
}
