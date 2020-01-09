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

#include "imxdevice.h"

#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/threads.h>

#include <phoenix/arch/imxrt.h>

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

struct {
	addr_t pdev;
	addr_t pconf;

	addr_t pstr_0;
	addr_t pstr_man;
	addr_t pstr_prod;

	addr_t phid_reports;

	addr_t pIN;
	addr_t pOUT;
	uint8_t *IN;
	uint8_t *OUT;

	usb_common_data_t *usb_data;

	usb_dc_t *dc;
} imxdevice_common;


static void init_endpt(usb_endpoint_desc_t *endpt)
{
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_caps.mult = 0;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_caps.zlt = 1;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_caps.max_pkt_len = endpt->wMaxPacketSize;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_caps.ios = 0;

	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_ctrl.type = endpt->bmAttributes & 0x03;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_ctrl.data_toggle = 1;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_ctrl.data_inhibit = 0;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].rx_ctrl.stall = 0;

	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_caps.mult = 0;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_caps.zlt = 1;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_caps.max_pkt_len = endpt->wMaxPacketSize;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_caps.ios = 0;

	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_ctrl.type = endpt->bmAttributes & 0x03;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_ctrl.data_toggle = 1;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_ctrl.data_inhibit = 0;
	imxdevice_common.usb_data->in_endpt[imxdevice_common.usb_data->endNb].tx_ctrl.stall = 0;

	imxdevice_common.usb_data->endNb++;
}


static void init_strDesc(usb_desc_list_t *it, int *localOffset, int strOrder)
{
	usb_string_desc_t *str_0;
	usb_functional_desc_t *str_man;
	usb_functional_desc_t *str_prod;

	switch (strOrder) {
		case 0:
			str_0 = imxdevice_common.usb_data->local_conf + *localOffset;
			imxdevice_common.pstr_0 = (((uint32_t)va2pa(str_0)) & ~0xfff) + ((uint32_t)str_0 & 0xfff);
			memcpy(str_0, &it->descriptors[0], sizeof(usb_string_desc_t));
			*localOffset += it->descriptors[0].bFunctionLength;
			break;
		case 1:
			str_man = imxdevice_common.usb_data->local_conf + *localOffset;
			imxdevice_common.pstr_man = (((uint32_t)va2pa(str_man)) & ~0xfff) + ((uint32_t)str_man & 0xfff);
			memcpy(str_man, &it->descriptors[0], it->descriptors[0].bFunctionLength);
			*localOffset += it->descriptors[0].bFunctionLength;
			break;
		case 2:
			str_prod = imxdevice_common.usb_data->local_conf + *localOffset;
			imxdevice_common.pstr_prod = (((uint32_t)va2pa(str_prod)) & ~0xfff) + ((uint32_t)str_prod & 0xfff);
			memcpy(str_prod, &it->descriptors[0], it->descriptors[0].bFunctionLength);
			*localOffset += it->descriptors[0].bFunctionLength;
		default:
			break;
	}
}


int init_desc(usb_conf_t *conf, usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	int localOffset = 0;
	uint32_t string_desc_count = 0;

	usb_device_desc_t *dev;
	usb_configuration_desc_t *cfg;
	usb_functional_desc_t *hid_reports;

	usb_desc_list_t *it;

	imxdevice_common.dc = dc_in;
	imxdevice_common.usb_data = usb_data_in;

	memset(imxdevice_common.usb_data->local_conf, 0, USB_BUFFER_SIZE);

	/* Endpoints */
	imxdevice_common.IN = imxdevice_common.usb_data->local_conf + 0x500;
	imxdevice_common.OUT = imxdevice_common.usb_data->local_conf + 0x700;

	imxdevice_common.pIN = ((va2pa((void *)imxdevice_common.IN)) & ~0xfff) + ((uint32_t)imxdevice_common.IN & 0xfff);
	imxdevice_common.pOUT = ((va2pa((void *)imxdevice_common.OUT)) & ~0xfff) + ((uint32_t)imxdevice_common.OUT & 0xfff);

	/* Extract mandatory descriptors to mapped memory */
	it = conf->descriptors_head;
	for (; it != NULL; it = it->next) {
		/* Control data cannot be bigger than 0x500 */
		if (localOffset > 0x500)
			return -ENOMEM	;

		switch(it->descriptors->bDescriptorType) {
			case USB_DESC_DEVICE:
				dev = imxdevice_common.usb_data->local_conf + localOffset;
				imxdevice_common.pdev = (((uint32_t)va2pa(dev)) & ~0xfff) + ((uint32_t)dev & 0xfff);
				memcpy(dev, &it->descriptors[0], sizeof(usb_device_desc_t));
				localOffset += dev->bLength;
				break;

			case USB_DESC_CONFIG:
				cfg = imxdevice_common.usb_data->local_conf + localOffset;
				imxdevice_common.pconf = (((uint32_t)va2pa(cfg)) & ~0xfff) + ((uint32_t)cfg & 0xfff);
				memcpy(cfg, &it->descriptors[0], sizeof(usb_configuration_desc_t));
				localOffset += cfg->bLength;
				break;

			case USB_DESC_INTERFACE:
				memcpy(imxdevice_common.usb_data->local_conf + localOffset, &it->descriptors[0], it->descriptors[0].bFunctionLength);
				localOffset += it->descriptors[0].bFunctionLength;
				break;

			case USB_DESC_ENDPOINT:
				memcpy(imxdevice_common.usb_data->local_conf + localOffset, &it->descriptors[0], it->descriptors[0].bFunctionLength);
				localOffset += it->descriptors[0].bFunctionLength;
				init_endpt((usb_endpoint_desc_t *)&it->descriptors[0]);
				break;

			case USB_DESC_TYPE_HID:
				memcpy(imxdevice_common.usb_data->local_conf + localOffset, &it->descriptors[0], it->descriptors[0].bFunctionLength);
				localOffset += it->descriptors[0].bFunctionLength;
				break;

			case USB_DESC_TYPE_CDC_CS_INTERFACE:
				memcpy(imxdevice_common.usb_data->local_conf + localOffset, &it->descriptors[0], it->descriptors[0].bFunctionLength);
				localOffset += it->descriptors[0].bFunctionLength;
				break;

			case USB_DESC_TYPE_HID_REPORT:
				hid_reports = imxdevice_common.usb_data->local_conf + localOffset;
				imxdevice_common.phid_reports = (((uint32_t)va2pa(hid_reports)) & ~0xfff) + ((uint32_t)hid_reports & 0xfff);
				memcpy(hid_reports, &it->descriptors[0].bDescriptorSubtype, it->descriptors[0].bFunctionLength - 2);
				localOffset += it->descriptors[0].bFunctionLength - 2;
				break;

			case USB_DESC_STRING:
				init_strDesc(it, &localOffset, string_desc_count++);
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


static dtd_t *dtd_get(int endpt, int dir)
{
	int qh = endpt * 2 + dir;
	uint32_t base_addr;
	dtd_t *ret;

	base_addr = ((uint32_t)imxdevice_common.dc->endptqh[qh].head & ~((imxdevice_common.dc->endptqh[qh].size * sizeof(dtd_t)) - 1));

	ret = imxdevice_common.dc->endptqh[qh].tail++;
	imxdevice_common.dc->endptqh[qh].tail = (dtd_t *)(base_addr | ((uint32_t)imxdevice_common.dc->endptqh[qh].tail & (((imxdevice_common.dc->endptqh[qh].size) * sizeof(dtd_t)) - 1)));

	return ret;
}


static int dtd_build(dtd_t *dtd, uint32_t paddr, uint32_t size)
{
	if (size > USB_BUFFER_SIZE)
		return -EINVAL;

	dtd->dtd_next = 1;
	dtd->dtd_token = size << 16;

	dtd->dtd_token |= 1 << 7;
	dtd->buff_ptr[0] = paddr;

	return EOK;
}


static int dtd_exec(int endpt, uint32_t paddr, uint32_t sz, int dir)
{
	int shift;
	uint32_t offs;
	dtd_t *dtd;
	int qh = (endpt << 1) + dir;

	dtd = dtd_get(endpt, dir);

	dtd_build(dtd, paddr, sz);

	shift = endpt + ((qh & 1) ? 16 : 0);
	offs = (uint32_t)dtd & (((imxdevice_common.dc->endptqh[qh].size) * sizeof(dtd_t)) - 1);

	imxdevice_common.dc->endptqh[qh].dtd_next = (imxdevice_common.dc->endptqh[qh].base + offs) & ~1;
	imxdevice_common.dc->endptqh[qh].dtd_token &= ~(1 << 6);
	imxdevice_common.dc->endptqh[qh].dtd_token &= ~(1 << 7);

	/* prime the endpoint and wait for it to prime */
	while ((*(imxdevice_common.dc->base + endptprime) & (1 << shift)));
	*(imxdevice_common.dc->base + endptprime) |= 1 << shift;
	while (!(*(imxdevice_common.dc->base + endptprime) & (1 << shift)) && (*(imxdevice_common.dc->base + endptstat) & (1 << shift)));

	while (!(*(imxdevice_common.dc->base + endptcomplete) & (1 << shift)));
	*(imxdevice_common.dc->base + endptcomplete) |= 1 << shift;

	while (*(imxdevice_common.dc->base + usbsts) & 1);
	*(imxdevice_common.dc->base + usbsts) |= 1;
	imxdevice_common.dc->endptqh[qh].head += 1;

	return EOK;
}


int dc_setup(usb_setup_packet_t *setup)
{
	int res = EOK;
	uint32_t fsz;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) != REQUEST_TYPE_STANDARD)
		return EOK;

	switch (setup->bRequest) {
		case REQ_SET_ADDRESS:
			if (setup->wValue) {
				imxdevice_common.dc->status = DC_ADDRESS;

				imxdevice_common.dc->dev_addr = setup->wValue << 25;
				imxdevice_common.dc->dev_addr |= 1 << 24;

				*(imxdevice_common.dc->base + deviceaddr) = imxdevice_common.dc->dev_addr;
				imxdevice_common.dc->op = DC_OP_INIT;
				dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN);
			}
			else if (imxdevice_common.dc->status != DC_CONFIGURED) {
				imxdevice_common.dc->status = DC_DEFAULT;
			}
			break;

		case REQ_SET_CONFIGURATION:
			if (imxdevice_common.dc->status == DC_ADDRESS) {
				imxdevice_common.dc->status = DC_CONFIGURED;
				dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN);
			}
			break;

		case REQ_GET_DESCRIPTOR:
			if (setup->wValue >> 8 == USB_DESC_DEVICE) {
				dtd_exec(0, imxdevice_common.pdev, sizeof(usb_device_desc_t), USB_ENDPT_DIR_IN);
			}
			else if (setup->wValue >> 8 == USB_DESC_CONFIG) {
				dtd_exec(0, imxdevice_common.pconf, setup->wLength, USB_ENDPT_DIR_IN);
			}
			else if (setup->wValue >> 8 == USB_DESC_STRING) {
				if ((setup->wValue & 0xff) == 0)
					dtd_exec(0, imxdevice_common.pstr_0, MIN(sizeof(usb_string_desc_t), setup->wLength), USB_ENDPT_DIR_IN);
				else if ((setup->wValue & 0xff) == 1)
					dtd_exec(0, imxdevice_common.pstr_man, MIN(56, setup->wLength), USB_ENDPT_DIR_IN);
				else if ((setup->wValue & 0xff) == 2)
					dtd_exec(0, imxdevice_common.pstr_prod, MIN(30, setup->wLength), USB_ENDPT_DIR_IN);
				else if ((setup->wValue & 0xff) == 4) {
					dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN);
					dtd_exec(0, imxdevice_common.pOUT, 71, USB_ENDPT_DIR_OUT);
					break;
				}
			}
			else if (setup->wValue >> 8 == USB_DESC_TYPE_HID_REPORT) {
				dtd_exec(0, imxdevice_common.phid_reports, 76, USB_ENDPT_DIR_IN);
			}
			dtd_exec(0, imxdevice_common.pOUT, 0x40	, USB_ENDPT_DIR_OUT);
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
			if (setup->wValue != 0 || setup->wIndex != 0 || setup->wLength != 1)
				return res;

			if (imxdevice_common.dc->status != DC_CONFIGURED)
				imxdevice_common.OUT[0] = 0;
			else
				imxdevice_common.OUT[1] = 1;

			dtd_exec(0, imxdevice_common.pOUT, setup->wLength, USB_ENDPT_DIR_OUT);
			break;

		default:
			if (*(uint32_t *)setup == 0xdeadc0de) {
				imxdevice_common.dc->op = DC_OP_EXIT;
			}
			else {
				fsz = setup->wValue << 16;
				fsz |= setup->wIndex;
				dtd_exec(0, imxdevice_common.pOUT, setup->wLength, USB_ENDPT_DIR_OUT);
				dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN);
				imxdevice_common.OUT[0] = 0;

				dtd_exec(1, imxdevice_common.pOUT, 0x80, USB_ENDPT_DIR_OUT);
				imxdevice_common.dc->op = DC_OP_RECEIVE;
			}
			break;
	}

	return res;
}


int dc_class_setup(usb_setup_packet_t *setup)
{
	int res = EOK;

	if (EXTRACT_REQ_TYPE(setup->bmRequestType) != REQUEST_TYPE_CLASS)
		return EOK;

	switch (setup->bRequest) {
		case CLASS_REQ_SET_IDLE:
			dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN);
			break;

		case CLASS_REQ_SET_REPORT:
			dtd_exec(0, imxdevice_common.pOUT, 64 + setup->wLength, USB_ENDPT_DIR_OUT); /* read data to buffer with URB struct*/
			imxdevice_common.usb_data->read_buffer.length = setup->wLength;
			imxdevice_common.dc->op = DC_OP_RECEIVE; /* mark that data is ready */
			break;

		case CLASS_REQ_GET_IDLE:
		case CLASS_REQ_GET_PROTOCOL:
		case CLASS_REQ_GET_REPORT:
		case CLASS_REQ_SET_PROTOCOL:
			break;

		case CLASS_REQ_SET_LINE_CODING:
			dtd_exec(0, imxdevice_common.pOUT, 64 + setup->wLength, USB_ENDPT_DIR_OUT); /* read data to buffer with URB struct*/
			imxdevice_common.usb_data->read_buffer.length = setup->wLength;
			imxdevice_common.dc->op = DC_OP_RECEIVE; /* mark that data is ready */
			break;

		case CLASS_REQ_SET_CONTROL_LINE_STATE:
			imxdevice_common.usb_data->read_buffer.length = setup->wLength;
			imxdevice_common.dc->op = DC_OP_RECEIVE; /* mark that data is ready */
			break;

		default:
			break;
	}

	return res;
}


/* high frequency interrupts */
int dc_hf_intr(void)
{
	usb_setup_packet_t setup;
	uint32_t status;
	int endpt = 0;

	if ((status = *(imxdevice_common.dc->base + endptsetupstat)) & 0x1) {
		/* trip wire set */
		while (!((status >> endpt) & 1))
			endpt++;
		do {
			*(imxdevice_common.dc->base + usbcmd) |= 1 << 13;
			memcpy(&setup, imxdevice_common.dc->endptqh[endpt].setup_buff, sizeof(usb_setup_packet_t));
		} while (!(*(imxdevice_common.dc->base + usbcmd) & 1 << 13));

		*(imxdevice_common.dc->base + endptsetupstat) |= 1 << endpt;
		*(imxdevice_common.dc->base + usbcmd) &= ~(1 << 13);
		*(imxdevice_common.dc->base + endptflush) |= 0xffffffff;
		*(imxdevice_common.dc->base + usbsts) |= 1;

		imxdevice_common.dc->endptqh[0].head = imxdevice_common.dc->endptqh[0].tail;
		imxdevice_common.dc->endptqh[1].head = imxdevice_common.dc->endptqh[1].tail;
		imxdevice_common.dc->endptqh[2].head = imxdevice_common.dc->endptqh[2].tail;

		while (*(imxdevice_common.dc->base + endptsetupstat) & 1);

		dc_setup(&setup);
		dc_class_setup(&setup);
	}

	return 1;
}


/* low frequency interrupts */
int dc_lf_intr(void)
{
	if ((*(imxdevice_common.dc->base + usbsts) & 1 << 6)) {

		*(imxdevice_common.dc->base + endptsetupstat) = *(imxdevice_common.dc->base + endptsetupstat);
		*(imxdevice_common.dc->base + endptcomplete) = *(imxdevice_common.dc->base + endptcomplete);

		while (*(imxdevice_common.dc->base + endptprime));

		*(imxdevice_common.dc->base + endptflush) = 0xffffffff;

		while (*(imxdevice_common.dc->base + portsc1) & 1 << 8);

		*(imxdevice_common.dc->base + usbsts) |= 1 << 6;
		imxdevice_common.dc->status = DC_DEFAULT;
	}

	return 1;
}


int usbclient_send(usb_endpoint_conf_t *ep, const void *data, unsigned int len)
{
	if (len > USB_BUFFER_SIZE)
		return -1;

	if (ep->direction != USB_ENDPT_DIR_IN)
		return -1;

	memcpy(imxdevice_common.IN, data, len);
	dtd_exec(ep->id, imxdevice_common.pIN, len, ep->direction);

	return len;
}


int usbclient_receive(usb_endpoint_conf_t *ep, void *data, unsigned int len)
{
	int32_t result = -1;

	while (imxdevice_common.dc->op != DC_OP_EXIT) {
		mutexLock(imxdevice_common.dc->lock);
		while (imxdevice_common.dc->op == DC_OP_NONE)
			condWait(imxdevice_common.dc->cond, imxdevice_common.dc->lock, 0);
		mutexUnlock(imxdevice_common.dc->lock);

		if (imxdevice_common.dc->op == DC_OP_RECEIVE) {
			memcpy(data, (const char *)imxdevice_common.OUT, imxdevice_common.usb_data->read_buffer.length); /* copy data to buffer */
			result = imxdevice_common.usb_data->read_buffer.length;

			imxdevice_common.usb_data->read_buffer.length = 0;
			imxdevice_common.dc->op = DC_OP_NONE;
			dtd_exec(0, imxdevice_common.pIN, 0, USB_ENDPT_DIR_IN); /* ACK */
			break;
		}
	}

	return result;
}
