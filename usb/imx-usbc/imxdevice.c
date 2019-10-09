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


void init_desc(usbclient_conf_t *conf, usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	imxdevice_common.usb_data = usb_data_in;
	imxdevice_common.dc = dc_in;

	usbclient_desc_dev_t *dev;
	usbclient_desc_conf_t *cfg;
	usbclient_desc_intf_t *intf;
	usbclient_desc_gen_t *hid;
	usbclient_desc_ep_t *endpt;

	usbclient_desc_str_zr_t *str_0;
	usbclient_desc_gen_t *str_man;
	usbclient_desc_gen_t *str_prod;

	usbclient_desc_gen_t *hid_reports;

	memset(imxdevice_common.usb_data->local_conf, 0, USB_BUFFER_SIZE);

	/* Virtual addresses offsets */
	dev = imxdevice_common.usb_data->local_conf;
	cfg = (usbclient_desc_conf_t *)(dev + 1);
	intf = (usbclient_desc_intf_t *)(cfg + 1);
	hid = (usbclient_desc_gen_t *)(intf + 1);
	endpt = (usbclient_desc_ep_t *)(((uint8_t*)hid) + 9);
	str_0 = (usbclient_desc_str_zr_t *)(endpt + 1);
	str_man = (usbclient_desc_gen_t *)(str_0 + 1);
	str_prod = (usbclient_desc_gen_t *)(((uint8_t*)str_man) + 56);
	hid_reports = (usbclient_desc_gen_t *)(((uint8_t*)str_prod) + 28);

	/* Physical addresses offsets */
	imxdevice_common.pdev = (((uint32_t)va2pa(dev)) & ~0xfff) + ((uint32_t)dev & 0xfff);
	imxdevice_common.pconf = (((uint32_t)va2pa(cfg)) & ~0xfff) + ((uint32_t)cfg & 0xfff);

	imxdevice_common.pstr_0 = (((uint32_t)va2pa(str_0)) & ~0xfff) + ((uint32_t)str_0 & 0xfff);
	imxdevice_common.pstr_man = (((uint32_t)va2pa(str_man)) & ~0xfff) + ((uint32_t)str_man & 0xfff);
	imxdevice_common.pstr_prod = (((uint32_t)va2pa(str_prod)) & ~0xfff) + ((uint32_t)str_prod & 0xfff);
	imxdevice_common.phid_reports = (((uint32_t)va2pa(hid_reports)) & ~0xfff) + ((uint32_t)hid_reports & 0xfff);

	/* Endpoints */
	imxdevice_common.IN = imxdevice_common.usb_data->local_conf + 0x500;
	imxdevice_common.OUT = imxdevice_common.usb_data->local_conf + 0x700;

	imxdevice_common.pIN = ((va2pa((void *)imxdevice_common.IN)) & ~0xfff) + ((uint32_t)imxdevice_common.IN & 0xfff);
	imxdevice_common.pOUT = ((va2pa((void *)imxdevice_common.OUT)) & ~0xfff) + ((uint32_t)imxdevice_common.OUT & 0xfff);


	uint32_t string_desc_count = 0;
	/* Extract mandatory descriptors to mapped memory */
	usbclient_desc_list_t *it = conf->descriptors_head;
	for (; it != NULL; it = it->next) {
		switch(it->descriptors->desc_type) {
			case USBCLIENT_DESC_TYPE_DEV:
				memcpy(dev, &it->descriptors[0], sizeof(usbclient_desc_dev_t));
				break;
			case USBCLIENT_DESC_TYPE_CFG:
				memcpy(cfg, &it->descriptors[0], sizeof(usbclient_desc_conf_t));
				break;
			case USBCLIENT_DESC_TYPE_INTF:
				memcpy(intf, &it->descriptors[0], sizeof(usbclient_desc_intf_t));
				break;
			case USBCLIENT_DESC_TYPE_ENDPT:
				memcpy(endpt, &it->descriptors[0], sizeof(usbclient_desc_ep_t));
				/* Initialize endpoint */

				/* For now hardcode only one endpoint */
				imxdevice_common.usb_data->in_endpt.rx_caps.mult = 0;
				imxdevice_common.usb_data->in_endpt.rx_caps.zlt = 1;
				imxdevice_common.usb_data->in_endpt.rx_caps.max_pkt_len = endpt->max_pkt_sz;
				imxdevice_common.usb_data->in_endpt.rx_caps.ios = 0;

				imxdevice_common.usb_data->in_endpt.rx_ctrl.type = (endpt->attr_bmp & 0x03);
				imxdevice_common.usb_data->in_endpt.rx_ctrl.data_toggle = 1;
				imxdevice_common.usb_data->in_endpt.rx_ctrl.data_inhibit = 0;
				imxdevice_common.usb_data->in_endpt.rx_ctrl.stall = 0;

				imxdevice_common.usb_data->in_endpt.tx_caps.mult = 0;
				imxdevice_common.usb_data->in_endpt.tx_caps.zlt = 1;
				imxdevice_common.usb_data->in_endpt.tx_caps.max_pkt_len = endpt->max_pkt_sz;
				imxdevice_common.usb_data->in_endpt.tx_caps.ios = 0;

				imxdevice_common.usb_data->in_endpt.tx_ctrl.type = (endpt->attr_bmp & 0x03);
				imxdevice_common.usb_data->in_endpt.tx_ctrl.data_toggle = 1;
				imxdevice_common.usb_data->in_endpt.tx_ctrl.data_inhibit = 0;
				imxdevice_common.usb_data->in_endpt.tx_ctrl.stall = 0;
				break;
			case USBCLIENT_DESC_TYPE_HID:
				memcpy(hid, &it->descriptors[0], 9);
				break;
			case USBCLIENT_DESC_TYPE_HID_REPORT:
				/* Copy only data section, because HID report descriptor is sent raw */
				memcpy(hid_reports, &it->descriptors[0].data, it->descriptors[0].len - 2);
				break;
			case USBCLIENT_DESC_TYPE_STR:
				if (string_desc_count == 0)
					memcpy(str_0, &it->descriptors[0], sizeof(usbclient_desc_str_zr_t));
				else if (string_desc_count == 1)
					memcpy(str_man, &it->descriptors[0], it->descriptors[0].len);
				else if (string_desc_count == 2)
					memcpy(str_prod, &it->descriptors[0], it->descriptors[0].len);

				string_desc_count++;
				break;
			case USBCLIENT_DESC_TYPE_DEV_QUAL:
			case USBCLIENT_DESC_TYPE_OTH_SPD_CFG:
			case USBCLIENT_DESC_TYPE_INTF_PWR:
			default:
				/* Not implemented yet */
				break;
		}
	}
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


int dc_setup(setup_packet_t *setup)
{
	int res = EOK;
	uint32_t fsz;

	if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_STANDARD)
		return EOK;

	switch (setup->req_code) {
		case REQ_SET_ADDR:
			if (setup->val) {
				imxdevice_common.dc->status = DC_ADDRESS;

				imxdevice_common.dc->dev_addr = setup->val << 25;
				imxdevice_common.dc->dev_addr |= 1 << 24;

				*(imxdevice_common.dc->base + deviceaddr) = imxdevice_common.dc->dev_addr;
				imxdevice_common.dc->op = DC_OP_INIT;
				dtd_exec(0, imxdevice_common.pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			}
			else if (imxdevice_common.dc->status != DC_CONFIGURED) {
				imxdevice_common.dc->status = DC_DEFAULT;
			}
			break;

		case REQ_SET_CONFIG:
			if (imxdevice_common.dc->status == DC_ADDRESS) {
				imxdevice_common.dc->status = DC_CONFIGURED;
				dtd_exec(0, imxdevice_common.pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			}
			break;

		case REQ_GET_DESC:
			if (setup->val >> 8 == USBCLIENT_DESC_TYPE_DEV) {
				dtd_exec(0, imxdevice_common.pdev, sizeof(usbclient_desc_dev_t), USBCLIENT_ENDPT_DIR_IN);
			}
			else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_CFG) {
				dtd_exec(0, imxdevice_common.pconf, setup->len, USBCLIENT_ENDPT_DIR_IN);
			}
			else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_STR) {
				if ((setup->val & 0xff) == 0)
					dtd_exec(0, imxdevice_common.pstr_0, MIN(sizeof(usbclient_desc_str_zr_t), setup->len), USBCLIENT_ENDPT_DIR_IN);
				else if ((setup->val & 0xff) == 1)
					dtd_exec(0, imxdevice_common.pstr_man, MIN(56, setup->len), USBCLIENT_ENDPT_DIR_IN);
				else if ((setup->val & 0xff) == 2)
					dtd_exec(0, imxdevice_common.pstr_prod, MIN(28, setup->len), USBCLIENT_ENDPT_DIR_IN);
			}
			else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_HID_REPORT) {
				dtd_exec(0, imxdevice_common.phid_reports, 76, USBCLIENT_ENDPT_DIR_IN);
			}
			dtd_exec(0, imxdevice_common.pOUT, 0x40, USBCLIENT_ENDPT_DIR_OUT);
			break;

		case REQ_CLR_FEAT:
		case REQ_GET_STS:
		case REQ_GET_INTF:
		case REQ_SET_INTF:
		case REQ_SET_FEAT:
		case REQ_SET_DESC:
		case REQ_SYNCH_FRAME:
			break;

		case REQ_GET_CONFIG:
			if (setup->val != 0 || setup->idx != 0 || setup->len != 1)
				return res;

			if (imxdevice_common.dc->status != DC_CONFIGURED)
				imxdevice_common.OUT[0] = 0;
			else
				imxdevice_common.OUT[1] = 1;

			dtd_exec(0, imxdevice_common.pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
			break;

		default:
			if (*(uint32_t *)setup == 0xdeadc0de) {
				imxdevice_common.dc->op = DC_OP_EXIT;
			}
			else {
				fsz = setup->val << 16;
				fsz |= setup->idx;
				dtd_exec(0, imxdevice_common.pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
				dtd_exec(0, imxdevice_common.pIN, 0, USBCLIENT_ENDPT_DIR_IN);
				imxdevice_common.OUT[0] = 0;

				dtd_exec(1, imxdevice_common.pOUT, 0x80, USBCLIENT_ENDPT_DIR_OUT);
				imxdevice_common.dc->op = DC_OP_RECEIVE;
			}
			break;
	}

	return res;
}


int dc_class_setup(setup_packet_t *setup)
{
	int res = EOK;

	if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_CLASS)
		return EOK;

	switch (setup->req_code) {
		case CLASS_REQ_SET_IDLE:
			dtd_exec(0, imxdevice_common.pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			break;

		case CLASS_REQ_SET_REPORT:
			dtd_exec(0, imxdevice_common.pOUT, 64 + setup->len, USBCLIENT_ENDPT_DIR_OUT); /* read data to buffer with URB struct*/
			imxdevice_common.usb_data->read_buffer.length = setup->len;
			imxdevice_common.dc->op = DC_OP_RECEIVE; /* mark that data is ready */
			break;

		case CLASS_REQ_GET_IDLE:
		case CLASS_REQ_GET_PROTOCOL:
		case CLASS_REQ_GET_REPORT:
		case CLASS_REQ_SET_PROTOCOL:
		default:
			break;
	}

	return res;
}


/* high frequency interrupts */
int dc_hf_intr(void)
{
	setup_packet_t setup;
	uint32_t status;
	int endpt = 0;

	if ((status = *(imxdevice_common.dc->base + endptsetupstat)) & 0x1) {
		/* trip wire set */
		while (!((status >> endpt) & 1))
			endpt++;
		do {
			*(imxdevice_common.dc->base + usbcmd) |= 1 << 13;
			memcpy(&setup, imxdevice_common.dc->endptqh[endpt].setup_buff, sizeof(setup_packet_t));
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


int usbclient_send(usbclient_ep_t *ep, const void *data, unsigned int len)
{
	if (len > USB_BUFFER_SIZE)
		return -1;

	if (ep->direction != USBCLIENT_ENDPT_DIR_IN)
		return -1;

	memcpy(imxdevice_common.IN, data, len);
	dtd_exec(ep->id, imxdevice_common.pIN, len, ep->direction);

	return len;
}


int usbclient_receive(usbclient_ep_t *ep, void *data, unsigned int len)
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
			dtd_exec(0, imxdevice_common.pIN, 0, USBCLIENT_ENDPT_DIR_IN); /* ACK */
			break;
		}
	}

	return result;
}
