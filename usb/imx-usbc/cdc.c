/*
 * Phoenix-RTOS
 *
 * cdc - USB Communication Device Class
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>

#include "cdc.h"


struct {
	usbclient_conf_t config;

	usbclient_desc_list_t dev;
	usbclient_desc_list_t conf;

	usbclient_desc_list_t comIface;
	usbclient_desc_list_t header;
	usbclient_desc_list_t call;
	usbclient_desc_list_t acm;
	usbclient_desc_list_t unio;
	usbclient_desc_list_t comEp;

	usbclient_desc_list_t dataIface;
	usbclient_desc_list_t dataEpOUT;
	usbclient_desc_list_t dataEpIN;

	usbclient_desc_list_t str0;
	usbclient_desc_list_t strman;
	usbclient_desc_list_t strprod;
} cdc_common;


/* Device descriptor */
static const usbclient_desc_dev_t ddev = { .len = sizeof(usbclient_desc_dev_t), .desc_type = USBCLIENT_DESC_TYPE_DEV, .bcd_usb = 0x0002,
	.dev_class = 0x0, .dev_subclass = 0, .dev_prot = 0, .max_pkt_sz0 = 64,
	.vend_id = 0x16f9, .prod_id = 0x0003, .bcd_dev = 0x0200,
	.man_str = 1, .prod_str = 2, .sn_str = 0,
	.num_conf = 1 };


/* Configuration descriptor */
static const usbclient_desc_conf_t dconfig = { .len = 9, .desc_type = USBCLIENT_DESC_TYPE_CFG,
	.total_len = sizeof(usbclient_desc_conf_t) + sizeof(usbclient_desc_intf_t) + sizeof(usbclient_desc_cdc_header_t) + sizeof(usbclient_desc_cdc_call_t)
	+ sizeof(usbclient_desc_cdc_acm_t) + sizeof(usbclient_desc_cdc_union_t) + sizeof(usbclient_desc_ep_t) + sizeof(usbclient_desc_intf_t) + sizeof(usbclient_desc_ep_t) + sizeof(usbclient_desc_ep_t),
	.num_intf = 2, .conf_val = 1, .conf_str = 0, .attr_bmp = 0xc0, .max_pow = 5 };


/* Communication Interface Descriptor */
static const usbclient_desc_intf_t dComIntf =  { .len = 9, .desc_type = USBCLIENT_DESC_TYPE_INTF, .intf_num = 0, .alt_set = 0,
	.num_endpt = 1, .intf_class = 0x02, .intf_subclass = 0x02, .intf_prot = 0x00, .intf_str = 4 };


static const usbclient_desc_cdc_header_t dHeader = { .len = 5, .type = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, .sub_type = 0, .bcd_cdc = 0x0110 };


static const usbclient_desc_cdc_call_t dCall = { .len = 5, .type = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, .sub_type = 0x01, .cap = 0x01, .data_int = 0x1 };


static const usbclient_desc_cdc_acm_t dAcm = { .len = 4, .type = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, .sub_type = 0x02, .cap = 0x03 };


static const usbclient_desc_cdc_union_t dUnion = { .len = 5, .type = USB_DESCRIPTOR_TYPE_CDC_CS_INTERFACE, .sub_type = 0x06, .con_int = 0x0, .sub_con_int = 0x1};


/* Communication Interrupt Endpoint IN */
static const usbclient_desc_ep_t dComEp = { .len = 7, .desc_type = USBCLIENT_DESC_TYPE_ENDPT, .endpt_addr = 0x83	, /* direction IN */
	.attr_bmp = 0x03, .max_pkt_sz = 16, .interval = 0x08
};


/* CDC Data Interface Descriptor */
static const usbclient_desc_intf_t dDataIntf = { .len = 9, .desc_type = USBCLIENT_DESC_TYPE_INTF, .intf_num = 1, .alt_set = 0,
	 .num_endpt = 2, .intf_class = 0x0a, .intf_subclass = 0x00, .intf_prot = 0x00, .intf_str = 0
};


/* Data Bulk Endpoint OUT */
static const usbclient_desc_ep_t depOUT = { .len = 7, .desc_type = USBCLIENT_DESC_TYPE_ENDPT, .endpt_addr = 0x01, /* direction OUT */
	.attr_bmp = 0x02, .max_pkt_sz = 0x0200, .interval = 0
};


/* Data Bulk Endpoint IN */
static const usbclient_desc_ep_t depIN = { .len = 7, .desc_type = USBCLIENT_DESC_TYPE_ENDPT, .endpt_addr = 0x82, /* direction IN */
	.attr_bmp = 0x02, .max_pkt_sz = 0x0200, .interval = 1
};


/* String Data */
static const usbclient_desc_str_t dstrman = {
	.len = 2 * 18 + 2,
	.desc_type = USBCLIENT_DESC_TYPE_STR,
	.string = {	'N', 0, 'X', 0, 'P', 0, ' ', 0, 'S', 0, 'E', 0, 'M', 0, 'I', 0, 'C', 0, 'O', 0, 'N', 0, 'D', 0, 'U', 0, 'C', 0, 'T', 0,
				'O', 0, 'R', 0, 'S', 0 }
};


static const usbclient_desc_str_zr_t dstr0 = {
	.len = sizeof(usbclient_desc_str_zr_t),
	.desc_type = USBCLIENT_DESC_TYPE_STR,
	.w_langid0 = 0x0409 /* English */
};


static const usbclient_desc_str_t dstrprod = {
	.len = 11 * 2 + 2,
	.desc_type = USBCLIENT_DESC_TYPE_STR,
	.string = { 'M', 0, 'C', 0, 'U', 0, ' ', 0, 'V', 0, 'I', 0, 'R', 0, 'T', 0, 'U', 0, 'A', 0, 'L', 0 }
};


int cdc_recv(char *data, unsigned int len)
{
	return usbclient_receive(&cdc_common.config.endpoint_list.endpoints[2], data, len);
}


int cdc_send(const char *data, unsigned int len)
{
	return usbclient_send(&cdc_common.config.endpoint_list.endpoints[1], data, len);
}


void cdc_destroy(void)
{
	usbclient_destroy();
}


int cdc_init(void)
{
	int res;

	usbclient_ep_list_t endpoints = {
		.size = 3,
		.endpoints = {
			{ .id = 0, .type = USBCLIENT_ENDPT_TYPE_CONTROL, .direction = 0 },
			{ .id = 1, .type = USBCLIENT_ENDPT_TYPE_INTR, .direction = USBCLIENT_ENDPT_DIR_IN },
			{ .id = 2, .type = USBCLIENT_ENDPT_TYPE_BULK, .direction = USBCLIENT_ENDPT_DIR_OUT },
			{ .id = 3, .type = USBCLIENT_ENDPT_TYPE_BULK, .direction = USBCLIENT_ENDPT_DIR_IN }
		}
	};

	cdc_common.dev.size = 1;
	cdc_common.dev.descriptors = (usbclient_desc_gen_t *)&ddev;
	cdc_common.dev.next = &cdc_common.conf;

	cdc_common.conf.size = 1;
	cdc_common.conf.descriptors = (usbclient_desc_gen_t *)&dconfig;
	cdc_common.conf.next = &cdc_common.comIface;

	cdc_common.comIface.size = 1;
	cdc_common.comIface.descriptors = (usbclient_desc_gen_t *)&dComIntf;
	cdc_common.comIface.next = &cdc_common.header;

	cdc_common.header.size = 1;
	cdc_common.header.descriptors = (usbclient_desc_gen_t *)&dHeader;
	cdc_common.header.next = &cdc_common.call;

	cdc_common.call.size = 1;
	cdc_common.call.descriptors = (usbclient_desc_gen_t *)&dCall;
	cdc_common.call.next = &cdc_common.acm;

	cdc_common.acm.size = 1;
	cdc_common.acm.descriptors = (usbclient_desc_gen_t *)&dAcm;
	cdc_common.acm.next = &cdc_common.unio;

	cdc_common.unio.size = 1;
	cdc_common.unio.descriptors = (usbclient_desc_gen_t *)&dComEp;
	cdc_common.unio.next = &cdc_common.comEp;

	cdc_common.comEp.size = 1;
	cdc_common.comEp.descriptors = (usbclient_desc_gen_t *)&dUnion;
	cdc_common.comEp.next = &cdc_common.dataIface;

	cdc_common.dataIface.size = 1;
	cdc_common.dataIface.descriptors = (usbclient_desc_gen_t *)&dDataIntf;
	cdc_common.dataIface.next = &cdc_common.dataEpOUT;

	cdc_common.dataEpOUT.size = 1;
	cdc_common.dataEpOUT.descriptors = (usbclient_desc_gen_t *)&depOUT;
	cdc_common.dataEpOUT.next = &cdc_common.dataEpIN;

	cdc_common.dataEpIN.size = 1;
	cdc_common.dataEpIN.descriptors = (usbclient_desc_gen_t *)&depIN;
	cdc_common.dataEpIN.next = &cdc_common.str0;

	cdc_common.str0.size = 1;
	cdc_common.str0.descriptors = (usbclient_desc_gen_t *)&dstr0;
	cdc_common.str0.next = &cdc_common.strman;

	cdc_common.strman.size = 1;
	cdc_common.strman.descriptors = (usbclient_desc_gen_t *)&dstrman;
	cdc_common.strman.next = &cdc_common.strprod;

	cdc_common.strprod.size = 1;
	cdc_common.strprod.descriptors = (usbclient_desc_gen_t *)&dstrprod;
	cdc_common.strprod.next = NULL;

	cdc_common.config.endpoint_list = endpoints;
	cdc_common.config.descriptors_head = &cdc_common.dev;

	if ((res = usbclient_init(&cdc_common.config)) != EOK)
		return res;

	return EOK;
}
