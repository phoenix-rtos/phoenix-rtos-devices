/*
 * Phoenix-RTOS
 *
 * psd - Serial Download Protocol client
 *
 * HID support
 *
 * Copyright 2019 Phoenix Systems
 * Author: Bartosz Ciesla, Pawel Pisarczyk, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#include <errno.h>
#include <stdio.h>

#include "hid.h"


struct {
	usbclient_conf_t config;

	usbclient_desc_list_t dev;
	usbclient_desc_list_t conf;
	usbclient_desc_list_t iface;

	usbclient_desc_list_t hid;
	usbclient_desc_list_t ep;

	usbclient_desc_list_t str0;
	usbclient_desc_list_t strman;
	usbclient_desc_list_t strprod;

	usbclient_desc_list_t hidreport;
} hid_common;


static const hid_desc_report_t dhidreport = { 2 + 76, USBCLIENT_DESC_TYPE_HID_REPORT,
	/* Raw HID report descriptor - compatibile with IMX6ULL SDP protocol */
	{ 0x06, 0x00, 0xff, 0x09, 0x01, 0xa1, 0x01, 0x85, 0x01, 0x19,
	  0x01, 0x29, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08,
	  0x95, 0x10, 0x91, 0x02, 0x85, 0x02, 0x19, 0x01, 0x29, 0x01,
	  0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x80, 0x95, 0x40, 0x91,
	  0x02, 0x85, 0x03, 0x19, 0x01, 0x29, 0x01, 0x15, 0x00, 0x26,
	  0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x85, 0x04,
	  0x19, 0x01, 0x29, 0x01, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75,
	  0x08, 0x95, 0x40, 0x81, 0x02, 0xc0 }
};


static const usbclient_desc_ep_t dep = { .len = 7, .desc_type = USBCLIENT_DESC_TYPE_ENDPT, .endpt_addr = 0x81, /* direction IN */
	.attr_bmp = 0x03, .max_pkt_sz = 64, .interval = 0x01
};


static const hid_desc_t dhid = { 9, USBCLIENT_DESC_TYPE_HID, 0x0110, 0x0, 1, 0x22, 76 };


static const usbclient_desc_intf_t diface = { .len = 9, .desc_type = USBCLIENT_DESC_TYPE_INTF, .intf_num = 0, .alt_set = 0,
	.num_endpt = 1, .intf_class = 0x03, .intf_subclass = 0x00, .intf_prot = 0x00, .intf_str = 2
};


static const usbclient_desc_conf_t dconfig = { .len = 9, .desc_type = USBCLIENT_DESC_TYPE_CFG,
	.total_len = sizeof(usbclient_desc_conf_t) + sizeof(usbclient_desc_intf_t) + sizeof(dhid) + sizeof(usbclient_desc_ep_t),
	.num_intf = 1, .conf_val = 1, .conf_str = 1, .attr_bmp = 0xc0, .max_pow = 5
};


static const usbclient_desc_str_zr_t dstr0 = {
	.len = sizeof(usbclient_desc_str_zr_t),
	.desc_type = USBCLIENT_DESC_TYPE_STR,
	.w_langid0 = 0x0409 /* English */
};


int hid_recv(char *data, unsigned int len)
{
	return usbclient_receive(&hid_common.config.endpoint_list.endpoints[0], data, len);
}


int hid_send(const char *data, unsigned int len)
{
	return usbclient_send(&hid_common.config.endpoint_list.endpoints[1], data, len);
}


void hid_destroy(void)
{
	usbclient_destroy();
}


int hid_init(const hid_dev_setup_t* dev_setup)
{
	int res;

	usbclient_ep_list_t endpoints = {
		.size = 2,
		.endpoints = {
			{ .id = 0, .type = USBCLIENT_ENDPT_TYPE_CONTROL, .direction = 0                     /* for control endpoint it's ignored */},
			{ .id = 1, .type = USBCLIENT_ENDPT_TYPE_INTR, .direction = USBCLIENT_ENDPT_DIR_IN } /* IN interrupt endpoint required for HID */
		}
	};

	hid_common.dev.size = 1;
	hid_common.dev.descriptors = (usbclient_desc_gen_t *)&dev_setup->ddev;
	hid_common.dev.next = &hid_common.conf;

	hid_common.conf.size = 1;
	hid_common.conf.descriptors = (usbclient_desc_gen_t *)&dconfig;
	hid_common.conf.next = &hid_common.iface;

	hid_common.iface.size = 1;
	hid_common.iface.descriptors = (usbclient_desc_gen_t *)&diface;
	hid_common.iface.next = &hid_common.hid;

	hid_common.hid.size = 1;
	hid_common.hid.descriptors = (usbclient_desc_gen_t *)&dhid;
	hid_common.hid.next = &hid_common.ep;

	hid_common.ep.size = 1;
	hid_common.ep.descriptors = (usbclient_desc_gen_t *)&dep;
	hid_common.ep.next = &hid_common.str0;

	hid_common.str0.size = 1;
	hid_common.str0.descriptors = (usbclient_desc_gen_t *)&dstr0;
	hid_common.str0.next = &hid_common.strman;

	hid_common.strman.size = 1;
	hid_common.strman.descriptors = (usbclient_desc_gen_t *)&dev_setup->dstrman;
	hid_common.strman.next = &hid_common.strprod;

	hid_common.strprod.size = 1;
	hid_common.strprod.descriptors = (usbclient_desc_gen_t *)&dev_setup->dstrprod;
	hid_common.strprod.next = &hid_common.hidreport;

	hid_common.hidreport.size = 1;
	hid_common.hidreport.descriptors = (usbclient_desc_gen_t *)&dhidreport;
	hid_common.hidreport.next = NULL;

	hid_common.config.endpoint_list = endpoints;
	hid_common.config.descriptors_head = &hid_common.dev;

	if ((res = usbclient_init(&hid_common.config)) != EOK)
		return res;

	return EOK;
}
