/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Bartosz Ciesla
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _USBCLIENT_H_
#define _USBCLIENT_H_

#include <stdint.h>

/* Library interface*/

/* Descriptors */
/* Descriptor types */
typedef enum {
	USBCLIENT_DESC_TYPE_DEV = 1,		/* device */
	USBCLIENT_DESC_TYPE_CFG,			/* configuration */
	USBCLIENT_DESC_TYPE_STR,			/* string */
	USBCLIENT_DESC_TYPE_INTF,			/* interface */
	USBCLIENT_DESC_TYPE_ENDPT,			/* endpoint */
	USBCLIENT_DESC_TYPE_DEV_QUAL,		/* device qualifier */
	USBCLIENT_DESC_TYPE_OTH_SPD_CFG,	/* other speed configuration */
	USBCLIENT_DESC_TYPE_INTF_PWR,		/* interface power */
	USBCLIENT_DESC_TYPE_HID = 0x21,		/* HID */
	USBCLIENT_DESC_TYPE_HID_REPORT		/* HID report */
} usbclient_descriptor_type_t;

/* Device descriptor */
typedef struct _usbclient_desc_dev_t {
	uint8_t	len;			/* size of descriptor */
	uint8_t	desc_type;		/* descriptor type */
	uint16_t bcd_usb;		/* usb specification in BCD */
	uint8_t	dev_class;		/* device class code (USB-IF)*/
	uint8_t	dev_subclass;	/* device subclass code (USB-IF)*/
	uint8_t	dev_prot;		/* protocol code  (USB-IF)*/
	uint8_t	max_pkt_sz0;	/* max packet size for endpoint0 */
	uint16_t vend_id;		/* vendor id (USB-IF) */
	uint16_t prod_id;		/* product id */
	uint16_t bcd_dev;		/* device release number in BCD */
	uint8_t	man_str;		/* manufacturer string index */
	uint8_t	prod_str;		/* product string index */
	uint8_t	sn_str;			/* serial number string index */
	uint8_t	num_conf;		/* number of possible configurations */
} __attribute__((packed)) usbclient_desc_dev_t;

/* Configuration descriptor */
typedef struct _usbclient_desc_conf_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint16_t total_len;		/* total bytes returned for this configuration */
	uint8_t	num_intf;		/* number of interfaces supported */
	uint8_t	conf_val;		/* value to use for SET_CONFIGURATION request */
	uint8_t	conf_str;		/* configuration string index */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint8_t	max_pow;		/* maximum power consumption */
} __attribute__((packed)) usbclient_desc_conf_t;

/* Interface descriptor */
typedef struct _usbclient_desc_intf_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	intf_num;		/* number of this interface */
	uint8_t	alt_set;		/* value for the alternate setting */
	uint8_t	num_endpt;		/* number of endpoints */
	uint8_t	intf_class;		/* interface class code */
	uint8_t	intf_subclass;	/* interface subclass code */
	uint8_t	intf_prot;		/* interface protocol code */
	uint8_t	intf_str;       /* interface string index */
} __attribute__((packed)) usbclient_desc_intf_t;

/* Endpoint descriptor */
typedef struct _usbclient_desc_ep_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	endpt_addr;		/* endpoint address */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint16_t max_pkt_sz;	/* maximum packet size */
	uint8_t	interval;		/* polling interval for data transfers */
} __attribute__((packed)) usbclient_desc_ep_t;

/* String descriptor zero */
typedef struct _usbclient_desc_str_zr_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint16_t w_langid0;
} __attribute__((packed)) usbclient_desc_str_zr_t;

/* Generic descriptor
 * Used when there is no defined descriptor (e.g. HID descriptor or Report descriptor) */
typedef struct _usbclient_desc_gen_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	data[0];	/* other fields */
} __attribute__((packed)) usbclient_desc_gen_t;

/* Descriptor list of arrays */
typedef struct _usbclient_desc_list_t {
	uint32_t size;									/* array size */
	usbclient_desc_gen_t* descriptors;	/* array containing all descriptors for given type*/
	struct _usbclient_desc_list_t* next;		/* pointer to next descriptor type */
} usbclient_desc_list_t;

/* Endpoints */
/* Endpoint types */
typedef enum {
	USBCLIENT_ENDPT_TYPE_CONTROL = 0,
	USBCLIENT_ENDPT_TYPE_ISO,
	USBCLIENT_ENDPT_TYPE_BULK,
	USBCLIENT_ENDPT_TYPE_INTR
} usbclient_ep_type_t;

/* Endpoint direction */
typedef enum {
	USBCLIENT_ENDPT_DIR_OUT,
	USBCLIENT_ENDPT_DIR_IN
} usbclient_ep_dir_t;

/* Endpoint configuration */
typedef struct _usbclient_ep_t {
	u8 id;
	usbclient_ep_type_t type;
	usbclient_ep_dir_t direction; /* ignored for control endpoint */
} usbclient_ep_t;

/* High speed device can have 15 IN and 15 OUT endpoints
 * in addition to control endpoint (mandatory, both in and out) */
#define USBCLIENT_MAX_ENDPOINTS 31

/* Endpoints configuration list */
typedef struct _usbclient_ep_list_t {
	uint32_t size;
	usbclient_ep_t endpoints[USBCLIENT_MAX_ENDPOINTS];
} usbclient_ep_list_t;

/* Library configuration structure */
typedef struct _usbclient_conf_t {
	usbclient_ep_list_t endpoint_list;
	usbclient_desc_list_t* descriptors_head;
} usbclient_conf_t;

/* Initialize library with given configuration */
extern int usbclient_init(usbclient_conf_t *conf);
/* Cleanup data */
extern int usbclient_destroy(void);

/* Send data on given endpoint - blocking */
extern int usbclient_send(usbclient_ep_t *ep, const void *data, unsigned int len);
/* Receive data from given endpoint - blocking */
extern int usbclient_receive(usbclient_ep_t *ep, void *data, unsigned int len);

#endif /* _USBCLIENT_H_ */
