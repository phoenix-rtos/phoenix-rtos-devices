/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
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
	USBCLIENT_DESC_TYPE_HID = 0x21		/* HID */
} usbclient_descriptor_type_t;

/* Device descriptor */
typedef struct _usbclient_descriptor_device_t {
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
} __attribute__((packed)) usbclient_descriptor_device_t;

/* Configuration descriptor */
typedef struct _usbclient_descriptor_configuration_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint16_t total_len;		/* total bytes returned for this configuration */
	uint8_t	num_intf;		/* number of interfaces supported */
	uint8_t	conf_val;		/* value to use for SET_CONFIGURATION request */
	uint8_t	conf_str;		/* configuration string index */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint8_t	max_pow;		/* maximum power consumption */
} __attribute__((packed)) usbclient_descriptor_configuration_t;

/* Interface descriptor */
typedef struct _usbclient_descriptor_interface_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	intf_num;		/* number of this interface */
	uint8_t	alt_set;		/* value for the alternate setting */
	uint8_t	num_endpt;		/* number of endpoints */
	uint8_t	intf_class;		/* interface class code */
	uint8_t	intf_subclass;	/* interface subclass code */
	uint8_t	intf_prot;		/* interface protocol code */
	uint8_t	intf_str;       /* interface string index */
} __attribute__((packed)) usbclient_descriptor_interface_t;

/* Endpoint descriptor */
typedef struct _usbclient_descriptor_endpoint_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	endpt_addr;		/* endpoint address */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint16_t max_pkt_sz;	/* maximum packet size */
	uint8_t	interval;		/* polling interval for data transfers */
} __attribute__((packed)) usbclient_descriptor_endpoint_t;

/* Generic descriptor
 * Used when there is no defined descriptor (e.g. HID descriptor or Report descriptor) */
typedef struct _usbclient_descriptor_generic_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	data[0];	/* other fields */
} __attribute__((packed)) usbclient_descriptor_generic_t;

/* Descriptor list of arrays */
typedef struct _usbclient_descriptor_list_t {
	uint32_t size;									/* array size */
	usbclient_descriptor_generic_t* descriptors;	/* array containing all descriptors for given type*/
	struct _usbclient_descriptor_list_t* next;		/* pointer to next descriptor type */
} usbclient_descriptor_list_t;

/* Endpoints */
/* Endpoint types */
typedef enum {
	USBCLIENT_ENDPT_TYPE_CONTROL = 0,
	USBCLIENT_ENDPT_TYPE_ISO,
	USBCLIENT_ENDPT_TYPE_BULK,
	USBCLIENT_ENDPT_TYPE_INTR
} usbclient_endpoint_type_t;

/* Endpoint direction */
typedef enum {
	USBCLIENT_ENDPT_DIR_OUT,
	USBCLIENT_ENDPT_DIR_IN
} usbclient_endpoint_direction_t;

/* Endpoint configuration */
typedef struct _usbclient_endpoint_t {
	usbclient_endpoint_type_t type;
	usbclient_endpoint_direction_t direction; /* ignored for control endpoint */
} usbclient_endpoint_t;

/* High speed device can have 15 IN and 15 OUT endpoints
 * in addition to control endpoint (mandatory, both in and out) */
#define USBCLIENT_MAX_ENDPOINTS 31

/* Endpoints configuration list */
typedef struct _usbclient_endpoints_list_t {
	uint32_t size;
	usbclient_endpoint_t endpoints[USBCLIENT_MAX_ENDPOINTS];
} usbclient_endpoints_list_t;

/* Library configuration structure */
typedef struct _usbclient_config_t {
	usbclient_endpoints_list_t endpoint_list;
	usbclient_descriptor_list_t* descriptors_head;
} usbclient_config_t;

/* Initialize library with given configuration */
extern int32_t usbclient_init(usbclient_config_t* config);
/* Cleanup data */
extern void usbclient_destroy(void);

/* Send data on given endpoint - blocking */
extern int32_t usbclient_send_data(usbclient_endpoint_t* endpoint, const uint8_t* data, uint32_t len);
/* Receive data from given endpoint - blocking */
extern int32_t usbclient_receive_data(usbclient_endpoint_t* endpoint, uint8_t* data, uint32_t len);

#endif /* _USBCLIENT_H_ */
