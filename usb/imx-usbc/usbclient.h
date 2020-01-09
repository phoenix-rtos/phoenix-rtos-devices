/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019 Phoenix Systems
 * Author: Kamil Amanowicz, Bartosz Ciesla, Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _USBCLIENT_H_
#define _USBCLIENT_H_

#include <stdint.h>

#include <usb.h>

/* descriptor list of arrays */
typedef struct usb_desc_list {
	uint32_t size;								/* array size */
	usb_functional_desc_t *descriptors;	        /* array containing all descriptors for given type*/
	struct usb_desc_list *next;		        /* pointer to next descriptor type */
} usb_desc_list_t;


/* endpoint configuration */
typedef struct usb_endpoint_conf {
	uint8_t id;
	uint8_t type;
	uint8_t direction; /* ignored for control endpoint */
} usb_endpoint_conf_t;


/* endpoints configuration list */
typedef struct usb_endpoint_list {
	uint32_t size;
	usb_endpoint_conf_t endpoints[USB_MAX_ENDPOINTS];
} usb_endpoint_list_t;


/* library configuration structure */
typedef struct usb_conf_t {
	usb_endpoint_list_t endpoint_list;
	usb_desc_list_t *descriptors_head;
} usb_conf_t;


/* Initialize library with given configuration */
extern int usbclient_init(usb_conf_t *conf);


/* Cleanup data */
extern int usbclient_destroy(void);


/* Send data on given endpoint - blocking */
extern int usbclient_send(usb_endpoint_conf_t *ep, const void *data, unsigned int len);


/* Receive data from given endpoint - blocking */
extern int usbclient_receive(usb_endpoint_conf_t *ep, void *data, unsigned int len);


#endif /* _USBCLIENT_H_ */
