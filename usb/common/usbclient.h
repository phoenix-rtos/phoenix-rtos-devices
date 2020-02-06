/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019, 2020 Phoenix Systems
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


/* Library configuration structure */
typedef struct _usb_desc_list_t {
	struct usb_desc_list_t *next, *prev;
	usb_functional_desc_t *descriptor;
} usb_desc_list_t;


/* Initialize library with given configuration */
extern int usbclient_init(usb_desc_list_t *desList);


/* Cleanup data */
extern int usbclient_destroy(void);


/* Send data on given endpoint - blocking */
extern int usbclient_send(int endpt, const void *data, unsigned int len);


/* Receive data from given endpoint - blocking */
extern int usbclient_receive(int endpt, void *data, unsigned int len);


#endif /* _USBCLIENT_H_ */
