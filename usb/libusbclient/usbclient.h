/*
 * Phoenix-RTOS
 *
 * usbclient - usb device controller driver
 *
 * Copyright 2019-2021 Phoenix Systems
 * Author: Kamil Amanowicz, Bartosz Ciesla, Hubert Buczynski, Gerard Swiderski
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
	struct _usb_desc_list_t *next, *prev;
	usb_functional_desc_t *descriptor;
} usb_desc_list_t;


enum {
	USBCLIENT_EV_DISCONNECT,
	USBCLIENT_EV_CONNECT,
	USBCLIENT_EV_RESET,
	USBCLIENT_EV_INIT,
	USBCLIENT_EV_CONFIGURED,
	USBCLIENT_EV_FAULT,
};


enum {
	CLASS_SETUP_ACK = 0,       /* Send ACK after return from Class Setup callback */
	CLASS_SETUP_ENDP0 = -1,    /* Wake up ENDP0, and receive data in endpoint     */
	CLASS_SETUP_NOACTION = -2, /* Class Setup default action (ignore request)     */
	CLASS_SETUP_ERROR = -3,
};


/* Initialize library with given configuration */
extern int usbclient_init(usb_desc_list_t *desList);


/* Cleanup data */
extern int usbclient_destroy(void);


/* Send data on given endpoint - blocking */
extern int usbclient_send(int endpt, const void *data, unsigned int len);


/* Receive data from given endpoint - blocking */
extern int usbclient_receive(int endpt, void *data, unsigned int len);


/* Set user context for handlers */
extern void usbclient_setUserContext(void *ctxUser);


/* Set general event callback handler */
extern void usbclient_setEventCallback(void (*cbEvent)(int, void *));


/* Set class setup callback handler */
extern void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *));


#endif /* _USBCLIENT_H_ */
