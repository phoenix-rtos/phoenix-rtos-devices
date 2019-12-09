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

#ifndef _IMXDEVICE_H_
#define _IMXDEVICE_H_

#define USB_BUFFER_SIZE 0x1000
#define ENDPOINT_NUMBER 15

#include <sys/types.h>
#include "usbclient.h"

/* device controller structures */

/* data transfer descriptor */
typedef struct _dtd_t {
	uint32_t dtd_next;
	uint32_t dtd_token;
	uint32_t buff_ptr[5];
	uint8_t	padding[4];
} __attribute__((packed)) dtd_t;


/* endpoint queue head */
typedef struct _dqh_t {
	uint32_t caps;
	uint32_t dtd_current;

	/* overlay area for dtd */
	uint32_t dtd_next;
	uint32_t dtd_token;
	uint32_t buff_ptr[5];

	uint32_t reserved;

	/* setup packet buffer */
	uint32_t setup_buff[2];

	/* head and tail for dtd list */
	dtd_t *head;
	dtd_t *tail;
	uint32_t base;
	uint32_t size;
} __attribute__((packed)) dqh_t;


/* dcd structures */

/* dc states */
enum {
	DC_POWERED,
	DC_ATTACHED,
	DC_DEFAULT,
	DC_ADDRESS,
	DC_CONFIGURED
};


/* dc operation */
enum {
	DC_OP_NONE,
	DC_OP_RECEIVE,
	DC_OP_EXIT,
	DC_OP_INIT,
	DC_TEMP
};


/* usb spec related stuff */
typedef struct _endpt_caps_t {
	uint8_t  mult;
	uint8_t  zlt;
	uint16_t max_pkt_len;
	uint8_t  ios;
} endpt_caps_t;


typedef struct _endpt_ctrl_t {
	uint8_t type;
	uint8_t data_toggle;
	uint8_t data_inhibit;
	uint8_t stall;
} endpt_ctrl_t;


typedef struct _endpt_init_t {
	endpt_caps_t rx_caps;
	endpt_ctrl_t rx_ctrl;
	endpt_caps_t tx_caps;
	endpt_ctrl_t tx_ctrl;
} endpt_init_t;


/* reqeuest codes */
enum {
	REQ_GET_STS = 0,
	REQ_CLR_FEAT,
	REQ_SET_FEAT = 3,
	REQ_SET_ADDR = 5,
	REQ_GET_DESC,
	REQ_SET_DESC,
	REQ_GET_CONFIG,
	REQ_SET_CONFIG,
	REQ_GET_INTF,
	REQ_SET_INTF,
	REQ_SYNCH_FRAME
};


/* class reqeuest codes */
enum {
	CLASS_REQ_GET_REPORT = 1,
	CLASS_REQ_GET_IDLE,
	CLASS_REQ_GET_PROTOCOL,
	CLASS_REQ_SET_REPORT = 9,
	CLASS_REQ_SET_IDLE,
	CLASS_REQ_SET_PROTOCOL,
	CLASS_REQ_SET_LINE_CODING = 0x20,
	CLASS_REQ_SET_CONTROL_LINE_STATE = 0x22
};


enum {
	REQ_TYPE_STANDARD,
	REQ_TYPE_CLASS,
	REQ_TYPE_VENDOR
};


#define EXTRACT_REQ_TYPE(req_type) (((req_type) >> 5) & 0x3)


/* setup packet structure */
typedef struct _setup_packet_t {
	uint8_t	req_type;		/* reqest type */
	uint8_t	req_code;		/* request code */
	uint16_t val;			/* value */
	uint16_t idx;			/* index */
	uint16_t len;			/* length */
} __attribute__((packed)) setup_packet_t;


typedef struct _usb_dc_t {
	volatile uint32_t *base;
	dqh_t *endptqh;
	uint32_t status;
	uint32_t dev_addr;
	handle_t cond;
	handle_t lock;
	handle_t inth;
	uint8_t op;
	setup_packet_t setup;
} usb_dc_t;


typedef struct {
	uint32_t length;
	void *data;
} buffer_t;


typedef struct{
	buffer_t read_buffer;
	addr_t pread_buffer;

	buffer_t write_buffer;
	addr_t pwrite_buffer;

	void *local_conf;
	endpt_init_t in_endpt[ENDPOINT_NUMBER];
	int endNb;
} usb_common_data_t;


/* device cotroller register offsets */
enum {
	/* identification regs */
	id = 0x0, hwgeneral, hwhost, hwdevice, hwtxbuf, hwrxbuf,

	/* operational regs */
	gptimer0ld	= 0x20, gptimer0ctrl, gptimer1ld, gptimer1ctrl, sbuscfg,

	/* capability regs */
	caplength = 0x40, hciversion = 0x40, hcsparams, hccparams,
	dciversion = 0x48, dccparams,

	/* operational regs cont. */
	usbcmd = 0x50, usbsts, usbintr, frindex,
	periodiclistbase = 0x55, deviceaddr = 0x55, asynclistaddr = 0x56,
	endpointlistaddr = 0x56, burstsize = 0x58, txfilltunning, endptnak = 0x5E,
	endptnaken, configflag, portsc1, otgsc = 0x69, usbmode, endptsetupstat,
	endptprime, endptflush, endptstat, endptcomplete, endptctrl0, endptctrl1,
	endptctrl2, endptctrl3, endptctrl4, endptctrl5, endptctrl6, endptctrl7
};


extern int init_desc(usbclient_conf_t *conf, usb_common_data_t *usb_data_in, usb_dc_t *dc_in);


extern int dc_lf_intr(void);


extern int dc_hf_intr(void);


extern int dc_class_setup(setup_packet_t *setup);


extern int dc_setup(setup_packet_t *setup);


#endif /* _IMXDEVICE_H_ */
