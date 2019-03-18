/*
 * Phoenix-RTOS
 *
 * dummyfs - usb device controller driver
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _USB_H_
#define _USB_H_

#include <stdint.h>

typedef struct _mod_t mod_t;

extern int init_usb(void);
extern int bulk_endpt_init(void);
extern void destroy_usb(void);

/* host/device cotroller register offsets */
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
	DC_OP_INIT
};

/* endpoint types */
enum {
	ENDPT_CONTROL = 0,
	ENDPT_ISO,
	ENDPT_BULK,
	ENDPT_INTR
};


typedef struct _mod_t {
	uint32_t size;
	void *data;
	char name[64];
	char args[128];
} mod_t;


#define MOD_MAX 8

typedef struct _usb_dc_t {
	volatile uint32_t *base;
	dqh_t *endptqh;
	uint32_t status;
	uint32_t dev_addr;
	handle_t cond;
	handle_t lock;
	handle_t inth;
	uint32_t mods_cnt;
	mod_t mods[MOD_MAX];
	uint8_t op;
} usb_dc_t;

extern usb_dc_t dc;

/* directions */
enum {
	DIR_OUT = 0,
	DIR_IN = 1
};


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

/* usb spec related stuff */

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


/* descriptor types */

enum {
	DESC_DEV = 1,		/* device */
	DESC_CFG,			/* configuration */
	DESC_STR,			/* string */
	DESC_INTF,			/* interface */
	DESC_ENDPT,			/* endpoint */
	DESC_DEV_QUAL,		/* device qualifier */
	DESC_OTH_SPD_CFG,	/* other speed configuration */
	DESC_INTF_PWR		/* interface power */
};


/* setup packet structure */
typedef struct _setup_packet_t {
	uint8_t	req_type;		/* reqest type */
	uint8_t	req_code;		/* request code */
	uint16_t val;			/* value */
	uint16_t idx;			/* index */
	uint16_t len;			/* length */
} __attribute__((packed)) setup_packet_t;


/*device descriptor */
typedef struct  _dev_desc_t {
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
} __attribute__((packed)) dev_desc_t;


/* device qualifier */
typedef struct _dev_qual_desc_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint16_t bcd_usb;
	uint8_t	dev_class;
	uint8_t	dev_subclass;
	uint8_t	dev_prot;
	uint8_t	max_pkt_sz0;
	uint8_t	num_conf;
} __attribute__((packed)) dev_qual_desc_t;


/* configuration */
typedef struct _conf_desc_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint16_t total_len;		/* total bytes returned for this configuration */
	uint8_t	num_intf;		/* number of interfaces supported */
	uint8_t	conf_val;		/* value to use for SET_CONFIGURATION request */
	uint8_t	conf_str;		/* configuration string index */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint8_t	max_pow;		/* maximum power consumption */
} __attribute__((packed)) conf_desc_t;


/* other speed configuration */
//typedef conf_desc_t oth_speed_conf_t;


/* interface */
typedef struct _intf_desc_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	intf_num;		/* number of this interface */
	uint8_t	alt_set;		/* value for the alternate setting */
	uint8_t	num_endpt;		/* number of endpoints */
	uint8_t	intf_class;		/* interface class code */
	uint8_t	intf_subclass;	/* interface subclass code */
	uint8_t	intf_prot;		/* interface protocol code */
	uint8_t	intf_str;       /* interface string index */
} __attribute__((packed)) intf_desc_t;


/* endpoint */
typedef struct _endpt_desc_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t	endpt_addr;		/* endpoint address */
	uint8_t	attr_bmp;		/* attributes bitmap */
	uint16_t max_pkt_sz;		/* maximum packet size */
	uint8_t	interval;		/* polling interval for data transfers */
} __attribute__((packed)) endpt_desc_t;

#endif /* _USB_H_ */
