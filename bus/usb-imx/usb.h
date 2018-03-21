/*
 * Phoenix-RTOS
 *
 * Operating system kernel
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
	u32 dtd_next;
	u32 dtd_token;
	u32 buff_ptr[5];
	u8	padding[4];
} __attribute__((packed)) dtd_t;


/* endpoint queue head */
typedef struct _dqh_t {
	u32 caps;
	u32 dtd_current;

	/* overlay area for dtd */
	u32 dtd_next;
	u32 dtd_token;
	u32 buff_ptr[5];

	u32 reserved;

	/* setup packet buffer */
	u32 setup_buff[2];

	/* head and tail for dtd list */
	dtd_t *head;
	dtd_t *tail;
	u32 base;
	u32 size;
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
	u32 size;
	void *data;
	char name[64];
	char args[128];
} mod_t;


#define MOD_MAX 8

typedef struct _usb_dc_t {
	volatile u32 *base;
	dqh_t *endptqh;
	u32	status;
	u32 dev_addr;
	handle_t cond;
	handle_t lock;
	u32 mods_cnt;
	mod_t mods[MOD_MAX];
	u8 op;
} usb_dc_t;


/* directions */
enum {
	DIR_OUT = 0,
	DIR_IN = 1
};


typedef struct _endpt_caps_t {
	u8	mult;
	u8	zlt;
	u16 max_pkt_len;
	u8  ios;
} endpt_caps_t;


typedef struct _endpt_ctrl_t {
	u8 type;
	u8 data_toggle;
	u8 data_inhibit;
	u8 stall;
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
	u8	req_type;		/* reqest type */
	u8	req_code;		/* request code */
	u16 val;			/* value */
	u16 idx;			/* index */
	u16 len;			/* length */
} __attribute__((packed)) setup_packet_t;


/*device descriptor */
typedef struct  _dev_desc_t {
	u8	len;			/* size of descriptor */
	u8	desc_type;		/* descriptor type */
	u16 bcd_usb;		/* usb specification in BCD */
	u8	dev_class;		/* device class code (USB-IF)*/
	u8	dev_subclass;	/* device subclass code (USB-IF)*/
	u8	dev_prot;		/* protocol code  (USB-IF)*/
	u8	max_pkt_sz0;	/* max packet size for endpoint0 */
	u16 vend_id;		/* vendor id (USB-IF) */
	u16 prod_id;		/* product id */
	u16 bcd_dev;		/* device release number in BCD */
	u8	man_str;		/* manufacturer string index */
	u8	prod_str;		/* product string index */
	u8	sn_str;			/* serial number string index */
	u8	num_conf;		/* number of possible configurations */
} __attribute__((packed)) dev_desc_t;


/* device qualifier */
typedef struct _dev_qual_desc_t {
	u8	len;
	u8	desc_type;
	u16	bcd_usb;
	u8	dev_class;
	u8	dev_subclass;
	u8	dev_prot;
	u8	max_pkt_sz0;
	u8	num_conf;
} __attribute__((packed)) dev_qual_desc_t;


/* configuration */
typedef struct _conf_desc_t {
	u8	len;
	u8	desc_type;
	u16 total_len;		/* total bytes returned for this configuration */
	u8	num_intf;		/* number of interfaces supported */
	u8	conf_val;		/* value to use for SET_CONFIGURATION request */
	u8	conf_str;		/* configuration string index */
	u8	attr_bmp;		/* attributes bitmap */
	u8	max_pow;		/* maximum power consumption */
} __attribute__((packed)) conf_desc_t;


/* other speed configuration */
//typedef conf_desc_t oth_speed_conf_t;


/* interface */
typedef struct _intf_desc_t {
	u8	len;
	u8	desc_type;
	u8	intf_num;		/* number of this interface */
	u8	alt_set;		/* value for the alternate setting */
	u8	num_endpt;		/* number of endpoints */
	u8	intf_class;		/* interface class code */
	u8	intf_subclass;	/* interface subclass code */
	u8	intf_prot;		/* interface protocol code */
	u8	intf_str;       /* interface string index */
} __attribute__((packed)) intf_desc_t;


/* endpoint */
typedef struct _endpt_desc_t {
	u8	len;
	u8	desc_type;
	u8	endpt_addr;		/* endpoint address */
	u8	attr_bmp;		/* attributes bitmap */
	u16 max_pkt_sz;		/* maximum packet size */
	u8	interval;		/* polling interval for data transfers */
} __attribute__((packed)) endpt_desc_t;

#endif /* _USB_H_ */
