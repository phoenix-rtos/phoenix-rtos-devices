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

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h> /* to set mode for /init */
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/debug.h>
#include <sys/interrupt.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "usbclient.h"

#define USB_ADDR 0x02184000
#define USB_SIZE 0x1000
#define PAGE_SIZE 4096

#define MAX_SPRINT_BUF 1024
volatile uint8_t sprint[MAX_SPRINT_BUF];
volatile uint8_t *sprint_buf = &sprint;

#define int_printf(...) \
do { \
	sprint_buf[0] = 1; \
	sprintf(sprint_buf + 1, __VA_ARGS__); \
} while(0);

#define print_msg() \
do { \
	if (sprint_buf[0]) { \
		printf("%s", sprint_buf + 1); \
		sprint_buf[0] = 0; \
	} \
} while(0);

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

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
	CLASS_REQ_SET_PROTOCOL
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
} usb_dc_t;

usb_dc_t dc = { 0 };

/* Physical addresses for USB controller */
addr_t pdev;
addr_t pconf;

addr_t pstr_0;
addr_t pstr_man;
addr_t pstr_prod;

addr_t phid_reports;

addr_t pIN;
addr_t pOUT;
u8 *IN;
u8 *OUT;

//#define BUFFER_SIZE (1025 + 64)
#define BUFFER_SIZE (0x1000)
typedef struct {
	uint32_t length;
	void *data;
} buffer_t;

static buffer_t read_buffer;
addr_t pread_buffer;
static buffer_t write_buffer;
addr_t pwrite_buffer;

static endpt_init_t in_endpt;
static int dtd_exec(int endpt, u32 paddr, u32 sz, int dir);
int endpt_init(int endpt, endpt_init_t *endpt_init);


static int dc_setup(setup_packet_t *setup)
{
	if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_STANDARD) {
		return EOK;
	}

	int res = EOK;
	u32 fsz;

	switch (setup->req_code) {
		case REQ_SET_ADDR:
			if (setup->val) {
				dc.status = DC_ADDRESS;
				dc.dev_addr = setup->val << 25;
				dc.dev_addr |= 1 << 24;
				*(dc.base + deviceaddr) = dc.dev_addr;
				dc.op = DC_OP_INIT;
				dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			} else if (dc.status != DC_CONFIGURED)
				dc.status = DC_DEFAULT;
			break;

		case REQ_SET_CONFIG:
			if (dc.status == DC_ADDRESS) {
				dc.status = DC_CONFIGURED;
				dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			}
			break;

		case REQ_GET_DESC:
			if (setup->val >> 8 == USBCLIENT_DESC_TYPE_DEV)
				dtd_exec(0, pdev, sizeof(usbclient_desc_dev_t), USBCLIENT_ENDPT_DIR_IN);
			else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_CFG)
				dtd_exec(0, pconf, setup->len, USBCLIENT_ENDPT_DIR_IN);
			else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_STR) {

				if ((setup->val & 0xff) == 0) {
					dtd_exec(0, pstr_0, MIN(sizeof(usbclient_desc_str_zr_t), setup->len), USBCLIENT_ENDPT_DIR_IN);
				} else if ((setup->val & 0xff) == 1) {
					dtd_exec(0, pstr_man, MIN(56, setup->len), USBCLIENT_ENDPT_DIR_IN);
				} else if ((setup->val & 0xff) == 2) {
					dtd_exec(0, pstr_prod, MIN(28, setup->len), USBCLIENT_ENDPT_DIR_IN);
				}
			} else if (setup->val >> 8 == USBCLIENT_DESC_TYPE_HID_REPORT) {
				dtd_exec(0, phid_reports, 76, USBCLIENT_ENDPT_DIR_IN);
			}
			dtd_exec(0, pOUT, 0x40, USBCLIENT_ENDPT_DIR_OUT);
			break;

		case REQ_CLR_FEAT:
		case REQ_GET_STS:
		case REQ_GET_INTF:
		case REQ_SET_INTF:
		case REQ_SET_FEAT:
		case REQ_SET_DESC:
		case REQ_SYNCH_FRAME:
			break;

		case REQ_GET_CONFIG:
			if (setup->val != 0 || setup->idx != 0 || setup->len != 1)
				return res;
			if (dc.status != DC_CONFIGURED)
				OUT[0] = 0;
			else
				OUT[1] = 1;

			dtd_exec(0, pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
			break;

		default:
			if (*(u32 *)setup == 0xdeadc0de)
				dc.op = DC_OP_EXIT;
			else {
				fsz = setup->val << 16;
				fsz |= setup->idx;
				dtd_exec(0, pOUT, setup->len, USBCLIENT_ENDPT_DIR_OUT);
				dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
				//strcpy(dc.mods[dc.mods_cnt].name, (const char *)OUT);
				//dc.mods[dc.mods_cnt].size = fsz;
				OUT[0] = 0;
				dtd_exec(1, pOUT, 0x80, USBCLIENT_ENDPT_DIR_OUT);
				//strcpy(dc.mods[dc.mods_cnt].args, (const char *)OUT);
				dc.op = DC_OP_RECEIVE;
				//dc.mods_cnt++;
			}
			break;
	}

	return res;
}

static int dc_class_setup(setup_packet_t *setup)
{
	if (EXTRACT_REQ_TYPE(setup->req_type) != REQ_TYPE_CLASS) {
		return EOK;
	}

	int res = EOK;
	u32 fsz;

	switch (setup->req_code) {
		case CLASS_REQ_SET_IDLE:
			dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN);
			break;
		case CLASS_REQ_SET_REPORT:
			dtd_exec(0, pOUT, 64 + setup->len, USBCLIENT_ENDPT_DIR_OUT); /* read data to buffer with URB struct*/
			read_buffer.length = setup->len;
			dc.op = DC_OP_RECEIVE; /* mark that data is ready */
			break;
		case CLASS_REQ_GET_IDLE:
		case CLASS_REQ_GET_PROTOCOL:
		case CLASS_REQ_GET_REPORT:
		case CLASS_REQ_SET_PROTOCOL:
		default:
			break;
	}

	return res;
}

/* high frequency interrupts */
static int dc_hf_intr(void)
{
	setup_packet_t setup;
	u32 status;
	int endpt = 0;

	if ((status = *(dc.base + endptsetupstat)) & 0x1) {
		/* trip wire set */
		while (!((status >> endpt) & 1))
			endpt++;
		do {
			*(dc.base + usbcmd) |= 1 << 13;
			memcpy(&setup, dc.endptqh[endpt].setup_buff, sizeof(setup_packet_t));
		} while (!(*(dc.base + usbcmd) & 1 << 13));

		*(dc.base + endptsetupstat) |= 1 << endpt;
		*(dc.base + usbcmd) &= ~(1 << 13);
		*(dc.base + endptflush) |= 0xffffffff;
		*(dc.base + usbsts) |= 1;

		dc.endptqh[0].head = dc.endptqh[0].tail;
		dc.endptqh[1].head = dc.endptqh[1].tail;
		dc.endptqh[2].head = dc.endptqh[2].tail;

		while (*(dc.base + endptsetupstat) & 1);

		dc_setup(&setup);
		dc_class_setup(&setup);
	}

	return 1;
}


/* low frequency interrupts */
static int dc_lf_intr(void)
{
	if ((*(dc.base + usbsts) & 1 << 6)) {

		*(dc.base + endptsetupstat) = *(dc.base + endptsetupstat);
		*(dc.base + endptcomplete) = *(dc.base + endptcomplete);

		while (*(dc.base + endptprime));

		*(dc.base + endptflush) = 0xffffffff;

		while(*(dc.base + portsc1) & 1 << 8);

		*(dc.base + usbsts) |= 1 << 6;
		dc.status = DC_DEFAULT;
	}

	return 1;
}


static int dc_intr(unsigned int intr, void *data)
{

	dc_hf_intr();
	dc_lf_intr();

	return 0;
}


static int ctrlqh_init(void)
{
	u32 qh_addr;

	/* map queue head list */
	dc.endptqh = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

	if (dc.endptqh == MAP_FAILED)
		return -ENOMEM;

	memset((void *)dc.endptqh, 0, 0x1000);

	qh_addr = ((u32)va2pa((void *)dc.endptqh)) & ~0xfff;

	dc.endptqh[0].caps =  0x40 << 16; /* max 64 bytes */
	dc.endptqh[0].caps |= 0x1 << 29;
	dc.endptqh[0].caps |=  0x1 << 15; /* ios */
	dc.endptqh[0].dtd_next = 0x1; /* invalid */

	dc.endptqh[1].caps =  0x40 << 16;
	dc.endptqh[1].caps |= 0x1 << 29;
	dc.endptqh[1].caps |=  0x1 << 15;
	dc.endptqh[1].dtd_next = 1;

	dc.endptqh[0].base = (((u32)va2pa(dc.endptqh)) & ~0xfff) + (32 * sizeof(dqh_t));
	dc.endptqh[0].size = 0x10;
	dc.endptqh[0].head = (dtd_t *)(dc.endptqh + 32);
	dc.endptqh[0].tail = (dtd_t *)(dc.endptqh + 32);

	dc.endptqh[1].base = (((u32)va2pa(dc.endptqh)) & ~0xfff) + (48 * sizeof(dqh_t));
	dc.endptqh[1].size = 0x10;
	dc.endptqh[1].head = (dtd_t *)(dc.endptqh + 48);
	dc.endptqh[1].tail = (dtd_t *)(dc.endptqh + 48);

	*(dc.base + endpointlistaddr) = qh_addr;
	*(dc.base + endptprime) |= 1;
	*(dc.base + endptprime) |= 1 << 16;

	return EOK;
}


static int dtd_init(int endpt)
{
	dtd_t *buff;
	int qh = endpt * 2;

	if (!endpt)
		return -EINVAL;

	buff = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

	if (buff == MAP_FAILED)
		return -ENOMEM;

	memset(buff, 0, 0x1000);

	dc.endptqh[qh].base = (((u32)va2pa(buff)) & ~0xfff);
	dc.endptqh[qh].size = 0x40;
	dc.endptqh[qh].head = buff;
	dc.endptqh[qh].tail = buff;
	dc.endptqh[++qh].base = (((u32)va2pa(buff)) & ~0xfff) + (64 * sizeof(dqh_t));
	dc.endptqh[++qh].size = 0x40;
	dc.endptqh[++qh].head = buff + 64;
	dc.endptqh[++qh].tail = buff + 64;

	return EOK;
}


static dtd_t *dtd_get(int endpt, int dir)
{
	int qh = endpt * 2 + dir;
	u32 base_addr;
	dtd_t *ret;

	base_addr = ((u32)dc.endptqh[qh].head & ~((dc.endptqh[qh].size * sizeof(dtd_t)) - 1));

	ret = dc.endptqh[qh].tail++;
	dc.endptqh[qh].tail = (dtd_t *)(base_addr | ((u32)dc.endptqh[qh].tail & (((dc.endptqh[qh].size) * sizeof(dtd_t)) - 1)));

	return ret;
}


static int dtd_build(dtd_t *dtd, u32 paddr, u32 size)
{
	if (size > 0x1000)
		return -EINVAL;

	dtd->dtd_next = 1;
	if (size)
		dtd->dtd_token = size << 16;
	else
		dtd->dtd_token = 0;
	dtd->dtd_token |= 1 << 7;

	dtd->buff_ptr[0] = paddr;

	return EOK;
}


static int dtd_exec_chain(int endpt, void *vaddr, int sz, int dir)
{
	printf("dtd_exec_chain: start\n");
	u32 qh, offs, shift;
	int i, dcnt, dtdn;
	dtd_t *prev, *current, *first;
	int size = sz;

again:
	prev = NULL;
	dcnt = 1;
	dtdn = 0;

	first = dtd_get(endpt, dir);
	current = first;

	while (sz > 0 && dtdn < 64) {

		if (prev != NULL) {
			dcnt++;
			current = dtd_get(endpt, dir);
			prev->dtd_next = ((va2pa(current)) & ~0xfff) + ((u32)current & 0xffe);
		}

		memset(current, 0, sizeof(dtd_t));
		current->dtd_token = (sz < 0x4000 ? sz : 0x4000) << 16;
		current->dtd_token |= 1 << 7;

		i = 0;

		while (i < 4 && sz > 0) {
			current->buff_ptr[i] = ((va2pa((void *)vaddr)) & ~0xfff) + ((addr_t)vaddr & 0xfff);
			vaddr = (void *)((addr_t)vaddr & ~0xfff) + 0x1000;
			i++;
			sz -= 0x1000;
		}

		prev = current;
		dtdn++;
	}

	current->dtd_next = 1;

	qh = (endpt << 1)  + dir;
	offs = (u32)first & (((dc.endptqh[qh].size) * sizeof(dtd_t)) - 1);
	shift = endpt + ((qh & 1) ? 16 : 0);

	dc.endptqh[qh].dtd_next = (dc.endptqh[qh].base + offs) & ~1;
	dc.endptqh[qh].dtd_token &= ~(1 << 6);
	dc.endptqh[qh].dtd_token &= ~(1 << 7);

	/* prime the endpoint and wait for it to prime */
	while ((*(dc.base + endptprime) & (1 << shift)));
	*(dc.base + endptprime) |= 1 << shift;
	while (!(*(dc.base + endptprime) & (1 << shift)) && (*(dc.base + endptstat) & (1 << shift)));

	while ((current->dtd_token >> 7) & 1) usleep(10000);
	dc.endptqh[qh].head = dc.endptqh[qh].tail;

	if (sz > 0)
		goto again;

	return size;
}


static int dtd_exec(int endpt, u32 paddr, u32 sz, int dir)
{
	int shift;
	u32 offs;
	dtd_t *dtd;
	int qh = (endpt << 1) + dir;

	dtd = dtd_get(endpt, dir);

	dtd_build(dtd, paddr, sz);

	shift = endpt + ((qh & 1) ? 16 : 0);
	offs = (u32)dtd & (((dc.endptqh[qh].size) * sizeof(dtd_t)) - 1);

	dc.endptqh[qh].dtd_next = (dc.endptqh[qh].base + offs) & ~1;
	dc.endptqh[qh].dtd_token &= ~(1 << 6);
	dc.endptqh[qh].dtd_token &= ~(1 << 7);

	/* prime the endpoint and wait for it to prime */
	while ((*(dc.base + endptprime) & (1 << shift)));
	*(dc.base + endptprime) |= 1 << shift;
	while (!(*(dc.base + endptprime) & (1 << shift)) && (*(dc.base + endptstat) & (1 << shift)));

	while (!(*(dc.base + endptcomplete) & (1 << shift)));
	*(dc.base + endptcomplete) |= 1 << shift;
	while (*(dc.base + usbsts) & 1);
	*(dc.base + usbsts) |= 1;
	dc.endptqh[qh].head += 1;

	return EOK;
}


int endpt_init(int endpt, endpt_init_t *endpt_init)
{
	u32 setup = 0;
	int res;
	int qh_rx = endpt * 2 + USBCLIENT_ENDPT_DIR_OUT;
	int qh_tx = endpt * 2 + USBCLIENT_ENDPT_DIR_IN;

	if (endpt == 0)
		return -EINVAL;

	if ((res = dtd_init(endpt)) != EOK)
		return res;

	dc.endptqh[qh_rx].caps =  endpt_init->rx_caps.max_pkt_len << 16;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.ios << 15;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.zlt << 29;
	dc.endptqh[qh_rx].caps |= endpt_init->rx_caps.mult << 30;
	dc.endptqh[qh_rx].dtd_next = 1;

	dc.endptqh[qh_tx].caps =  endpt_init->tx_caps.max_pkt_len << 16;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.ios << 15;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.zlt << 29;
	dc.endptqh[qh_tx].caps |= endpt_init->tx_caps.mult << 30;
	dc.endptqh[qh_tx].dtd_next = 1;

	setup |= endpt_init->rx_ctrl.type << 2;
	setup |= endpt_init->tx_ctrl.type << 18;
	setup |= endpt_init->rx_ctrl.data_toggle << 6;
	setup |= endpt_init->tx_ctrl.data_toggle << 22;

	*(dc.base + endptctrl0 + endpt) = setup;
	*(dc.base + endptctrl0 + endpt) |= 1 << 7;
	*(dc.base + endptctrl0 + endpt) |= 1 << 23;

	return EOK;
}


static void init_desc(usbclient_conf_t *config, void *local_conf)
{
	usbclient_desc_dev_t *dev;
	usbclient_desc_conf_t *cfg;
	usbclient_desc_intf_t *intf;
	usbclient_desc_gen_t *hid;
	usbclient_desc_ep_t *endpt;

	usbclient_desc_str_zr_t *str_0;
	usbclient_desc_gen_t *str_man;
	usbclient_desc_gen_t *str_prod;

	usbclient_desc_gen_t *hid_reports;

	memset(local_conf, 0, 0x1000);

	/* Virtual addresses offsets */
	dev = local_conf;
	cfg = (usbclient_desc_conf_t*)(dev + 1);
	intf = (usbclient_desc_intf_t*)(cfg + 1);
	hid = (usbclient_desc_gen_t*)(intf + 1);
	endpt = (usbclient_desc_ep_t*)(((uint8_t*)hid) + 9);
	str_0 = (usbclient_desc_str_zr_t*)(endpt + 1);
	str_man = (usbclient_desc_gen_t*)(str_0 + 1);
	str_prod = (usbclient_desc_gen_t*)(((uint8_t*)str_man) + 56);
	hid_reports = (usbclient_desc_gen_t*)(((uint8_t*)str_prod) + 28);

	/* Physical addresses offsets */
	pdev = (((u32)va2pa(dev)) & ~0xfff) + ((u32)dev & 0xfff);
	pconf = (((u32)va2pa(cfg)) & ~0xfff) + ((u32)cfg & 0xfff);

	pstr_0 = (((u32)va2pa(str_0)) & ~0xfff) + ((u32)str_0 & 0xfff);
	pstr_man = (((u32)va2pa(str_man)) & ~0xfff) + ((u32)str_man & 0xfff);
	pstr_prod = (((u32)va2pa(str_prod)) & ~0xfff) + ((u32)str_prod & 0xfff);
	phid_reports = (((u32)va2pa(hid_reports)) & ~0xfff) + ((u32)hid_reports & 0xfff);

	/* Endpoints */
	IN = local_conf + 0x500;
	OUT = local_conf + 0x700;

	pIN = ((va2pa((void *)IN)) & ~0xfff) + ((u32)IN & 0xfff);
	pOUT = ((va2pa((void *)OUT)) & ~0xfff) + ((u32)OUT & 0xfff);

	uint32_t string_desc_count = 0;
	/* Extract mandatory descriptors to mapped memory */
	usbclient_desc_list_t* it = config->descriptors_head;
	for (; it != NULL; it = it->next) {
		/* TODO: consider more than one descriptor in array */
		switch(it->descriptors->desc_type) {
			case USBCLIENT_DESC_TYPE_DEV:
				memcpy(dev, &it->descriptors[0], sizeof(usbclient_desc_dev_t));
				break;
			case USBCLIENT_DESC_TYPE_CFG:
				memcpy(cfg, &it->descriptors[0], sizeof(usbclient_desc_conf_t));
				break;
			case USBCLIENT_DESC_TYPE_INTF:
				memcpy(intf, &it->descriptors[0], sizeof(usbclient_desc_intf_t));
				break;
			case USBCLIENT_DESC_TYPE_ENDPT:
				memcpy(endpt, &it->descriptors[0], sizeof(usbclient_desc_ep_t));
				/* Initialize endpoint */
				/* For now hardcode only one endpoint */
				in_endpt.rx_caps.mult = 0;
				in_endpt.rx_caps.zlt = 1;
				in_endpt.rx_caps.max_pkt_len = endpt->max_pkt_sz;
				in_endpt.rx_caps.ios = 0;

				in_endpt.rx_ctrl.type = USBCLIENT_ENDPT_TYPE_INTR; /* TODO: hardcoded value, extract form  descriptor */
				in_endpt.rx_ctrl.data_toggle = 1;
				in_endpt.rx_ctrl.data_inhibit = 0;
				in_endpt.rx_ctrl.stall = 0;

				in_endpt.tx_caps.mult = 0;
				in_endpt.tx_caps.zlt = 1;
				in_endpt.tx_caps.max_pkt_len = endpt->max_pkt_sz;
				in_endpt.tx_caps.ios = 0;

				in_endpt.tx_ctrl.type = USBCLIENT_ENDPT_TYPE_INTR; /* TODO: hardcoded value, extract form  descriptor */;
				in_endpt.tx_ctrl.data_toggle = 1;
				in_endpt.tx_ctrl.data_inhibit = 0;
				in_endpt.tx_ctrl.stall = 0;
				break;
			case USBCLIENT_DESC_TYPE_HID:
				memcpy(hid, &it->descriptors[0], 9);
				break;
			case USBCLIENT_DESC_TYPE_HID_REPORT:
				/* Copy only data section, because HID report descriptor is sent raw */
				memcpy(hid_reports, &it->descriptors[0].data, it->descriptors[0].len - 2);
				break;
			case USBCLIENT_DESC_TYPE_STR:
				if (string_desc_count == 0) {
					memcpy(str_0, &it->descriptors[0], sizeof(usbclient_desc_str_zr_t));
				} else if (string_desc_count == 1) {
					memcpy(str_man, &it->descriptors[0], it->descriptors[0].len);
				} else if (string_desc_count == 2) {
					memcpy(str_prod, &it->descriptors[0], it->descriptors[0].len);
				}
				string_desc_count++;
				break;
			case USBCLIENT_DESC_TYPE_DEV_QUAL:
			case USBCLIENT_DESC_TYPE_OTH_SPD_CFG:
			case USBCLIENT_DESC_TYPE_INTF_PWR:
			default:
				/* Not implemented yet */
				break;
		}
	}
}

char __attribute__((aligned(8))) stack[4096];

static void *local_conf;

int usbclient_init(usbclient_conf_t *config)
{
	int res = 0;
	/* Buffers init */
	read_buffer.data = mmap(NULL, BUFFER_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
	pread_buffer = (((u32)va2pa(read_buffer.data)) & ~0xfff) + ((u32)read_buffer.data & 0xfff);
	write_buffer.data = mmap(NULL, BUFFER_SIZE, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
	pwrite_buffer = (((u32)va2pa(write_buffer.data)) & ~0xfff) + ((u32)write_buffer.data & 0xfff);

	local_conf = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

	init_desc(config, local_conf);

	dc.base = mmap(NULL, USB_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, USB_ADDR);

	if (dc.base == MAP_FAILED)
		return -ENOMEM;

	dc.lock = 0;
	dc.cond = 0;
	dc.dev_addr = 0;

	if (mutexCreate(&dc.lock) != EOK)
		return 0;
	if (condCreate(&dc.cond) != EOK)
		return 0;

	interrupt(75, dc_intr, NULL, dc.cond, &dc.inth);

	*(dc.base + endptflush) = 0xffffffff;
	*(dc.base + usbcmd) &= ~1;
	*(dc.base + usbcmd) |= 1 << 1;
	dc.status = DC_POWERED;
	while (*(dc.base + usbcmd) & (1 << 1));

	/* set usb mode to device */
	*(dc.base + usbmode) |= 2;
	/* trip wire mode (setup lockout mode disabled) */
	*(dc.base + usbmode) |= 1 << 3;
	/* map queue heads list and init control endpoint */
	if ((res = ctrlqh_init()) != EOK)
		return res;

	*(dc.base + usbintr) |= 0x57;

	dc.status = DC_ATTACHED;
	*(dc.base + usbcmd) |= 1;

	while (dc.op != DC_OP_EXIT) {
		print_msg();
		if (dc.op == DC_OP_INIT) {
			res = endpt_init(1, &in_endpt); /* hardcode endpoint initialization */
			return res;
		}
	}
	return EOK;
}

int usbclient_destroy(void)
{
	/* stopping device controller */
	*(dc.base + usbintr) = 0;
	*(dc.base + usbcmd) &= ~1;
	munmap(local_conf, 0x1000);
	munmap((void *)dc.base, 0x1000);
	munmap((void *)((u32)dc.endptqh[2].head & ~0xfff), 0x1000);
	munmap((void *)dc.endptqh, 0x1000);
	return 0;
}

int usbclient_send_data(usbclient_ep_t *endpoint, const void *data, uint32_t len)
{
	int32_t result = -1;
	return result;
}

int usbclient_receive_data(usbclient_ep_t *endpoint, void *data, uint32_t len)
{
	int32_t result = -1;
	int modn = 0;
	while (dc.op != DC_OP_EXIT) {
		mutexLock(dc.lock);
		while (dc.op == DC_OP_NONE)
			condWait(dc.cond, dc.lock, 0);
		mutexUnlock(dc.lock);

		if (dc.op == DC_OP_RECEIVE) {
			/* Copy data to buffer */
			/* TODO: take data len into account when copying */
			memcpy(data, (const char *)OUT, read_buffer.length); /* copy data to buffer */
			result = read_buffer.length;
			read_buffer.length = 0;
			dc.op = DC_OP_NONE;
			dtd_exec(0, pIN, 0, USBCLIENT_ENDPT_DIR_IN); /* ACK */
			break;
		}
	}
	return result;
}
