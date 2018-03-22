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

#include <stdio.h>
#include <stdlib.h>
#include <sys/threads.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/debug.h>
#include <sys/interrupt.h>
#include <errno.h>
#include <string.h>
#include <unistd.h>

#include "usb.h"

#define USB_ADDR 0x02184000
#define USB_SIZE 0x1000
#define PAGE_SIZE 4096


static struct _usb_dc_t dc = { 0 };

u32 pdev;
u32 pconf;

u32 pIN;
u32 pOUT;
u8 *IN;
u8 *OUT;


static int dtd_exec(int endpt, u32 paddr, u32 sz, int dir);


static int dc_setup(setup_packet_t *setup)
{
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
				dtd_exec(0, pIN, 0, DIR_IN);
			} else if (dc.status != DC_CONFIGURED)
				dc.status = DC_DEFAULT;
			break;

		case REQ_SET_CONFIG:
			if (dc.status == DC_ADDRESS) {
				dc.status = DC_CONFIGURED;
				dtd_exec(0, pIN, 0, DIR_IN);
			}
			break;

		case REQ_GET_DESC:
			if (setup->val >> 8 == DESC_DEV)
				dtd_exec(0, pdev, sizeof(dev_desc_t), DIR_IN);
			else
				dtd_exec(0, pconf, setup->len, DIR_IN);
			dtd_exec(0, pOUT, 0x40, DIR_OUT);
			break;

		case REQ_CLR_FEAT:
		case REQ_GET_STS:
		case REQ_GET_CONFIG:
		case REQ_GET_INTF:
		case REQ_SET_INTF:
		case REQ_SET_FEAT:
		case REQ_SET_DESC:
		case REQ_SYNCH_FRAME:
			break;

		default:
			if (*(u32 *)setup == 0xdeadc0de) {
				dc.op = DC_OP_EXIT;
				dtd_exec(0, pIN, 0, DIR_IN);
			} else {
				fsz = setup->val << 16;
				fsz |= setup->idx;
				dtd_exec(0, pOUT, setup->len, DIR_OUT);
				dtd_exec(0, pIN, 0, DIR_IN);
				strcpy(dc.mods[dc.mods_cnt].name, (const char *)OUT);
				dc.mods[dc.mods_cnt].size = fsz;
				OUT[0] = 0;
				dtd_exec(1, pOUT, 0x80, DIR_OUT);
				strcpy(dc.mods[dc.mods_cnt].args, (const char *)OUT);
				dc.op = DC_OP_RECEIVE;
			}
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

	if (dc.endptqh == NULL)
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

	if (buff == NULL)
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


dtd_t *dtd_get(int endpt, int dir)
{
	int qh = endpt * 2 + dir;
	u32 base_addr;
	dtd_t *ret;

	base_addr = ((u32)dc.endptqh[qh].head & ~((dc.endptqh[qh].size * sizeof(dtd_t)) - 1));

	ret = dc.endptqh[qh].tail++;
	dc.endptqh[qh].tail = (dtd_t *)(base_addr | ((u32)dc.endptqh[qh].tail & (((dc.endptqh[qh].size) * sizeof(dtd_t)) - 1)));

	return ret;
}


int dtd_build(dtd_t *dtd, u32 paddr, u32 size)
{
	if (size > 0x1000)
		return -EINVAL;

	memset(dtd, 0, sizeof(dtd_t));

	dtd->dtd_next = 1;
	if (size)
		dtd->dtd_token = size << 16;
	else
		dtd->dtd_token = 0;
	dtd->dtd_token |= 1 << 7;

	dtd->buff_ptr[0] = paddr;

	return EOK;
}


static int dtd_exec_chain(int endpt, u32 vaddr, int sz, int dir)
{
	u32 qh, offs, shift;
	int i, dcnt = 1;
	dtd_t *prev = NULL, *current, *first;

	first = dtd_get(endpt, dir);
	current = first;

	while (sz > 0) {

		if (prev != NULL) {
			dcnt++;
			current = dtd_get(endpt, dir);
			prev->dtd_next = (((u32)va2pa(current)) & ~0xfff) + ((u32)current & 0xffe);
		}

		memset(current, 0, sizeof(dtd_t));
		current->dtd_token = (sz < 0x4000 ? sz : 0x4000) << 16;
		current->dtd_token |= 1 << 7;

		i = 0;

		while (i < 4 && sz > 0) {
			current->buff_ptr[i] = (((u32)va2pa((void *)vaddr)) & ~0xfff) + (vaddr & 0xfff);
			vaddr = (vaddr & ~0xfff) + 0x1000;
			i++;
			sz -= 0x1000;
		}

		prev = current;
	}

	current->dtd_next = 1;

	qh = (endpt << 1)  + dir;
	offs = (u32)first & (((dc.endptqh[qh].size) * sizeof(dtd_t)) - 1);
	shift = endpt + ((qh & 1) ? 16 : 0);

	dc.endptqh[qh].dtd_next = (dc.endptqh[qh].base + offs) & ~1;
	dc.endptqh[qh].dtd_token &= ~(1 << 6);
	dc.endptqh[qh].dtd_token &= ~(1 << 7);

	*(dc.base + endptprime) |= 1 << shift;
	while (!(*(dc.base + endptprime) & (1 << shift)) && (*(dc.base + endptstat) & (1 << shift)));

	while (!(*(dc.base + endptcomplete) & (1 << shift)));
	*(dc.base + endptcomplete) |= 1 << shift;
	while (*(dc.base + usbsts) & 1);
	*(dc.base + usbsts) |= 1;
	dc.endptqh[qh].head = dc.endptqh[qh].tail;
	return 0;
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
	int qh_rx = endpt * 2 + DIR_OUT;
	int qh_tx = endpt * 2 + DIR_IN;

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


static void init_desc(void *conf)
{
	dev_desc_t *dev;
	conf_desc_t *cfg;
	intf_desc_t *intf;
	endpt_desc_t *endpt;

	memset(conf, 0, 0x1000);

	dev = conf;
	cfg = conf + sizeof(dev_desc_t);
	intf = conf + sizeof(conf_desc_t) + sizeof(dev_desc_t);
	endpt = conf + sizeof(conf_desc_t) + sizeof(intf_desc_t) + sizeof(dev_desc_t);

	pdev = (((u32)va2pa(dev)) & ~0xfff) + ((u32)dev & 0xfff);
	pconf = (((u32)va2pa(cfg)) & ~0xfff) + ((u32)cfg & 0xfff);

	IN = conf + 0x500;
	OUT = conf + 0x700;

	pIN = (((u32)va2pa((void *)IN)) & ~0xfff) + ((u32)IN & 0xfff);
	pOUT = (((u32)va2pa((void *)OUT)) & ~0xfff) + ((u32)OUT & 0xfff);

	dev->len = sizeof(dev_desc_t);
	dev->desc_type = DESC_DEV;
	dev->bcd_usb = 0x200;
	dev->dev_class = 0xff;
	dev->dev_subclass = 0xc0;
	dev->dev_prot = 0xde;
	dev->max_pkt_sz0 = 64;
	dev->vend_id = 0x15a2;
	dev->prod_id = 0x007d;
	dev->bcd_dev = 0x0001;
	dev->man_str = 0;
	dev->prod_str = 0;
	dev->sn_str = 0;
	dev->num_conf = 1;

	cfg->len = 9;
	cfg->desc_type = 2;
	cfg->total_len = 25;
	cfg->num_intf = 1;
	cfg->conf_val = 1;
	cfg->conf_str = 0;
	cfg->attr_bmp = 0xC0;
	cfg->max_pow = 10;

	intf->len = 9;
	intf->desc_type = 4;
	intf->intf_num = 0;
	intf->alt_set = 0;
	intf->num_endpt = 1;
	intf->intf_class = 0xff;
	intf->intf_subclass = 0xb1;
	intf->intf_prot = 0x7e;
	intf->intf_str = 0;

	endpt->len = 7;
	endpt->desc_type = 5;
	endpt->endpt_addr = 0x01;
	endpt->attr_bmp = 0x02;
	endpt->max_pkt_sz = 512;
	endpt->interval = 0x06;

}


static void mod_lookup(void *arg)
{
	msg_t msg;
	unsigned int rid;
	int size;
	int mod_no = 0;

	while (1) {

		msgRecv(1, &msg, &rid);

		switch (msg.type) {

			case mtLookup:
				msg.o.lookup.res.id = mod_no;
				msg.o.lookup.res.port = 1;
				break;
			case mtRead:
				if (msg.o.size > dc.mods[mod_no - 1].size - msg.i.io.offs)
					size = dc.mods[mod_no - 1].size - msg.i.io.offs;
				else
					size = msg.o.size;
				memcpy(msg.o.data, dc.mods[mod_no - 1].data + msg.i.io.offs, size);
				msg.o.io.err = EOK;
				break;
			case mtGetAttr:
				msg.o.attr.val = dc.mods[mod_no].size;
				mod_no++;
				break;
			default:
				break;
		}

		msgRespond(1, &msg, rid);
	}

	portDestroy((u32)arg);
	endthread();
}


static void printf_mockup(void *arg)
{
	msg_t msg;
	unsigned int rid;
	int offs = 0;
	int size;
	char buff[16] = { 0 };
	while (1) {
		msgRecv(0, &msg, &rid);
		offs = 0;
		size = msg.i.size;
		if (size > 0) {
			if (size >= 15) {
				memcpy(buff, msg.i.data + offs, 15);
				buff[15] = 0;
			} else {
				memcpy(buff, msg.i.data + offs, size);
				buff[size] = 0;
			}
			debug(buff);
			offs += 15;
			size -= 15;
		}
		msgRespond(0, &msg, rid);
	}
}

char __attribute__((aligned(8))) stack0[2048];
char __attribute__((aligned(8))) stack[2048];

int main(void)
{
	endpt_init_t bulk_endpt;
	int res;

	oid_t oid;
	char path[65];
	char *arg_tok;
	char *argv[16] = { 0 };
	int argc;
	u32 port;
	u32 uart_port;
	int cnt = 0;

	void *conf = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

	init_desc(conf);

	bulk_endpt.rx_caps.mult = 0;
	bulk_endpt.rx_caps.zlt = 1;
	bulk_endpt.rx_caps.max_pkt_len = 512;
	bulk_endpt.rx_caps.ios = 0;

	bulk_endpt.rx_ctrl.type = ENDPT_BULK;
	bulk_endpt.rx_ctrl.data_toggle = 1;
	bulk_endpt.rx_ctrl.data_inhibit = 0;
	bulk_endpt.rx_ctrl.stall = 0;

	bulk_endpt.tx_caps.mult = 0;
	bulk_endpt.tx_caps.zlt = 1;
	bulk_endpt.tx_caps.max_pkt_len = 512;
	bulk_endpt.tx_caps.ios = 0;

	bulk_endpt.tx_ctrl.type = ENDPT_BULK;
	bulk_endpt.tx_ctrl.data_toggle = 1;
	bulk_endpt.tx_ctrl.data_inhibit = 0;
	bulk_endpt.tx_ctrl.stall = 0;

	dc.base = mmap(NULL, USB_SIZE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, USB_ADDR);

	dc.lock = 0;
	dc.cond = 0;
	dc.dev_addr = 0;

	if (dc.base == NULL)
		return -ENOMEM;

	if (mutexCreate(&dc.lock) != EOK)
		return 0;
	if (condCreate(&dc.cond) != EOK)
		return 0;

	interrupt(75, dc_intr, NULL, dc.cond);

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

		mutexLock(dc.lock);
		condWait(dc.cond, dc.lock, 5000);
		mutexUnlock(dc.lock);

		if (dc.op == DC_OP_RECEIVE) {

			if (dc.mods_cnt >= MOD_MAX) {
				//printf("Maximum modules number reached (%d), stopping usb...\n", MOD_MAX);
				break;
			} else
				dc.op = DC_OP_NONE;

			dc.mods[dc.mods_cnt].data = mmap(NULL, (dc.mods[dc.mods_cnt].size + 0xfff) & ~0xfff, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);
			dtd_exec_chain(1, (u32)dc.mods[dc.mods_cnt].data, dc.mods[dc.mods_cnt].size, DIR_OUT);
			dc.mods_cnt++;
		} else if (dc.op == DC_OP_INIT && endpt_init(1, &bulk_endpt) != EOK) {
			dc.op = DC_OP_NONE;
			return 0;
		}
	}

//	if (dc.op == DC_OP_EXIT)
//		printf("Modules transfer done (%d), stopping usb...\n", dc.mods_cnt);

	/* stopping device controller */
	*(dc.base + usbcmd) &= ~1;
	munmap(conf, 0x1000);
	munmap((void *)dc.base, 0x1000);
	munmap((void *)((u32)dc.endptqh[2].head & ~0xfff), 0x1000);
	munmap((void *)dc.endptqh, 0x1000);

	if (portCreate(&uart_port) != EOK)
		return 0;

	if (portRegister(uart_port, "/p", &oid) != EOK)
		return 0;

	if (beginthread(printf_mockup, 4, stack0, sizeof(stack0), (void *)port) != EOK)
		return 0;

	if (portCreate(&port) != EOK)
		return 0;

	if (portRegister(port, "/init", &oid) != EOK)
		return 0;

	if (beginthread(mod_lookup, 4, stack, sizeof(stack), (void *)port) != EOK)
		return 0;

	memcpy(path, "/init/", 6);

//	portDestroy(uart_port);

	while (cnt < dc.mods_cnt) {
		argc = 0;

		arg_tok = strtok(dc.mods[cnt].args, ",");

		while (arg_tok != NULL && argc < 15){
			argv[argc] = arg_tok;
			arg_tok = strtok(NULL, ",");
			argc++;
		}
		argv[argc] = NULL;

		memcpy(&path[6], dc.mods[cnt].name, strlen(dc.mods[cnt].name) + 1);
		if (vfork() == 0) {
			execve(path, argv, NULL);
			//if(execve(path, argv, NULL) != EOK)
			//	printf("Failed to start %s\n", &path[6]);
			//return 0;
		}
		cnt++;
		//usleep(200000);
	}

	//printf("Bye\n");
	while (1) usleep(99999999);
	return EOK;
}
