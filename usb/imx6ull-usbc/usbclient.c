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


usb_dc_t dc = { 0 };

addr_t pdev;
addr_t pconf;

addr_t pIN;
addr_t pOUT;
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

			dtd_exec(0, pOUT, setup->len, DIR_OUT);
			break;

		default:
			if (*(u32 *)setup == 0xdeadc0de)
				dc.op = DC_OP_EXIT;
			else {
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
				dc.mods_cnt++;
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

	pIN = ((va2pa((void *)IN)) & ~0xfff) + ((u32)IN & 0xfff);
	pOUT = ((va2pa((void *)OUT)) & ~0xfff) + ((u32)OUT & 0xfff);

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


//extern int dummyfs_create(oid_t *dir, const char *name, oid_t *oid, int type, int mode, oid_t *dev);
//extern int dummyfs_link(oid_t *dir, const char *name, oid_t *oid);
//extern int dummyfs_write(oid_t *oid, offs_t offs, char *buff, unsigned int len);
//extern int dummyfs_lookup(oid_t *dir, const char *name, oid_t *res, oid_t *dev);
//extern int dummyfs_setattr(oid_t *oid, int type, int attr);


char __attribute__((aligned(8))) stack[4096];

void exec_modules(void *arg)
{
	char path[65];
	char *arg_tok;
	char *argv[16] = { 0 };
	int argc;
	int cnt = 0;
	int x;

	oid_t toid = { 0 };
	oid_t root = { 0 };
	oid_t init = { 0 };
	oid_t tmp;

	memcpy(path, "/init/", 6);
	//dummyfs_lookup(NULL, ".", &tmp, &root);
	//dummyfs_create(&root, "init", &init, otDir, 0, NULL);
	//dummyfs_setattr(&init, atMode, S_IFDIR | 0777);

	while (cnt < dc.mods_cnt) {
		argc = 1;

		x = 0;
		if (dc.mods[cnt].name[0] == 'X')
			x++;

		//if (dummyfs_create(&init, dc.mods[cnt].name + 1, &toid, otFile, S_IFREG, NULL) == EOK)
		//	dummyfs_write(&toid, 0, dc.mods[cnt].data, dc.mods[cnt].size);

		if (x) {

			arg_tok = strtok(dc.mods[cnt].args, ",");

			while (arg_tok != NULL && argc < 15){
				argv[argc] = arg_tok;
				arg_tok = strtok(NULL, ",");
				argc++;
			}
			argv[argc] = NULL;

			memcpy(&path[6], dc.mods[cnt].name + 1, strlen(dc.mods[cnt].name));
			argv[0] = path;
			if (vfork() == 0) {
				if(execve(path, argv, NULL) != EOK)
					printf("Failed to start %s\n", &path[6]);
				endthread(); //prevent crash if execve fails
			}
		}

		munmap(dc.mods[cnt].data, (dc.mods[dc.mods_cnt].size + 0xfff) & ~0xfff);
		cnt++;
	}
	endthread();
}


static endpt_init_t bulk_endpt;
static void *conf;

int init_usb(void)
{
    int res = 0;
	conf = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_UNCACHED, OID_NULL, 0);

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

	return EOK;
}

int bulk_endpt_init(void) {
	return endpt_init(1, &bulk_endpt);
}

void destroy_usb(void)
{
	/* stopping device controller */
	*(dc.base + usbintr) = 0;
	*(dc.base + usbcmd) &= ~1;
	munmap(conf, 0x1000);
	munmap((void *)dc.base, 0x1000);
	munmap((void *)((u32)dc.endptqh[2].head & ~0xfff), 0x1000);
	munmap((void *)dc.endptqh, 0x1000);
}
