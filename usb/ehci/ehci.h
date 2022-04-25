/*
 * Phoenix-RTOS
 *
 * USB EHCI host controller
 *
 * Copyright 2021 Phoenix Systems
 * Author: Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _USB_EHCI_H_
#define _USB_EHCI_H_

#define USBSTS_AS    (1 << 15)
#define USBSTS_PS    (1 << 14)
#define USBSTS_RCL   (1 << 13)
#define USBSTS_URI   (1 << 6)
#define USBSTS_SRI   (1 << 7)
#define USBSTS_SLI   (1 << 8)
#define USBSTS_ULPII (1 << 10)
#define USBSTS_HCH   (1 << 12)
#define USBSTS_IAA   (1 << 5)
#define USBSTS_SEI   (1 << 4)
#define USBSTS_FRI   (1 << 3)
#define USBSTS_PCI   (1 << 2)
#define USBSTS_UEI   (1 << 1)
#define USBSTS_UI    (1 << 0)

#define EHCI_INTRMASK (USBSTS_PCI | USBSTS_UEI | USBSTS_UI)

#define USBCMD_ASE (1 << 5)
#define USBCMD_IAA (1 << 6)

#define PORTSC_PTS_1 (3 << 30)
#define PORTSC_STS   (1 << 29)
#define PORTSC_PPTW  (1 << 28)
#define PORTSC_PSPD  (3 << 26)
#define PORTSC_PTS_2 (1 << 25)
#define PORTSC_PFSC  (1 << 24)
#define PORTSC_PHCD  (1 << 23)

#define PORTSC_WKOC     (1 << 22)
#define PORTSC_WKDSCNNT (1 << 21)
#define PORTSC_WKCNT    (1 << 20)
#define PORTSC_PTC      (0xf << 16)
#define PORTSC_PIC      (3 << 14)
#define PORTSC_PO       (1 << 13)
#define PORTSC_PP       (1 << 12)
#define PORTSC_LS       (3 << 10)
#define PORTSC_HSP      (1 << 9)
#define PORTSC_PR       (1 << 8)
#define PORTSC_SUSP     (1 << 7)

#define PORTSC_FPR (1 << 6)
#define PORTSC_OCC (1 << 5)
#define PORTSC_OCA (1 << 4)
#define PORTSC_PEC (1 << 3)

#define PORTSC_ENA (1 << 2)
#define PORTSC_CSC (1 << 1)
#define PORTSC_CCS (1 << 0)

#define PORTSC_PSPD_HS (2 << 26)

#define QTD_DT            (1 << 31)
#define QTD_LEN(token)    (((token) >> 16) & 0x7fff)
#define QTD_IOC           (1 << 15)
#define QTD_CERR(token)   (((token) >> 10) & 0x3)
#define QTD_PID(token)    (((token) >> 8) & 0x3)
#define QTD_ACTIVE        (1 << 7)
#define QTD_HALTED        (1 << 6)
#define QTD_BUFERR        (1 << 5)
#define QTD_BABBLE        (1 << 4)
#define QTD_XACT          (1 << 3)
#define QTD_MISSED_UFRAME (1 << 2)
#define QTD_SPLIT         (1 << 1)
#define QTD_PING          (1 << 0)
#define QTD_OFFSET(buf)   ((buf) & 0xfff)
#define QTD_ERRMASK       (QTD_XACT | QTD_BABBLE | QTD_BUFERR | QTD_HALTED)

#define QTD_PTR(addr)   ((uint32_t)va2pa((void *)(addr)->hw) & ~0x1f)
#define QTD_PTR_INVALID 0x1

#define EHCI_TRANS_ERRORS 3

#define QH_CTRL           (1 << 27)
#define QH_PACKLEN(info0) (((info0) >> 16) & 0x7ff)
#define QH_DEVADDR(info0) ((info0) & 0x7f)
#define QH_HEAD           (1 << 15)
#define QH_DT             (1 << 14)
#define QH_HIGH_SPEED     (2 << 12)
#define QH_LOW_SPEED      (1 << 12)
#define QH_FULL_SPEED     (0 << 12)
#define QH_INACTIVATE     (1 << 7)

#define QH_SMASK   0x000000ff
#define QH_CMASK   0x0000ff00
#define QH_HUBADDR 0x007f0000
#define QH_HUBPORT 0x3f800000
#define QH_MULT    0xc0000000

#define QH_PTR(addr)   (((uint32_t)va2pa((void *)(addr)->hw) & ~0x1f) | 0x2)
#define QH_PTR_INVALID 0x1

/* 'change' bits cleared by writing 1 */
#define PORTSC_CBITS (PORTSC_CSC | PORTSC_PEC | PORTSC_OCC)


#define EHCI_PAGE_SIZE        4096
#define EHCI_PERIODIC_ALIGN   4096
#define EHCI_MAX_QTD_BUF_SIZE (4 * EHCI_PAGE_SIZE)

#define EHCI_MAX_QTD_POOL 20
#define EHCI_MAX_QH_POOL  10

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
	endptctrl2, endptctrl3, endptctrl4, endptctrl5, endptctrl6, endptctrl7,
};


enum { usb_otg1_ctrl = 0x200, usb_otg2_ctrl, usb_otg1_phy_ctrl = usb_otg2_ctrl + 5, usb_otg2_phy_ctrl };

enum { ehci_item_itd = 0, ehci_item_qh, ehci_item_sitd, ehci_item_fstn };


struct qtd {
	uint32_t next;
	uint32_t altnext;
	uint32_t token;
	uint32_t buf[5];
};


struct qh {
	uint32_t horizontal;
	uint32_t info[2];
	uint32_t current;

	/* Overlay area */
	uint32_t nextQtd;
	uint32_t altnextQtd;
	uint32_t token;
	uint32_t buf[5];
};


typedef struct _ehci_qtd {
	struct _ehci_qtd *prev, *next;
	volatile struct qtd *hw;
	struct _ehci_qh *qh;
	uint32_t paddr;
	size_t bytes;
} ehci_qtd_t;


typedef struct _ehci_qh {
	struct _ehci_qh *prev, *next;
	volatile struct qh *hw;
	volatile struct qtd *lastQtd;
	unsigned period; /* [ms], interrupt transfer only */
	unsigned phase;  /* [ms], interrupt transfer only */
	unsigned uframe; /* interrupt transfer and high-speed only */
} ehci_qh_t;


typedef struct {
	char stack[1024] __attribute__((aligned(8)));
	uint32_t *periodicList;
	ehci_qh_t *asyncList;
	ehci_qh_t **periodicNodes;

	ehci_qh_t *qhPool;
	size_t nqhs;
	ehci_qtd_t *qtdPool;
	size_t nqtds;

	handle_t irqCond, irqHandle, irqLock, asyncLock, periodicLock;
	volatile unsigned portResetChange;
	volatile unsigned status;
	volatile unsigned portsc;
} ehci_t;


int phy_init(hcd_t *hcd);


void phy_enableHighSpeedDisconnect(hcd_t *hcd, int enable);


int ehci_roothubReq(usb_dev_t *hub, usb_transfer_t *t);


uint32_t ehci_getHubStatus(usb_dev_t *hub);


#endif /* _USB_EHCI_H_ */
