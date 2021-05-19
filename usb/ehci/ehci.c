/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/ehci.c
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/list.h>
#include <sys/interrupt.h>

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <usbhost.h>
#include <stdio.h>

#include <hub.h>
#include <hcd.h>

#define TRACE(x, ...) fprintf(stderr, "ehci: " x "\n", ##__VA_ARGS__);
#define TRACE_FAIL(x, ...) fprintf(stderr, "ehci error: " x "\n", ##__VA_ARGS__);
#define FUN_TRACE fprintf(stderr, "ehci trace: %s\n", __PRETTY_FUNCTION__);

#define USBSTS_AS  (1 << 15)
#define USBSTS_PS  (1 << 14)
#define USBSTS_RCL (1 << 13)
#define USBSTS_URI (1 << 6)
#define USBSTS_SRI (1 << 7)
#define USBSTS_SLI (1 << 8)
#define USBSTS_ULPII (1 << 10)
#define USBSTS_HCH (1 << 12)
#define USBSTS_IAA (1 << 5)
#define USBSTS_SEI (1 << 4)
#define USBSTS_FRI (1 << 3)
#define USBSTS_PCI (1 << 2)
#define USBSTS_UEI (1 << 1)
#define USBSTS_UI  (1 << 0)

#define USBCMD_ASE (1 << 5)
#define USBCMD_IAA (1 << 6)

#define PORTSC_PTS_1 (3 << 30)
#define PORTSC_STS (1 << 29)
#define PORTSC_PPTW (1 << 28)
#define PORTSC_PSPD (3 << 26)
#define PORTSC_PTS_2 (1 << 25)
#define PORTSC_PFSC (1 << 24)
#define PORTSC_PHCD (1 << 23)

#define PORTSC_WKOC (1 << 22)
#define PORTSC_WKDSCNNT (1 << 21)
#define PORTSC_WKCNT (1 << 20)
#define PORTSC_PTC (0xf << 16)
#define PORTSC_PIC (3 << 14)
#define PORTSC_PO (1 << 13)
#define PORTSC_PP (1 << 12)
#define PORTSC_LS (3 << 10)
#define PORTSC_HSP (1 << 9)
#define PORTSC_PR (1 << 8)
#define PORTSC_SUSP (1 << 7)

#define PORTSC_FPR (1 << 6)
#define PORTSC_OCC (1 << 5)
#define PORTSC_OCA (1 << 4)
#define PORTSC_PEC (1 << 3)

#define PORTSC_ENA (1 << 2)
#define PORTSC_CSC (1 << 1)
#define PORTSC_CCS (1 << 0)

/* 'change' bits cleared by writing 1 */
#define PORTSC_CBITS (PORTSC_CSC | PORTSC_PEC | PORTSC_OCC)


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

	otg1_ctrl = 0x100, otg2_ctrl,
};


enum { usb_otg1_ctrl = 0x200, usb_otg2_ctrl, usb_otg1_phy_ctrl = usb_otg2_ctrl + 5, usb_otg2_phy_ctrl };


typedef struct link_pointer {
	unsigned terminate : 1;
	unsigned type : 2;
	unsigned zero : 2;
	unsigned pointer : 27;
} link_pointer_t;


struct itd {
	link_pointer_t next;

	union {
		struct {
			unsigned offset      : 12;
			unsigned page_select :  3;
			unsigned ioc         :  1;
			unsigned length      : 12;
			unsigned status      :  4;
		};
		unsigned raw;
	} transactions[8];

	union {
		struct {
			unsigned short reserved;
			unsigned short pointer;
		} buffers[7];

		struct {
			unsigned device_address  :  7;
			unsigned reserved        :  1;
			unsigned endpoint        :  4;
			unsigned page0           : 16;
			unsigned max_packet_size : 11;
			unsigned direction       :  1;
			unsigned page1           : 16;
			unsigned mult            :  2;
		};
	};
};


struct sitd {
	link_pointer_t next;

	unsigned device_addr : 7;
	unsigned reserved0   : 1;
	unsigned endpoint    : 4;
	unsigned reserved1   : 4;
	unsigned hub_addr    : 7;
	unsigned reserved2   : 1;
	unsigned port_number : 7;
	unsigned direction   : 1;

	unsigned split_start_mask : 8;
	unsigned split_completion_mask : 8;
	unsigned reserved3 : 16;

	unsigned status : 8;
	unsigned split_progress_mask : 8;
	unsigned bytes_to_transfer : 10;
	unsigned reserved4 : 4;
	unsigned page_select : 1;
	unsigned ioc : 1;

	unsigned current_offset : 12;
	unsigned page0          : 20;

	unsigned transaction_count    :  3;
	unsigned transaction_position :  2;
	unsigned reserved5            :  7;
	unsigned page1                : 20;

	link_pointer_t back_link;
};


struct qtd {
	link_pointer_t next;
	link_pointer_t alt_next;

	unsigned ping_state : 1;
	unsigned split_state : 1;
	unsigned missed_uframe : 1;
	unsigned transaction_error : 1;
	unsigned babble : 1;
	unsigned buffer_error : 1;
	unsigned halted : 1;
	unsigned active : 1;

	unsigned pid_code : 2;
	unsigned error_counter : 2;
	unsigned current_page : 3;
	unsigned ioc : 1;
	unsigned bytes_to_transfer : 15;
	unsigned data_toggle : 1;

	union {
		struct {
			unsigned reserved : 12;
			unsigned page     : 20;
		} buffers[5];

		struct {
			unsigned offset : 12;
			unsigned page0  : 20;
		};
	};
};


struct qh {
	link_pointer_t horizontal;

	unsigned device_addr : 7;
	unsigned inactivate : 1;
	unsigned endpoint : 4;
	unsigned endpoint_speed : 2;
	unsigned data_toggle : 1;
	unsigned head_of_reclamation : 1;
	unsigned max_packet_len : 11;
	unsigned control_endpoint : 1;
	unsigned nak_count_reload : 4;

	unsigned interrupt_schedule_mask : 8;
	unsigned split_completion_mask : 8;
	unsigned hub_addr : 7;
	unsigned port_number : 7;
	unsigned pipe_multiplier : 2;

	link_pointer_t current_qtd;

	struct qtd transfer_overlay;

	/* non-hardware fields */
	struct qtd *last;
	struct qh *next, *prev;
};


typedef struct ehci {
	link_pointer_t *periodic_list;
	volatile struct qh *async_head;

	handle_t irq_cond, irq_handle, irq_lock, aai_cond, async_lock;
	volatile unsigned portResetChange;
	volatile unsigned status;
	volatile unsigned port_change;
	volatile unsigned portsc;

	handle_t common_lock;
} ehci_t;


static int ehci_irqHandler(unsigned int n, void *data)
{
	hcd_t *hcd = (hcd_t *) data;
	ehci_t *ehci = (ehci_t *) hcd->priv;

	ehci->status = *(hcd->base + usbsts);
	*(hcd->base + usbsts) = ehci->status & 0x1f;

	return -!(ehci->status & 0x1f);
}


static void ehci_printStatus(ehci_t *ehci)
{
	TRACE("%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
		ehci->status & USBSTS_UI ? "UI " : "",
		ehci->status & USBSTS_UEI ? "UEI " : "",
		ehci->status & USBSTS_PCI ? "PCI " : "",
		ehci->status & USBSTS_FRI ? "FRI " : "",
		ehci->status & USBSTS_SEI ? "SEI " : "",
		ehci->status & USBSTS_IAA ? "AAI " : "",
		ehci->status & USBSTS_URI ? "URI " : "",
		ehci->status & USBSTS_SRI ? "SRI " : "",
		ehci->status & USBSTS_SLI ? "SLI " : "",
		ehci->status & USBSTS_ULPII ? "ULPII " : "",
		ehci->status & USBSTS_HCH ? "HCH " : "",
		ehci->status & USBSTS_RCL ? "RCL " : "",
		ehci->status & USBSTS_PS ? "PS " : "",
		ehci->status & USBSTS_AS ? "AS " : "");
	TRACE("portsc: %x", ehci->portsc);
}


static void ehci_printPortStatus(hcd_t *hcd)
{
	unsigned portsc = *(hcd->base + portsc1);

	printf("%s: %x  ", "PORTSC_PTS_1", portsc & PORTSC_PTS_1);
	printf("%s: %x  ", "PORTSC_STS", portsc & PORTSC_STS);
	printf("%s: %x  ", "PORTSC_PPTW", portsc & PORTSC_PPTW);
	printf("%s: %x  ", "PORTSC_PSPD", portsc & PORTSC_PSPD);
	printf("%s: %x  ", "PORTSC_PTS_2", portsc & PORTSC_PTS_2);
	printf("%s: %x  ", "PORTSC_PFSC", portsc & PORTSC_PFSC);
	printf("%s: %x  ", "PORTSC_PHCD", portsc & PORTSC_PHCD);

	printf("%s: %x  ", "PORTSC_WKOC", portsc & PORTSC_WKOC);
	printf("%s: %x  ", "PORTSC_WKDSCNNT", portsc & PORTSC_WKDSCNNT);
	printf("%s: %x  ", "PORTSC_WKCNT", portsc & PORTSC_WKCNT);
	printf("%s: %x  ", "PORTSC_PTC", portsc & PORTSC_PTC);
	printf("%s: %x  ", "PORTSC_PIC", portsc & PORTSC_PIC);
	printf("%s: %x  ", "PORTSC_PO", portsc & PORTSC_PO);
	printf("%s: %x  ", "PORTSC_PP", portsc & PORTSC_PP);
	printf("%s: %x  ", "PORTSC_LS", portsc & PORTSC_LS);
	printf("%s: %x  ", "PORTSC_HSP", portsc & PORTSC_HSP);
	printf("%s: %x  ", "PORTSC_PR", portsc & PORTSC_PR);
	printf("%s: %x  ", "PORTSC_SUSP", portsc & PORTSC_SUSP);
	printf("%s: %x  ", "PORTSC_FPR", portsc & PORTSC_FPR);
	printf("%s: %x  ", "PORTSC_OCC", portsc & PORTSC_OCC);
	printf("%s: %x  ", "PORTSC_OCA", portsc & PORTSC_OCA);
	printf("%s: %x  ", "PORTSC_PEC", portsc & PORTSC_PEC);

	printf("%s: %x  ", "PORTSC_ENA", portsc & PORTSC_ENA);
	printf("%s: %x  ", "PORTSC_CSC", portsc & PORTSC_CSC);
	printf("%s: %x  ", "PORTSC_CCS", portsc & PORTSC_CCS);
	printf("\n");
}


static void ehci_irqThread(void *arg)
{
	hcd_t *hcd = (hcd_t *) arg;
	ehci_t *ehci = (ehci_t *) hcd->priv;

	mutexLock(ehci->irq_lock);
	for (;;) {
		condWait(ehci->irq_cond, ehci->irq_lock, 0);
		/* TODO: handle irq */
	}
}


static int ehci_deviceAttached(hcd_t *hcd)
{
	return *(hcd->base + portsc1) & PORTSC_CCS;
}


static void ehci_resetPort(hcd_t *hcd, int port)
{
	ehci_t *ehci = (ehci_t *) hcd->priv;
	volatile int *reg = (hcd->base + portsc1) + (port - 1);

	*reg |= PORTSC_PR;
	*reg &= ~PORTSC_ENA;
	/* This is imx deviation. According to ehci documentation
	 * it is up to software to set the PR bit 0 after waiting 20ms */
	while (*reg & PORTSC_PR) ;
	*reg |= PORTSC_ENA;

	ehci->portResetChange = 1 << port;
	/* Turn port power on */
	*reg |= PORTSC_PP;
}

int ehci_urbSubmit(hcd_t *hcd, usb_urb_t *urb)
{
	return 0;
}

static int hub_statusChanged(usb_hub_t *hub, uint32_t *status)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *) hcd->priv;
	volatile int *portsc;
	int i;

	*status = 0;

	/* Status not changed */
	if ((ehci->status & USBSTS_PCI) == 0)
		return 0;

	ehci->status &= ~USBSTS_PCI;

	for (i = 0; i < hub->nports; i++) {
		portsc = hcd->base + portsc1 + i;
		if (*portsc & PORTSC_CSC)
			*status |= 1 << (i + 1);
	}

	return 0;
}

static int hub_getPortStatus(usb_hub_t *hub, int port, usb_port_status_t *status)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *) hcd->priv;
	int val;

	if (port > hub->nports)
		return -1;

	status->wPortChange = 0;
	status->wPortStatus = 0;

	val = *(hcd->base + portsc1 + port - 1);
	if (val & PORTSC_CCS)
		status->wPortStatus |= USB_PORT_STAT_CONNECTION;

	if (val & PORTSC_CSC)
		status->wPortChange |= USB_PORT_STAT_C_CONNECTION;

	if (val & PORTSC_ENA)
		status->wPortStatus |= USB_PORT_STAT_ENABLE;

	if (val & PORTSC_PEC)
		status->wPortChange |= USB_PORT_STAT_C_ENABLE;

	if (val & PORTSC_OCA)
		status->wPortStatus |= USB_PORT_STAT_OVERCURRENT;

	if (val & PORTSC_OCC)
		status->wPortChange |= USB_PORT_STAT_C_OVERCURRENT;

	if (val & PORTSC_SUSP)
		status->wPortStatus |= USB_PORT_STAT_SUSPEND;

	if (val & PORTSC_PR)
		status->wPortStatus |= USB_PORT_STAT_RESET;

	if ((val & PORTSC_LS) == (1 << 10))
		status->wPortStatus |= USB_PORT_STAT_LOW_SPEED;

	if (val & PORTSC_PP)
		status->wPortStatus |= USB_PORT_STAT_POWER;

	if ((val & PORTSC_PTC) != 0)
		status->wPortStatus |= USB_PORT_STAT_TEST;

	if (ehci->portResetChange & (1 << port))
		status->wPortChange |= USB_PORT_STAT_C_RESET;

	/* TODO: set HIGH SPEED and INDICATOR */
	return 0;
}

static int hub_setPortFeature(usb_hub_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;

	switch (wValue) {
	case USB_PORT_FEAT_RESET:
		ehci_resetPort(hcd, port);
		break;
	case USB_PORT_FEAT_SUSPEND:
	case USB_PORT_FEAT_POWER:
	case USB_PORT_FEAT_TEST:
	case USB_PORT_FEAT_INDICATOR:
		/* TODO */
		break;
	default:
		return -1;
	}

	return 0;
}

static int hub_clearPortFeature(usb_hub_t *hub, int port, uint16_t wValue)
{
	hcd_t *hcd = hub->hcd;
	ehci_t *ehci = (ehci_t *) hcd->priv;
	volatile int *portsc = hcd->base + portsc1 + port - 1;
	uint32_t val = *portsc;

	if (port > hub->nports)
		return -1;

	val &= ~PORTSC_CBITS;
	switch (wValue) {
	/* For 'change' features, ack the change */
	case USB_PORT_FEAT_C_CONNECTION:
		*portsc = val | PORTSC_CSC;
		break;
	case USB_PORT_FEAT_C_ENABLE:
		*portsc = val | PORTSC_PEC;
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		*portsc = val | PORTSC_OCC;
		break;
	case USB_PORT_FEAT_C_RESET:
		ehci->portResetChange &= ~(1 << port);
		break;
	case USB_PORT_FEAT_ENABLE:
		/* Disable port */
		*portsc = val & ~PORTSC_ENA;
		break;
	case USB_PORT_FEAT_POWER:
	case USB_PORT_FEAT_INDICATOR:
	case USB_PORT_FEAT_SUSPEND:
	case USB_PORT_FEAT_C_SUSPEND:
		/* TODO */
		break;
	default:
		return -1;
	}

	return 0;
}

void phy_init(hcd_t *hcd);

static void *ehci_periodicListAlloc(void)
{
	uintptr_t addr, tmp;

	addr = (uintptr_t) mmap(NULL, 4096, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);
	if (addr % 4096 == 0)
		return (void *) addr;

	munmap((void *) addr, 4096);

	addr = (uintptr_t) mmap(NULL, 8192, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);
	tmp = ((addr + 4096) / 4096) * 4096;

	/* unmap memory prefix */
	if (tmp != addr)
		munmap((void *) addr, tmp - addr);

	/* unmap memory sufix */
	if (tmp + 4096 != addr)
		munmap((void *) tmp + 4096, (addr + 8192) - (tmp + 4096));

	return (void *) tmp;
}

int ehci_init(hcd_t *hcd)
{
	ehci_t *ehci;
	int i;

	if (sizeof(struct qh) > 64 || sizeof(struct qtd) > 64) {
		fprintf(stderr, "qh is %d bytes, qtd is %d bytes\n", sizeof(struct qh), sizeof(struct qtd));
		exit(1);
	}

	if ((ehci = malloc(sizeof(ehci_t))) == NULL) {
		fprintf(stderr, "ehci: Can't allocate hcd ehci!\n");
		return -ENOMEM;
	}

	hcd->priv = ehci;
	phy_init(hcd);

	ehci->async_head = NULL;
	condCreate(&ehci->aai_cond);
	condCreate(&ehci->irq_cond);
	mutexCreate(&ehci->irq_lock);
	mutexCreate(&ehci->async_lock);

	ehci->periodic_list = ehci_periodicListAlloc();
	for (i = 0; i < 1024; ++i)
		ehci->periodic_list[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	/* Reset controller */
	*(hcd->base + usbcmd) |= 2;
	while (*(hcd->base + usbcmd) & 2) ;

	/* Set host mode */
	*(hcd->base + usbmode) |= 3;

	/* Enable interrupts */
	*(hcd->base + usbintr) = 0x1f & ~(1 << 3);

	/* Set periodic frame list */
	*(hcd->base + periodiclistbase) = va2pa(ehci->periodic_list);

	/* Set interrupts threshold, frame list size, turn controller on */
	*(hcd->base + usbcmd) |= (1 << 4) | 1;

	/* Route all ports to this host controller */
	*(hcd->base + configflag) = 1;

	hcd->roothub->statusChanged = hub_statusChanged;
	hcd->roothub->getPortStatus = hub_getPortStatus;
	hcd->roothub->clearPortFeature = hub_clearPortFeature;
	hcd->roothub->setPortFeature = hub_setPortFeature;
	hcd->roothub->nports = *(hcd->base + hcsparams) & 0xf;
	strncpy(hcd->roothub->dev->name, "USB 2.0 root hub", sizeof(hcd->roothub->dev->name));
	/* TODO: set root hub device descriptors */

	beginthread(ehci_irqThread, 2, malloc(0x1000), 0x1000, hcd);
	interrupt(112 + 16, ehci_irqHandler, hcd, ehci->irq_cond, &ehci->irq_handle);

	return 0;
}


#if 0
void ehci_shutdown(void)
{
	*(hcd->base + usbcmd) &= ~USBCMD_ASE;
	*(hcd->base + portsc1) &= PORTSC_PP;
	phy_disableClock();
}
#endif

void ehci_dumpRegisters(hcd_t *hcd, FILE *stream)
{
	printf("DUMP REGISTERS\n");
	fprintf(stream, "%18s: %08x", "id", *(hcd->base + id));
	fprintf(stream, "%18s: %08x\n", "hwgeneral", *(hcd->base + hwgeneral));
	fprintf(stream, "%18s: %08x", "hwhost", *(hcd->base + hwhost));
	fprintf(stream, "%18s: %08x\n", "hwdevice", *(hcd->base + hwdevice));
	fprintf(stream, "%18s: %08x", "hwtxbuf", *(hcd->base + hwtxbuf));
	fprintf(stream, "%18s: %08x\n", "hwrxbuf", *(hcd->base + hwrxbuf));
	fprintf(stream, "%18s: %08x", "gptimer0ld", *(hcd->base + gptimer0ld));
	fprintf(stream, "%18s: %08x\n", "gptimer0ctrl", *(hcd->base + gptimer0ctrl));
	fprintf(stream, "%18s: %08x", "gptimer1ld", *(hcd->base + gptimer1ld));
	fprintf(stream, "%18s: %08x\n", "gptimer1ctrl", *(hcd->base + gptimer1ctrl));
	fprintf(stream, "%18s: %08x", "sbuscfg", *(hcd->base + sbuscfg));
	fprintf(stream, "%18s: %08x\n", "caplength", *(hcd->base + caplength));
	fprintf(stream, "%18s: %08x", "hciversion", *(hcd->base + hciversion));
	fprintf(stream, "%18s: %08x\n", "hcsparams", *(hcd->base + hcsparams));
	fprintf(stream, "%18s: %08x", "hccparams", *(hcd->base + hccparams));
	fprintf(stream, "%18s: %08x\n", "dciversion", *(hcd->base + dciversion));
	fprintf(stream, "%18s: %08x", "dccparams", *(hcd->base + dccparams));
	fprintf(stream, "%18s: %08x\n", "usbcmd", *(hcd->base + usbcmd));
	fprintf(stream, "%18s: %08x", "usbsts", *(hcd->base + usbsts));
	fprintf(stream, "%18s: %08x\n", "usbintr", *(hcd->base + usbintr));
	fprintf(stream, "%18s: %08x", "frindex", *(hcd->base + frindex));
	fprintf(stream, "%18s: %08x\n", "periodiclistbase", *(hcd->base + periodiclistbase));
	fprintf(stream, "%18s: %08x", "deviceaddr", *(hcd->base + deviceaddr));
	fprintf(stream, "%18s: %08x\n", "asynclistaddr", *(hcd->base + asynclistaddr));
	fprintf(stream, "%18s: %08x", "endpointlistaddr", *(hcd->base + endpointlistaddr));
	fprintf(stream, "%18s: %08x\n", "burstsize", *(hcd->base + burstsize));
	fprintf(stream, "%18s: %08x", "txfilltunning", *(hcd->base + txfilltunning));
	fprintf(stream, "%18s: %08x\n", "endptnak", *(hcd->base + endptnak));
	fprintf(stream, "%18s: %08x", "endptnaken", *(hcd->base + endptnaken));
	fprintf(stream, "%18s: %08x\n", "configflag", *(hcd->base + configflag));
	fprintf(stream, "%18s: %08x", "portsc1", *(hcd->base + portsc1));
	fprintf(stream, "%18s: %08x\n", "otgsc", *(hcd->base + otgsc));
	fprintf(stream, "%18s: %08x", "usbmode", *(hcd->base + usbmode));
	fprintf(stream, "%18s: %08x\n", "endptsetupstat", *(hcd->base + endptsetupstat));
	fprintf(stream, "%18s: %08x", "endptprime", *(hcd->base + endptprime));
	fprintf(stream, "%18s: %08x\n", "endptflush", *(hcd->base + endptflush));
	fprintf(stream, "%18s: %08x", "endptstat", *(hcd->base + endptstat));
	fprintf(stream, "%18s: %08x\n", "endptcomplete", *(hcd->base + endptcomplete));
	fprintf(stream, "%18s: %08x", "endptctrl0", *(hcd->base + endptctrl0));
	fprintf(stream, "%18s: %08x\n", "endptctrl1", *(hcd->base + endptctrl1));
	fprintf(stream, "%18s: %08x", "endptctrl2", *(hcd->base + endptctrl2));
	fprintf(stream, "%18s: %08x\n", "endptctrl3", *(hcd->base + endptctrl3));
	fprintf(stream, "%18s: %08x", "endptctrl4", *(hcd->base + endptctrl4));
	fprintf(stream, "%18s: %08x\n", "endptctrl5", *(hcd->base + endptctrl5));
	fprintf(stream, "%18s: %08x", "endptctrl6", *(hcd->base + endptctrl6));
	fprintf(stream, "%18s: %08x\n", "endptctrl7", *(hcd->base + endptctrl7));
	fprintf(stream, "%18s: %08x", "otg1_ctrl", *(hcd->base + otg1_ctrl));
	fprintf(stream, "%18s: %08x\n\n", "otg2_ctrl", *(hcd->base + otg2_ctrl));
}


static void ehci_dumpQueue(FILE *stream, struct qh *qh)
{
	fprintf(stream, "%18s: %08x\n", "device_addr", qh->device_addr);
	fprintf(stream, "%18s: %08x\n", "inactivate", qh->inactivate);
	fprintf(stream, "%18s: %08x\n", "endpoint", qh->endpoint);
	fprintf(stream, "%18s: %08x\n", "endpoint_speed", qh->endpoint_speed);
	fprintf(stream, "%18s: %08x\n", "data_toggle", qh->data_toggle);
	fprintf(stream, "%18s: %08x\n", "head_of_reclamation", qh->head_of_reclamation);
	fprintf(stream, "%18s: %08x\n", "max_packet_len ", qh->max_packet_len );
	fprintf(stream, "%18s: %08x\n", "control_endpoint", qh->control_endpoint);
	fprintf(stream, "%18s: %08x\n", "nak_count_reload", qh->nak_count_reload);

	fprintf(stream, "%18s: %08x\n", "interrupt_schedule_mask", qh->interrupt_schedule_mask);
	fprintf(stream, "%18s: %08x\n", "split_completion_mask", qh->split_completion_mask);
	fprintf(stream, "%18s: %08x\n", "hub_addr", qh->hub_addr);
	fprintf(stream, "%18s: %08x\n", "port_number", qh->port_number);
	fprintf(stream, "%18s: %08x\n", "pipe_multiplier", qh->pipe_multiplier);

	fprintf(stream, "%18s: %08x\n", "ping_state", qh->transfer_overlay.ping_state);
	fprintf(stream, "%18s: %08x\n", "split_state", qh->transfer_overlay.split_state);
	fprintf(stream, "%18s: %08x\n", "missed_uframe", qh->transfer_overlay.missed_uframe);
	fprintf(stream, "%18s: %08x\n", "transaction_error", qh->transfer_overlay.transaction_error);
	fprintf(stream, "%18s: %08x\n", "babble", qh->transfer_overlay.babble);
	fprintf(stream, "%18s: %08x\n", "buffer_error", qh->transfer_overlay.buffer_error);
	fprintf(stream, "%18s: %08x\n", "halted", qh->transfer_overlay.halted);
	fprintf(stream, "%18s: %08x\n", "active", qh->transfer_overlay.active);

	fprintf(stream, "%18s: %p\n", "current", &qh->current_qtd);

	fprintf(stream, "%18s: %p\n", "next", &qh->transfer_overlay.next);
	fprintf(stream, "%18s: %08x\n", "pid_code", qh->transfer_overlay.pid_code);
	fprintf(stream, "%18s: %08x\n", "error_counter", qh->transfer_overlay.error_counter);
	fprintf(stream, "%18s: %08x\n", "current_page", qh->transfer_overlay.current_page);
	fprintf(stream, "%18s: %08x\n", "ioc", qh->transfer_overlay.ioc);
	fprintf(stream, "%18s: %08x\n", "bytes_to_transfer", qh->transfer_overlay.bytes_to_transfer);
	fprintf(stream, "%18s: %08x\n", "data_toggle", qh->transfer_overlay.data_toggle);

}


static void ehci_activate(struct qh *qh) {
	qh->transfer_overlay.active = 1;
}


static hcd_ops_t ehci_ops = {
	.type = "ehci",
	.init = ehci_init
};

__attribute__ ((constructor)) static void ehci_register(void)
{
	hcd_register(&ehci_ops);
}