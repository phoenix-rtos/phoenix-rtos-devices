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

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "ehci.h"
#include "phy.h"
#include "dma.h"

#define TRACE(x, ...) //fprintf(stderr, "ehci: " x "\n", ##__VA_ARGS__)
#define FUN_TRACE //fprintf(stderr, "ehci trace: %s\n", __PRETTY_FUNCTION__)

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

#define USB_OTG2_IRQ (32 + 42)
#define USB_OTG1_IRQ (32 + 43)

#define USB_ADDR 0x02184000

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


struct {
	volatile unsigned *base;
	volatile unsigned *usb2;
	link_pointer_t *periodic_list;
	volatile struct qh *async_head;

	handle_t irq_cond, irq_handle, irq_lock, aai_cond, async_lock;
	volatile unsigned status;
	volatile unsigned port_change;
	volatile unsigned portsc;

	handle_t common_lock;
} ehci_common;


void ehci_insertPeriodic(struct qh *qh)
{
	link_pointer_t ptr = { 0 };
	ptr.pointer = va2pa(qh) >> 5;
	ptr.type = framelist_qh;
	ehci_common.periodic_list[0] = ptr;
}


static int ehci_irqHandler(unsigned int n, void *data)
{
	ehci_common.status = *(ehci_common.usb2 + usbsts);
	*(ehci_common.usb2 + usbsts) = ehci_common.status & 0x1f;

	ehci_common.portsc = *(ehci_common.usb2 + portsc1);

	if (ehci_common.status & USBSTS_RCL) {
		/* Async schedule is empty */
	//	ehci_common.status &= ~USBSTS_RCL;
	}

	if (*(ehci_common.usb2 + portsc1) & PORTSC_PEC) {
		*(ehci_common.usb2 + portsc1) = PORTSC_PEC;
	}

	ehci_common.port_change = 0;

	if (ehci_common.status & USBSTS_PCI) {
		if (*(ehci_common.usb2 + portsc1) & PORTSC_CSC) {
			ehci_common.port_change = 1;
			*(ehci_common.usb2 + portsc1) = PORTSC_CSC;
		}
	}

	return -!(ehci_common.status & 0x1f);
}


void ehci_printStatus(void)
{
	TRACE("%s%s%s%s%s%s%s%s%s%s%s%s%s%s\n",
		ehci_common.status & USBSTS_UI ? "UI " : "",
		ehci_common.status & USBSTS_UEI ? "UEI " : "",
		ehci_common.status & USBSTS_PCI ? "PCI " : "",
		ehci_common.status & USBSTS_FRI ? "FRI " : "",
		ehci_common.status & USBSTS_SEI ? "SEI " : "",
		ehci_common.status & USBSTS_IAA ? "AAI " : "",
		ehci_common.status & USBSTS_URI ? "URI " : "",
		ehci_common.status & USBSTS_SRI ? "SRI " : "",
		ehci_common.status & USBSTS_SLI ? "SLI " : "",
		ehci_common.status & USBSTS_ULPII ? "ULPII " : "",
		ehci_common.status & USBSTS_HCH ? "HCH " : "",
		ehci_common.status & USBSTS_RCL ? "RCL " : "",
		ehci_common.status & USBSTS_PS ? "PS " : "",
		ehci_common.status & USBSTS_AS ? "AS " : "");
	TRACE("portsc: %x", ehci_common.portsc);
}


void ehci_printPortStatus(void)
{
	unsigned portsc = *(ehci_common.usb2 + portsc1);

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


void ehci_irqThread(void *callback)
{
	mutexLock(ehci_common.common_lock);
	for (;;) {
		condWait(ehci_common.irq_cond, ehci_common.common_lock, 0);

		ehci_printStatus();

//ehci_printPortStatus();
		if (ehci_common.status & USBSTS_IAA)
			condSignal(ehci_common.aai_cond);

		((void (*)(int))callback)(ehci_common.port_change);
	}
}


void ehci_linkQtd(struct qtd *prev, struct qtd *next)
{
	prev->next.pointer = va2pa(next) >> 5;
	prev->next.type = framelist_itd;
	prev->next.terminate = 0;
}


void ehci_consQtd(struct qtd *qtd, struct qh *qh)
{
	link_pointer_t head = qh->transfer_overlay.next;
	qh->transfer_overlay.next.pointer = va2pa(qtd) >> 5;
	qh->transfer_overlay.next.terminate = 0;
	qtd->next = head;

	if (qh->endpoint_speed != high_speed) {
		qtd->split_state = 0;
	}
}


void ehci_enqueue(struct qh *qh, struct qtd *first, struct qtd *last)
{
	FUN_TRACE;
	struct qtd *prev_last = qh->last;

	qh->last = last;
	last->next.terminate = 1;
	last->ioc = 1;

	prev_last->next.pointer = va2pa(first) >> 5;
	prev_last->next.type = framelist_itd;

	asm volatile ("dmb" ::: "memory");

	prev_last->next.terminate = 0;

	if (!qh->transfer_overlay.active) {
		TRACE("was not active");
		qh->transfer_overlay.next = prev_last->next;
	}
}


struct qtd *ehci_allocQtd(int token, char *buffer, size_t *size, int datax)
{
	struct qtd *result = NULL;
	size_t initial_size;
	int ix, offset;

	if ((result = dma_alloc64()) == NULL)
		return NULL;

	result->data_toggle = datax;
	result->pid_code = token;

	result->next.terminate = 1;
	result->alt_next.terminate = 1;
	result->active = 1;
	result->error_counter = 3;

	if (buffer != NULL) {
		initial_size = *size;
		offset = (uintptr_t)buffer & (SIZE_PAGE - 1);
		result->offset = offset;

		for (ix = 0; ix < 5 && *size; ++ix) {
			result->buffers[ix].page = va2pa(buffer) >> 12;
			buffer += SIZE_PAGE - offset;

			if (*size > SIZE_PAGE - offset)
				*size -= SIZE_PAGE - offset;
			else
				*size = 0;

			offset = 0;
		}

		result->bytes_to_transfer = initial_size - *size;
	}

	return result;
}


void ehci_freeQtd(struct qtd *qtd)
{
	if (qtd->babble) {
		TRACE("babble!");
	}
	if (qtd->transaction_error) {
		TRACE("transaction error!");
	}
	if (qtd->buffer_error) {
		TRACE("buffer error");
	}

	dma_free64(qtd);
}


struct qh *ehci_allocQh(int address, int endpoint, int transfer, int speed, int max_packet_len)
{
	struct qh *result = dma_alloc64();

	if (result == NULL)
		return NULL;

	result->horizontal.terminate = 1;

	result->device_addr = address;
	result->endpoint_speed = speed;
	result->data_toggle = transfer == transfer_control ? 1 : 0;

	result->current_qtd.terminate = 1;
	result->transfer_overlay.next.terminate = 1;

	result->max_packet_len = max_packet_len;
	result->endpoint = endpoint;
	result->control_endpoint = transfer == transfer_control && speed != high_speed;
	result->nak_count_reload = 3;

	if (transfer == transfer_interrupt) {
		result->interrupt_schedule_mask = 0xff;
		result->split_completion_mask = 0xff;
	}

	result->last = &result->transfer_overlay;

	return result;
}


int ehci_dequeue(struct qh *qh, struct qtd *first, struct qtd *last)
{
	FUN_TRACE;

	if (qh->last == last)
		qh->last = &qh->transfer_overlay;

	return 0;
}


int ehci_qtdError(struct qtd *qtd)
{
	return qtd->transaction_error;
}


int ehci_qtdFinished(struct qtd *qtd)
{
	return !qtd->active || qtd->halted;
}


void ehci_qhSetAddress(struct qh *qh, int address)
{
	qh->device_addr = address;
}


int ehci_qtdRemainingBytes(struct qtd *qtd)
{
	return qtd->bytes_to_transfer;
}


void ehci_freeQh(struct qh *qh)
{
	FUN_TRACE;

	mutexLock(ehci_common.async_lock);
	if (ehci_common.async_head != NULL && /*hack*/!qh->interrupt_schedule_mask) {
		*(ehci_common.usb2 + usbcmd) |= USBCMD_IAA;

		while (!(ehci_common.status & USBSTS_IAA)) {
			TRACE("waiting for IAA %s", (ehci_common.status & USBSTS_AS) ? "async on" : "async off, dupa");
			if (condWait(ehci_common.aai_cond, ehci_common.common_lock, 1000000) < 0) {
				TRACE("aii timeout");
				break;
			}
		}

		TRACE("got IAA");

		ehci_common.status &= ~USBSTS_IAA;
	}
	mutexUnlock(ehci_common.async_lock);
	dma_free64(qh);
}


void ehci_linkQh(struct qh *qh)
{
	FUN_TRACE;
	int first = ehci_common.async_head == NULL;

	mutexLock(ehci_common.async_lock);
	LIST_ADD(&ehci_common.async_head, qh);

	qh->horizontal.pointer = va2pa(qh->next) >> 5;
	qh->horizontal.type = framelist_qh;
	qh->horizontal.terminate = 0;

	qh->prev->horizontal.pointer = va2pa(qh) >> 5;
	qh->prev->horizontal.type = framelist_qh;
	qh->prev->horizontal.terminate = 0;
	mutexUnlock(ehci_common.async_lock);

	if (first) {
		TRACE("first qh");
		qh->head_of_reclamation = 1;

		*(ehci_common.usb2 + asynclistaddr) = va2pa(qh);
		*(ehci_common.usb2 + usbcmd) |= USBCMD_ASE;
		while (!(*(ehci_common.usb2 + usbsts) & USBSTS_AS)) ;
	}
}


void ehci_unlinkQh(struct qh *unlink)
{
	FUN_TRACE;

	if (unlink->head_of_reclamation)
		unlink->next->head_of_reclamation = 1;

	mutexLock(ehci_common.async_lock);
	if (unlink == unlink->next) {
		TRACE("disabling ASE");
		ehci_common.async_head = NULL;
		*(ehci_common.usb2 + usbcmd) &= ~USBCMD_ASE;
		while (*(ehci_common.usb2 + usbsts) & USBSTS_AS) ;
		*(ehci_common.usb2 + asynclistaddr) = 1;
	}
	else if (unlink == ehci_common.async_head) {
		ehci_common.async_head = unlink->next;
		*(ehci_common.usb2 + asynclistaddr) = va2pa(unlink->next);
	}

	unlink->prev->horizontal = unlink->horizontal;
	LIST_REMOVE(&ehci_common.async_head, unlink);
	mutexUnlock(ehci_common.async_lock);
}


int ehci_deviceAttached(void)
{
	return *(ehci_common.usb2 + portsc1) & PORTSC_CCS;
}


void ehci_resetPort(void)
{
	//*(ehci_common.usb2 + portsc1) &= ~PORTSC_PP;
	/* Reset port */
	for (int i = 0; i < 10; ++i) {
		*(ehci_common.usb2 + portsc1) |= PORTSC_PR;
		*(ehci_common.usb2 + portsc1) &= ~PORTSC_ENA;
		while (*(ehci_common.usb2 + portsc1) & PORTSC_PR) ;
		*(ehci_common.usb2 + portsc1) |= PORTSC_ENA;
	}

	/* Turn port power on */
	*(ehci_common.usb2 + portsc1) |= PORTSC_PP;

	usleep(500 * 1000);

	/* Configure port */
	//*(ehci_common.usb2 + portsc1) |= PORTSC_WKOC | PORTSC_WKCNT | PORTSC_WKDSCNNT;

	//ehci_printPortStatus();
	//ehci_printStatus();
}


void ehci_init(void *event_callback, handle_t common_lock)
{
	int i;

	if (sizeof(struct qh) > 64 || sizeof(struct qtd) > 64) {
		fprintf(stderr, "qh is %d bytes, qtd is %d bytes\n", sizeof(struct qh), sizeof(struct qtd));
		exit(1);
	}


	ehci_common.common_lock = common_lock;
	ehci_common.async_head = NULL;
	phy_init();
	condCreate(&ehci_common.aai_cond);
	condCreate(&ehci_common.irq_cond);
	mutexCreate(&ehci_common.irq_lock);
	mutexCreate(&ehci_common.async_lock);

	ehci_common.periodic_list = mmap(NULL, SIZE_PAGE, PROT_WRITE | PROT_READ, MAP_ANONYMOUS | MAP_UNCACHED, OID_NULL, 0);

	for (i = 0; i < 1024; ++i)
		ehci_common.periodic_list[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	ehci_common.base = mmap(NULL, 4 * SIZE_PAGE, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, USB_ADDR);

	/* Offset into USB2 */
	ehci_common.usb2 = ehci_common.base + 128;

	beginthread(ehci_irqThread, 4, malloc(0x4000), 0x4000, event_callback);
	interrupt(USB_OTG2_IRQ, ehci_irqHandler, NULL, ehci_common.irq_cond, &ehci_common.irq_handle);

	/* Reset controller */
	*(ehci_common.usb2 + usbcmd) |= 2;
	while (*(ehci_common.usb2 + usbcmd) & 2) ;

	/* Set host mode */
	*(ehci_common.usb2 + usbmode) |= 3;

	/* Enable interrupts */
	*(ehci_common.usb2 + usbintr) = 0x1f & ~(1 << 3);

	/* Set periodic frame list */
	*(ehci_common.usb2 + periodiclistbase) = va2pa(ehci_common.periodic_list);

	/* Set interrupts threshold, frame list size, turn controller on */
	*(ehci_common.usb2 + usbcmd) |= (1 << 4) | 1;

	/* Route all ports to this host controller */
	*(ehci_common.usb2 + configflag) = 1;
}


#if 0
void ehci_shutdown(void)
{
	*(ehci_common.usb2 + usbcmd) &= ~USBCMD_ASE;
	*(ehci_common.usb2 + portsc1) &= PORTSC_PP;
	phy_disableClock();
}
#endif

void ehci_dumpRegisters(FILE *stream)
{
	fprintf(stream, "%18s: %08x", "id", *(ehci_common.usb2 + id));
	fprintf(stream, "%18s: %08x\n", "hwgeneral", *(ehci_common.usb2 + hwgeneral));
	fprintf(stream, "%18s: %08x", "hwhost", *(ehci_common.usb2 + hwhost));
	fprintf(stream, "%18s: %08x\n", "hwdevice", *(ehci_common.usb2 + hwdevice));
	fprintf(stream, "%18s: %08x", "hwtxbuf", *(ehci_common.usb2 + hwtxbuf));
	fprintf(stream, "%18s: %08x\n", "hwrxbuf", *(ehci_common.usb2 + hwrxbuf));
	fprintf(stream, "%18s: %08x", "gptimer0ld", *(ehci_common.usb2 + gptimer0ld));
	fprintf(stream, "%18s: %08x\n", "gptimer0ctrl", *(ehci_common.usb2 + gptimer0ctrl));
	fprintf(stream, "%18s: %08x", "gptimer1ld", *(ehci_common.usb2 + gptimer1ld));
	fprintf(stream, "%18s: %08x\n", "gptimer1ctrl", *(ehci_common.usb2 + gptimer1ctrl));
	fprintf(stream, "%18s: %08x", "sbuscfg", *(ehci_common.usb2 + sbuscfg));
	fprintf(stream, "%18s: %08x\n", "caplength", *(ehci_common.usb2 + caplength));
	fprintf(stream, "%18s: %08x", "hciversion", *(ehci_common.usb2 + hciversion));
	fprintf(stream, "%18s: %08x\n", "hcsparams", *(ehci_common.usb2 + hcsparams));
	fprintf(stream, "%18s: %08x", "hccparams", *(ehci_common.usb2 + hccparams));
	fprintf(stream, "%18s: %08x\n", "dciversion", *(ehci_common.usb2 + dciversion));
	fprintf(stream, "%18s: %08x", "dccparams", *(ehci_common.usb2 + dccparams));
	fprintf(stream, "%18s: %08x\n", "usbcmd", *(ehci_common.usb2 + usbcmd));
	fprintf(stream, "%18s: %08x", "usbsts", *(ehci_common.usb2 + usbsts));
	fprintf(stream, "%18s: %08x\n", "usbintr", *(ehci_common.usb2 + usbintr));
	fprintf(stream, "%18s: %08x", "frindex", *(ehci_common.usb2 + frindex));
	fprintf(stream, "%18s: %08x\n", "periodiclistbase", *(ehci_common.usb2 + periodiclistbase));
	fprintf(stream, "%18s: %08x", "deviceaddr", *(ehci_common.usb2 + deviceaddr));
	fprintf(stream, "%18s: %08x\n", "asynclistaddr", *(ehci_common.usb2 + asynclistaddr));
	fprintf(stream, "%18s: %08x", "endpointlistaddr", *(ehci_common.usb2 + endpointlistaddr));
	fprintf(stream, "%18s: %08x\n", "burstsize", *(ehci_common.usb2 + burstsize));
	fprintf(stream, "%18s: %08x", "txfilltunning", *(ehci_common.usb2 + txfilltunning));
	fprintf(stream, "%18s: %08x\n", "endptnak", *(ehci_common.usb2 + endptnak));
	fprintf(stream, "%18s: %08x", "endptnaken", *(ehci_common.usb2 + endptnaken));
	fprintf(stream, "%18s: %08x\n", "configflag", *(ehci_common.usb2 + configflag));
	fprintf(stream, "%18s: %08x", "portsc1", *(ehci_common.usb2 + portsc1));
	fprintf(stream, "%18s: %08x\n", "otgsc", *(ehci_common.usb2 + otgsc));
	fprintf(stream, "%18s: %08x", "usbmode", *(ehci_common.usb2 + usbmode));
	fprintf(stream, "%18s: %08x\n", "endptsetupstat", *(ehci_common.usb2 + endptsetupstat));
	fprintf(stream, "%18s: %08x", "endptprime", *(ehci_common.usb2 + endptprime));
	fprintf(stream, "%18s: %08x\n", "endptflush", *(ehci_common.usb2 + endptflush));
	fprintf(stream, "%18s: %08x", "endptstat", *(ehci_common.usb2 + endptstat));
	fprintf(stream, "%18s: %08x\n", "endptcomplete", *(ehci_common.usb2 + endptcomplete));
	fprintf(stream, "%18s: %08x", "endptctrl0", *(ehci_common.usb2 + endptctrl0));
	fprintf(stream, "%18s: %08x\n", "endptctrl1", *(ehci_common.usb2 + endptctrl1));
	fprintf(stream, "%18s: %08x", "endptctrl2", *(ehci_common.usb2 + endptctrl2));
	fprintf(stream, "%18s: %08x\n", "endptctrl3", *(ehci_common.usb2 + endptctrl3));
	fprintf(stream, "%18s: %08x", "endptctrl4", *(ehci_common.usb2 + endptctrl4));
	fprintf(stream, "%18s: %08x\n", "endptctrl5", *(ehci_common.usb2 + endptctrl5));
	fprintf(stream, "%18s: %08x", "endptctrl6", *(ehci_common.usb2 + endptctrl6));
	fprintf(stream, "%18s: %08x\n", "endptctrl7", *(ehci_common.usb2 + endptctrl7));
	fprintf(stream, "%18s: %08x", "otg1_ctrl", *(ehci_common.usb2 + otg1_ctrl));
	fprintf(stream, "%18s: %08x\n\n", "otg2_ctrl", *(ehci_common.usb2 + otg2_ctrl));
}


void ehci_dumpQueue(FILE *stream, struct qh *qh)
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







void ehci_activate(struct qh *qh) {
	qh->transfer_overlay.active = 1;
}