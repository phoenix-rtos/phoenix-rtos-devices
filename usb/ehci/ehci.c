/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/ehci.c
 *
 * Copyright 2018, 2021 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski, Maciej Purski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/list.h>
#include <sys/interrupt.h>
#include <sys/minmax.h>

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

#include "ehci.h"
#include "mem.h"

#define TRACE(x, ...) /*fprintf(stderr, "ehci: " x "\n", ##__VA_ARGS__);*/
#define TRACE_FAIL(x, ...) fprintf(stderr, "ehci error: " x "\n", ##__VA_ARGS__);
#define FUN_TRACE /*fprintf(stderr, "ehci trace: %s\n", __PRETTY_FUNCTION__);*/

struct qtd_node {
	struct qtd_node *prev, *next;
	struct qtd *qtd;
};


struct qh_node {
	struct qh_node *prev, *next;
	struct qtd *last;
	struct qh *qh;
};


static void ehci_dumpRegisters(hcd_t *hcd, FILE *stream);
static void ehci_dumpQueue(FILE *stream, struct qh *qh, struct qtd_node *qtd);
static void ehci_printPortStatus(hcd_t *hcd);
static void ehci_printStatus(ehci_t *ehci);


static int ehci_linkQtd(struct qtd *prev, struct qtd *next)
{
	FUN_TRACE;
	prev->next.pointer = va2pa(next) >> 5;
	prev->next.type = ehci_item_itd;
	prev->next.terminate = 0;

	return 0;
}


static void ehci_consQtd(struct qtd *qtd, struct qh *qh)
{
	FUN_TRACE;
	link_pointer_t head = qh->transfer_overlay.next;
	qh->transfer_overlay.next.pointer = va2pa(qtd) >> 5;
	qh->transfer_overlay.next.terminate = 0;
	qtd->next = head;

	if (qh->endpoint_speed != usb_high_speed) {
		qtd->split_state = 0;
	}
}


static void ehci_enqueue(struct qh_node *qh_node, struct qtd *first, struct qtd *last)
{
	FUN_TRACE;
	struct qtd *prev_last = qh_node->last;

	//printf("ehci enqueue first: %x last: %x\n", first, last);

	qh_node->last = last;
	last->next.terminate = 1;
	last->ioc = 1;

	prev_last->next.pointer = va2pa(first) >> 5;
	prev_last->next.type = ehci_item_itd;

	asm volatile ("dmb" ::: "memory");

	prev_last->next.terminate = 0;
}


static void ehci_continue(struct qh_node *qh_node, struct qtd *last)
{
	FUN_TRACE;
	struct qh *qh = qh_node->qh;

	if (qh_node->last == last) {
		qh_node->last = &qh->transfer_overlay;
		qh_node->last->next = last->next;
	}
	else if (qh->transfer_overlay.active == 0 && qh->current_qtd.pointer == va2pa(last) >> 5) {
		qh->transfer_overlay.next = last->next;
	}
}


static struct qtd *ehci_allocQtd(int token, size_t maxpacksz, char *buffer, size_t *size, int datax)
{
	FUN_TRACE;
	struct qtd *qtd = NULL;
	size_t bytes;
	int i, offs;

	if ((qtd = ehci_alloc()) == NULL)
		return NULL;

	qtd->data_toggle = datax;
	qtd->pid_code = token;

	qtd->next.terminate = 1;
	qtd->alt_next.terminate = 1;
	qtd->active = 1;
	qtd->error_counter = 3;

	if (buffer != NULL) {
		qtd->offset = (uintptr_t)buffer & (EHCI_PAGE_SIZE - 1);
		qtd->page0 = va2pa(buffer) >> 12;
		offs = min(EHCI_PAGE_SIZE - qtd->offset, *size);
		bytes += offs;
		buffer += offs;

		for (i = 1; i < 5 && bytes != *size; i++) {
			qtd->buffers[i].page = va2pa(buffer) >> 12;
			offs = min(*size - bytes, EHCI_PAGE_SIZE);
			/* If the data does not fit one qtd, don't leave a trailing short packet */
			if (i == 4 && bytes + offs < *size)
				offs = ((bytes + offs) & ~(maxpacksz - 1)) - bytes;

			bytes += offs;
			buffer += offs;
		}

		qtd->bytes_to_transfer = bytes;
		*size -= bytes;
	}

	return qtd;
}


static void ehci_freeQtd(struct qtd *qtd)
{
	FUN_TRACE;
	if (qtd->babble) {
		TRACE("babble!");
	}
	if (qtd->transaction_error) {
		TRACE("transaction error!");
	}
	if (qtd->buffer_error) {
		TRACE("buffer error");
	}

	ehci_free(qtd);
}


static struct qh *ehci_allocQh(struct qh_node *qh_node, usb_endpoint_t *ep)
{
	FUN_TRACE;
	struct qh *qh;

	if ((qh = ehci_alloc()) == NULL)
		return NULL;

	qh->horizontal.terminate = 1;

	qh->device_addr = ep->device->address;
	qh->endpoint_speed = ep->device->speed;
	qh->data_toggle = ep->type == usb_transfer_control ? 1 : 0;

	qh->current_qtd.terminate = 1;
	qh->transfer_overlay.next.terminate = 1;

	qh->max_packet_len = ep->max_packet_len;
	qh->endpoint = ep->number;
	qh->nak_count_reload = 3;

	if (ep->type == usb_transfer_interrupt) {
		qh->interrupt_schedule_mask = 0xff;
		qh->split_completion_mask = 0xff;
	}

	if (ep->type == usb_transfer_control && ep->device->speed != usb_high_speed)
		qh->control_endpoint = 1;

	qh_node->last = &qh->transfer_overlay;
	qh_node->qh = qh;

	return qh;
}


int ehci_qtdError(struct qtd *qtd)
{
	return qtd->transaction_error;
}


int ehci_qtdBabble(struct qtd *qtd)
{
	return qtd->babble;
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


// void ehci_freeQh(struct qh *qh)
// {
// 	FUN_TRACE;

// 	mutexLock(ehci->async_lock);
// 	if (ehci->async_head != NULL && /*hack*/!qh->interrupt_schedule_mask) {
// 		*(hcd->base + usbcmd) |= USBCMD_IAA;

// 		while (!(ehci->status & USBSTS_IAA)) {
// 			TRACE("waiting for IAA %s", (ehci->status & USBSTS_AS) ? "async on" : "async off, dupa");
// 			if (condWait(ehci->aai_cond, ehci->common_lock, 1000000) < 0) {
// 				TRACE("aii timeout");
// 				break;
// 			}
// 		}

// 		TRACE("got IAA");

// 		ehci->status &= ~USBSTS_IAA;
// 	}
// 	mutexUnlock(ehci->async_lock);
// 	ehci_free(qh);
// }


static void ehci_linkQh(hcd_t *hcd, struct qh_node *node)
{
	FUN_TRACE;
	ehci_t *ehci = (ehci_t *)hcd->priv;
	int first = ehci->async_head == NULL;
	struct qh *prev;

	mutexLock(ehci->async_lock);

	LIST_ADD(&ehci->async_head, node);

	node->qh->horizontal.pointer = va2pa(node->next->qh) >> 5;
	node->qh->horizontal.type = ehci_item_qh;
	node->qh->horizontal.terminate = 0;


	prev = node->prev->qh;
	prev->horizontal.pointer = va2pa(node->qh) >> 5;
	prev->horizontal.type = ehci_item_qh;
	prev->horizontal.terminate = 0;
	mutexUnlock(ehci->async_lock);

	if (first) {
		node->qh->head_of_reclamation = 1;

		*(hcd->base + asynclistaddr) = va2pa(node->qh);
		*(hcd->base + usbcmd) |= USBCMD_ASE;
		while ((*(hcd->base + usbsts) & USBSTS_AS) == 0);
	}
}


static void ehci_qtdsFree(struct qtd_node *qtds)
{
	struct qtd_node *e, *n;

	if ((e = qtds) != NULL) {
		do {
			n = e->next;
			ehci_freeQtd(e->qtd);
			free(e);
		}
		while ((e = n) != qtds);
	}
}

static void ehci_qtdsDeactivate(struct qtd_node *qtds)
{
	struct qtd_node *e = qtds;

	if (e != NULL) {
		do {
			e->qtd->active = 0;
		} while ((e = e->next) != qtds);
	}
}


void ehci_unlinkQh(hcd_t *hcd, struct qh_node *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;

	if (qh->qh->head_of_reclamation)
		qh->next->qh->head_of_reclamation = 1;

	if (qh->qh == qh->next->qh) {
		ehci->async_head = NULL;
		*(hcd->base + usbcmd) &= ~USBCMD_ASE;
		while (*(hcd->base + usbsts) & USBSTS_AS);
		*(hcd->base + asynclistaddr) = 1;
	}
	else if (qh == ehci->async_head) {
		ehci->async_head = qh->next;
		*(hcd->base + asynclistaddr) = va2pa(qh->next->qh);
	}

	qh->prev->qh->horizontal = qh->qh->horizontal;
	LIST_REMOVE(&ehci->async_head, qh);
}

static int ehci_irqHandler(unsigned int n, void *data)
{
	hcd_t *hcd = (hcd_t *)data;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	ehci->status = *(hcd->base + usbsts);
	*(hcd->base + usbsts) = ehci->status & 0x1f;

	if (*(hcd->base + portsc1) & PORTSC_PEC) {
		*(hcd->base + portsc1) = PORTSC_PEC;
	}

	return -!(ehci->status & 0x1f);
}

static int ehci_transferStatus(hcd_t *hcd, usb_transfer_t *t)
{
	FUN_TRACE;
	struct qtd_node *qtds = (struct qtd_node *)t->hcdpriv;
	int finished = ehci_qtdFinished(qtds->prev->qtd);
	int error = 0;

	do {
		if (ehci_qtdError(qtds->qtd)) {
			TRACE_FAIL("transaction error");
			error++;
			ehci_dumpQueue(stderr, ((struct qh_node *)t->endpoint->hcdpriv)->qh, t->hcdpriv);
		}

		if (ehci_qtdBabble(qtds->qtd)) {
			TRACE_FAIL("babble");
			error++;
		}

		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	t->finished = finished;
	t->error = error;

	return finished;
}


static void ehci_irqThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	ehci_t *ehci = (ehci_t *)hcd->priv;
	usb_transfer_t *t, *p;
	struct qh_node *qh_node;
	struct qtd_node *qtd_node;
	int res;

	mutexLock(ehci->irq_lock);
	for (;;) {
		condWait(ehci->irq_cond, ehci->irq_lock, 0);
		mutexLock(hcd->transLock);
		if ((t = hcd->transfers) != NULL) {
			do {
				qh_node = (struct qh_node *)t->endpoint->hcdpriv;
				qtd_node = (struct qtd_node *)t->hcdpriv;

				if (ehci_transferStatus(hcd, t)) {
					/* Transfer finished */
					ehci_continue(qh_node, qtd_node->prev->qtd);
					p = t->prev;
					LIST_REMOVE(&hcd->transfers, t);
					ehci_qtdsFree(qtd_node);
					t->hcdpriv = NULL;
					usb_transferFinished(t);
					t = p;
				}
			} while (hcd->transfers && (t = t->next) != hcd->transfers);
		}
		mutexUnlock(hcd->transLock);
	}
}


static int ehci_deviceAttached(hcd_t *hcd)
{
	return *(hcd->base + portsc1) & PORTSC_CCS;
}


static void *ehci_periodicListAlloc(void)
{
	return NULL;
}


static void ehci_activate(struct qh *qh) {
	qh->transfer_overlay.active = 1;
}

static void ehci_transferFree(hcd_t *hcd, usb_transfer_t *t)
{
	/* TODO */
}


static int ehci_addQtd(struct qtd_node **list, int token, size_t maxpacksz, char *buf, size_t size, int dt)
{
	struct qtd_node *tmp;
	size_t remaining = size;

	do {
		if ((tmp = malloc(sizeof(struct qtd_node))) == NULL)
			return -ENOMEM;

		if ((tmp->qtd = ehci_allocQtd(token, maxpacksz, buf + size - remaining, &remaining, dt)) == NULL) {
			free(tmp);
			return -ENOMEM;
		}

		LIST_ADD(list, tmp);
		dt = !dt;
	} while (remaining > 0);

	return 0;
}


static int ehci_transferEnqueue(hcd_t *hcd, usb_transfer_t *t)
{
	FUN_TRACE;
	usb_endpoint_t *ep = t->endpoint;
	struct qh *qh;
	struct qh_node *qh_node;
	struct qtd_node *qtds = NULL;
	int token = t->direction == usb_dir_in ? in_token : out_token;

	if (ep->hcdpriv == NULL) {
		if ((qh_node = malloc(sizeof(struct qh_node))) == NULL)
			return -ENOMEM;

		if ((qh = ehci_allocQh(qh_node, ep)) == NULL) {
			free(qh_node);
			return -ENOMEM;
		}

		qh_node->qh = qh;
		ep->hcdpriv = qh_node;

		ehci_linkQh(hcd, qh_node);
	}
	else {
		qh_node = (struct qh_node *)ep->hcdpriv;
		if (qh_node->qh->device_addr != ep->device->address)
			qh_node->qh->device_addr = ep->device->address;
		if (qh_node->qh->max_packet_len != ep->max_packet_len)
			qh_node->qh->max_packet_len = ep->max_packet_len;
	}

	/* Setup stage */
	if (t->type == usb_transfer_control) {
		if (ehci_addQtd(&qtds, setup_token, ep->max_packet_len, (char *)t->setup, sizeof(usb_setup_packet_t), 0) < 0) {
			ehci_qtdsFree(qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Data stage */
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk) {
		if (ehci_addQtd(&qtds, token, ep->max_packet_len, t->buffer, t->size, 1) < 0) {
			ehci_qtdsFree(qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Status stage */
	if (t->type == usb_transfer_control) {
		token = (token == in_token) ? out_token : in_token;
		if (ehci_addQtd(&qtds, token, ep->max_packet_len, NULL, 0, 1) < 0) {
			ehci_qtdsFree(qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* No qtds allocated */
	if (qtds == NULL)
		return -1;

	t->hcdpriv = qtds;
	do {
		ehci_linkQtd(qtds->qtd, qtds->next->qtd);
		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	qtds = (struct qtd_node *)t->hcdpriv;

	mutexLock(hcd->transLock);
	LIST_ADD(&hcd->transfers, t);
	ehci_enqueue(qh_node, qtds->qtd, qtds->prev->qtd);
	mutexUnlock(hcd->transLock);

	return 0;
}

static void ehci_epDestroy(hcd_t *hcd, usb_endpoint_t *ep)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	struct qh_node *qh = (struct qh_node *)ep->hcdpriv;

	if (qh != NULL) {
		mutexLock(ehci->async_lock);

		while (qh->qh->transfer_overlay.active);

		ehci_unlinkQh(hcd, qh);
		ehci_free(qh->qh);
		free(qh);

		mutexUnlock(ehci->async_lock);
	}
}

static void ehci_devDestroy(hcd_t *hcd, usb_device_t *dev)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	struct qh_node *qh;
	usb_transfer_t *t, *p;
	usb_endpoint_t *ep;
	int i, j;

	/* Deactivate device's qtds */
	if ((t = hcd->transfers) != NULL) {
		mutexLock(hcd->transLock);
		do {
			if (t->endpoint->device == dev)
				ehci_qtdsDeactivate((struct qtd_node *)t->hcdpriv);
		} while ((t = t->next) != hcd->transfers);
		mutexUnlock(hcd->transLock);
	}

	/* Remove endpoints Queue Heads */
	ep = dev->eps;
	do {
		ehci_epDestroy(hcd, ep);
		ep = ep->next;
	} while (ep != dev->eps);

	/* Remove transfers */
	if ((t = hcd->transfers) != NULL) {
		mutexLock(hcd->transLock);
		do {
			if (t->endpoint->device == dev) {
				p = t->prev;
				LIST_REMOVE(&hcd->transfers, t);
				ehci_transferStatus(hcd, t);
				usb_transferFinished(t);
				t = p;
				ehci_qtdsFree(t->hcdpriv);
				t->hcdpriv = NULL;
			}

		} while (hcd->transfers && (t = t->next) != hcd->transfers);
		mutexUnlock(hcd->transLock);
	}
}

static int ehci_init(hcd_t *hcd)
{
	ehci_t *ehci;
	int nports;
	int i;

	if ((ehci = calloc(1, sizeof(ehci_t))) == NULL) {
		fprintf(stderr, "ehci: Can't allocate hcd ehci!\n");
		return -ENOMEM;
	}

	if ((ehci->periodic_list = ehci_allocPage()) == NULL) {
		free(ehci);
		return -ENOMEM;
	}

	hcd->priv = ehci;
	phy_init(hcd);

	nports = *(hcd->base + hcsparams) & 0xf;
	if (ehci_rootHubInit(hcd->roothub, nports) != 0) {
		ehci_freePage(ehci->periodic_list);
		free(ehci);
		return -ENOMEM;
	}

	ehci->async_head = NULL;

	condCreate(&ehci->aai_cond);
	condCreate(&ehci->irq_cond);
	mutexCreate(&ehci->irq_lock);
	mutexCreate(&ehci->async_lock);

	for (i = 0; i < 1024; ++i)
		ehci->periodic_list[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	/* Reset controller */
	*(hcd->base + usbcmd) |= 2;
	while (*(hcd->base + usbcmd) & 2);

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

	beginthread(ehci_irqThread, 2, malloc(0x1000), 0x1000, hcd);
	interrupt(112 + 16, ehci_irqHandler, hcd, ehci->irq_cond, &ehci->irq_handle);


	return 0;
}


static hcd_ops_t ehci_ops = {
	.type = "ehci",
	.init = ehci_init,
	.transferEnqueue = ehci_transferEnqueue,
	.devDestroy = ehci_devDestroy
};


__attribute__ ((constructor)) static void ehci_register(void)
{
	hcd_register(&ehci_ops);
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


static void ehci_dumpRegisters(hcd_t *hcd, FILE *stream)
{
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


static void ehci_dumpQueue(FILE *stream, struct qh *qh, struct qtd_node *qtd)
{
	struct qtd_node *tmp = qtd;

	fprintf(stream, "%18s: %08x\n", "horizontal", qh->horizontal);
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

	fprintf(stream, "%18s: %08x\n", "current", qh->current_qtd);

	fprintf(stream, "%18s: %08x\n", "next", qh->transfer_overlay.next);
	fprintf(stream, "%18s: %08x\n", "alternate", qh->transfer_overlay.alt_next);
	fprintf(stream, "%18s: %08x\n", "pid_code", qh->transfer_overlay.pid_code);
	fprintf(stream, "%18s: %08x\n", "error_counter", qh->transfer_overlay.error_counter);
	fprintf(stream, "%18s: %08x\n", "current_page", qh->transfer_overlay.current_page);
	fprintf(stream, "%18s: %08x\n", "ioc", qh->transfer_overlay.ioc);
	fprintf(stream, "%18s: %08x\n", "bytes_to_transfer", qh->transfer_overlay.bytes_to_transfer);
	fprintf(stream, "%18s: %08x\n", "data_toggle", qh->transfer_overlay.data_toggle);
	fprintf(stream, "%18s: %08x\n", "page0", qh->transfer_overlay.page0);

	do {
		printf("==========QTD===========\n");
		fprintf(stream, "%18s: %08x\n", "ping_state", tmp->qtd->ping_state);
		fprintf(stream, "%18s: %08x\n", "split_state", tmp->qtd->split_state);
		fprintf(stream, "%18s: %08x\n", "missed_uframe", tmp->qtd->missed_uframe);
		fprintf(stream, "%18s: %08x\n", "transaction_error", tmp->qtd->transaction_error);
		fprintf(stream, "%18s: %08x\n", "babble", tmp->qtd->babble);
		fprintf(stream, "%18s: %08x\n", "buffer_error", tmp->qtd->buffer_error);
		fprintf(stream, "%18s: %08x\n", "halted", tmp->qtd->halted);
		fprintf(stream, "%18s: %08x\n", "active", tmp->qtd->active);

		fprintf(stream, "%18s: %08x\n", "next", tmp->qtd->next);
		fprintf(stream, "%18s: %08x\n", "pid_code", tmp->qtd->pid_code);
		fprintf(stream, "%18s: %08x\n", "error_counter", tmp->qtd->error_counter);
		fprintf(stream, "%18s: %08x\n", "current_page", tmp->qtd->current_page);
		fprintf(stream, "%18s: %08x\n", "ioc", tmp->qtd->ioc);
		fprintf(stream, "%18s: %08x\n", "bytes_to_transfer", tmp->qtd->bytes_to_transfer);
		fprintf(stream, "%18s: %08x\n", "data_toggle", tmp->qtd->data_toggle);
		fprintf(stream, "%18s: %08x\n", "page0", tmp->qtd->page0);
		tmp = tmp->next;
	} while (tmp != qtd);
}
