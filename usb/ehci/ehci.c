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

#define EHCI_PERIODIC_SIZE 1024

struct qtd_node {
	struct qtd_node *prev, *next;
	struct qtd *qtd;
};


struct qh_node {
	struct qh_node *prev, *next;
	struct qtd *last;
	struct qh *qh;
};


static int ehci_linkQtd(struct qtd *prev, struct qtd *next)
{
	prev->next.pointer = va2pa(next) >> 5;
	prev->next.type = ehci_item_itd;
	prev->next.terminate = 0;

	return 0;
}


static void ehci_enqueue(struct qh_node *qh_node, struct qtd *first, struct qtd *last)
{
	struct qtd *prev_last = qh_node->last;

	qh_node->last = last;
	last->next.terminate = 1;
	last->ioc = 1;

	prev_last->next.pointer = va2pa(first) >> 5;
	prev_last->next.type = ehci_item_itd;

	asm volatile ("dmb" ::: "memory");

	prev_last->next.terminate = 0;
}


// static void ehci_insertPeriodic(struct qh *qh)
// {
// 	link_pointer_t ptr = { 0 };
// 	ptr.pointer = va2pa(qh) >> 5;
// 	ptr.type = ehci_item_fstn;
// 	ehci_common.periodic_list[0] = ptr;
// }


static void ehci_continue(struct qh_node *qh_node, struct qtd *last)
{
	struct qh *qh = qh_node->qh;

	if (qh_node->last == last) {
		qh_node->last = &qh->transferOverlay;
		qh_node->last->next = last->next;
	}
	else if (qh->transferOverlay.active == 0 && qh->currentQtd.pointer == va2pa(last) >> 5) {
		qh->transferOverlay.next = last->next;
	}
}


static struct qtd *ehci_allocQtd(int token, size_t maxpacksz, char *buffer, size_t *size, int datax)
{
	struct qtd *qtd = NULL;
	size_t bytes = 0;
	int i, offs;

	if ((qtd = ehci_alloc()) == NULL)
		return NULL;

	qtd->dt = datax;
	qtd->pid = token;

	qtd->next.terminate = 1;
	qtd->altNext.terminate = 1;
	qtd->active = 1;
	qtd->errorCounter = 3;

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
				offs = (((bytes + offs) / maxpacksz) * maxpacksz) - bytes;

			bytes += offs;
			buffer += offs;
		}

		qtd->bytesToTransfer = bytes;
		*size -= bytes;
	}

	return qtd;
}


static struct qh *ehci_allocQh(struct qh_node *qh_node, usb_endpoint_t *ep)
{
	struct qh *qh;

	if ((qh = ehci_alloc()) == NULL)
		return NULL;

	qh->horizontal.terminate = 1;

	qh->devAddr = ep->device->address;
	qh->epSpeed = ep->device->speed;
	qh->dt = ep->type == usb_transfer_control ? 1 : 0;

	qh->currentQtd.terminate = 1;
	qh->transferOverlay.next.terminate = 1;

	qh->maxPacketLen = ep->maxPacketLen;
	qh->ep = ep->number;
	qh->nakCountReload = 3;

	if (ep->type == usb_transfer_interrupt) {
		if (ep->device->speed == usb_high_speed) {

		} else {
			qh->smask = 0xff;
		}
		/* TODO: handle SPLIT transactions */
		qh->cmask = 0xff;
	}

	if (ep->type == usb_transfer_control && ep->device->speed != usb_high_speed)
		qh->ctrlEp = 1;

	qh_node->last = &qh->transferOverlay;
	qh_node->qh = qh;

	return qh;
}

// void ehci_freeQh(struct qh *qh)
// {
// 	FUN_TRACE;

// 	mutexLock(ehci->async_lock);
// 	if (ehci->asyncList != NULL && /*hack*/!qh->interrupt_schedule_mask) {
// 		*(hcd->base + usbcmd) |= USBCMD_IAA;

// 		while (!(ehci->status & USBSTS_IAA)) {
// 			TRACE("waiting for IAA %s", (ehci->status & USBSTS_AS) ? "async on" : "async off, dupa");
// 			if (condWait(ehci->aai_cond, ehci->commonLock, 1000000) < 0) {
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
	ehci_t *ehci = (ehci_t *)hcd->priv;
	int first = ehci->asyncList == NULL;
	struct qh *prev;

	mutexLock(ehci->asyncLock);

	LIST_ADD(&ehci->asyncList, node);

	node->qh->horizontal.pointer = va2pa(node->next->qh) >> 5;
	node->qh->horizontal.type = ehci_item_qh;
	node->qh->horizontal.terminate = 0;

	prev = node->prev->qh;
	prev->horizontal.pointer = va2pa(node->qh) >> 5;
	prev->horizontal.type = ehci_item_qh;
	prev->horizontal.terminate = 0;
	mutexUnlock(ehci->asyncLock);

	if (first) {
		node->qh->headOfReclamation = 1;

		*(hcd->base + asynclistaddr) = va2pa(node->qh);
		*(hcd->base + usbcmd) |= USBCMD_ASE;
		while ((*(hcd->base + usbsts) & USBSTS_AS) == 0);
	}
}


static void ehci_qtdsFree(struct qtd_node **head)
{
	struct qtd_node *q;

	while ((q = *head) != NULL) {
		LIST_REMOVE(head, q);
		ehci_free(q->qtd);
		free(q);
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

	if (qh->qh->headOfReclamation)
		qh->next->qh->headOfReclamation = 1;

	if (qh->qh == qh->next->qh) {
		ehci->asyncList = NULL;
		*(hcd->base + usbcmd) &= ~USBCMD_ASE;
		while (*(hcd->base + usbsts) & USBSTS_AS);
		*(hcd->base + asynclistaddr) = 1;
	}
	else if (qh == ehci->asyncList) {
		ehci->asyncList = qh->next;
		*(hcd->base + asynclistaddr) = va2pa(qh->next->qh);
	}

	qh->prev->qh->horizontal = qh->qh->horizontal;
	LIST_REMOVE(&ehci->asyncList, qh);
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
	struct qtd_node *qtds = (struct qtd_node *)t->hcdpriv;
	int finished;
	int error = 0;

	finished = !qtds->prev->qtd->active || qtds->prev->qtd->halted;
	do {
		if (qtds->qtd->transactionError)
			error++;

		if (qtds->qtd->babble)
			error++;

		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	t->finished = finished;
	t->error = error;
	/* TODO: fixme */
	t->transfered = t->size - qtds->prev->qtd->bytesToTransfer;

	return finished;
}


static void ehci_updateTransfersStatus(hcd_t *hcd, usb_device_t *dev)
{
	struct qh_node *qh;
	struct qtd_node *qtd;
	usb_transfer_t *t, *n;
	int cont;

	if ((t = hcd->transfers) == NULL)
		return;

	do {
		qh = (struct qh_node *)t->ep->hcdpriv;
		qtd = (struct qtd_node *)t->hcdpriv;
		cont = 0;
		n = t->next;
		if (ehci_transferStatus(hcd, t) != 0) {
			/* Transfer finished */
			ehci_continue(qh, qtd->prev->qtd);
			if (t->type == usb_transfer_bulk || t->type == usb_transfer_control) {
				LIST_REMOVE(&hcd->transfers, t);
				ehci_qtdsFree(&qtd);
				t->hcdpriv = NULL;
			}

			usb_transferFinished(t);
			if (n != t)
				cont = 1;
		}
	} while (hcd->transfers && ((t = n) != hcd->transfers || cont));
}


static void ehci_irqThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	mutexLock(ehci->irqLock);
	for (;;) {
		condWait(ehci->irqCond, ehci->irqLock, 0);
		mutexLock(hcd->transLock);
		ehci_updateTransfersStatus(hcd, NULL);
		mutexUnlock(hcd->transLock);
	}
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
	usb_endpoint_t *ep = t->ep;
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
		if (qh_node->qh->devAddr != ep->device->address)
			qh_node->qh->devAddr = ep->device->address;
		if (qh_node->qh->maxPacketLen != ep->maxPacketLen)
			qh_node->qh->maxPacketLen = ep->maxPacketLen;
	}

	/* Setup stage */
	if (t->type == usb_transfer_control) {
		if (ehci_addQtd(&qtds, setup_token, ep->maxPacketLen, (char *)t->setup, sizeof(usb_setup_packet_t), 0) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Data stage */
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk) {
		if (ehci_addQtd(&qtds, token, ep->maxPacketLen, t->buffer, t->size, 1) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Status stage */
	if (t->type == usb_transfer_control) {
		token = (token == in_token) ? out_token : in_token;
		if (ehci_addQtd(&qtds, token, ep->maxPacketLen, NULL, 0, 1) < 0) {
			ehci_qtdsFree(&qtds);
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
		mutexLock(ehci->asyncLock);

		while (qh->qh->transferOverlay.active);

		ehci_unlinkQh(hcd, qh);
		ehci_free(qh->qh);
		free(qh);

		mutexUnlock(ehci->asyncLock);
	}
}


static void ehci_devDestroy(hcd_t *hcd, usb_device_t *dev)
{
	usb_transfer_t *t;
	usb_endpoint_t *ep;

	/* Deactivate device's qtds */
	if ((t = hcd->transfers) != NULL) {
		mutexLock(hcd->transLock);
		do {
			if (t->ep->device == dev)
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
	mutexLock(hcd->transLock);
	ehci_updateTransfersStatus(hcd, dev);
	mutexUnlock(hcd->transLock);
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

	if ((ehci->periodicList = ehci_allocPage()) == NULL) {
		free(ehci);
		return -ENOMEM;
	}

	hcd->priv = ehci;
	phy_init(hcd);

	nports = *(hcd->base + hcsparams) & 0xf;
	if (ehci_rootHubInit(hcd->roothub, nports) != 0) {
		ehci_freePage(ehci->periodicList);
		free(ehci);
		return -ENOMEM;
	}

	ehci->asyncList = NULL;

	condCreate(&ehci->aaiCond);
	condCreate(&ehci->irqCond);
	mutexCreate(&ehci->irqLock);
	mutexCreate(&ehci->asyncLock);

	for (i = 0; i < EHCI_PERIODIC_SIZE; ++i)
		ehci->periodicList[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	/* Reset controller */
	*(hcd->base + usbcmd) |= 2;
	while (*(hcd->base + usbcmd) & 2);

	/* Set host mode */
	*(hcd->base + usbmode) |= 3;

	/* Enable interrupts */
	*(hcd->base + usbintr) = 0x1f & ~(1 << 3);

	/* Set periodic frame list */
	*(hcd->base + periodiclistbase) = va2pa(ehci->periodicList);

	/* Set interrupts threshold, frame list size, turn controller on */
	*(hcd->base + usbcmd) |= (1 << 4) | 1;

	/* Route all ports to this host controller */
	*(hcd->base + configflag) = 1;

	beginthread(ehci_irqThread, 2, malloc(0x1000), 0x1000, hcd);
	interrupt(112 + 16, ehci_irqHandler, hcd, ehci->irqCond, &ehci->irqHandle);


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
