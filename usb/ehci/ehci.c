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
#include "ehci-mem.h"

#define EHCI_PERIODIC_SIZE 128

struct qtd_node {
	struct qtd_node *prev, *next;
	struct qtd *qtd;
};


struct qh_node {
	struct qh_node *prev, *next;
	struct qtd *last;
	struct qh *qh;
	unsigned period; /* [ms], interrupt transfer only */
	unsigned phase; /* [ms], interrupt transfer only */
	unsigned uframe; /* interrupt transfer and high-speed only */
};


static int ehci_linkQtd(struct qtd *prev, struct qtd *next)
{
	prev->next.pointer = va2pa(next) >> 5;
	prev->next.type = 0;
	prev->next.terminate = 0;

	return 0;
}


static void ehci_enqueue(struct qh_node *qhnode, struct qtd *first, struct qtd *last)
{
	struct qtd *prevlast = qhnode->last;

	qhnode->last = last;
	last->next.terminate = 1;
	last->ioc = 1;

	prevlast->next.pointer = va2pa(first) >> 5;
	prevlast->next.type = 0;

	asm volatile ("dmb" ::: "memory");

	prevlast->next.terminate = 0;
}


static void ehci_continue(struct qh_node *qhnode, struct qtd *last)
{
	struct qh *qh = qhnode->qh;

	if (qhnode->last == last) {
		qhnode->last = &qh->transferOverlay;
		qhnode->last->next = last->next;
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
		qtd->offset = (uintptr_t)va2pa(buffer) & (EHCI_PAGE_SIZE - 1);
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


static struct qh *ehci_allocQh(struct qh_node *node, usb_endpoint_t *ep)
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
		if (node->qh->epSpeed == usb_high_speed) {
			node->period = ((1 << (ep->interval - 1))) >> 3;
			/* Assume, that for 1-8 microframes period, we send it every microframe */
			if (node->period == 0)
				node->period = 1;
		}
		else {
			node->period = 1;
			while (node->period * 2 < ep->interval)
				node->period *= 2;
		}
		fprintf(stderr, "ehci: allocQh period: %d\n", node->period);
	}

	if (ep->type == usb_transfer_control && ep->device->speed != usb_high_speed)
		qh->ctrlEp = 1;

	node->last = &qh->transferOverlay;
	node->qh = qh;

	return qh;
}


static void ehci_allocPeriodicBandwidth(ehci_t *ehci, struct qh_node *node)
{
	struct qh_node *tmp;
	unsigned int i, n, best;
	unsigned int ucnt[8] = { 0 };

	best = (unsigned)-1;
	node->phase = 0;
	node->uframe = 0xff;

	/* Find the best periodicList index (phase) to begin Qh linking */
	for (i = 0; i < node->period && i < EHCI_PERIODIC_SIZE; i++) {
		n = 0;
		/* Count Qhs linked to this periodic index */
		tmp = ehci->periodicNodes[i];
		while (tmp != NULL) {
			n++;
			tmp = tmp->next;
		}

		if (n < best) {
			best = n;
			node->phase = i;
		}
	}
	fprintf(stderr, "ehci: allocPeriodicBandwidth period: %d phase: %d\n", node->period, node->phase);

	/* Find the best microframe in a frame. For periods equal to 1, send it every microframe */
	if (node->qh->epSpeed == usb_high_speed && node->period > 1) {
		tmp = ehci->periodicNodes[node->phase];
		while (tmp != NULL) {
			if (tmp->uframe != 0xff)
				ucnt[tmp->uframe]++;
			tmp = tmp->next;
		}

		best = (unsigned)-1;
		node->uframe = 0;
		for (i = 0; i < 8; i++) {
			if (ucnt[i] < best) {
				node->uframe = i;
				best = ucnt[i];
			}
		}
	}
}


static void ehci_linkPeriodicQh(hcd_t *hcd, struct qh_node *node)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	struct qh_node *t;
	int i;

	mutexLock(ehci->periodicLock);
	ehci_allocPeriodicBandwidth(ehci, node);
	node->qh->smask = (node->uframe != 0xff) ? (1 << node->uframe) : 0xff;
	/* TODO: Handle SPLIT transactions */
	node->qh->cmask = 0xff;

	t = ehci->periodicNodes[node->phase];
	while (t && t->next != NULL && t->next->period >= node->period)
		t = t->next;

	if (t == NULL || t->period < node->period) {
		/* New first element */
		node->next = ehci->periodicNodes[node->phase];

		for (i = node->phase; i < EHCI_PERIODIC_SIZE; i += node->period) {
			ehci->periodicNodes[i] = node;
			ehci->periodicList[i].pointer = va2pa(node->qh) >> 5;
			ehci->periodicList[i].type = ehci_item_qh;
			ehci->periodicList[i].terminate = 0;
		}
	}
	else {
		/* Insert inside */
		node->next = t->next;
		t->next = node;
		node->qh->horizontal.pointer = va2pa(node->next->qh) >> 5;
		node->qh->horizontal.type = ehci_item_qh;
		node->qh->horizontal.terminate = 0;

		t->qh->horizontal.pointer = va2pa(node->qh) >> 5;
		t->qh->horizontal.type = ehci_item_qh;
		t->qh->horizontal.terminate = 0;
	}

	if (node->next == NULL) {
		/* New last element */
		node->qh->horizontal.terminate = 1;
	}

	mutexUnlock(ehci->periodicLock);
}


static void ehci_linkAsyncQh(hcd_t *hcd, struct qh_node *node)
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


void ehci_unlinkQhAsync(hcd_t *hcd, struct qh_node *qh)
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

void ehci_unlinkQhPeriodic(hcd_t *hcd, struct qh_node *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	struct qh_node *tmp;
	int i;

	/* TODO: do we have to stop the periodic queue? */
	for (i = 0; i < EHCI_PERIODIC_SIZE; i++) {
		/* Count Qhs linked to this periodic index */
		tmp = ehci->periodicNodes[i];

		if (tmp == qh) {
			if (qh->next != NULL) {
				ehci->periodicList[i].pointer = va2pa(qh->next->qh) >> 5;
			}
			else {
				ehci->periodicList[i].pointer = 0;
				ehci->periodicList[i].terminate = 1;
			}
			ehci->periodicNodes[i] = qh->next;
		}
		else {
			while (tmp != NULL && tmp->next != qh)
				tmp = tmp->next;

			if (tmp != NULL && tmp->next == qh) {
				tmp->next = qh->next;
				if (tmp->next != NULL) {
					tmp->qh->horizontal.pointer = va2pa(tmp->next->qh) >> 5;
				}
				else {
					tmp->qh->horizontal.pointer = 0;
					tmp->qh->horizontal.terminate = 1;
				}
			}
		}
	}
}


static int ehci_irqHandler(unsigned int n, void *data)
{
	hcd_t *hcd = (hcd_t *)data;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	ehci->status = *(hcd->base + usbsts);
	*(hcd->base + usbsts) = ehci->status & 0x1f;

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
	t->transferred = t->size - qtds->prev->qtd->bytesToTransfer;

	return finished;
}


static void ehci_updateTransfersStatus(hcd_t *hcd, usb_dev_t *dev)
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
			LIST_REMOVE(&hcd->transfers, t);
			ehci_qtdsFree(&qtd);
			t->hcdpriv = NULL;

			t->handler(t);
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
	struct qh_node *node;
	struct qtd_node *qtds = NULL;
	int token = t->direction == usb_dir_in ? in_token : out_token;

	if (t->ep->device == hcd->roothub)
		return ehci_roothubReq(t);

	if (ep->hcdpriv == NULL) {
		if ((node = malloc(sizeof(struct qh_node))) == NULL)
			return -ENOMEM;

		if ((qh = ehci_allocQh(node, ep)) == NULL) {
			free(node);
			return -ENOMEM;
		}

		node->qh = qh;
		ep->hcdpriv = node;

		if (t->type == usb_transfer_bulk || t->type == usb_transfer_control)
			ehci_linkAsyncQh(hcd, node);
		else
			ehci_linkPeriodicQh(hcd, node);
	}
	else {
		node = (struct qh_node *)ep->hcdpriv;
		if (node->qh->devAddr != ep->device->address)
			node->qh->devAddr = ep->device->address;
		if (node->qh->maxPacketLen != ep->maxPacketLen)
			node->qh->maxPacketLen = ep->maxPacketLen;
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
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk ||
		t->type == usb_transfer_interrupt) {
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
	ehci_enqueue(node, qtds->qtd, qtds->prev->qtd);
	mutexUnlock(hcd->transLock);

	return 0;
}


static void ehci_epDestroy(hcd_t *hcd, usb_endpoint_t *ep)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	struct qh_node *qh = (struct qh_node *)ep->hcdpriv;

	if (ep->type == usb_transfer_bulk || ep->type == usb_transfer_control) {
		mutexLock(ehci->asyncLock);
		while (qh->qh->transferOverlay.active);
		ehci_unlinkQhAsync(hcd, qh);
		mutexUnlock(ehci->asyncLock);
	}
	else if (ep->type == usb_transfer_interrupt) {
		mutexLock(ehci->periodicLock);
		ehci_unlinkQhPeriodic(hcd, qh);
		mutexUnlock(ehci->periodicLock);
	}

	ehci_free(qh->qh);
	free(qh);
}


static void ehci_devDestroy(hcd_t *hcd, usb_dev_t *dev)
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
		if (ep->hcdpriv != NULL)
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
	int i;

	if ((ehci = calloc(1, sizeof(ehci_t))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		return -ENOMEM;
	}

	/* TODO: allocate less memory */
	if ((ehci->periodicList = ehci_allocPage()) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		free(ehci);
		return -ENOMEM;
	}

	if ((ehci->periodicNodes = calloc(EHCI_PERIODIC_SIZE, sizeof(struct qh_node *))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_freePage(ehci->periodicList);
		free(ehci);
		return -ENOMEM;
	}

	hcd->priv = ehci;

	phy_init(hcd);
	ehci->asyncList = NULL;

	if (condCreate(&ehci->irqCond) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_freePage(ehci->periodicList);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->irqLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_freePage(ehci->periodicList);
		resourceDestroy(ehci->irqCond);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->asyncLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_freePage(ehci->periodicList);
		resourceDestroy(ehci->irqCond);
		resourceDestroy(ehci->irqLock);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->periodicLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_freePage(ehci->periodicList);
		resourceDestroy(ehci->irqCond);
		resourceDestroy(ehci->irqLock);
		resourceDestroy(ehci->asyncLock);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}


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

	/* Set interrupts threshold, frame list size - 128 bytes, turn controller on */
	*(hcd->base + usbcmd) |= (1 << 4) | (3 << 2) | 1;

	/* Route all ports to this host controller */
	*(hcd->base + configflag) = 1;

	beginthread(ehci_irqThread, 2, malloc(1024), 1024, hcd);
	interrupt(hcd->info->irq, ehci_irqHandler, hcd, ehci->irqCond, &ehci->irqHandle);

	return 0;
}


static const hcd_ops_t ehci_ops = {
	.type = "ehci",
	.init = ehci_init,
	.transferEnqueue = ehci_transferEnqueue,
	.devDestroy = ehci_devDestroy,
	.getRoothubStatus = ehci_getHubStatus
};


__attribute__ ((constructor)) static void ehci_register(void)
{
	hcd_register(&ehci_ops);
}
