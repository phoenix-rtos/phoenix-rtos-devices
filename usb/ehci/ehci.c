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

#define EHCI_PERIODIC_SIZE 128


static inline void ehci_memDmb(void)
{
	asm volatile ("dmb" ::: "memory");
}


static int ehci_linkQtd(ehci_qtd_t *prev, ehci_qtd_t *next)
{
	prev->hw->next.pointer = va2pa(next->hw) >> 5;
	prev->hw->next.type = 0;
	prev->hw->next.terminate = 0;

	return 0;
}


static void ehci_enqueue(ehci_qh_t *qh, ehci_qtd_t *first, ehci_qtd_t *last)
{
	volatile struct qtd *prevlast = qh->qtdlast;

	qh->qtdlast = last->hw;
	last->hw->next.terminate = 1;
	last->hw->ioc = 1;

	prevlast->next.pointer = va2pa(first->hw) >> 5;
	prevlast->next.type = 0;
	ehci_memDmb();

	prevlast->next.terminate = 0;
}


static void ehci_continue(ehci_qh_t *qh, ehci_qtd_t *last)
{
	if (qh->qtdlast == last->hw) {
		qh->qtdlast = &qh->hw->transferOverlay;
		qh->qtdlast->next = last->hw->next;
	}
	else if (qh->hw->transferOverlay.active == 0 && qh->hw->currentQtd.pointer == va2pa(last->hw) >> 5) {
		qh->hw->transferOverlay.next = last->hw->next;
	}
}


static ehci_qtd_t *ehci_allocQtd(int token, size_t maxpacksz, char *buffer, size_t *size, int datax)
{
	ehci_qtd_t *qtd;
	size_t bytes = 0;
	int i, offs;

	if ((qtd = malloc(sizeof(ehci_qtd_t))) == NULL)
		return NULL;

	if ((qtd->hw = usb_alloc(sizeof(struct qtd))) == NULL)
		return NULL;

	qtd->hw->dt = datax;
	qtd->hw->pid = token;

	qtd->hw->next.terminate = 1;
	qtd->hw->altNext.terminate = 1;
	qtd->hw->active = 1;
	qtd->hw->errorCounter = 3;

	if (buffer != NULL) {
		qtd->hw->offset = (uintptr_t)va2pa(buffer) & (EHCI_PAGE_SIZE - 1);
		qtd->hw->page0 = va2pa(buffer) >> 12;
		offs = min(EHCI_PAGE_SIZE - qtd->hw->offset, *size);
		bytes += offs;
		buffer += offs;

		for (i = 1; i < 5 && bytes != *size; i++) {
			qtd->hw->buffers[i].page = va2pa(buffer) >> 12;
			offs = min(*size - bytes, EHCI_PAGE_SIZE);
			/* If the data does not fit one qtd, don't leave a trailing short packet */
			if (i == 4 && bytes + offs < *size)
				offs = (((bytes + offs) / maxpacksz) * maxpacksz) - bytes;

			bytes += offs;
			buffer += offs;
		}

		qtd->hw->bytesToTransfer = bytes;
		*size -= bytes;
	}

	return qtd;
}


static ehci_qh_t *ehci_allocQh(usb_pipe_t *pipe)
{
	ehci_qh_t *qh;

	if ((qh = malloc(sizeof(ehci_qh_t))) == NULL)
		return NULL;

	if ((qh->hw = usb_alloc(sizeof(struct qh))) == NULL)
		return NULL;

	qh->hw->horizontal.terminate = 1;
	qh->hw->devAddr = pipe->dev->address;
	qh->hw->epSpeed = pipe->dev->speed;
	qh->hw->dt = pipe->type == usb_transfer_control ? 1 : 0;

	qh->hw->currentQtd.terminate = 1;
	qh->hw->transferOverlay.next.terminate = 1;

	qh->hw->maxPacketLen = pipe->maxPacketLen;
	qh->hw->ep = pipe->num;
	qh->hw->nakCountReload = 3;

	if (pipe->type == usb_transfer_interrupt) {
		if (pipe->dev->speed == usb_high_speed) {
			qh->period = ((1 << (pipe->interval - 1))) >> 3;
			/* Assume, that for 1-8 microframes period, we send it every microframe */
			if (qh->period == 0)
				qh->period = 1;
		}
		else {
			qh->period = 1;
			while (qh->period * 2 < pipe->interval)
				qh->period *= 2;
		}
	}

	if (pipe->type == usb_transfer_control && pipe->dev->speed != usb_high_speed)
		qh->hw->ctrlEp = 1;

	qh->qtdlast = &qh->hw->transferOverlay;

	return qh;
}


static void ehci_allocPeriodicBandwidth(ehci_t *ehci, ehci_qh_t *qh)
{
	ehci_qh_t *tmp;
	unsigned int i, n, best;
	unsigned int ucnt[8] = { 0 };

	best = (unsigned)-1;
	qh->phase = 0;
	qh->uframe = 0xff;

	/* Find the best periodicList index (phase) to begin Qh linking */
	for (i = 0; i < qh->period && i < EHCI_PERIODIC_SIZE; i++) {
		n = 0;
		/* Count Qhs linked to this periodic index */
		tmp = ehci->periodicNodes[i];
		while (tmp != NULL) {
			n++;
			tmp = tmp->next;
		}

		if (n < best) {
			best = n;
			qh->phase = i;
		}
	}

	/* Find the best microframe in a frame. For periods equal to 1, send it every microframe */
	if (qh->hw->epSpeed == usb_high_speed && qh->period > 1) {
		for (tmp = ehci->periodicNodes[qh->phase]; tmp != NULL; tmp = tmp->next) {
			if (tmp->uframe != 0xff)
				ucnt[tmp->uframe]++;
		}

		best = (unsigned)-1;
		qh->uframe = 0;
		for (i = 0; i < 8; i++) {
			if (ucnt[i] < best) {
				qh->uframe = i;
				best = ucnt[i];
			}
		}
	}
}


static void ehci_linkPeriodicQh(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	ehci_qh_t *t;
	int i;

	mutexLock(ehci->periodicLock);
	ehci_allocPeriodicBandwidth(ehci, qh);
	qh->hw->smask = (qh->uframe != 0xff) ? (1 << qh->uframe) : 0xff;
	/* TODO: Handle SPLIT transactions */
	qh->hw->cmask = 0xff;

	t = ehci->periodicNodes[qh->phase];
	while (t != NULL && t->next != NULL && t->next->period >= qh->period)
		t = t->next;

	if (t == NULL || t->period < qh->period) {
		/* New first element */
		qh->next = ehci->periodicNodes[qh->phase];

		for (i = qh->phase; i < EHCI_PERIODIC_SIZE; i += qh->period) {
			ehci->periodicNodes[i] = qh;
			ehci->periodicList[i].pointer = va2pa(qh->hw) >> 5;
			ehci->periodicList[i].type = ehci_item_qh;
			ehci->periodicList[i].terminate = 0;
		}
	}
	else {
		/* Insert inside */
		qh->next = t->next;
		t->next = qh;
		qh->hw->horizontal.pointer = va2pa(qh->next->hw) >> 5;
		qh->hw->horizontal.type = ehci_item_qh;
		qh->hw->horizontal.terminate = 0;

		t->hw->horizontal.pointer = va2pa(qh->hw) >> 5;
		t->hw->horizontal.type = ehci_item_qh;
		t->hw->horizontal.terminate = 0;
	}

	if (qh->next == NULL) {
		/* New last element */
		qh->hw->horizontal.terminate = 1;
	}

	mutexUnlock(ehci->periodicLock);
}


static void ehci_linkAsyncQh(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	int first = (ehci->asyncList == NULL) ? 1 : 0;
	ehci_qh_t *prev;

	mutexLock(ehci->asyncLock);

	LIST_ADD(&ehci->asyncList, qh);

	qh->hw->horizontal.pointer = va2pa(qh->next->hw) >> 5;
	qh->hw->horizontal.type = ehci_item_qh;
	qh->hw->horizontal.terminate = 0;

	prev = qh->prev;
	prev->hw->horizontal.pointer = va2pa(qh->hw) >> 5;
	prev->hw->horizontal.type = ehci_item_qh;
	prev->hw->horizontal.terminate = 0;
	mutexUnlock(ehci->asyncLock);

	if (first) {
		qh->hw->headOfReclamation = 1;

		*(hcd->base + asynclistaddr) = va2pa(qh->hw);
		*(hcd->base + usbcmd) |= USBCMD_ASE;
		while ((*(hcd->base + usbsts) & USBSTS_AS) == 0)
			;
	}
}


static void ehci_qtdsFree(ehci_qtd_t **head)
{
	ehci_qtd_t *q;

	while ((q = *head) != NULL) {
		LIST_REMOVE(head, q);
		usb_free(q->hw, sizeof(struct qtd));
		free(q);
	}
}

static void ehci_qtdsDeactivate(ehci_qtd_t *qtds)
{
	ehci_qtd_t *e = qtds;

	if (e != NULL) {
		do {
			e->hw->active = 0;
		} while ((e = e->next) != qtds);
	}
}


void ehci_unlinkQhAsync(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;

	if (qh->hw->headOfReclamation)
		qh->next->hw->headOfReclamation = 1;

	if (qh->hw == qh->next->hw) {
		ehci->asyncList = NULL;
		*(hcd->base + usbcmd) &= ~USBCMD_ASE;
		while (*(hcd->base + usbsts) & USBSTS_AS)
			;
		*(hcd->base + asynclistaddr) = 1;
	}
	else if (qh == ehci->asyncList) {
		ehci->asyncList = qh->next;
		*(hcd->base + asynclistaddr) = va2pa(qh->next->hw);
	}

	qh->prev->hw->horizontal = qh->hw->horizontal;
	LIST_REMOVE(&ehci->asyncList, qh);
}

void ehci_unlinkQhPeriodic(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	ehci_qh_t *tmp;
	int i;

	/* TODO: do we have to stop the periodic queue? */
	for (i = 0; i < EHCI_PERIODIC_SIZE; i++) {
		/* Count Qhs linked to this periodic index */
		tmp = ehci->periodicNodes[i];

		if (tmp == qh) {
			if (qh->next != NULL) {
				ehci->periodicList[i].pointer = va2pa(qh->next->hw) >> 5;
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
					tmp->hw->horizontal.pointer = va2pa(tmp->next->hw) >> 5;
				}
				else {
					tmp->hw->horizontal.pointer = 0;
					tmp->hw->horizontal.terminate = 1;
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

	if (ehci->status & USBSTS_PCI)
		ehci->portsc = *(hcd->base + portsc1);

	return -!(ehci->status & 0x1f);
}


static int ehci_qtdsCheck(hcd_t *hcd, usb_transfer_t *t, int *status)
{
	ehci_qtd_t *qtds = (ehci_qtd_t *)t->hcdpriv;
	int error = 0;
	int finished = 0;

	*status = 0;
	do {
		if (qtds->hw->transactionError || qtds->hw->babble)
			error++;

		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	if (error > 0) {
		finished = 1;
		*status = -error;
	}

	/* Finished no error */
	if (!qtds->prev->hw->active || qtds->prev->hw->halted) {
		finished = 1;
		*status = t->size - qtds->prev->hw->bytesToTransfer;
	}

	return finished;
}


static void ehci_transUpdate(hcd_t *hcd)
{
	ehci_qh_t *qh;
	ehci_qtd_t *qtd;
	usb_transfer_t *t, *n;
	int cont;
	int status;

	if ((t = hcd->transfers) == NULL)
		return;

	do {
		qh = (ehci_qh_t *)t->pipe->hcdpriv;
		qtd = (ehci_qtd_t *)t->hcdpriv;
		cont = 0;
		n = t->next;

		if (ehci_qtdsCheck(hcd, t, &status)) {
			ehci_continue(qh, qtd->prev);
			ehci_qtdsFree(&qtd);
			LIST_REMOVE(&hcd->transfers, t);
			t->hcdpriv = NULL;
			usb_transferFinished(t, status);
			if (n != t)
				cont = 1;
		}
	} while (hcd->transfers && ((t = n) != hcd->transfers || cont));
}


static void ehci_portStatusChanged(hcd_t *hcd)
{
	usb_dev_t *hub = hcd->roothub;
	uint32_t status;

	status = ehci_getHubStatus(hub);

	if (status != 0 && !usb_transferCheck(hub->statusTransfer)) {
		memcpy(hub->statusTransfer->buffer, &status, sizeof(status));
		usb_transferFinished(hub->statusTransfer, hub->statusTransfer->size);
	}
}


static void ehci_irqThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	mutexLock(ehci->irqLock);
	for (;;) {
		condWait(ehci->irqCond, ehci->irqLock, 0);

		if (ehci->status & (USBSTS_UI | USBSTS_UEI)) {
			mutexLock(hcd->transLock);
			ehci_transUpdate(hcd);
			mutexUnlock(hcd->transLock);
		}

		if (ehci->status & USBSTS_PCI)
			ehci_portStatusChanged(hcd);
	}
}


static int ehci_addQtd(ehci_qtd_t **list, int token, size_t maxpacksz, char *buf, size_t size, int dt)
{
	ehci_qtd_t *tmp;
	size_t remaining = size;

	do {
		if ((tmp = ehci_allocQtd(token, maxpacksz, buf + size - remaining, &remaining, dt)) == NULL) {
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
	usb_pipe_t *pipe = t->pipe;
	ehci_qh_t *qh;
	ehci_qtd_t *qtds = NULL;
	int token = t->direction == usb_dir_in ? in_token : out_token;

	if (usb_isRoothub(t->pipe->dev))
		return ehci_roothubReq(t);

	if (pipe->hcdpriv == NULL) {
		if ((qh = ehci_allocQh(pipe)) == NULL)
			return -ENOMEM;

		pipe->hcdpriv = qh;

		if (t->type == usb_transfer_bulk || t->type == usb_transfer_control)
			ehci_linkAsyncQh(hcd, qh);
		else
			ehci_linkPeriodicQh(hcd, qh);
	}
	else {
		qh = (ehci_qh_t *)pipe->hcdpriv;
		if (qh->hw->devAddr != pipe->dev->address)
			qh->hw->devAddr = pipe->dev->address;
		if (qh->hw->maxPacketLen != pipe->maxPacketLen)
			qh->hw->maxPacketLen = pipe->maxPacketLen;
	}

	/* Setup stage */
	if (t->type == usb_transfer_control) {
		if (ehci_addQtd(&qtds, setup_token, pipe->maxPacketLen, (char *)t->setup, sizeof(usb_setup_packet_t), 0) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Data stage */
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk ||
		t->type == usb_transfer_interrupt) {
		if (ehci_addQtd(&qtds, token, pipe->maxPacketLen, t->buffer, t->size, 1) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Status stage */
	if (t->type == usb_transfer_control) {
		token = (token == in_token) ? out_token : in_token;
		if (ehci_addQtd(&qtds, token, pipe->maxPacketLen, NULL, 0, 1) < 0) {
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
		ehci_linkQtd(qtds, qtds->next);
		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	qtds = (ehci_qtd_t *)t->hcdpriv;

	mutexLock(hcd->transLock);
	LIST_ADD(&hcd->transfers, t);
	ehci_enqueue(qh, qtds, qtds->prev);
	mutexUnlock(hcd->transLock);

	return 0;
}


static void ehci_pipeDestroy(hcd_t *hcd, usb_pipe_t *pipe)
{
	usb_transfer_t *t;
	ehci_qh_t *qh;
	ehci_t *ehci = (ehci_t *)hcd->priv;

	if (pipe->hcdpriv == NULL)
		return;

	qh = (ehci_qh_t *)pipe->hcdpriv;
	mutexLock(hcd->transLock);

	t = hcd->transfers;
	/* Deactivate device's qtds */
	if (t != NULL) {
		do {
			if (t->pipe == pipe) {
				ehci_qtdsDeactivate((ehci_qtd_t *)t->hcdpriv);
			}
			t = t->next;
		} while (t != hcd->transfers);
		ehci_transUpdate(hcd);
	}
	mutexUnlock(hcd->transLock);

	if (pipe->type == usb_transfer_bulk || pipe->type == usb_transfer_control) {
		mutexLock(ehci->asyncLock);
		while (qh->hw->transferOverlay.active)
			;
		ehci_unlinkQhAsync(hcd, qh);
		mutexUnlock(ehci->asyncLock);
	}
	else if (pipe->type == usb_transfer_interrupt) {
		mutexLock(ehci->periodicLock);
		ehci_unlinkQhPeriodic(hcd, qh);
		mutexUnlock(ehci->periodicLock);
	}
	pipe->hcdpriv = NULL;
	usb_free(qh->hw, sizeof(struct qh));
	free(qh);
}


static int ehci_init(hcd_t *hcd)
{
	ehci_t *ehci;
	int i;

	if ((ehci = calloc(1, sizeof(ehci_t))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		return -ENOMEM;
	}

	if ((ehci->periodicList = usb_allocAligned(EHCI_PERIODIC_SIZE * sizeof(link_pointer_t), 4096)) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		free(ehci);
		return -ENOMEM;
	}

	if ((ehci->periodicNodes = calloc(EHCI_PERIODIC_SIZE, sizeof(ehci_qh_t *))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		free(ehci);
		return -ENOMEM;
	}

	hcd->priv = ehci;

	if (phy_init(hcd) != 0) {
		fprintf(stderr, "ehci: Phy init failed!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		free(ehci->periodicNodes);
		free(ehci);
		return -EINVAL;
	}

	ehci->asyncList = NULL;

	if (condCreate(&ehci->irqCond) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->irqLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		resourceDestroy(ehci->irqCond);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->asyncLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		resourceDestroy(ehci->irqCond);
		resourceDestroy(ehci->irqLock);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->periodicLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		resourceDestroy(ehci->irqCond);
		resourceDestroy(ehci->irqLock);
		resourceDestroy(ehci->asyncLock);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}

	for (i = 0; i < EHCI_PERIODIC_SIZE; ++i)
		ehci->periodicList[i] = (link_pointer_t) { .type = 0, .zero = 0, .pointer = 0, .terminate = 1 };

	if (beginthread(ehci_irqThread, 2, ehci->stack, sizeof(ehci->stack), hcd) != 0) {
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(link_pointer_t));
		resourceDestroy(ehci->irqCond);
		resourceDestroy(ehci->irqLock);
		resourceDestroy(ehci->asyncLock);
		free(ehci->periodicNodes);
		free(ehci);
		return -ENOMEM;
	}
	interrupt(hcd->info->irq, ehci_irqHandler, hcd, ehci->irqCond, &ehci->irqHandle);

	/* Reset controller */
	*(hcd->base + usbcmd) |= 2;
	while (*(hcd->base + usbcmd) & 2)
		;

	/* Set host mode */
	*(hcd->base + usbmode) |= 3;

	/* Enable interrupts */
	*(hcd->base + usbintr) = USBSTS_UI | USBSTS_UEI;

	/* Set periodic frame list */
	*(hcd->base + periodiclistbase) = va2pa(ehci->periodicList);

	/* Set interrupts threshold, frame list size - 128 bytes, turn controller on */
	*(hcd->base + usbcmd) |= (1 << 4) | (3 << 2) | 1;

	/* Route all ports to this host controller */
	*(hcd->base + configflag) = 1;

	return 0;
}


static const hcd_ops_t ehci_ops = {
	.type = "ehci",
	.init = ehci_init,
	.transferEnqueue = ehci_transferEnqueue,
	.pipeDestroy = ehci_pipeDestroy,
	.getRoothubStatus = ehci_getHubStatus
};


__attribute__((constructor)) static void ehci_register(void)
{
	hcd_register(&ehci_ops);
}
