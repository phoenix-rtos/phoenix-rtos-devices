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


static void ehci_startAsync(hcd_t *hcd)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;

	*(hcd->base + asynclistaddr) = va2pa((void *)ehci->asyncList->hw);
	*(hcd->base + usbcmd) |= USBCMD_ASE;
	ehci_memDmb();
	while ((*(hcd->base + usbsts) & USBSTS_AS) == 0)
		;
}


static void ehci_stopAsync(hcd_t *hcd)
{
	*(hcd->base + usbcmd) &= ~USBCMD_ASE;
	ehci_memDmb();
	while ((*(hcd->base + usbsts) & USBSTS_AS) != 0)
		;
}


static void ehci_qtdLink(ehci_qtd_t *prev, ehci_qtd_t *next)
{
	prev->hw->next = QTD_PTR(next);
	ehci_memDmb();
}


static void ehci_enqueue(ehci_qh_t *qh, ehci_qtd_t *first, ehci_qtd_t *last)
{
	last->hw->next = QTD_PTR_INVALID;
	last->hw->token |= QTD_IOC;

	/* No qtds linked */
	if (qh->lastQtd == NULL)
		qh->hw->nextQtd = QTD_PTR(first);
	else
		qh->lastQtd->next = QTD_PTR(first);
	ehci_memDmb();

	qh->lastQtd = last->hw;
}


static void ehci_continue(ehci_t *ehci, ehci_qh_t *qh, ehci_qtd_t *last)
{
	mutexLock(ehci->asyncLock);
	if (qh->lastQtd == last->hw) {
		qh->lastQtd = NULL;
		qh->hw->nextQtd = QTD_PTR_INVALID;
	}
	else if ((qh->hw->token & QTD_ACTIVE) == 0 && (qh->hw->current == QTD_PTR(last))) {
		qh->hw->nextQtd = last->hw->next;
	}
	ehci_memDmb();
	mutexUnlock(ehci->asyncLock);
}


static ehci_qtd_t *ehci_qtdAlloc(int pid, size_t maxpacksz, char *data, size_t *size, int datax)
{
	ehci_qtd_t *qtd;
	size_t bytes = 0;
	int i, offs;

	if ((qtd = malloc(sizeof(ehci_qtd_t))) == NULL)
		return NULL;

	if ((qtd->hw = usb_alloc(sizeof(struct qtd))) == NULL)
		return NULL;

	qtd->hw->token = (datax << 31) | (pid << 8) | (EHCI_TRANS_ERRORS << 10) | QTD_ACTIVE;

	qtd->hw->next = QTD_PTR_INVALID;
	qtd->hw->altnext = QTD_PTR_INVALID;

	if (data != NULL) {
		qtd->hw->buf[0] = (uintptr_t)va2pa(data);
		offs = min(EHCI_PAGE_SIZE - QTD_OFFSET(qtd->hw->buf[0]), *size);
		bytes += offs;
		data += offs;

		for (i = 1; i < 5 && bytes != *size; i++) {
			qtd->hw->buf[i] = va2pa(data) & ~0xfff;
			offs = min(*size - bytes, EHCI_PAGE_SIZE);
			/* If the data does not fit one qtd, don't leave a trailing short packet */
			if (i == 4 && bytes + offs < *size)
				offs = (((bytes + offs) / maxpacksz) * maxpacksz) - bytes;

			bytes += offs;
			data += offs;
		}

		qtd->hw->token |= bytes << 16;
		*size -= bytes;
	}

	return qtd;
}


static ehci_qh_t *ehci_qhAlloc(void)
{
	ehci_qh_t *qh;

	if ((qh = malloc(sizeof(ehci_qh_t))) == NULL)
		return NULL;

	if ((qh->hw = usb_alloc(sizeof(struct qh))) == NULL)
		return NULL;

	qh->hw->info[0] = 0;
	qh->hw->info[1] = 0;
	qh->hw->token = 0;
	qh->hw->horizontal = QH_PTR_INVALID;
	qh->hw->current = QTD_PTR_INVALID;
	qh->hw->nextQtd = QTD_PTR_INVALID;
	qh->hw->altnextQtd = QTD_PTR_INVALID;

	qh->next = NULL;
	qh->prev = NULL;
	qh->period = 0;
	qh->uframe = 0;
	qh->phase = 0;
	qh->lastQtd = NULL;

	return qh;
}


static void ehci_qhConf(ehci_qh_t *qh, usb_pipe_t *pipe)
{
	qh->hw->info[0] = pipe->dev->address;
	qh->hw->info[0] |= (pipe->num << 8);
	qh->hw->info[0] |= (pipe->dev->speed << 12);
	qh->hw->info[0] |= (pipe->type == usb_transfer_control) ? QH_DT : 0;
	qh->hw->info[0] |= (pipe->maxPacketLen << 16);

	if (pipe->type == usb_transfer_control && pipe->dev->speed != usb_high_speed)
		qh->hw->info[0] |= QH_CTRL;

	qh->hw->info[0] |= (3 << 28); /* NAK count reload */
	qh->hw->info[1] = 0;

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
}


static void ehci_bandAlloc(ehci_t *ehci, ehci_qh_t *qh)
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
	if ((qh->hw->info[0] & QH_HIGH_SPEED) && qh->period > 1) {
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


static void ehci_qhLinkPeriodic(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	ehci_qh_t *t;
	int i;

	mutexLock(ehci->periodicLock);
	ehci_bandAlloc(ehci, qh);
	qh->hw->info[1] = (qh->uframe != 0xff) ? (1 << qh->uframe) : QH_SMASK;
	qh->hw->info[1] |= QH_CMASK;
	/* TODO: Handle SPLIT transactions */

	t = ehci->periodicNodes[qh->phase];
	while (t != NULL && t->next != NULL && t->next->period >= qh->period)
		t = t->next;

	if (t == NULL || t->period < qh->period) {
		/* New first element */
		qh->next = ehci->periodicNodes[qh->phase];

		for (i = qh->phase; i < EHCI_PERIODIC_SIZE; i += qh->period) {
			ehci->periodicNodes[i] = qh;
			ehci->periodicList[i] = QH_PTR(qh);
		}
	}
	else {
		/* Insert inside */
		qh->next = t->next;
		t->next = qh;
		qh->hw->horizontal = QH_PTR(qh->next);
		t->hw->horizontal = QH_PTR(qh);
	}

	if (qh->next == NULL) {
		/* New last element */
		qh->hw->horizontal |= QH_PTR_INVALID;
	}
	ehci_memDmb();
	mutexUnlock(ehci->periodicLock);
}


static void ehci_qhLinkAsync(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;

	mutexLock(ehci->asyncLock);

	/* Insert after dummy qh */
	qh->next = ehci->asyncList->next;
	qh->prev = ehci->asyncList;

	qh->next->prev = qh;
	ehci->asyncList->next = qh;

	qh->hw->horizontal = ehci->asyncList->hw->horizontal;
	ehci->asyncList->hw->horizontal = QH_PTR(qh);
	ehci_memDmb();

	mutexUnlock(ehci->asyncLock);
}


static void ehci_qtdsFree(ehci_qtd_t **head)
{
	ehci_qtd_t *q;

	while ((q = *head) != NULL) {
		LIST_REMOVE(head, q);
		usb_free((void *)q->hw, sizeof(struct qtd));
		free(q);
	}
}

static void ehci_qtdsDeactivate(ehci_qtd_t *qtds)
{
	ehci_qtd_t *e = qtds;

	if (e != NULL) {
		do {
			e->hw->token &= ~QTD_ACTIVE;
		} while ((e = e->next) != qtds);
	}
	ehci_memDmb();
}


static void ehci_qhUnlinkAsync(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;

	mutexLock(ehci->asyncLock);

	ehci_stopAsync(hcd);
	qh->prev->hw->horizontal = qh->hw->horizontal;
	ehci_startAsync(hcd);
	ehci_memDmb();

	qh->prev->next = qh->next;
	qh->next->prev = qh->prev;

	mutexUnlock(ehci->asyncLock);
}


void ehci_qhUnlinkPeriodic(hcd_t *hcd, ehci_qh_t *qh)
{
	ehci_t *ehci = (ehci_t *)hcd->priv;
	ehci_qh_t *tmp;
	int i;

	mutexLock(ehci->periodicLock);
	/* TODO: do we have to stop the periodic queue? */
	for (i = 0; i < EHCI_PERIODIC_SIZE; i++) {
		/* Count Qhs linked to this periodic index */
		tmp = ehci->periodicNodes[i];

		if (tmp == qh) {
			if (qh->next != NULL) {
				ehci->periodicList[i] = QH_PTR(qh->next);
			}
			else {
				ehci->periodicList[i] = QH_PTR_INVALID;
			}
			ehci->periodicNodes[i] = qh->next;
		}
		else {
			while (tmp != NULL && tmp->next != qh)
				tmp = tmp->next;

			if (tmp != NULL && tmp->next == qh) {
				tmp->next = qh->next;
				if (tmp->next != NULL) {
					tmp->hw->horizontal = QH_PTR(tmp->next);
				}
				else {
					tmp->hw->horizontal = QH_PTR_INVALID;
				}
			}
		}
	}
	ehci_memDmb();
	mutexUnlock(ehci->periodicLock);
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
		if (qtds->hw->token & (QTD_XACT | QTD_BABBLE | QTD_BUFERR))
			error++;

		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	if (error > 0) {
		finished = 1;
		*status = -error;
	}

	/* Finished no error */
	if (!(qtds->prev->hw->token & QTD_ACTIVE) || (qtds->prev->hw->token & QTD_HALTED)) {
		finished = 1;
		*status = t->size - QTD_LEN(qtds->prev->hw->token);
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
			ehci_continue(hcd->priv, qh, qtd->prev);
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


static int ehci_qtdAdd(ehci_qtd_t **list, int token, size_t maxpacksz, char *buf, size_t size, int dt)
{
	ehci_qtd_t *tmp;
	size_t remaining = size;

	do {
		if ((tmp = ehci_qtdAlloc(token, maxpacksz, buf + size - remaining, &remaining, dt)) == NULL) {
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
		if ((qh = ehci_qhAlloc()) == NULL)
			return -ENOMEM;

		ehci_qhConf(qh, pipe);
		pipe->hcdpriv = qh;

		if (t->type == usb_transfer_bulk || t->type == usb_transfer_control)
			ehci_qhLinkAsync(hcd, qh);
		else
			ehci_qhLinkPeriodic(hcd, qh);
	}
	else {
		qh = (ehci_qh_t *)pipe->hcdpriv;

		/* Update fields, which might have been changed */
		if (QH_DEVADDR(qh->hw->info[0]) != pipe->dev->address)
			qh->hw->info[0] = (qh->hw->info[0] & ~0x7f) | pipe->dev->address;

		if (QH_PACKLEN(qh->hw->info[0]) != pipe->maxPacketLen)
			qh->hw->info[0] = (qh->hw->info[0] & ~(0x7ff << 16)) | (pipe->maxPacketLen << 16);
	}

	/* Setup stage */
	if (t->type == usb_transfer_control) {
		if (ehci_qtdAdd(&qtds, setup_token, pipe->maxPacketLen, (char *)t->setup, sizeof(usb_setup_packet_t), 0) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Data stage */
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk ||
		t->type == usb_transfer_interrupt) {
		if (ehci_qtdAdd(&qtds, token, pipe->maxPacketLen, t->buffer, t->size, 1) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Status stage */
	if (t->type == usb_transfer_control) {
		token = (token == in_token) ? out_token : in_token;
		if (ehci_qtdAdd(&qtds, token, pipe->maxPacketLen, NULL, 0, 1) < 0) {
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
		ehci_qtdLink(qtds, qtds->next);
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

	if (pipe->hcdpriv == NULL)
		return;

	qh = (ehci_qh_t *)pipe->hcdpriv;

	if (pipe->type == usb_transfer_bulk || pipe->type == usb_transfer_control)
		ehci_qhUnlinkAsync(hcd, qh);
	else if (pipe->type == usb_transfer_interrupt)
		ehci_qhUnlinkPeriodic(hcd, qh);

	mutexLock(hcd->transLock);
	t = hcd->transfers;
	/* Deactivate device's qtds */
	if (t != NULL) {
		do {
			if (t->pipe == pipe)
				ehci_qtdsDeactivate((ehci_qtd_t *)t->hcdpriv);
			t = t->next;
		} while (t != hcd->transfers);
		ehci_transUpdate(hcd);
	}
	mutexUnlock(hcd->transLock);

	pipe->hcdpriv = NULL;
	usb_free((void *)qh->hw, sizeof(struct qh));
	free(qh);
}


static void ehci_free(ehci_t *ehci)
{
	if (ehci->periodicList != NULL)
		usb_freeAligned(ehci->periodicList, EHCI_PERIODIC_SIZE * sizeof(uint32_t));

	if (ehci->irqCond != 0)
		resourceDestroy(ehci->irqCond);

	if (ehci->irqLock != 0)
		resourceDestroy(ehci->irqLock);

	if (ehci->asyncLock != 0)
		resourceDestroy(ehci->asyncLock);

	free(ehci->periodicNodes);
	free(ehci);
}


static int ehci_init(hcd_t *hcd)
{
	ehci_t *ehci;
	ehci_qh_t *qh;
	int i;

	if ((ehci = calloc(1, sizeof(ehci_t))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		return -ENOMEM;
	}

	if ((ehci->periodicList = usb_allocAligned(EHCI_PERIODIC_SIZE * sizeof(uint32_t), EHCI_PERIODIC_ALIGN)) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	if ((ehci->periodicNodes = calloc(EHCI_PERIODIC_SIZE, sizeof(ehci_qh_t *))) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	hcd->priv = ehci;

	if (phy_init(hcd) != 0) {
		fprintf(stderr, "ehci: Phy init failed!\n");
		ehci_free(ehci);
		return -EINVAL;
	}

	if (condCreate(&ehci->irqCond) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->irqLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->asyncLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	if (mutexCreate(&ehci->periodicLock) < 0) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}

	/* Initialize Async List with a dummy qh to optimize
	 * accesses and make them safer */
	if ((qh = ehci_qhAlloc()) == NULL) {
		fprintf(stderr, "ehci: Out of memory!\n");
		ehci_free(ehci);
		return -ENOMEM;
	}
	qh->hw->info[0] |= QH_HEAD;
	qh->hw->horizontal = QH_PTR(qh);
	LIST_ADD(&ehci->asyncList, qh);

	for (i = 0; i < EHCI_PERIODIC_SIZE; ++i)
		ehci->periodicList[i] = QH_PTR_INVALID;

	if (beginthread(ehci_irqThread, 2, ehci->stack, sizeof(ehci->stack), hcd) != 0) {
		ehci_free(ehci);
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

	ehci_startAsync(hcd);

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
