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


static void ehci_continue(struct qh_node *qh_node, struct qtd *last)
{
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
	struct qtd *qtd = NULL;
	size_t bytes = 0;
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
				offs = (((bytes + offs) / maxpacksz) * maxpacksz) - bytes;

			bytes += offs;
			buffer += offs;
		}

		qtd->bytes_to_transfer = bytes;
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
	struct qtd_node *qtds = (struct qtd_node *)t->hcdpriv;
	int finished;
	int error = 0;

	finished = !qtds->prev->qtd->active || qtds->prev->qtd->halted;
	do {
		if (qtds->qtd->transaction_error)
			error++;

		if (qtds->qtd->babble)
			error++;

		qtds = qtds->next;
	} while (qtds != t->hcdpriv);

	t->finished = finished;
	t->error = error;
	t->transfered = t->size - qtds->prev->qtd->bytes_to_transfer;

	return finished;
}


static void ehci_irqThread(void *arg)
{
	hcd_t *hcd = (hcd_t *)arg;
	ehci_t *ehci = (ehci_t *)hcd->priv;
	usb_transfer_t *t, *n;
	struct qh_node *qh_node;
	struct qtd_node *qtd_node;
	int cont;

	mutexLock(ehci->irq_lock);
	for (;;) {
		condWait(ehci->irq_cond, ehci->irq_lock, 0);
		mutexLock(hcd->transLock);
		if ((t = hcd->transfers) != NULL) {
			do {
				qh_node = (struct qh_node *)t->endpoint->hcdpriv;
				qtd_node = (struct qtd_node *)t->hcdpriv;
				cont = 0;
				n = t->next;
				if (ehci_transferStatus(hcd, t) != 0) {
					/* Transfer finished */
					ehci_continue(qh_node, qtd_node->prev->qtd);
					LIST_REMOVE(&hcd->transfers, t);
					ehci_qtdsFree(&qtd_node);
					t->hcdpriv = NULL;
					usb_transferFinished(t);
					if (n != t)
						cont = 1;
				}
			} while (hcd->transfers && ((t = n) != hcd->transfers || cont));
		}
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
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Data stage */
	if ((t->type == usb_transfer_control && t->size > 0) || t->type == usb_transfer_bulk) {
		if (ehci_addQtd(&qtds, token, ep->max_packet_len, t->buffer, t->size, 1) < 0) {
			ehci_qtdsFree(&qtds);
			t->hcdpriv = NULL;
			return -ENOMEM;
		}
	}

	/* Status stage */
	if (t->type == usb_transfer_control) {
		token = (token == in_token) ? out_token : in_token;
		if (ehci_addQtd(&qtds, token, ep->max_packet_len, NULL, 0, 1) < 0) {
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
	usb_transfer_t *t, *n;
	usb_endpoint_t *ep;

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
	mutexLock(hcd->transLock);
	if ((t = hcd->transfers) != NULL) {
		do {
			if (t->endpoint->device == dev) {
				n = t->next;
				LIST_REMOVE(&hcd->transfers, t);
				ehci_transferStatus(hcd, t);
				ehci_qtdsFree((struct qtd_node **)&t->hcdpriv);
				t->hcdpriv = NULL;
				usb_transferFinished(t);
				if (hcd->transfers != NULL && n != hcd->transfers) {
					t = n;
					continue;
				}
			}

		} while (hcd->transfers && (t = t->next) != hcd->transfers);
	}
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
