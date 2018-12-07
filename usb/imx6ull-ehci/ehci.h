/*
 * Phoenix-RTOS
 *
 * Operating system kernel
 *
 * ehci/ehci.h
 *
 * Copyright 2018 Phoenix Systems
 * Copyright 2007 Pawel Pisarczyk
 * Author: Jan Sikorski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX6ULL_EHCI_H_
#define _IMX6ULL_EHCI_H_


enum { framelist_itd = 0, framelist_qh, framelist_sitd, framelist_fstn };


enum { transfer_control, transfer_interrupt, transfer_bulk, transfer_isochronous };


enum { full_speed = 0, low_speed, high_speed };


struct itd;
struct sitd;
struct qtd;
struct qh;


extern int ehci_qtdRemainingBytes(struct qtd *qtd);


extern int ehci_dequeue(struct qh *qh, struct qtd *first, struct qtd *last);


extern void ehci_enqueue(struct qh *qh, struct qtd *first, struct qtd *last);


extern int ehci_deviceAttached(void);


extern void ehci_resetPort(void);


extern void ehci_init(void *event_callback, handle_t common_lock);


extern void ehci_dumpRegisters(FILE *stream);


extern void ehci_dumpQueue(FILE *stream, struct qh *qh);


extern void ehci_consQtd(struct qtd *qtd, struct qh *qh);


extern void ehci_freeQtd(struct qtd *qtd);


extern void ehci_freeQh(struct qh *qh);


extern void ehci_linkQh(struct qh *qh);


extern void ehci_unlinkQh(struct qh *unlink);


extern void ehci_linkQtd(struct qtd *prev, struct qtd *next);


extern int ehci_qhFinished(struct qh *qh);


extern void ehci_qhSetAddress(struct qh *qh, int address);


extern int ehci_await(int timeout);


extern int ehci_qtdError(struct qtd *qtd);


extern int ehci_qtdFinished(struct qtd *qtd);


extern struct qtd *ehci_allocQtd(int token, char *buffer, size_t *size, int datax);


extern struct qh *ehci_allocQh(int address, int endpoint, int speed, int transfer, int max_packet_len);

#endif
