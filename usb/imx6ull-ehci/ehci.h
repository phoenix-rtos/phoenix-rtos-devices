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


extern void ehci_unlinkQh(struct qh *prev, struct qh *unlink, struct qh *next);


extern void ehci_linkQtd(struct qtd *prev, struct qtd *next);


extern int ehci_qhFinished(struct qh *qh);


extern int ehci_await(int timeout);


extern int ehci_qtdError(struct qtd *qtd);


extern int ehci_qtdFinished(struct qtd *qtd);


extern struct qtd *ehci_allocQtd(int token, char *buffer, size_t *size, int datax);


extern struct qh *ehci_allocQh(int address, int endpoint, int speed, int transfer, int max_packet_len);

#endif
