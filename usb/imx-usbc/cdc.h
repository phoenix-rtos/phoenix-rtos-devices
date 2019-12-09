/*
 * Phoenix-RTOS
 *
 * cdc - USB Communication Device Class
 *
 * Copyright 2019 Phoenix Systems
 * Author: Hubert Buczynski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _CDC_H_
#define _CDC_H_


#include "usbclient.h"


/* String descriptor */
typedef struct _usbclient_desc_str_t {
	uint8_t	len;
	uint8_t	desc_type;
	uint8_t string[256];
} __attribute__((packed)) usbclient_desc_str_t;


/* CDC Header functional descriptor  */
typedef struct _usbclient_desc_cdc_header_t {
	uint8_t len;
	uint8_t type;
	uint8_t sub_type;
	uint16_t bcd_cdc;
} __attribute__((packed)) usbclient_desc_cdc_header_t;


/* CDC ACM functional descriptor  */
typedef struct _usbclient_desc_cdc_acm_t {
	uint8_t len;
	uint8_t type;
	uint8_t sub_type;
	uint8_t cap;
} __attribute__((packed)) usbclient_desc_cdc_acm_t;


/* CDC Union functional descriptor  */
typedef struct _usbclient_desc_cdc_union_t {
	uint8_t len;
	uint8_t type;
	uint8_t sub_type;
	uint8_t con_int;
	uint8_t sub_con_int;
} __attribute__((packed)) usbclient_desc_cdc_union_t;


/* CDC Call management functional descriptor  */
typedef struct _usbclient_desc_cdc_call_t {
	uint8_t len;
	uint8_t type;
	uint8_t sub_type;
	uint8_t cap;
	uint8_t data_int;
} __attribute__((packed)) usbclient_desc_cdc_call_t;


/* Line Coding Request */
typedef struct _cdc_line_coding_t {
	uint32_t line_speed;
	uint8_t stop_bits;
	uint8_t parity;
	uint8_t len;
} __attribute__((packed)) cdc_line_coding_t;


int cdc_init(void);


int cdc_recv(char *data, unsigned int len);


int cdc_send(const char *data, unsigned int len);


void cdc_destroy(void);


#endif
