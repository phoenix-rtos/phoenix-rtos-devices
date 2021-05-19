/*
 * Phoenix-RTOS
 *
 * client - usb client
 *
 * Copyright 2019-2021 Phoenix Systems
 * Author: Kamil Amanowicz, Hubert Buczynski, Gerard Swiderski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _IMX_CLIENT_H_
#define _IMX_CLIENT_H_

#include <sys/types.h>

#include <usbclient.h>


#define USB_BUFFER_SIZE 0x1000

#define ENDPOINTS_NUMBER 8
#define ENDPOINTS_DIR_NB 2

#define MIN(X, Y)           (((X) < (Y)) ? (X) : (Y))
#define VM_2_PHYM(addr)     ((((uint32_t)va2pa(addr)) & ~0xfff) + ((uint32_t)addr & 0xfff))

#define DTD_SIZE(dtd)       ((dtd->dtd_token >> 16) & 0x7fff)
#define DTD_ERROR(dtd)      (dtd->dtd_token & (0xd << 3))
#define DTD_ACTIVE(dtd)     (dtd->dtd_token & (1 << 7))


/* device controller structures */

/* data transfer descriptor */
typedef struct _dtd_t {
	uint32_t dtd_next;
	uint32_t dtd_token;
	uint32_t buff_ptr[5];
	uint8_t	padding[4];
} __attribute__((packed)) dtd_t;


/* endpoint queue head */
typedef struct _dqh_t {
	uint32_t caps;
	uint32_t dtd_current;

	/* overlay area for dtd */
	uint32_t dtd_next;
	uint32_t dtd_token;
	uint32_t buff_ptr[5];

	uint32_t reserved;

	/* setup packet buffer */
	uint32_t setup_buff[2];

	/* head and tail for dtd list */
	dtd_t *head;
	dtd_t *tail;
	uint32_t base;
	uint32_t size;
} __attribute__((packed)) dqh_t;


/* dcd structures */

/* dc states */
enum {
	DC_POWERED,
	DC_ATTACHED,
	DC_DEFAULT,
	DC_ADDRESS,
	DC_CONFIGURED
};


/* dc operation */
enum {
	DC_OP_NONE,
	DC_OP_RCV_ENDP0,
	DC_OP_RCV_ERR,
	DC_OP_EXIT,
	DC_OP_INIT,
	DC_OP_INIT_ERR,
};


typedef struct _usb_dc_t {
	volatile uint32_t *base;
	void *dtdMem;
	volatile dqh_t *endptqh;
	uint32_t status;
	uint32_t dev_addr;
	uint8_t connected;

	void *ctxUser;
	void (*cbEvent)(int, void *);
	int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *);

	handle_t irqCond;
	handle_t irqLock;

	handle_t endp0Cond;
	handle_t endp0Lock;

	handle_t inth;
	volatile uint8_t op;
	usb_setup_packet_t setup;

	volatile int endptFailed;
	volatile int runIrqThread;
	volatile uint32_t setupstat;
} usb_dc_t;


/* usb spec related stuff */
typedef struct _endpt_caps_t {
	uint8_t  mult;
	uint8_t  zlt;
	uint16_t max_pkt_len;
	uint8_t  ios;
	uint8_t init;
} endpt_caps_t;


typedef struct _endpt_ctrl_t {
	uint8_t type;
	uint8_t data_toggle;
	uint8_t data_inhibit;
	uint8_t stall;
} endpt_ctrl_t;


typedef struct _usb_buffer_t {
	uint8_t *vBuffer;
	addr_t pBuffer;

	int16_t len;
} usb_buffer_t;


typedef struct _endpt_data_t {
	endpt_caps_t caps[ENDPOINTS_DIR_NB];
	endpt_ctrl_t ctrl[ENDPOINTS_DIR_NB];

	usb_buffer_t buf[ENDPOINTS_DIR_NB];
} endpt_data_t;


typedef struct{
	char *setupMem;
	endpt_data_t endpts[ENDPOINTS_NUMBER];
} usb_common_data_t;


/* Descriptors Manager's functions */

extern int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in);


extern int desc_classSetup(const usb_setup_packet_t *setup);


extern int desc_setup(const usb_setup_packet_t *setup);



/* Controller's functions */

extern int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);


extern void ctrl_setAddress(uint32_t addr);


extern void ctrl_initQtdMem(void);


extern void ctrl_initQtd(void);


extern int ctrl_endptInit(int endpt, endpt_data_t *ctrl_endptInit);


extern int ctrl_lfIrq(void);


extern int ctrl_hfIrq(void);


extern dtd_t *ctrl_execTransfer(int endpt, uint32_t paddr, uint32_t sz, int dir);


extern void ctrl_reset(void);


#endif /* _IMX_CLIENT_H_ */
