/*
 * Phoenix-RTOS
 *
 * USB Client Header for STM32N6 (DWC2 OTG HS) - Device Mode
 *
 * Copyright 2025 Phoenix Systems
 * Author: Olaf Czerwinski, Radosław Szewczyk
 *
 * %LICENSE%
 */

#ifndef _STM32_CLIENT_H_
#define _STM32_ CLIENT_H_

#include <sys/types.h>
#include <usbclient.h>
#include <stdint.h>
#include <sys/mman.h>

#include "phy.h"


#define USB_BUFFER_SIZE 0x1000
/* MANUAL REGISTER OFFSETS (Byte Addresses) */
/* Global */
#define GRXSTSP    (0x020 / 4)
#define GRXFSIZ    (0x024 / 4)
#define DIEPTXF0   (0x028 / 4)
#define DIEPEMPMSK (0x834 / 4)


// #define OFF_DIEPTXF1 0x104

/* Packet IN - tx registers */
#define DIEPCTL0      (0x900 / 4)
#define DIEPMSK       (0x810 / 4)
#define DIEPINT0      (0x908 / 4)
#define DIEPINTxWrMsk 0x297F
#define DIEPTSIZ0     (0x910 / 4)


/* Packet OUT - rx registers */
#define DOEPCTL0      (0xB00 / 4)
#define DOEPMSK       (0x814 / 4)
#define DOEPINT0      (0xB08 / 4)
#define DOEPINTxWrMsk 0xF17F
#define DOEPTSIZ0     (0xB10 / 4)
#define DTXFSTS0      (0x918 / 4)

#define DAINT    (0x818 / 4)
#define DAINTMSK (0x81C / 4)

/* Endpoint Base */
#define OFF_INEP_BASE 0x900 / 4
#define EP_STRIDE     (0x20 / 4)


/* States */
#define DC_DEFAULT    0
#define DC_ADDRESS    1
#define DC_CONFIGURED 2

/* Operations */
#define DC_OP_NONE      0
#define DC_OP_INIT      1
#define DC_OP_EXIT      2
#define DC_OP_RCV_ENDP0 3
#define DC_OP_RCV_ERR   4

/* Limits */
#define ENDPOINTS_NUMBER 9
#define ENDPOINTS_DIR_NB 2

/* FIFO Offsets */
#define FIFO_BASE_OFF (0x1000 / 4)
#define FIFO_SIZE     (0x1000UL)

/* FIFO RX, TX Configuration */
#define MAX_PCKT_SZ_EPO_TX_WORDS (0x10)   // 16 words
#define RX_FIFO_DEPTH_WORDS      (0x200)  // 512 words - required minumim is 28 words (for max 64B EP0)
#define TX0FSA                   RX_FIFO_DEPTH_WORDS
#define TX0FD                    MAX_PCKT_SZ_EPO_TX_WORDS
#define MAX_PCKT_SZ_EP0_TX_B     MAX_PCKT_SZ_EPO_TX_WORDS * 4

#define DIEPCTL_MPSIZ_Msk 0x7FF

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#undef VM_2_PHYM
#define VM_2_PHYM(addr) ((uint32_t)(addr))

/* Access Macro - 32-bit aligned read/write */
#define REG32(base, off) (*(volatile uint32_t *)((uintptr_t)(base) + (off)))

typedef struct _usb_dc_t {
	volatile uint32_t *base;

	uint32_t status;
	uint32_t dev_addr;
	volatile uint32_t setupstat;

	uint8_t connected;

	void *ctxUser;
	void (*cbEvent)(int, void *);
	int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *);

	handle_t irqCond;
	handle_t irqLock;
	handle_t endp0Cond;
	handle_t endp0Lock;
	handle_t inth;

	volatile int runIrqThread;
	volatile int pending_event;
	volatile int endptFailed;

	volatile uint8_t op;
	usb_setup_packet_t setup;
} usb_dc_t;


typedef struct _usb_buffer_t {
	uint8_t *vBuffer;
	addr_t pBuffer;
	int16_t len;
} usb_buffer_t;

typedef struct _endpt_data_t {
	uint16_t max_pkt_len[ENDPOINTS_DIR_NB];
	uint8_t type[ENDPOINTS_DIR_NB];
	uint8_t data_toggle[ENDPOINTS_DIR_NB];
	usb_buffer_t buf[ENDPOINTS_DIR_NB];
} endpt_data_t;

typedef struct {
	char *setupMem;
	endpt_data_t endpts[ENDPOINTS_NUMBER];
	uint32_t bufforek[ENDPOINTS_DIR_NB];
} usb_common_data_t;


/* Descriptors Manager's functions */

extern int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in);


extern int desc_setup(const usb_setup_packet_t *setup);


extern int desc_classSetup(const usb_setup_packet_t *setup);


/* Controller's functions */
extern int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern int ctrl_hfIrq(void);
extern void ctrl_lfIrq(void);
extern int ctrl_reset(void);
extern int ctrl_ep0SetOnEnumdne(void);
extern int ctrl_execTransfer(int endpt, uint8_t *virtAddr, int nBytes);
void ctrl_setAddress(uint32_t addr);


int usbclient_init(usb_desc_list_t *desList);
int usbclient_send(int endpt, const void *data, unsigned int len);
int usbclient_receive(int endpt, void *data, unsigned int len);
int usbclient_destroy(void);

void usbclient_setUserContext(void *ctxUser);
void usbclient_setEventCallback(void (*cbEvent)(int, void *));
void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *));

#endif
