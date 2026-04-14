/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * Device-side CDC ACM driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#ifndef _STM32_CLIENT_H_
#define _STM32_CLIENT_H_

#include <sys/types.h>
#include <usbclient.h>
#include <stdint.h>
#include <sys/mman.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <errno.h>

#include "registers.h"

#define EP0_TX_BUFFER_SIZE 0x80U
#define EP0_RX_BUFFER_SIZE 0x40U
#define USB_BUFFER_SIZE    0x1000
#define USB_OTG_TIMEOUT    0xF000000UL

#define USB_SPEED_FS 0x2U
#define USB_SPEED_HS 0x0U

#define DEVICE_SPEED USB_SPEED_FS

/* States */
#define DC_DEFAULT    0U
#define DC_ADDRESS    1U
#define DC_CONFIGURED 2U

/* Operations */
#define DC_OP_NONE      0U
#define DC_OP_INIT      1U
#define DC_OP_EXIT      2U
#define DC_OP_RCV_ENDP0 3U
#define DC_OP_RCV_ERR   4U

/* Limits */
#define ENDPOINTS_NUMBER   9U
#define ENDPOINTS_DIR_NB   2U
#define MAX_PACKET_SIZE_FS 0x40UL
#define MAX_PACKET_SIZE_HS 0x200UL

/* FIFO Offsets */
#define FIFO_BASE_OFF (0x1000 / 4)
#define FIFO_SIZE     (0x1000UL)

/* FIFO RX, TX Configuration */
#define RX_FIFO_DEPTH (0x200UL) /* 512 words - required minumim is 28 words (for max 64B EP0) */
#define TX0FD         (0x40UL)  /* 64 words - EP0 IN */
#define TX1FD         (0x10UL)  /* 16 words - CDC Control Interrupt IN */
#define TX2FD         (0x180UL) /* 384 words - CDC Data Bulk IN */
#define TX0FSA        RX_FIFO_DEPTH
#define TX1FSA        (TX0FSA + TX0FD)
#define TX2FSA        (TX1FSA + TX1FD)

#define DIEPINTxWrMsk     0x297fUL
#define DOEPINTxWrMsk     0xf17fUL
#define DIEPCTL_MPSIZ_Msk 0x7ff
#define GINTSTSWrMsk      0xf8f0fc0aUL

#define IS_IRQ(REG, IRQ) (((REG >> IRQ) & 1U) == 1U)

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

#define DAINTMSK_IN(x)  (1UL << x)
#define DAINTMSK_OUT(x) (1UL << (x + 16UL))


typedef struct
{
	uint32_t status;
	uint32_t total_length;
	uint32_t rem_length;
	uint32_t maxpacket;
	uint16_t is_used;
	uint16_t bInterval;
} usb_endpointtypedef_t;


typedef struct _usb_dc_t {

	void *ctxUser;
	void (*cbEvent)(int, void *);
	int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *);
	int currEvent, prevEvent;

	volatile uint32_t *base;

	uint32_t devAddr;

	uint8_t connected;
	uint8_t enumSpeed;
	uint8_t isPortOpened;

	int threadNum;
	handle_t irqCond;
	handle_t irqLock;
	handle_t endp0Cond;
	handle_t endp0Lock;
	handle_t inth;

	semaphore_t semBulkTx, semBulkRx;

	volatile uint8_t runIrqThread;

	usb_setup_packet_t setupPacket;
	uint8_t ep0buffTX[EP0_TX_BUFFER_SIZE];
	uint8_t ep0buffRX[EP0_RX_BUFFER_SIZE];

	uint8_t deviceState;
	uint8_t ep0State;

	_Atomic uint32_t pending_event;
	_Atomic uint32_t daintClear;
	_Atomic uint32_t irqPendingDIEPINT[ENDPOINTS_NUMBER];
	_Atomic uint32_t irqPendingDOEPINT[ENDPOINTS_NUMBER];
	uint32_t diepmsk;
} usb_dc_t;

typedef struct _usb_buffer_t {
	uint8_t *vBuffer;
	uint16_t size;
	uint8_t *head, *tail;
} usb_buffer_t;


typedef struct usb_xfer_desc {
	uint16_t len;
	uint8_t *head, *tail;
} usb_xfer_desc_t;

/*
 * Endpoint structure for stm32n6 usb
 * Params:
 *  - xfer_buf - pointer to buffer with received / transmit data
 *  - xfer_size - size of the transaction
 *  - xfer_len - length of the buffer
 *  - xfer_count - partial transfer length in case of multipacket transfer
 *  - is_in - (1 - true / 0 - false)  - is it IN endpoint
 */
typedef struct _stm32n6_endpt_data_t {
	/* Current transfer parameters*/
	uint8_t *xfer_buf;
	uint32_t xfer_len;
	uint32_t xfer_count;
	uint32_t xfer_size;
	volatile uint8_t xfer_active;
	volatile uint8_t xfer_failed;
	/* Endpoint static data */
	usb_buffer_t dataBuf;
	uint8_t is_in;
	uint8_t epNum;
	uint8_t type;
	uint32_t maxpacket;
} stm32n6_endpt_data_t;


typedef struct _stm32n6_endpt_t {
	stm32n6_endpt_data_t in;
	stm32n6_endpt_data_t out;
} stm32n6_endpt_t;


typedef struct {
	char *setupMem;
	stm32n6_endpt_t endpts[ENDPOINTS_NUMBER];
} usb_common_data_t;


enum {
	USB_EP_DIR_IN,
	USB_EP_DIR_OUT
};


enum {
	USB_EP_TYPE_CTRL = 0U,
	USB_EP_TYPE_ISOC = 1U,
	USB_EP_TYPE_BULK = 2U,
	USB_EP_TYPE_INTR = 3U
};

/*  Device Status */
#define USBD_STATE_DEFAULT    0x01U
#define USBD_STATE_ADDRESSED  0x02U
#define USBD_STATE_CONFIGURED 0x03U
#define USBD_STATE_SUSPENDED  0x04U
#define USBD_STATE_ERROR      0xFFU


/*  EP0 State */
#define USBD_EP0_IDLE       0x00U
#define USBD_EP0_SETUP      0x01U
#define USBD_EP0_DATA_IN    0x02U
#define USBD_EP0_DATA_OUT   0x03U
#define USBD_EP0_STATUS_IN  0x04U
#define USBD_EP0_STATUS_OUT 0x05U
#define USBD_EP0_STALL      0x06U

/* Received packet status */
#define RX_PKTSTS_GOUT_NAK    0x1U
#define RX_PKTSTS_OUT_RXED    0x2U
#define RX_PKTSTS_OUT_CMPLT   0x3U
#define RX_PKTSTS_SETUP_CMPLT 0x4U
#define RX_PKTSTS_SETUP_RXED  0x6U


void usbclient_buffDestory(void *addrs, uint32_t size);
void *usbclient_allocBuff(uint32_t size);

extern void ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern void ctrl_hifiq_handler(uint32_t *irqStatus);
extern void ctrl_lifiq_handler(uint32_t irqStatus);
extern void ctrl_setAddress(uint32_t addr);

extern void clbc_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern void clbc_reset(void);
extern int clbc_resetUSBSS(void);
extern void clbc_enumdne(void);
extern void clbc_ep0OutStart(void);
extern void clbc_epStall(stm32n6_endpt_t *ep);
extern int clbc_setDevMode(void);
extern void clbc_rxFifoData(void);
extern int clbc_flushTxFifo(uint8_t num);
extern int clbc_flushRxFifo(void);
extern void clbc_sendEpData(uint8_t epNum);
extern void clbc_receiveEpData(uint8_t *dest, uint16_t nBytes);
extern void clbc_epTransmit(uint8_t epNum, uint8_t *dataBuf, uint32_t len);

/* RX on control endpoint */
extern int clbc_ep0Receive(usb_xfer_desc_t *desc);

/* RX on data endpoint*/
extern void clbc_epReceive(uint8_t epNum, uint8_t *dataBuf, uint32_t len);
extern void clbc_epReceiveCont(uint8_t epNum);

extern void _clbc_USBRST(void);
extern void _clbc_ENUMDNE(void);
extern void _clbc_OEPINT(void);
extern void _clbc_IEPINT(void);

extern int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern int desc_setup(const usb_setup_packet_t *setup);
extern int desc_classSetup(const usb_setup_packet_t *setup);
extern void desc_epActivation(stm32n6_endpt_data_t *ep);
extern void desc_epDeactivation(stm32n6_endpt_data_t *ep);


#endif
