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
#define _STM32_CLIENT_H_

#include <sys/types.h>
#include <usbclient.h>
#include <stdint.h>
#include <sys/mman.h>

#include "phy.h"
#include "registers.h"


#define USB_BUFFER_SIZE 0x1000

// #define OFF_DIEPTXF1 0x104

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
#define MAX_PCKT_SZ_EPO_TX_WORDS (0x40)   // 64 words
#define RX_FIFO_DEPTH_WORDS      (0x200)  // 512 words - required minumim is 28 words (for max 64B EP0)
#define TX0FSA                   RX_FIFO_DEPTH_WORDS
#define TX0FD                    MAX_PCKT_SZ_EPO_TX_WORDS
#define MAX_PCKT_SZ_EP0_TX_B     MAX_PCKT_SZ_EPO_TX_WORDS * 4

#define DIEPINTxWrMsk     0x297F
#define DOEPINTxWrMsk     0xF17F
#define DIEPCTL_MPSIZ_Msk 0x7FF


#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))

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

	uint8_t deviceState;
	uint8_t ep0State;

	usb_endpointtypedef_t epIn[ENDPOINTS_NUMBER];
	usb_endpointtypedef_t epOut[ENDPOINTS_NUMBER];

	uint32_t ep0DataLen;
} usb_dc_t;


typedef struct _usb_buffer_t {
	uint8_t *vBuffer;
	addr_t pBuffer;
	int16_t len;
} usb_buffer_t;


/**
 * Endpoint structure for stm32n6 usb
 * Params:
 *  - xfer_buf - pointer to buffer with received / transmit data
 *  - xfer_len - length of the buffer
 *  - xfer_count - partial transfer lenth in case of multipacket transfer
 *  - is_in - (1 - true / 0 - false)  - is it IN endpoint
 */
typedef struct _stm32n6_endpt_data_t {
	uint8_t *xfer_buf;
	uint32_t xfer_len;
	uint32_t xfer_count;
	uint8_t is_in;
	uint8_t epNum;
	uint8_t type;
	uint32_t maxpacket;
	uint32_t xfer_size;
} stm32n6_endpt_data_t;


typedef struct _stm32n6_endpt_t {
	stm32n6_endpt_data_t in;
	stm32n6_endpt_data_t out;
} stm32n6_endpt_t;


typedef struct {
	char *setupMem;
	stm32n6_endpt_t endpts[ENDPOINTS_NUMBER];
	uint32_t bufforek[ENDPOINTS_DIR_NB];
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


/*  EP0 State */
#define USBD_EP0_IDLE       0x00U
#define USBD_EP0_SETUP      0x01U
#define USBD_EP0_DATA_IN    0x02U
#define USBD_EP0_DATA_OUT   0x03U
#define USBD_EP0_STATUS_IN  0x04U
#define USBD_EP0_STATUS_OUT 0x05U
#define USBD_EP0_STALL      0x06U


int usbclient_init(usb_desc_list_t *desList);
int usbclient_send(int endpt, const void *data, unsigned int len);
int usbclient_receive(int endpt, void *data, unsigned int len);
int usbclient_destroy(void);

void usbclient_setUserContext(void *ctxUser);
void usbclient_setEventCallback(void (*cbEvent)(int, void *));
void usbclient_setClassCallback(int (*cbClassSetup)(const usb_setup_packet_t *, void *, unsigned int, void *));


extern int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern int ctrl_hfIrq(void);
extern void ctrl_lfIrq(void);
extern void ctrl_setAddress(uint32_t addr);
extern int ctrl_execTransfer(int endpt, uint8_t *virtAddr, int nBytes);

extern int clbc_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern void clbc_reset(void);
extern void clbc_enumdne(void);
extern void clbc_dataInStage(uint8_t epNum);
extern void clbc_ep0OutStart(void);
extern void clbc_endptInit(void);
extern void clbc_rxFifoData(int debug);
extern void clbc_flushTxFifo(uint32_t num);
extern void clbc_sendEpData(int ep, uint8_t *virtAddr, int nBytes);
extern int clbc_epTransmit(uint8_t epNum, uint8_t *dataBuf, uint32_t len);


extern int desc_init(usb_desc_list_t *desList, usb_common_data_t *usb_data_in, usb_dc_t *dc_in);
extern int desc_setup(const usb_setup_packet_t *setup);
extern int desc_classSetup(const usb_setup_packet_t *setup);


#endif
