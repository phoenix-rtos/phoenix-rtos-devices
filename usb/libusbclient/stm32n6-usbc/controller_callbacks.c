/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * IRQ thread callbacks
 *
 * Copyright 2025 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdatomic.h>
#include "client.h"
#include "phy.h"

#define DOEPINT_RESET_MASK 0xF17FU
#define DIEPINT_RESET_MASK 0x297FU

#define UNALIGNED32_READ(addr)       (((const struct unaligned32 *)(addr))->var)
#define UNALIGNED32_WRITE(addr, val) (void)((((struct unaligned32 *)(addr))->var) = (val))

#define USB_REG(x) clbc_common.dc->base[x]


struct __attribute__((packed)) unaligned32 {
	uint32_t var;
};


struct {
	usb_dc_t *dc;
	usb_common_data_t *data;
} clbc_common;


void clbc_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	clbc_common.dc = dc_in;
	clbc_common.data = usb_data_in;
}


static void activateEndpoint(int epNum, int epDir, uint32_t epType, uint32_t maxPacket)
{
	if (epDir == USB_EP_DIR_IN) {
		clbc_common.data->endpts[epNum].in.type = epType;

		USB_REG(DAINTMSK) |= (0xFFFF & ((uint32_t)(1UL << (epNum & 0xFU))));

		/* EPs 1-8 are disabled by the core when USBRST event detected */
		if (((USB_REG(DIEPCTL0 + epNum * EP_STRIDE) >> 15) & 1) == 0) {
			USB_REG(DIEPCTL0 + epNum * EP_STRIDE) |= ((maxPacket & 0x7FF) | ((epType & 0x3U) << DIEPCTL_EPTYP) |
					((epNum & 0xFU) << DIEPCTL_TXFNUM) | (0x1UL << DIEPCTL_SD0PID) |
					(0x1UL << DIEPCTL_USBAEP));
		}
	}
	else {
		clbc_common.data->endpts[epNum].out.type = epType;

		USB_REG(DAINTMSK) |= ((0xFFFF << 16) & ((uint32_t)(1UL << (epNum & 0xFU))) << 16);

		/* EPs 1-8 are disabled by the core when USBRST event detected */
		if (((USB_REG(DOEPCTL0 + epNum * EP_STRIDE) >> 15) & 1) == 0) {
			USB_REG(DOEPCTL0 + epNum * EP_STRIDE) |= ((maxPacket & 0x7FF) | ((epType & 0x3U) << DOEPCTL_EPTYP) |
					(0x1UL << DOEPCTL_SD0PID) | (0x1UL << DOEPCTL_USBAEP));
		}
	}
}


void clbc_reset(void)
{
	activateEndpoint(0, USB_EP_DIR_OUT, USB_EP_TYPE_CTRL, 64U);
	clbc_common.data->endpts[0].out.maxpacket = 64;
	clbc_common.data->endpts[0].out.dataBuf.vBuffer = clbc_common.dc->ep0buffRX;
	clbc_common.data->endpts[0].out.dataBuf.head = clbc_common.data->endpts[0].out.dataBuf.vBuffer;
	clbc_common.data->endpts[0].out.dataBuf.tail = clbc_common.data->endpts[0].out.dataBuf.vBuffer;
	clbc_common.data->endpts[0].out.dataBuf.size = (uint16_t)EP0_RX_BUFFER_SIZE;

	activateEndpoint(0, USB_EP_DIR_IN, USB_EP_TYPE_CTRL, 64U);
	clbc_common.data->endpts[0].in.maxpacket = 64;
	clbc_common.data->endpts[0].in.dataBuf.vBuffer = clbc_common.dc->ep0buffTX;
	clbc_common.data->endpts[0].in.dataBuf.head = clbc_common.data->endpts[0].in.dataBuf.vBuffer;
	clbc_common.data->endpts[0].in.dataBuf.tail = clbc_common.data->endpts[0].in.dataBuf.vBuffer;
	clbc_common.data->endpts[0].in.dataBuf.size = (uint16_t)EP0_TX_BUFFER_SIZE;
}


void clbc_ep0OutStart(void)
{
	uint32_t gSNPSiD = USB_REG(CID + 1);

	if (gSNPSiD > 0x4F54300AU) {
		if (((USB_REG(DOEPCTL0) >> DOEPCTL_EPENA) & 1U) == 1U) {
			return;
		}
	}

	USB_REG(DOEPTSIZ0) = 0U;
	USB_REG(DOEPTSIZ0) |= (1UL << 19);
	USB_REG(DOEPTSIZ0) |= (3U * 8U);
	USB_REG(DOEPTSIZ0) |= (0x3UL << 29);
}


void clbc_epStall(stm32n6_endpt_t *ep)
{
	/* stall IN endpoint */
	if (ep->in.epNum != 0U) {
		USB_REG(DIEPCTL0 + ep->in.epNum * EP_STRIDE) |= (1UL << DIEPCTL_EPDIS);
	}
	USB_REG(DIEPCTL0 + ep->in.epNum * EP_STRIDE) |= (1UL << DIEPCTL_STALL);

	/* stall OUT endpoint */
	if (ep->in.epNum != 0U) {
		USB_REG(DOEPCTL0 + ep->in.epNum * EP_STRIDE) |= (1UL << DOEPCTL_EPDIS);
	}
	USB_REG(DOEPCTL0 + ep->in.epNum * EP_STRIDE) |= (1UL << DOEPCTL_STALL);
}


static uint8_t getDevSpeed(void)
{
	uint8_t speed;
	uint32_t devEnumSpeed = ((USB_REG(DSTS) >> 1) & 3);

	if (devEnumSpeed == 0) {
		/* HIGH SPEED */
		speed = 0U;
	}
	else if (devEnumSpeed == 1 || devEnumSpeed == 3) {
		/* FULL SPEED */
		speed = 2U;
	}
	else {
		speed = 0xFU;
	}

	clbc_common.dc->enumSpeed = speed;
	return speed;
}


static void setTurnaroundTime(uint32_t hclk, uint8_t speed)
{
	uint32_t usbTrd;

	if (speed == 2U) {
		if ((hclk >= 14200000U) && (hclk < 15000000U)) {
			/* hclk Clock Range between 14.2-15 MHz */
			usbTrd = 0xFU;
		}
		else if ((hclk >= 15000000U) && (hclk < 16000000U)) {
			/* hclk Clock Range between 15-16 MHz */
			usbTrd = 0xEU;
		}
		else if ((hclk >= 16000000U) && (hclk < 17200000U)) {
			/* hclk Clock Range between 16-17.2 MHz */
			usbTrd = 0xDU;
		}
		else if ((hclk >= 17200000U) && (hclk < 18500000U)) {
			/* hclk Clock Range between 17.2-18.5 MHz */
			usbTrd = 0xCU;
		}
		else if ((hclk >= 18500000U) && (hclk < 20000000U)) {
			/* hclk Clock Range between 18.5-20 MHz */
			usbTrd = 0xBU;
		}
		else if ((hclk >= 20000000U) && (hclk < 21800000U)) {
			/* hclk Clock Range between 20-21.8 MHz */
			usbTrd = 0xAU;
		}
		else if ((hclk >= 21800000U) && (hclk < 24000000U)) {
			/* hclk Clock Range between 21.8-24 MHz */
			usbTrd = 0x9U;
		}
		else /* if(hclk >= 32000000) */
		{
			/* hclk Clock Range between 32-200 MHz */
			usbTrd = 0x9U;
		}
	}
	else if (speed == 0U) {
		usbTrd = 9U;
	}
	else {
		usbTrd = 9U;
	}

	USB_REG(GUSBCFG) &= ~(0xFUL << 10);
	USB_REG(GUSBCFG) |= (usbTrd << 10);
}


void clbc_enumdne(void)
{
	uint8_t speed = getDevSpeed();
	uint32_t hclkFreq = phy_getHclkFreq();

	setTurnaroundTime(hclkFreq, speed);
}


static void epStartXfer(stm32n6_endpt_data_t *epData)
{
	uint16_t pktcnt;

	if (epData->is_in == 1U) {
		/* IN Endpoint */
		USB_REG(DIEPTSIZ0 + epData->epNum * EP_STRIDE) &= ~(0x7FFFFUL);
		USB_REG(DIEPTSIZ0 + epData->epNum * EP_STRIDE) &= ~(0x3FFUL << 19);

		if (epData->xfer_len == 0U) {
			/* Zero Length Packet */
			USB_REG(DIEPTSIZ0 + epData->epNum * EP_STRIDE) |= (1UL << 19);
		}
		else {
			/* EP0 - send 1 packet at a time */
			if (epData->epNum == 0U) {

				if (epData->xfer_len > epData->maxpacket) {
					epData->xfer_len = epData->maxpacket;
				}
				USB_REG(DIEPTSIZ0) |= (1UL << 19);
				USB_REG(DIEPTSIZ0) |= (0x7FFFFUL & epData->xfer_len);
			}
			else {
				/* EP1-EP8 - whole transfer at once */
				pktcnt = (uint16_t)((epData->xfer_size + epData->maxpacket - 1U) / epData->maxpacket);
				USB_REG(DIEPTSIZ0 + epData->epNum * EP_STRIDE) |= ((pktcnt & 0x3FFUL) << 19);

				if (epData->type == USB_EP_TYPE_ISOC) {
					/**
					 * TODO: ISOC PACKETS
					 */
				}

				USB_REG(DIEPTSIZ0 + epData->epNum * EP_STRIDE) |= (0x7FFFFUL & epData->xfer_size);
			}
		}

		/* enable endpoint and clear NAK for this transfer */
		USB_REG(DIEPCTL0 + epData->epNum * EP_STRIDE) |= ((1UL << 26) | (1UL << 31));

		if (epData->type != USB_EP_TYPE_ISOC) {
			if (epData->xfer_len > 0U) {
				USB_REG(DIEPEMPMSK) |= (1UL << (epData->epNum & 0xF));
			}
		}
		else {
			if ((USB_REG(DSTS) & (1 << 8)) == 0U) {
				USB_REG(DIEPCTL0 + epData->epNum * EP_STRIDE) |= (1UL << 29);
			}
			else {
				USB_REG(DIEPCTL0 + epData->epNum * EP_STRIDE) |= (1UL << 28);
			}
			/**
			 * TODO: WRITE_USB_PACKET()
			 */
		}
	}
	else {
		/* OUT Endpoint */
		USB_REG(DOEPTSIZ0 + epData->epNum * EP_STRIDE) &= ~(0x3FFUL << 19UL);
		USB_REG(DOEPTSIZ0 + epData->epNum * EP_STRIDE) &= ~(0x7FFFFUL);

		if (epData->epNum == 0) {
			if (epData->xfer_size > epData->maxpacket) {
				epData->xfer_len = epData->maxpacket;
			}
			pktcnt = 1UL;
		}
		else {
			if (epData->xfer_size == 0) {
				pktcnt = 1UL;
			}
			else {
				pktcnt = (uint16_t)((epData->xfer_size + epData->maxpacket - 1U) / epData->maxpacket);
			}
		}

		USB_REG(DOEPTSIZ0 + epData->epNum * EP_STRIDE) |= (0x7FFFFUL & epData->xfer_size);
		USB_REG(DOEPTSIZ0 + epData->epNum * EP_STRIDE) |= ((pktcnt & 0x3FFUL) << 19);

		if (epData->type == USB_EP_TYPE_ISOC) {
			if ((USB_REG(DSTS) & (1 << 8)) == 0U) {
				USB_REG(DOEPCTL0 + epData->epNum * EP_STRIDE) |= (1UL << 29);
			}
			else {
				USB_REG(DOEPCTL0 + epData->epNum * EP_STRIDE) |= (1UL << 28);
			}
		}
		USB_REG(DOEPCTL0 + epData->epNum * EP_STRIDE) |= ((1UL << 26) | (1UL << 31));
	}
}


void clbc_epTransmit(uint8_t epNum, uint8_t *dataBuf, uint32_t len)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];

	/* lock semaphore for BULK IN */
	if (epNum == 2U) {
		semaphoreDown(&clbc_common.dc->semBulkTx, 0);
	}

	ep->in.xfer_buf = dataBuf;
	ep->in.xfer_size = len;
	ep->in.xfer_len = len;
	ep->in.xfer_count = 0U;
	ep->in.is_in = 1U;
	ep->in.epNum = epNum;
	ep->in.xfer_active = 1;

	epStartXfer(&ep->in);
}


void clbc_epTransmitCont(uint8_t epNum)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];

	ep->in.xfer_size = ep->in.xfer_size - ep->in.xfer_count;
	ep->in.xfer_len = ep->in.xfer_size;
	ep->in.xfer_count = 0U;

	epStartXfer(&ep->in);
}


int clbc_ep0Receive(usb_xfer_desc_t *desc)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[0];
	uint8_t availSpace = (uint8_t)ep->out.dataBuf.size - (uint8_t)(ep->out.dataBuf.tail - ep->out.dataBuf.head);
	uint32_t timeout = USB_OTG_TIMEOUT;

	if (desc->len > availSpace) {
		ep->out.dataBuf.head = ep->out.dataBuf.vBuffer;
		ep->out.dataBuf.tail = ep->out.dataBuf.vBuffer;
	}

	availSpace = (uint8_t)ep->out.dataBuf.size - (uint8_t)(ep->out.dataBuf.tail - ep->out.dataBuf.head);

	if (desc->len > availSpace) {
		return -ENOMEM;
	}

	desc->head = ep->out.dataBuf.tail;
	clbc_epReceive(0, ep->out.dataBuf.tail, desc->len);

	/* EP0 OUT data should be received immediately */
	while (ep->out.xfer_size < ep->out.xfer_count) {
		if (--timeout == 0UL) {
			return -EBUSY;
		};
	}
	ep->out.dataBuf.tail += desc->len;
	desc->tail = ep->out.dataBuf.tail;

	return EOK;
}


void clbc_epReceiveCont(uint8_t epNum)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];

	ep->out.xfer_size = ep->in.xfer_size - ep->in.xfer_count;
	ep->out.xfer_len = ep->in.xfer_size;
	ep->out.xfer_count = 0U;

	epStartXfer(&ep->out);
}


void clbc_epReceive(uint8_t epNum, uint8_t *dataBuf, uint32_t len)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];

	/* lock semaphore for BULK OUT */
	if (epNum == 2U) {
		semaphoreDown(&clbc_common.dc->semBulkRx, 0);
	}

	ep->out.xfer_buf = dataBuf;
	ep->out.xfer_size = len;
	ep->out.xfer_len = len;
	ep->out.xfer_count = 0U;
	ep->out.is_in = 0U;
	ep->out.epNum = epNum;
	ep->out.xfer_active = 1;

	epStartXfer(&ep->out);
}


void clbc_rxFifoData(void)
{
	uint8_t pktSts, epNum;
	uint16_t byteCount;
	uint8_t *dest;
	uint32_t grxstpStatus = USB_REG(GRXSTSP);
	stm32n6_endpt_t *ep;

	pktSts = ((grxstpStatus >> 17) & 0xF);
	epNum = (grxstpStatus & 0xF);
	byteCount = ((grxstpStatus >> 4) & 0x7FF);

	ep = &clbc_common.data->endpts[epNum];

	switch (pktSts) {

		case RX_PKTSTS_OUT_RXED:
			clbc_receiveEpData(ep->out.xfer_buf, byteCount);

			ep->out.xfer_buf += byteCount;
			ep->out.xfer_count += byteCount;

			break;
		case RX_PKTSTS_SETUP_RXED:
			dest = (uint8_t *)&clbc_common.dc->setupPacket;
			clbc_receiveEpData(dest, 8U);
			break;

		default:
			break;
	}
}


int clbc_flushTxFifo(uint8_t num)
{
	uint32_t timeout = USB_OTG_TIMEOUT;

	/* set Global IN NAK */
	USB_REG(DCTL) |= (1U << 7);
	while (((USB_REG(GINTSTS) >> 6) & 1U) == 0U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	/* check AHB master idle */
	timeout = USB_OTG_TIMEOUT;
	while (((USB_REG(GRSTCTL) >> 31) & 1U) == 0U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	USB_REG(GRSTCTL) = (num << 6) | (1U << 5);

	timeout = USB_OTG_TIMEOUT;
	while (((USB_REG(GRSTCTL) >> 5) & 1U)) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	/* clear Global IN NAK */
	timeout = USB_OTG_TIMEOUT;
	USB_REG(DCTL) |= (1U << 8);
	while (((USB_REG(GINTSTS) >> 6) & 1U) == 1U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	return EOK;
}


int clbc_flushRxFifo(void)
{
	uint32_t timeout = USB_OTG_TIMEOUT;

	while (((USB_REG(GRSTCTL) >> 31) & 1U) == 0U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}
	USB_REG(GRSTCTL) = (1U << 4);

	timeout = USB_OTG_TIMEOUT;
	while (((USB_REG(GRSTCTL) >> 4) & 1U) == 1U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	return EOK;
}


int clbc_resetUSBSS(void)
{
	uint32_t timeout = USB_OTG_TIMEOUT;

	while (((USB_REG(GRSTCTL) >> 31) & 1U) == 0U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	/* Wait before setting reset */
	timeout = 10U;
	while (timeout > 0U) {
		timeout--;
	}

	USB_REG(GRSTCTL) |= (1U);

	timeout = USB_OTG_TIMEOUT;
	while (((USB_REG(GRSTCTL)) & 1U) == 1U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	return EOK;
}


int clbc_setDevMode(void)
{
	uint32_t timeout = USB_OTG_TIMEOUT;

	/* clear current mode */
	USB_REG(GUSBCFG) &= ~(0x3UL << 29);

	/* set device mode */
	USB_REG(GUSBCFG) |= (1UL << 30);

	while ((USB_REG(GINTSTS) & 1U) == 1U) {
		if (--timeout == 0) {
			return -EBUSY;
		}
	}

	return EOK;
}


void clbc_sendEpData(uint8_t epNum)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];
	uint8_t *src = ep->in.xfer_buf;
	uint16_t len = ep->in.xfer_len;
	uint16_t count32b;
	uint16_t cnt;

	while ((ep->in.xfer_count < ep->in.xfer_len)) {
		len = (uint16_t)(ep->in.xfer_len - ep->in.xfer_count);

		if (len > ep->in.maxpacket)
			len = ep->in.maxpacket;

		count32b = (len + 3U) / 4U;

		if ((USB_REG(DTXFSTS0 + EP_STRIDE * epNum) & 0xFFFFUL) < count32b) {
			break;
		}

		src = ep->in.xfer_buf;

		for (cnt = 0; cnt < count32b; cnt++) {
			USB_REG(FIFO_BASE_OFF + epNum * (FIFO_SIZE / 4)) =
					UNALIGNED32_READ(src);
			src += sizeof(uint32_t);
		}

		ep->in.xfer_count += len;
		ep->in.xfer_buf += len;
	}

	if (ep->in.xfer_len <= ep->in.xfer_count) {
		/* mask TX FIFO empty interrupt */
		USB_REG(DIEPEMPMSK) &= ~(1UL << epNum);
	}
	else {
		/* unmask TX FIFO empty interrupt */
		USB_REG(DIEPEMPMSK) |= (1UL << epNum);
	}

	return;
}


void clbc_receiveEpData(uint8_t *dest, uint16_t nBytes)
{
	uint16_t count32b = nBytes >> 2U;
	uint8_t trailingBytes = nBytes % 4U;
	uint8_t *buff = dest;
	uint16_t cnt;
	uint32_t temp;

	for (cnt = 0; cnt < count32b; cnt++) {
		/* read 1 word from fifo*/
		UNALIGNED32_WRITE(buff, USB_REG(FIFO_BASE_OFF));
		buff += sizeof(uint32_t);
	}

	if (trailingBytes != 0U) {
		UNALIGNED32_WRITE(&temp, USB_REG(FIFO_BASE_OFF));
		cnt = 0;

		while (trailingBytes != 0) {
			*buff = *((uint8_t *)&temp + cnt);
			trailingBytes--;
			buff++;
			cnt++;
		}
	}
}


/* IRQs callbacks */
void _clbc_USBRST(void)
{
	/* clear all TX FIFOs*/
	clbc_flushTxFifo(16U);

	/* clear remote wake up */
	USB_REG(DCTL) &= ~(0x1UL);

	/* reset endpoints config and set SNACK */
	for (int i = 0; i < 9; i++) {
		USB_REG(DIEPINT0 + EP_STRIDE * i) = DIEPINT_RESET_MASK;
		USB_REG(DIEPCTL0 + EP_STRIDE * i) &= ~(0x1UL << DIEPCTL_STALL);
		USB_REG(DOEPINT0 + EP_STRIDE * i) = DOEPINT_RESET_MASK;
		USB_REG(DOEPCTL0 + EP_STRIDE * i) &= ~(0x1UL << DOEPCTL_STALL);
		USB_REG(DOEPCTL0 + EP_STRIDE * i) |= (0x1UL << DOEPCTL_SNAK);
	}

	/* unmask IN0 and OUT0 EPs IRQs*/
	USB_REG(DAINTMSK) |= DAINTMSK_IN(0UL);
	USB_REG(DAINTMSK) |= DAINTMSK_OUT(0UL);

	USB_REG(DOEPMSK) = 0UL;
	USB_REG(DOEPMSK) |= (0x1UL << DOEPMSK_STUMP) |
			(0x1UL << DOEPMSK_XFRCM) |
			(0x1UL << DOEPMSK_STSPHSRXM) |
			(0x1UL << DOEPMSK_NAKMSK);

	USB_REG(DIEPMSK) = 0UL;
	USB_REG(DIEPMSK) |= (0x1UL << DIEPMSK_TOM) |
			(0x1UL << DIEPMSK_XFRCM) |
			(0x1UL << DIEPMSK_EPDM);

	/* zero-out device address*/
	USB_REG(DCFG) &= ~(0x7FUL << DCFG_DAD);

	clbc_ep0OutStart();
}


void _clbc_ENUMDNE(void)
{
	/* zero out EP0 max packet size */
	USB_REG(DIEPCTL0) &= ~(0x7FFUL);

	/* clear global IN NAK */
	USB_REG(DCTL) |= (0x1UL << DCTL_CGINAK);

	/* get device speed */
	clbc_common.dc->enumSpeed = getDevSpeed();

	/* set turnaround time */
	clbc_enumdne();

	clbc_reset();
}


void _clbc_OEPINT(void)
{
	uint32_t epInt;
	uint8_t epNum = 0U;
	stm32n6_endpt_t *ep;
	uint32_t daintClear = atomic_fetch_and(&clbc_common.dc->daintClear, 0x0000FFFFU);

	daintClear &= 0xFFFF0000U;
	daintClear >>= 16U;

	while (daintClear != 0U) {

		ep = &clbc_common.data->endpts[epNum];

		if ((daintClear & 0x1U) != 0U) {
			epInt = clbc_common.dc->irqPendingDOEPINT[epNum];
			epInt &= USB_REG(DOEPMSK);

			/* XFRC */
			if (IS_IRQ(epInt, DOEPINT_XFRC)) {
				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_XFRC);

				/* Control transfer */
				if (epNum == 0U) {
					/* if STATUS OUT received */
					if (clbc_common.dc->ep0State == USBD_EP0_STATUS_OUT) {
						clbc_common.dc->ep0State = USBD_EP0_IDLE;
						ep->out.xfer_active = 0;
						/* prepare to receive next SETUP if it is ZLP */
						if (ep->out.xfer_len == 0U) {
							USB_REG(DOEPTSIZ0) = (3U << 29) | (1U << 19) | 8U;
						}
					}
					/* if DATA OUT received */
					else if (clbc_common.dc->ep0State == USBD_EP0_DATA_OUT) {

						/* More data to be received */
						if (ep->out.xfer_size > ep->out.xfer_count) {
							clbc_epReceiveCont(epNum);
						}
						/* Transfer completed */
						else {
							clbc_common.dc->ep0State = USBD_EP0_STATUS_IN;
							ep->out.xfer_active = 0;
						}
					}
				}
				/* only 1 data EP implemented */
				else {
					ep->out.xfer_active = 0;
					semaphoreUp(&clbc_common.dc->semBulkRx);
				}
			}

			/* STUP */
			if (IS_IRQ(epInt, DOEPINT_STUP)) {
				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_STUP);

				if (((epInt >> DOEPINT_STSPHSRX) & 0x1U) == 0x1U) {
					clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_STSPHSRX);
				}

				clbc_common.dc->ep0State = USBD_EP0_SETUP;

				desc_setup(&clbc_common.dc->setupPacket);
			}

			/* OTEPDIS */
			if (IS_IRQ(epInt, DOEPINT_OTEPDIS)) {
				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_OTEPDIS);
			}

			/* EPDISD */
			if (IS_IRQ(epInt, DOEPINT_EPDISD)) {
				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_EPDISD);

				if ((USB_REG(GINTSTS) >> 7) & 1) {
					USB_REG(DCTL) |= (1 << 10);
				}
			}

			/* STSPHSRX / OTEPSPR */
			if (IS_IRQ(epInt, DOEPINT_STSPHSRX)) {

				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_STSPHSRX);
			}

			/* NAK */
			if (IS_IRQ(epInt, DOEPINT_NAK)) {

				clbc_common.dc->irqPendingDOEPINT[epNum] &= ~(1UL << DOEPINT_NAK);

				/* check if more data are to be received */
				if (epNum == 0U) {
					if ((USB_REG(DOEPTSIZ0 + epNum * EP_STRIDE) & (0x3FFUL << DOEPTSIZ_PKTCNT)) != 0UL) {
						USB_REG(DOEPCTL0 + epNum * EP_STRIDE) |= ((1UL << DOEPCTL_CNAK) | (1UL << DOEPCTL_EPENA));
					}
				}
			}
		}
		epNum++;
		daintClear >>= 1U;
	}
}


void _clbc_IEPINT(void)
{
	volatile uint32_t epInt;
	uint32_t epMsk, epEmp;
	uint8_t epNum = 0U;
	stm32n6_endpt_t *ep;
	uint32_t daintClear = atomic_fetch_and(&clbc_common.dc->daintClear, 0xFFFF0000U);

	daintClear &= 0x0000FFFFU;

	while (daintClear != 0U) {

		ep = &clbc_common.data->endpts[epNum];

		if ((daintClear & 0x1U) != 0) {

			/*
			 * Add "TX FIFO empty interrupt mask" bit (local for every IEP) to the global
			 * mask register for all IEPs. Now the mask will extract all DIEPINTx interrupts
			 * (global and local).
			 */
			epMsk = USB_REG(DIEPMSK);
			epEmp = clbc_common.dc->diepmsk;

			epMsk |= ((epEmp >> (epNum & 0xFU)) & 0x1U) << 7;

			epInt = clbc_common.dc->irqPendingDIEPINT[epNum];
			epInt &= epMsk;

			/* XFRC */
			if (IS_IRQ(epInt, DIEPINT_XFRC)) {
				clbc_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_XFRC);

				/* mask TX FIFO empty interrupt */
				USB_REG(DIEPEMPMSK) &= ~(1UL << epNum);

				/* Control transfer*/
				if (epNum == 0) {
					/* prepare to receive next SETUP */
					USB_REG(DOEPTSIZ0) = (3UL << 29) | (1UL << 19) | 8UL;

					if (ep->in.xfer_size - ep->in.xfer_len > 0U) {
						clbc_epTransmitCont(epNum);
					}
					else {
						ep->in.xfer_active = 0;

						/* STATUS IN stage */
						if (ep->in.xfer_len == 0U && clbc_common.dc->ep0State == USBD_EP0_STATUS_IN) {
							clbc_common.dc->ep0State = USBD_EP0_IDLE;
						}
						/* DATA IN stage */
						else if (clbc_common.dc->ep0State == USBD_EP0_DATA_IN) {

							/* receive ZLP status packet */
							clbc_epReceive(0, NULL, 0);
							clbc_common.dc->ep0State = USBD_EP0_STATUS_OUT;
						}
					}
				}
				/* only 1 data EP implemented */
				else {
					/* Unmask SOF interrupt - masked in clbc_epTransmit() */
					ep->in.xfer_active = 0;
					semaphoreUp(&clbc_common.dc->semBulkTx);
				}
			}

			/* TOC */
			if (IS_IRQ(epInt, DIEPINT_TOC)) {
				clbc_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_TOC);
			}

			/* ITTXFE */
			if (IS_IRQ(epInt, DIEPINT_ITTXFE)) {
				clbc_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_ITTXFE);
			}

			/* INEPNE */
			if (IS_IRQ(epInt, DIEPINT_INEPNE)) {
				clbc_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_INEPNE);
			}

			/* EPDISD */
			if (IS_IRQ(epInt, DIEPINT_EPDISD)) {
				clbc_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_EPDISD);

				clbc_flushTxFifo(epNum);
			}
		}
		epNum++;
		daintClear >>= 1U;
	}
}
