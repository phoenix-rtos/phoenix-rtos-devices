#include "client.h"


#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed))
#endif
__PACKED_STRUCT T_UINT32_READ { uint32_t v; };
#define __UNALIGNED_UINT32_READ(addr) (((const struct T_UINT32_READ *)(const void *)(addr))->v)
struct {
	usb_dc_t *dc;
	usb_common_data_t *data;
} clbc_common;


int clbc_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	clbc_common.dc = dc_in;
	clbc_common.data = usb_data_in;

	return 0;
}


static void activateEndpoint(int epNum, int epDir, int epType, uint32_t maxPacket)
{
	if (epDir == USB_EP_DIR_IN) {
		clbc_common.data->endpts[epNum].in.type = epType;

		clbc_common.dc->base[DAINTMSK] |= (0xFFFF & ((uint32_t)(1UL << (epNum & 0xFU))));

		if (((clbc_common.dc->base[DIEPCTL0 + epNum * EP_STRIDE] >> 15) & 1) == 0) {
			clbc_common.dc->base[DIEPCTL0 + epNum * EP_STRIDE] |= (maxPacket & 0x7FF) |
					((uint32_t)epType << 18) | (epNum << 22) |
					(0x1UL << 28UL) | (0x1UL << 15UL);
		}
	}
	else {
		clbc_common.data->endpts[epNum].out.type = epType;

		clbc_common.dc->base[DAINTMSK] |= ((0xFFFF << 16) & ((uint32_t)(1UL << (epNum & 0xFU))) << 16);

		if (((clbc_common.dc->base[DOEPCTL0 + epNum * EP_STRIDE] >> 15) & 1) == 0) {
			clbc_common.dc->base[DOEPCTL0 + epNum * EP_STRIDE] |= (maxPacket & 0x7FF) |
					((uint32_t)epType << 18) | (epNum << 22) |
					(0x1UL << 28UL) | (0x1UL << 15UL);
		}
	}
}


void clbc_reset(void)
{
	activateEndpoint(0, USB_EP_DIR_OUT, USB_EP_TYPE_CTRL, 64U);
	clbc_common.data->endpts[0].in.maxpacket = 64;
	activateEndpoint(0, USB_EP_DIR_IN, USB_EP_TYPE_CTRL, 64U);
	clbc_common.data->endpts[0].out.maxpacket = 64;
}


void clbc_ep0OutStart(void)
{
	uint32_t gSNPSiD = clbc_common.dc->base[CID + 1];

	if (gSNPSiD > 0x4F54300AU) {
		if ((clbc_common.dc->base[DOEPCTL0] >> 31) & 1) {
			return;
		}
	}

	clbc_common.dc->base[DOEPTSIZ0] = 0U;
	clbc_common.dc->base[DOEPTSIZ0] |= ((0x3FFUL << 19UL) & (1UL << 19));
	clbc_common.dc->base[DOEPTSIZ0] |= (3U * 8U);
	clbc_common.dc->base[DOEPTSIZ0] |= (0x3UL << 29UL);
}


static void outXfrcComplete(int epNum)
{
	uint32_t gSNPSiD = clbc_common.dc->base[CID + 1];
	if (gSNPSiD == 0x4F54310AU) {
		if ((clbc_common.dc->base[DOEPINT0] >> 15) & 1) {
			clbc_common.dc->base[DOEPINT0] |= (1 << 15);
		}
		if ((clbc_common.dc->base[DOEPINT0] >> 5) & 1) {
			clbc_common.dc->base[DOEPINT0] |= (1 << 5);
		}

		// callback
	}
	else {
		if (epNum == 0) {
			clbc_ep0OutStart();
		}
	}
}


static uint8_t getDevSpeed(void)
{
	uint8_t speed;
	uint32_t devEnumSpeed = ((clbc_common.dc->base[DSTS] >> 1) & 3);

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
		else if ((hclk >= 24000000U) && (hclk < 27700000U)) {
			/* hclk Clock Range between 24-27.7 MHz */
			usbTrd = 0x8U;
		}
		else if ((hclk >= 27700000U) && (hclk < 32000000U)) {
			/* hclk Clock Range between 27.7-32 MHz */
			usbTrd = 0x7U;
		}
		else /* if(hclk >= 32000000) */
		{
			/* hclk Clock Range between 32-200 MHz */
			usbTrd = 0x6U;
		}
	}
	else if (speed == 0U) {
		usbTrd = 9U;
	}
	else {
		usbTrd = 9U;
	}

	clbc_common.dc->base[GUSBCFG] &= ~(0xFUL << 10UL);
	clbc_common.dc->base[GUSBCFG] |= (uint32_t)((usbTrd << 10) & (0xFUL << 10UL));
}


void clbc_enumdne(void)
{
	uint8_t speed = getDevSpeed();
	uint32_t hclkFreq = phy_getHclkFreq();

	setTurnaroundTime(hclkFreq, speed);
}


static void epStartXfer(stm32n6_endpt_data_t *epData)
{
	if (epData->is_in == 1U) {
		/* IN Endpoint */
		if (epData->xfer_len == 0U) {
			/* Zero Length Packet */
			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x3FFUL << 19UL);
			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] |= ((0x3FFUL << 19UL) & ((1UL << 19)));
			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x7FFFFUL);
		}
		else {
			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x3FFUL << 19UL);
			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x7FFFFUL);

			if (epData->epNum == 0) {
				if (epData->xfer_len > epData->maxpacket) {
					epData->xfer_len = epData->maxpacket;
				}
				// __asm__ volatile ("1: b 1b");
				clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] |= (1UL << 19);
			}
			else {
				uint16_t pktcnt = (uint16_t)((epData->xfer_len + epData->maxpacket - 1U) / epData->maxpacket);
				clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] |= ((0x3FFUL << 19UL) & (((uint32_t)pktcnt << 19)));

				if (epData->type == USB_EP_TYPE_ISOC) {
					/**
					 * TODO: ISOC PACKETS
					 */
					// USBx_INEP(epnum)->DIEPTSIZ &= ~(USB_OTG_DIEPTSIZ_MULCNT);
					// USBx_INEP(epnum)->DIEPTSIZ |= (USB_OTG_DIEPTSIZ_MULCNT & ((uint32_t)pktcnt << 29));
				}
			}

			clbc_common.dc->base[DIEPTSIZ0 + epData->epNum * EP_STRIDE] |= ((0x7FFFFUL) & epData->xfer_len);
		}

		clbc_common.dc->base[DIEPCTL0 + epData->epNum * EP_STRIDE] |= ((1 << 26) | (1 << 31));

		if (epData->type != USB_EP_TYPE_ISOC) {
			if (epData->xfer_len > 0U) {
				clbc_common.dc->base[DIEPEMPMSK] |= (1 << (epData->epNum & 0xF));
			}
		}
		else {
			if ((clbc_common.dc->base[DSTS] & (1 << 8)) == 0U) {
				clbc_common.dc->base[DIEPCTL0 + epData->epNum * EP_STRIDE] |= (1 << 29);
			}
			else {
				clbc_common.dc->base[DIEPCTL0 + epData->epNum * EP_STRIDE] |= (1 << 28);
			}
			/**
			 * TODO: WRITE_USB_PACKET()
			 */
		}
	}
	else {
		/* OUT Endpoint */
		clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x7FFFFUL);
		clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] &= ~(0x3FFUL << 19UL);

		if (epData->epNum == 0) {
			if (epData->xfer_len > 0) {
				epData->xfer_len = epData->maxpacket;
			}

			epData->xfer_size = epData->maxpacket;

			clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= (0x7FFFFUL & epData->xfer_size);
			clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= (1 << 19);
		}
		else {
			if (epData->xfer_len == 0) {
				clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= (0x7FFFFUL & epData->xfer_size);
				clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= (1 << 19);
			}
			else {
				uint16_t pktcnt = (uint16_t)((epData->xfer_len + epData->maxpacket - 1U) / epData->maxpacket);
				epData->xfer_size = epData->maxpacket * pktcnt;

				clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= ((0x3FFUL << 19UL) & (uint32_t)pktcnt << 19);
				clbc_common.dc->base[DOEPTSIZ0 + epData->epNum * EP_STRIDE] |= (0x7FFFFUL & epData->xfer_size);
			}
		}

		if (epData->type == USB_EP_TYPE_ISOC) {
			if ((clbc_common.dc->base[DSTS] & (1 << 8)) == 0U) {
				clbc_common.dc->base[DOEPCTL0 + epData->epNum * EP_STRIDE] |= (1 << 29);
			}
			else {
				clbc_common.dc->base[DOEPCTL0 + epData->epNum * EP_STRIDE] |= (1 << 28);
			}
		}

		clbc_common.dc->base[DOEPCTL0 + epData->epNum * EP_STRIDE] |= ((1 << 26) | (1 << 31));
	}
}


int clbc_epTransmit(uint8_t epNum, uint8_t *dataBuf, uint32_t len)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];

	ep->in.xfer_buf = dataBuf;
	ep->in.xfer_len = len;
	ep->in.xfer_count = 0U;
	ep->in.is_in = 1U;
	ep->in.epNum = epNum;

	epStartXfer(&ep->in);
}


int clbc_epReceive(uint8_t epNum, uint8_t *dataBuf, uint32_t len)
{
	stm32n6_endpt_t *ep = &clbc_common.data->endpts[epNum];
	ep->out.xfer_buf = dataBuf;
	ep->out.xfer_len = len;
	ep->out.xfer_count = 0U;
	ep->out.is_in = 0;
	ep->out.epNum = epNum;

	epStartXfer(&ep->out);
}


void clbc_dataInStage(uint8_t epNum)
{
	clbc_common.dc->ep0State = USBD_EP0_STATUS_IN;
	clbc_epReceive(0, NULL, 0);
	// callback receive
}


void clbc_endptInit(void)
{
	/* 1. Set the NAK bit for all OUT endpoints */
	for (int i = 0; i < ENDPOINTS_NUMBER; i++) {
		clbc_common.dc->base[DOEPCTL0 + EP_STRIDE * i] |= (1 << 27);
	}

	/* 2. Unmask interrupt bits: INEP0, OUTEP0 */
	clbc_common.dc->base[DAINTMSK] |= ((1) | (1 << 16));
	/* 2. Unmask interrupt bits: STUPM, XFRCM */
	clbc_common.dc->base[DOEPMSK] |= ((1 << 3) | (1));
	/* 2. Unmask interrupt bits: XFRCM, TOM */
	clbc_common.dc->base[DIEPMSK] |= ((1) | (1 << 3));

	/* 3. FIFO RAM: OTG_GRXFSIZ	-  Set Rx FIFO */
	clbc_common.dc->base[GRXFSIZ] = RX_FIFO_DEPTH_WORDS;
	/* 3. FIFO RAM: OTG_DIEPTXF0 */
	clbc_common.dc->base[DIEPTXF0] = (TX0FD << 16) | TX0FSA;

	/* 4. DOEPTSIZ0: STUPCNT = 3 */
	clbc_common.dc->base[DOEPTSIZ0] |= (3 << 29);

	/* 5. For now we don't use DMA as we use FIFO */
}


void clbc_rxFifoData(void)
{
	uint32_t grxstpStatus = clbc_common.dc->base[GRXSTSP];


	uint32_t pktstsVal = ((grxstpStatus >> 17) & 0xF);


	switch (pktstsVal) {

		case 2:
			/*
			READ REGULAR PACKER

			(void)USB_ReadPacket(USBx, ep->xfer_buff,
							   (uint16_t)((RegVal & USB_OTG_GRXSTSP_BCNT) >> 4));

			ep->xfer_buff += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;
			ep->xfer_count += (RegVal & USB_OTG_GRXSTSP_BCNT) >> 4;

			*/
			return;

		case 4:
			/* SETUP PACKET RECEIVED */
			return;

		case 6:
			/* READ SETUP PACKET */
			uint32_t wordsToRead = (((grxstpStatus >> 4) & 2047U) + 3) / 4;
			for (uint32_t i = 0; i < wordsToRead; i++) {
				/* read 1 word from fifo*/
				((uint32_t *)(&clbc_common.dc->setup))[i] = clbc_common.dc->base[FIFO_BASE_OFF];
			}
			break;

		default:
			break;
	}
}


void clbc_flushTxFifo(uint32_t num)
{
	uint32_t timeout = 200000;
	while (!((clbc_common.dc->base[GRSTCTL] >> 31) & 1)) {
		if (--timeout == 0) {
			return;
		}
	}

	clbc_common.dc->base[GRSTCTL] |= (1 << 5 | (num << 6));

	timeout = 200000;
	while (((clbc_common.dc->base[GRSTCTL] >> 5) & 1)) {
		if (--timeout == 0) {
			return;
		}
	}
}


void clbc_sendEpData(int ep, uint8_t *virtAddr, int nBytes)
{
	uint8_t *pSrc = virtAddr;
	uint32_t count32b = (((uint32_t)nBytes + 3U) / 4U);
	for (uint32_t i = 0U; i < count32b; i++) {
		clbc_common.dc->base[FIFO_BASE_OFF] = __UNALIGNED_UINT32_READ(pSrc);
		pSrc++;
		pSrc++;
		pSrc++;
		pSrc++;
	}
	return;
}
