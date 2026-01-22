#include <stdio.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/threads.h>
#include <sys/interrupt.h>
#include <errno.h>

#include "phy.h"
#include "client.h"
#include "helper.h"

#ifndef __PACKED_STRUCT
#define __PACKED_STRUCT struct __attribute__((packed, aligned(1)))
#endif

__PACKED_STRUCT T_UINT32_READ { uint32_t v; };
#define __UNALIGNED_UINT32_READ(addr) (((const struct T_UINT32_READ *)(const void *)(addr))->v)

typedef struct {
	int ep;
	uint8_t *virtAdress;
	int nBytes;
} dataToSendIN;

struct {
	usb_dc_t *dc;
	usb_common_data_t *data;
	dataToSendIN fifoTxPrep;
} ctrl_common;


enum {
	/* Core global control and status */
	usbotg_gotgctl = 0,
	usbotg_gotgint,
	usbotg_gahbcfg,
	usbotg_gusbcfg,
	usbotg_grstctl,
	usbotg_gintsts,
	usbotg_gintmsk,
	usbotg_grxstsr,
	usbotg_grxstsp,
	usbotg_grxfsiz,
	usbotg_dieptxf0,
	usbotg_hnptxsts,
	usbotg_gccfg = 14,
	usbotg_cid,
	usbotg_glpmcfg = 21,
	usbotg_hptxfsiz = 64,
	usbotg_dieptxf1,
	usbotg_dieptxf2,
	usbotg_dieptxf3,
	usbotg_dieptxf4,
	usbotg_dieptxf5,
	usbotg_dieptxf6,
	usbotg_dieptxf7,
	usbotg_dieptxf8,
	usbotg_dcfg = 512,
	usbotg_dctl,
	usbotg_dsts
};


volatile int mojstatus = 0;
volatile uint32_t tablicaStatusowRejestrow[10][3];
volatile int bereqineks = 0;
volatile uint8_t bereq[10];


void ctrl_endptInit(void)
{
	/* Endpoint initialization on USB reset */
	/* 1. Set the NAK bit for all OUT endpoints
	– SNAK = 1 in OTG_DOEPCTLx (for all OUT endpoints)
	2. Unmask the following interrupt bits
	– INEP0 = 1 in OTG_DAINTMSK (control 0 IN endpoint)
	– OUTEP0 = 1 in OTG_DAINTMSK (control 0 OUT endpoint)
	– STUPM = 1 in OTG_DOEPMSK
	– XFRCM = 1 in OTG_DOEPMSK
	– XFRCM = 1 in OTG_DIEPMSK
	– TOM = 1 in OTG_DIEPMSK
	3. Set up the data FIFO RAM for each of the FIFOs
	– Program the OTG_GRXFSIZ register, to be able to receive control OUT data and
	setup data. If thresholding is not enabled, at a minimum, this must be equal to 1
	max packet size of control endpoint 0 + 2 words (for the status of the control OUT
	data packet) + 10 words (for setup packets).
	– Program the OTG_DIEPTXF0 register (depending on the FIFO number chosen) to
	be able to transmit control IN data. At a minimum, this must be equal to 1 max
	packet size of control endpoint 0.
	4. Program the following fields in the endpoint-specific registers for control OUT endpoint 0
	to receive a SETUP packet
	– STUPCNT = 3 in OTG_DOEPTSIZ0 (to receive up to 3 back-to-back SETUP
	packets)
	5. For USB OTG in DMA mode, the OTG_DOEPDMA0 register must have a valid
	memory address to store any SETUP packets received.
	At this point, all initialization required to receive SETUP packets is done. */

	/* 1. Set the NAK bit for all OUT endpoints */
	for (int i = 0; i < ENDPOINTS_NUMBER; i++) {
		ctrl_common.dc->base[DOEPCTL0 + EP_STRIDE * i] |= (1 << 27);
	}

	/* 2. Unmask interrupt bits: INEP0, OUTEP0 */
	ctrl_common.dc->base[DAINTMSK] |= ((1) | (1 << 16));
	/* 2. Unmask interrupt bits: STUPM, XFRCM */
	ctrl_common.dc->base[DOEPMSK] |= ((1 << 3) | (1));
	/* 2. Unmask interrupt bits: XFRCM, TOM */
	ctrl_common.dc->base[DIEPMSK] |= ((1) | (1 << 3));

	/* 3. FIFO RAM: OTG_GRXFSIZ	-  Set Rx FIFO */
	ctrl_common.dc->base[GRXFSIZ] = RX_FIFO_DEPTH_WORDS;
	/* 3. FIFO RAM: OTG_DIEPTXF0 */
	ctrl_common.dc->base[DIEPTXF0] = (TX0FD << 16) | TX0FSA;

	/* 4. DOEPTSIZ0: STUPCNT = 3 */
	ctrl_common.dc->base[DOEPTSIZ0] |= (3 << 29);

	/* 5. For now we don't use DMA as we use FIFO */
}


int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	ctrl_common.dc = dc_in;
	ctrl_common.data = usb_data_in;

	ctrl_endptInit();
	return 0;
}


/* Page 3900 per rm0486 - Operational model */
static int ctrl_rxFifoData(void)
{
	/* 1. ... read the receive status pop register (OTG_GRXSTSP) */
	uint32_t grxstpStatus = ctrl_common.dc->base[GRXSTSP];

	// uint32_t epNum = (grxstpStatus & 0xF);

	/**
	 * GRXSTSP:
	 *
	 * [27] 	- STSPHST: 	Status phase start
	 * [24:21] 	- FRMNUM: 	Frame number
	 * [20:17] 	- PKTSTS: 	Packet status
	 * 						- 0001: Global OUT NAK (triggers an interrupt)
	 * 						- 0010: OUT data packet received
	 * 						- 0011: OUT transfer completed (triggers an interrupt)
	 * 						- 0100: SETUP transaction completed (triggers an interrupt)
	 * 						- 0110: SETUP data packet received
	 * 						- Others: Reserved
	 * [16:15] 	- DPID: 	Data PID
	 * [14:4] 	- BCNT: 	Byte count
	 * [3:0] 	- EPNUM: 	Endpoint number
	 */

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
			return 0;

		case 4:
			/* SETUP PACKET RECEIVED */
			return 0;

		case 6:
			/* READ SETUP PACKET */
			uint32_t wordsToRead = (((grxstpStatus >> 4) & 2047U) + 3) / 4;
			for (uint32_t i = 0; i < wordsToRead; i++) {
				/* read 1 word from fifo*/
				((uint32_t *)(&ctrl_common.dc->setup))[i] = ctrl_common.dc->base[FIFO_BASE_OFF];
			}
			break;

		default:
			break;
	}

	return 0;
}


static int ctrl_epInitOnEnumdne(void)
{
	/*1.On the Enumeration Done interrupt (ENUMDNE in OTG_GINTSTS), read the
	OTG_DSTS register to determine the enumeration speed.
	2.Program the MPSIZ field in OTG_DIEPCTL0 to set the maximum packet size. This
	step configures control endpoint 0. The maximum packet size for a control endpoint
	depends on the enumeration speed.
	3.For USB OTG in DMA mode, program the OTG_DOEPCTL0 register to enable control
	OUT endpoint 0, to receive a SETUP packet.
	At this point, the device is ready to receive SOF packets and is configured to perform control
	transfers on control endpoint 0.*/

	/* 1. READ DSTS*/
	uint32_t enum_speed = (ctrl_common.dc->base[DSTS] >> 1) & 3;

	/**
	 * TODO: TO VERIFY, RM  doens't specify the correlation MPSIZ <-> speed in TG_DIEPCTL0,
	 * used info about MPSIZ from DOEPCTL0 ([1:0] bits specified and set the other [10:2] bits to 0)
	 */
	switch (enum_speed) {
		case 0:
			/* HIGH SPEED */
			ctrl_common.dc->base[DIEPCTL0] &= ~2047U;
			ctrl_common.dc->base[DIEPCTL0] |= (MAX_PCKT_SZ_EP0_TX_B & DIEPCTL_MPSIZ_Msk);
			break;

		case 1:
			/* FULL SPEED */
			ctrl_common.dc->base[DIEPCTL0] &= ~2047U;
			ctrl_common.dc->base[DIEPCTL0] |= (MAX_PCKT_SZ_EP0_TX_B & DIEPCTL_MPSIZ_Msk);
			break;

		case 3:
			/* Reserved */
			break;

		default:
			break;
	}

	return 0;
}

#define STMCTRLDEBUG 0
#define maxmojstatus 6

void setDebug(uint32_t gintstsAsserted, uint32_t doepintAsserted, uint32_t diepintAsserted)
{
	if (STMCTRLDEBUG == 1 && maxmojstatus > mojstatus) {
		tablicaStatusowRejestrow[mojstatus][HELPER_GINSTSTS] = gintstsAsserted;
		tablicaStatusowRejestrow[mojstatus][HELPER_DOEPINT] = doepintAsserted;
		tablicaStatusowRejestrow[mojstatus][HELPER_DIEPINT] = diepintAsserted;
		mojstatus++;
	}
}

uint32_t wordsSending[10];
int iloscWyslanychSlow = 0;

volatile int lol2 = 0;
static void sendEpData(int ep, uint8_t *virtAddr, int nBytes)
{

	uint8_t *pSrc = virtAddr;
	uint32_t count32b = (((uint32_t)nBytes + 3U) / 4U);
	for (uint32_t i = 0U; i < count32b; i++) {
		ctrl_common.dc->base[FIFO_BASE_OFF] = __UNALIGNED_UINT32_READ(pSrc);
		pSrc++;
		pSrc++;
		pSrc++;
		pSrc++;
	}
	lol2 = 1;
	return;

	uint8_t buff[nBytes];
	uint32_t wordToSend[1];
	int fullWords = nBytes / 4;
	int bytesLeft = nBytes % 4;

	memcpy(buff, virtAddr, nBytes);

	uint32_t old = ctrl_common.dc->base[DTXFSTS0];

	/* send full words */
	for (int i = 0; i < fullWords; i++) {
		memcpy(wordToSend, buff + 4 * (uint8_t)i, 4);
		ctrl_common.dc->base[FIFO_BASE_OFF] = wordToSend[0];
		wordsSending[iloscWyslanychSlow] = wordToSend[0];
		iloscWyslanychSlow++;
		// __asm__ volatile ("1: b 1b");
	}

	/* send remaining bytes */
	if (bytesLeft > 0) {
		wordToSend[0] = 0;
		memcpy(wordToSend, buff + 4 * (uint8_t)fullWords, bytesLeft);
		ctrl_common.dc->base[FIFO_BASE_OFF] = wordToSend[0];
		wordsSending[iloscWyslanychSlow] = wordToSend[0];
		iloscWyslanychSlow++;
	}

	uint32_t new = ctrl_common.dc->base[DTXFSTS0];
	// __asm__ volatile ("1: b 1b");
}


void usb_flushTxFifo(uint32_t num)
{
	uint32_t timeout = 200000;
	while (!((ctrl_common.dc->base[GRSTCTL] >> 31) & 1)) {
		if (--timeout == 0) {
			return;
		}
	}

	ctrl_common.dc->base[GRSTCTL] |= (1 << 5 | (num << 6));

	timeout = 200000;
	while (((ctrl_common.dc->base[GRSTCTL] >> 5) & 1)) {
		if (--timeout == 0) {
			return;
		}
	}
}


static void ep0OutStart(void)
{
	uint32_t gSNPSiD = ctrl_common.dc->base[CID + 1];

	if (gSNPSiD > 0x4F54300AU) {
		if ((ctrl_common.dc->base[DOEPCTL0] >> 31) & 1) {
			return;
		}
	}

	ctrl_common.dc->base[DOEPTSIZ0] = 0U;
	ctrl_common.dc->base[DOEPTSIZ0] |= ((0x3FFUL << 19UL) & (1UL << 19));
	ctrl_common.dc->base[DOEPTSIZ0] |= (3U * 8U);
	ctrl_common.dc->base[DOEPTSIZ0] |= (0x3UL << 29UL);
}


static uint8_t getDevSpeed(void)
{
	uint8_t speed;
	uint32_t devEnumSpeed = ((ctrl_common.dc->base[DSTS] >> 1) & 3);

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

	ctrl_common.dc->base[GUSBCFG] &= ~(0xFUL << 10UL);
	ctrl_common.dc->base[GUSBCFG] |= (uint32_t)((usbTrd << 10) & (0xFUL << 10UL));
}


/* quick actions to perform on interrupt */
int ctrl_hfIrq(void)
{
	/* Read active interrupt flags */
	uint32_t gintstsAsserted = ctrl_common.dc->base[GINTSTS];
	uint32_t gintstsClear = (gintstsAsserted & GINTSTSWrMsk);

	if (gintstsClear == 0) {
		/* Invalid interrupt */
		return 0;
	}

	// frame number = dsts_fnsof

	/* MMIS */
	if ((gintstsClear >> 1) & 1) {
		ctrl_common.dc->base[GINTSTS] |= (1 << 1);
	}

	/* RXFLVL */
	if (gintstsClear >> 4) {
		ctrl_common.dc->base[GINTMSK] &= 0xFFFFFFEF;

		ctrl_rxFifoData();

		ctrl_common.dc->base[GINTMSK] |= (1 << 4);
	}

	/* OEPINT */
	if ((gintstsClear >> 19) & 1) {
		uint8_t epNum = 0U;

		uint32_t daintClear = ctrl_common.dc->base[DAINT];
		daintClear &= ctrl_common.dc->base[DAINTMSK];
		daintClear &= 0xFFFF0000;

		while (daintClear != 0U) {

			/* EP0 ? */
			if ((daintClear & 0x01U) != 0) {
				uint32_t epInt = ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE];
				epInt &= ctrl_common.dc->base[DOEPMSK];

				/* XFRC */
				if (epInt & 1) {
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= 1;
					// (void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
				}

				/* STUP */
				if ((epInt >> 3) & 1) {
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= (1 << 3);
					//(void)PCD_EP_OutSetupPacket_int(hpcd, epnum);
					desc_setup(&ctrl_common.dc->setup);
				}

				/* OTEPDIS */
				if ((epInt >> 4) & 1) {
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= (1 << 4);
				}

				/* EPDISD */
				if ((epInt >> 1) & 1) {
					/* CLEAR OUT ENDPOINT DISABLED INTERRUPT */

					if ((ctrl_common.dc->base[GINTSTS] >> 7) & 1) {
						ctrl_common.dc->base[DCTL] |= (1 << 10);
					}

					/* FOR ISOCHRONUS DATA */

					/*
					ep = &hpcd->OUT_ep[epnum];

					if (ep->is_iso_incomplete == 1U)
					{
						ep->is_iso_incomplete = 0U;

						#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
							hpcd->ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
						#else
							HAL_PCD_ISOOUTIncompleteCallback(hpcd, (uint8_t)epnum);
						#endif /* USE_HAL_PCD_REGISTER_CALLBACKS
					}
					*/

					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= (1 << 1);
				}

				/* STSPHSRX / OTEPSPR */
				if ((epInt >> 5) & 1) {
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= (1 << 5);
				}

				/* NAK */
				if ((epInt >> 13) & 1) {
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= (1 << 13);
				}
			}
			epNum++;
			daintClear >>= 1U;
		}
	} /* ENDIF GINTSTS_OEPINT */

	/* IEPINT */
	if ((gintstsClear >> 18) & 1) {
		uint8_t epNum = 0U;

		uint32_t daintClear = ctrl_common.dc->base[DAINT];
		daintClear &= ctrl_common.dc->base[DAINTMSK];
		daintClear &= 0xFFFF;

		while (daintClear != 0U) {

			/* EP0 ? */
			if ((daintClear & 0x01U) != 0) {
				uint32_t epInt = ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE];
				epInt &= ctrl_common.dc->base[DIEPMSK];

				/* XFRC */
				if (epInt & 1) {
					uint32_t fifoemptymsk = (uint32_t)(0x1UL << (epNum & 0xFU));
					ctrl_common.dc->base[DIEPEMPMSK] &= ~fifoemptymsk;

					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= 1;

					// HAL_PCD_DataInStageCallback(hpcd, (uint8_t)epnum);
				}

				/* TOC */
				if ((epInt >> 3) & 1) {
					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= (1 << 3);
				}

				/* ITTXFE */
				if ((epInt >> 4) & 1) {
					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= (1 << 4);
				}

				/* INEPNE */
				if ((epInt >> 6) & 1) {
					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= (1 << 6);
				}

				/* EPDISD */
				if ((epInt >> 1) & 1) {
					/* CLEAR OUT ENDPOINT DISABLED INTERRUPT */

					usb_flushTxFifo(epNum);

					/* FOR ISOCHRONUS DATA */

					/*
					ep = &hpcd->IN_ep[epnum];

					if (ep->is_iso_incomplete == 1U)
					{
						ep->is_iso_incomplete = 0U;

						#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
							hpcd->ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
						#else
							HAL_PCD_ISOINIncompleteCallback(hpcd, (uint8_t)epnum);
						#endif /* USE_HAL_PCD_REGISTER_CALLBACKS
					}
					*/

					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= (1 << 1);
				}

				/* TXFE */
				if ((epInt >> 7) & 1) {
					// (void)PCD_WriteEmptyTxFifo(hpcd, epnum);
					sendEpData(ctrl_common.fifoTxPrep.ep, ctrl_common.fifoTxPrep.virtAdress, ctrl_common.fifoTxPrep.nBytes);
				}
			}
			epNum++;
			daintClear >>= 1U;
		}
	} /* ENDIF GINTSTS_IEPINT */

	/* WKUPINT */
	if ((ctrl_common.dc->base[GINTSTS] >> 31) & 1) {

		/* RWUSIG */
		ctrl_common.dc->base[DCTL] &= ~(0x1);

		/*
		if (hpcd->LPM_State == LPM_L1)
		{
			hpcd->LPM_State = LPM_L0;

			#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
					hpcd->LPMCallback(hpcd, PCD_LPM_L0_ACTIVE);
			#else
					HAL_PCDEx_LPM_Callback(hpcd, PCD_LPM_L0_ACTIVE);
			#endif // USE_HAL_PCD_REGISTER_CALLBACKS
				}
				else
				{
			#if (USE_HAL_PCD_REGISTER_CALLBACKS == 1U)
					hpcd->ResumeCallback(hpcd);
			#else
					HAL_PCD_ResumeCallback(hpcd);
			#endif // USE_HAL_PCD_REGISTER_CALLBACKS
		}
	  */
		ctrl_common.dc->base[GINTSTS] |= (1 << 31);
	}

	/* USBSUSP */
	if ((gintstsClear >> 11) & 1) {

		// callback

		ctrl_common.dc->base[GINTSTS] |= (1 << 11);
	}


	/* LPMINT */


	/* USBRST */
	if ((gintstsClear >> 12) & 1) {

		ctrl_common.dc->base[DCTL] &= ~(0x1UL << 0UL);

		usb_flushTxFifo(0x10U);

		for (int i = 0; i < 9; i++) {
			ctrl_common.dc->base[DIEPINT0 + EP_STRIDE * i] = 0xFB7FU;
			ctrl_common.dc->base[DIEPCTL0 + EP_STRIDE * i] &= ~(0x1UL << 21UL);
			ctrl_common.dc->base[DOEPINT0 + EP_STRIDE * i] = 0xFB7FU;
			ctrl_common.dc->base[DOEPCTL0 + EP_STRIDE * i] &= ~(0x1UL << 21UL);
			ctrl_common.dc->base[DOEPCTL0 + EP_STRIDE * i] |= (0x1UL << 27UL);
		}
		ctrl_common.dc->base[DAINTMSK] |= 0x10001U;

		ctrl_common.dc->base[DOEPMSK] |= ((0x1UL << 3UL) | (0x1UL) | (0x1UL << 1UL) | (0x1UL << 5UL) | (0x1UL << 13UL));

		ctrl_common.dc->base[DIEPMSK] |= ((0x1UL << 3UL) | (0x1UL) | (0x1UL << 1UL));

		ctrl_common.dc->base[DCFG] &= ~(0x7FUL << 4UL);

		ep0OutStart();

		ctrl_common.dc->base[GINTSTS] |= (1 << 12);
	}

	/* ENUMDNE */
	if ((gintstsClear >> 13) & 1) {
		ctrl_common.dc->base[DIEPCTL0] &= ~(0x7FFUL);
		ctrl_common.dc->base[DCTL] |= (0x1UL << 8UL);

		uint8_t speed = getDevSpeed();
		uint32_t hclkFreq = phy_getHclkFreq();

		setTurnaroundTime(hclkFreq, speed);

		// callback

		ctrl_common.dc->base[GINTSTS] |= (1 << 13);
	}


	/* SOF */
	if ((gintstsClear >> 3) & 1) {

		// callback

		ctrl_common.dc->base[GINTSTS] |= (1 << 3);
	}


	/* GONAKEFFM / BOUTNAKEFF */
	if ((gintstsClear >> 7) & 1) {

		ctrl_common.dc->base[GINTSTS] |= (1 << 7);

		// for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
		// {
		// 	if (hpcd->OUT_ep[epnum].is_iso_incomplete == 1U)
		// 	{
		// 	/* disable the EP */
		// 	USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_SNAK);
		// 	USBx_OUTEP(epnum)->DOEPCTL |= (USB_OTG_DOEPCTL_EPDIS);
		// 	}
		// }
	}

	/* IISOIXFR */
	if ((gintstsClear >> 20) & 1) {

		// for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
		// {
		// 	RegVal = USBx_INEP(epnum)->DIEPCTL;

		// 	if ((hpcd->IN_ep[epnum].type == EP_TYPE_ISOC) &&
		// 		((RegVal & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA))
		// 	{
		// 	hpcd->IN_ep[epnum].is_iso_incomplete = 1U;

		// 	/* Abort current transaction and disable the EP */
		// 	(void)HAL_PCD_EP_Abort(hpcd, (uint8_t)(epnum | 0x80U));
		// 	}
		// }

		ctrl_common.dc->base[GINTSTS] |= (1 << 20);
	}

	/* PXFR_INCOMPISOOUT - incomplete iso interrupt */
	if ((gintstsClear >> 21) & 1) {
		// for (epnum = 1U; epnum < hpcd->Init.dev_endpoints; epnum++)
		// {
		// 	RegVal = USBx_OUTEP(epnum)->DOEPCTL;

		// 	if ((hpcd->OUT_ep[epnum].type == EP_TYPE_ISOC) &&
		// 		((RegVal & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA) &&
		// 		(((RegVal & (0x1UL << 16)) >> 16U) == (hpcd->FrameNumber & 0x1U)))
		// 	{
		// 	hpcd->OUT_ep[epnum].is_iso_incomplete = 1U;

		// 	USBx->GINTMSK |= USB_OTG_GINTMSK_GONAKEFFM;

		// 	if ((USBx->GINTSTS & USB_OTG_GINTSTS_BOUTNAKEFF) == 0U)
		// 	{
		// 		USBx_DEVICE->DCTL |= USB_OTG_DCTL_SGONAK;
		// 		break;
		// 	}
		// 	}
		// }
		ctrl_common.dc->base[GINTSTS] |= (1 << 21);
	}

	/* SRQINT */
	if ((gintstsClear >> 30) & 1) {

		// callback

		ctrl_common.dc->base[GINTSTS] |= (1 << 30);
	}


	/* OTGINT */
	if ((gintstsClear >> 2) & 1) {

		uint32_t regVal = ctrl_common.dc->base[GOTGINT];

		// callback using regval
		regVal &= 0x40004;

		ctrl_common.dc->base[GOTGINT] |= regVal;
	}


	return 0;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	uint32_t doepintAsserted = ctrl_common.dc->base[DOEPINT0];
	uint32_t doepintClear = (doepintAsserted & DOEPINTxWrMsk);
	uint32_t diepintAsserted = ctrl_common.dc->base[DIEPINT0];
	uint32_t diepintClear = (diepintAsserted & DIEPINTxWrMsk);
	/* READ ONLY */
	uint32_t deviceAllIrq = ctrl_common.dc->base[DAINT];

	/* Clear interrupt flags*/
	ctrl_common.dc->base[GINTSTS] |= gintstsClear;
	ctrl_common.dc->base[DOEPINT0] |= doepintClear;
	ctrl_common.dc->base[DIEPINT0] |= diepintClear;

	if ((gintstsAsserted >> 1) & 1) {
		// printf("mode mismatch interrupt");
	}

	// if (mojstatus > 6) {
	// 	return 1;
	// }
	/* USBRST */
	if ((gintstsAsserted >> 12) & 1) {
		setDebug(gintstsAsserted, doepintAsserted, diepintAsserted);
		if (ctrl_reset() < 0) {
			return -1;
		}
	}

	/* ENUMDNE */
	if ((gintstsAsserted >> 13) & 1) {
		if (ctrl_epInitOnEnumdne() < 0) {
			return -1;
		}
		if (ctrl_ep0SetOnEnumdne() < 0) {
			return -1;
		}
	}

	/* RXFLVL */
	if ((gintstsAsserted >> 4) & 1) {

		ctrl_common.dc->base[GINTMSK] &= 0xFFFFFFEF;

		setDebug(gintstsAsserted, doepintAsserted, diepintAsserted);

		ctrl_rxFifoData();

		ctrl_common.dc->base[GINTMSK] |= (1 << 4);
	}

	/* STUP */
	if ((doepintAsserted >> 3) & 1) {
		// mojstatus++;
		if (ctrl_common.dc->setup.bRequest == 5) {
			// printf("5s");
		}

		uint32_t numSetupPckRec = (ctrl_common.dc->base[DOEPTSIZ0] >> 29) & 3;
		setDebug(gintstsAsserted, doepintAsserted, diepintAsserted);
		desc_setup(&ctrl_common.dc->setup);
	}

	if ((gintstsAsserted >> 18) & 1) {

		if (ctrl_common.dc->base[DAINT] & 1) {
			/* TXFE - ready to send data*/
			if ((diepintAsserted >> 7) & 1) {
				sendEpData(ctrl_common.fifoTxPrep.ep, ctrl_common.fifoTxPrep.virtAdress, ctrl_common.fifoTxPrep.nBytes);
				// static volatile int debug_trap = 1;
				// while (debug_trap) {
				// 	// PROCESOR WAIT -- DEBUG - REMOVE IT
				// }
				// ctrl_common.dc->base[DOEPTSIZ0] &= 0xFFFFFF80;
				// ctrl_common.dc->base[DOEPTSIZ0] |= 0x8;
				// /* PKTCNT in DOEPTSIZ0 */
				// ctrl_common.dc->base[DOEPTSIZ0] |= (1 << 19);
				// /* SUPCNT = 1 (or 2 or 3) */
				// ctrl_common.dc->base[DOEPTSIZ0] |= (3 << 29);
				// /* 1. Program the OTG_DOEPCTLx register */
				// ctrl_common.dc->base[DOEPCTL0] |= ((1 << 31) | (1 << 26));

				setDebug(gintstsAsserted, doepintAsserted, diepintAsserted);
				ctrl_common.dc->base[DIEPEMPMSK] &= 0xFFFFFFFE;
			}
		}
	}

	// if (lol2 == 1) {
	// 	__asm__ volatile ("1: b 1b");
	// }

	/* XFRC */
	if (diepintAsserted & 1) {
		// if (~(diepintAsserted >> 4) & 1) {
		// __asm__ volatile("1: b 1b");
		// }
		setDebug(gintstsAsserted, doepintAsserted, diepintAsserted);
		/* prepare OUT ep for ZLP from host */

		// ctrl_ep0SetOnEnumdne();
		// mojstatus++;
		// if (mojstatus == 5) {
		// 	ctrl_common.dc->base[DCFG] &= ~(0x7F0);
		// 	if (ctrl_common.dc->dev_addr != 0) {
		// 		ctrl_common.dc->base[DCFG] |= ctrl_common.dc->dev_addr;
		// 		printf("ts");
		// 	}
		// }
	}


	// if ((diepintAsserted >> 7) & 1) {
	// 	desc_setup(&ctrl_common.dc->setup);
	// }


	return 0;
}


/* page 3904 per rm0486, section: Transfer Stop Programming for OUT endpoints */
int ctrl_reset(void)
{
	/*
1.Enable all OUT endpoints by setting
EPENA = 1 in all OTG_DOEPCTLx registers.

2. Flush the RxFIFO as follows
–Poll OTG_GRSTCTL.AHBIDL until it is 1. This indicates that AHB master is idle.
–Perform read modify write operation on OTG_GRSTCTL.RXFFLSH =1
–Poll OTG_GRSTCTL.RXFFLSH until it is 0, but also using a timeout of less than
10 milli-seconds (corresponds to minimum reset signaling duration). If 0 is seen
before the timeout, then the RxFIFO flush is successful. If at the moment the
timeout occurs, there is still a 1, (this may be due to a packet on EP0 coming from
the host) then go back (once only) to the previous step (“Perform read modify write
operation”).
3.Before disabling any OUT endpoint, the application must enable Global OUT NAK
mode in the core, according to the instructions in “Setting the global OUT NAK”. This
ensures that data in the RxFIFO is sent to the application successfully. Set SGONAK =
1 in OTG_DCTL
4.Wait for the GONAKEFF interrupt (OTG_GINTSTS)
5.Disable all active OUT endpoints by programming the following register bits:
–EPDIS = 1 in registers OTG_DOEPCTLx
–SNAK = 1 in registers OTG_DOEPCTLx
6. Wait for the EPDIS interrupt in OTG_DOEPINTx for each OUT endpoint programmed
in the previous step. The EPDIS interrupt in OTG_DOEPINTx indicates that the
corresponding OUT endpoint is completely disabled. When the EPDIS interrupt is
asserted, the following bits are cleared:
–EPENA = 0 in registers OTG_DOEPCTLx
–EPDIS = 0 in registers OTG_DOEPCTLx
–SNAK = 0 in registers OTG_DOEPCTLx*/
	/* 1. EPENA = 1 in all OTG_DOEPCTLx */

	// while ((ctrl_common.dc->base[GINTMSK] >> 7))

	usb_flushTxFifo(0x1F);
	return 0;
}


/* page 3905 per rm0486, section: Generic non-isochronous OUT data transfers - FOR NOW ONLY EP0*/
int ctrl_ep0SetOnEnumdne(void)
{
	/* 1. Program the OTG_DOEPTSIZx register*/
	/* SETUP packet is always single packet with size of 8 bites - XFRSIZ in DOEPTSIZ0 */
	ctrl_common.dc->base[DOEPTSIZ0] &= 0xFFFFFF80;
	ctrl_common.dc->base[DOEPTSIZ0] |= 0x8;

	/* PKTCNT in DOEPTSIZ0 */
	ctrl_common.dc->base[DOEPTSIZ0] |= (1 << 19);

	/* SUPCNT = 1 (or 2 or 3) */
	// ctrl_common.dc->base[DOEPTSIZ0] |= (3 << 29);

	/* 1. Program the OTG_DOEPCTLx register */
	ctrl_common.dc->base[DOEPCTL0] |= ((1 << 31) | (1 << 26));

	return 0;
}


int ctrl_awaitEpInit(void)
{
	/* Endpoint initialization on enumeration completion */
	/* 1. On the Enumeration Done interrupt (ENUMDNE in OTG_GINTSTS), read the
	OTG_DSTS register to determine the enumeration speed.
	2. Program the MPSIZ field in OTG_DIEPCTL0 to set the maximum packet size. This
	step configures control endpoint 0. The maximum packet size for a control endpoint
	depends on the enumeration speed.
	3. For USB OTG in DMA mode, program the OTG_DOEPCTL0 register to enable control
	OUT endpoint 0, to receive a SETUP packet. - N/A for now */

	uint32_t timeout = 0;

	while (((ctrl_common.dc->base[GINTSTS] >> 13) & 1) == 0) {
		usleep(50);
		timeout++;
		if (timeout > 0x0FFFFFFF) {
			printf(" USBC: timeout waiting for ENUMDNE in init\n");
			break;
		}
	}

	uint32_t enum_speed = (ctrl_common.dc->base[DSTS] >> 1) & 3;
	printf(" USBC: enum speed(0=Hs, 1=Fs, 3=Res): %d\n", enum_speed);
}


static void ctrl_writeFifo(void *base, int fifo_num, const void *data, uint32_t len)
{
}


int ctrl_execTransfer(int endpt, uint8_t *virtAddr, int nBytes)
{
	ctrl_common.fifoTxPrep.ep = endpt;
	ctrl_common.fifoTxPrep.virtAdress = virtAddr;
	ctrl_common.fifoTxPrep.nBytes = nBytes;

	/* setting XFERSIZ */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x7F);
	ctrl_common.dc->base[DIEPTSIZ0] |= (0x7F & nBytes);
	/* setting PKTCNT */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x180000);
	ctrl_common.dc->base[DIEPTSIZ0] |= (1 << 19);
	ctrl_common.dc->base[DIEPCTL0] |= (1 << 29);

	/* clear NAK and EPENA */
	ctrl_common.dc->base[DIEPCTL0] |= ((1 << 26) | (1 << 31));
	ctrl_common.dc->base[DIEPEMPMSK] |= 1;

	return 0;
}

void ctrl_setAddress(uint32_t addr)
{
}


void ctrl_rx_fifo_handler(void)
{
}


void print_hex(const void *ptr, size_t n)
{
	const uint8_t *data = (const uint8_t *)ptr;

	for (size_t i = 0; i < n; i++) {
		printf("%02X ", data[i]);

		if ((i + 1) % 16 == 0) {
			printf("\n");
		}
	}
	printf("\n");
}


void ctrl_lfIrq(void)
{
	// int statyczna = 4;
	// static int lol = 0;
	// if (mojstatus > statyczna) {
	// 	ctrl_common.dc->base[GINTMSK] &= 0xFFFFFFEF;
	// 	if (lol > 0) {
	// 		return;
	// 	}
	// 	lol++;
	// 	for (int i = 0; i < bereqineks; i++) {
	// 		printf("bereq nr %d: %d\n", i, bereq[i]);
	// 	}

	// 	for (int i = 0; i < mojstatus; i++) {
	// 		printf("\nnumer iteracji: %d\n", i);
	// 		for (int j = 0; j < 3; j++) {
	// 			helper_showRegisterInfo(tablicaStatusowRejestrow[i][j], j);
	// 		}
	// 	}
	// 	printf("\nbmRqType: %d\nbRq: %d\nwIndex: %d\nwLength: %d\nwValue: %d\n", ctrl_common.dc->setup.bmRequestType, ctrl_common.dc->setup.bRequest, ctrl_common.dc->setup.wIndex, ctrl_common.dc->setup.wLength, ctrl_common.dc->setup.wValue);
	// 	printf("\n\nGET_DESCRIPTOR_DEV_: ");
	// 	print_hex(ctrl_common.fifoTxPrep.virtAdress, ctrl_common.fifoTxPrep.nBytes);
	// 	printf("\nCo wysylam hostowi: ");
	// 	print_hex(wordsSending, iloscWyslanychSlow * 4);
	// }
	// ctrl_common.dc->base[GINTMSK] |= (1 << 4);
	// if (lol == 0 && lol2 == 1) {
	// 	phy_getHclkFreq();
	// 	lol=1;
	// }
}
