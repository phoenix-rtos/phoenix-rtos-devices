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


struct {
	usb_dc_t *dc;
	usb_common_data_t *data;
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

	/**
	 * TODO: RADEK	- jak jest zerowana flaga RXFLVL?
	 * 				- sprawdznanie ilości otrzymanych danych (STUP?) - tych co się zmniejszają o 1 za każdym razem jak czytamy z fifo
	 */

	/* 2. The application can mask the RXFLVL interrupt (in OTG_GINTSTS) */
	ctrl_common.dc->base[GINTMSK] &= 0xFFFFFFEF;


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

	if ((grxstpStatus >> 17) & 2) {
		/* read (BCNT+3)/4 words from FIFO - round up to full word*/
		uint32_t wordsToRead = (((grxstpStatus >> 4) & 2047U) + 3) / 4;
	}
	if ((grxstpStatus >> 17) & 6) {
		uint32_t wordsToRead = (((grxstpStatus >> 4) & 2047U) + 3) / 4;
		for (uint32_t i = 0; i < wordsToRead; i++) {
			/* read 1 word from fifo*/
			((uint32_t *)(&ctrl_common.dc->setup))[i] = ctrl_common.dc->base[FIFO_BASE_OFF];
		}
	}

	/* Unmask RXFLVL interrupt (in OTG_GINTSTS) */
	ctrl_common.dc->base[GINTMSK] |= (1 << 4);
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

		case 1:
			/* FULL SPEED */
			ctrl_common.dc->base[DIEPCTL0] &= ~2047U;
			break;

		case 3:
			/* Reserved */
			break;

		default:
			break;
	}

	return 0;
}

/**
 * TODO: RADEK
 * - action on RXFLVL - in the next episode...
 */
int ctrl_hfIrq(void)
{
	/* Read active interrupt flags */
	uint32_t intStatsAserted = ctrl_common.dc->base[GINTSTS];
	uint32_t intStatsClear = (intStatsAserted & OTG_GINTSTS_DEVICE_MASK);
	uint32_t doepintSts = ctrl_common.dc->base[DOEPINT0];

	/* Clear interrupt flags*/
	ctrl_common.dc->base[GINTSTS] |= intStatsClear;

	/**
	 * TODO: REMOVE IT
	 * For debug
	 */
	ctrl_common.dc->setupstat = intStatsAserted;


	/**
	 * ON DEVICE INIT
	 * 1. USB RESET
	 */
	if ((intStatsAserted >> 12) & 1) {
		if (ctrl_reset() < 0) {
			return -1;
		}
	}

	/**
	 * ON DEVICE INIT
	 * 2. ENUMDNE
	 */
	if ((intStatsAserted >> 12) & 1) {
		if (ctrl_epInitOnEnumdne() < 0) {
			return -1;
		}
		if (ctrl_ep0SetOnEnumdne() < 0) {
			return -1;
		}
	}

	/**
	 * ON DEVICE INIT
	 * 2. RXFLVL
	 */
	if ((intStatsAserted >> 4) & 1) {
		ctrl_rxFifoData();
		if (ctrl_common.dc->setup.bRequest == REQ_SET_ADDRESS) {
			printf("ps");
		}
	}

	/**
	 * ON DEVICE INIT
	 * 3. STUP
	 */
	if ((doepintSts >> 3) & 1) {
		// ctrl_common.dc->base[DOEPINT0] |= (1 << 3);
		uint32_t numSetupPckRec = (ctrl_common.dc->base[DOEPTSIZ0] >> 29) & 3;
		if (ctrl_common.dc->setup.bRequest == REQ_SET_ADDRESS) {
			printf("ps");
		}
		desc_setup(&ctrl_common.dc->setup);
	}


	return 1;
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

	return 0;
}


/* Transfer Stop Programming for OUT endpoints */
static int ctrl_transferStop(void)
{
	for (int i = 0; i < ENDPOINTS_NUMBER; i++) {
		ctrl_common.dc->base[DOEPCTL0 + EP_STRIDE * i] |= (1 << 31);
	}

	/* 2. Poll OTG_GRSTCTL.AHBIDL until it is 1 */
	while (!((ctrl_common.dc->base[GRSTCTL] >> 31) & 1))
		;

	/* 2. Perform read modify write operation on OTG_GRSTCTL.RXFFLSH =1 */
	ctrl_common.dc->base[GRSTCTL] |= (1 << 4);

	/* 2. Poll OTG_GRSTCTL.RXFFLSH until it is 0, but... */
	int timer = 0;
	while ((ctrl_common.dc->base[GRSTCTL] >> 4) & 1) {
		usleep(1);
		timer++;
		if (timer > 1e4) {
			timer = 0;
			/* go back (once only) to the previous step */
			ctrl_common.dc->base[GRSTCTL] |= (1 << 4);

			while ((ctrl_common.dc->base[GRSTCTL] >> 4) & 1) {
				usleep(1);
				timer++;
				if (timer > 1e4) {
					return -1;
				}
				break;
			}
		}
	}

	/* We can set SGONAK only after making sure the GONAKEFF i cleared */
	ctrl_common.dc->base[DCTL] |= (1 << 10);
	while ((ctrl_common.dc->base[GINTSTS] >> 7) & 1)
		;
	/* 3. enable Global OUT NAK mode in the core - Set SGONAK = 1 in OTG_DCTL*/
	ctrl_common.dc->base[DCTL] |= (1 << 9);
	ctrl_common.dc->base[GINTMSK] |= (1 << 7);

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
	ctrl_common.dc->base[DOEPTSIZ0] |= (3 << 29);

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
	uint8_t buff[nBytes];
	uint32_t wordToSend[1];
	int fullWords = nBytes / 4;
	int bytesLeft = nBytes % 4;

	memcpy(buff, virtAddr, nBytes);

	/* setting XFERSIZ */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x7F);
	ctrl_common.dc->base[DIEPTSIZ0] |= 0x7F & nBytes;
	/* setting PKTCNT */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x180000);
	ctrl_common.dc->base[DIEPTSIZ0] |= (1 << 19);

	/* clear NAK */
	ctrl_common.dc->base[DIEPCTL0] |= (1 << 26);
	/* set EPENA */
	ctrl_common.dc->base[DIEPCTL0] |= (1 << 31);

	/* send full words */
	for (int i = 0; i < fullWords; i++) {
		memcpy(wordToSend, buff + 4 * (uint8_t)i, 4);
		ctrl_common.dc->base[FIFO_BASE_OFF + RX_FIFO_DEPTH_WORDS] = wordToSend[0];
	}

	/* send remaining bytes */
	wordToSend[0] = 0;
	memcpy(wordToSend, buff + 4 * (uint8_t)fullWords, bytesLeft);
	ctrl_common.dc->base[FIFO_BASE_OFF + RX_FIFO_DEPTH_WORDS] = wordToSend[0];

	return 0;
}


void ctrl_setAddress(uint32_t addr)
{
}


void ctrl_rx_fifo_handler(void)
{
}


void ctrl_lfIrq(void)
{
}
