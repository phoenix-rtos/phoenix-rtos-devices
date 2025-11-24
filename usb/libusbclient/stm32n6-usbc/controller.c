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

#include "client.h"


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


volatile int dupa = 0;

/* quick actions to perform on interrupt */
int ctrl_hfIrq(void)
{
	/* Read active interrupt flags */
	uint32_t gintstsClear = ctrl_common.dc->base[GINTSTS];
	gintstsClear &= ctrl_common.dc->base[GINTMSK];

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


		if (dupa >= 1) {
			dupa++;
			clbc_rxFifoData(1);
		}
		else {
			clbc_rxFifoData(0);
		}


		ctrl_common.dc->base[GINTMSK] |= (1 << 4);
	}

	/* OEPINT */
	if ((gintstsClear >> 19) & 1) {
		uint8_t epNum = 0U;

		uint32_t daintClear = ctrl_common.dc->base[DAINT];
		daintClear &= ctrl_common.dc->base[DAINTMSK];
		daintClear &= 0xFFFF0000;
		daintClear >>= 16;

		while (daintClear != 0U) {
			if ((daintClear & 0x01U) != 0) {
				volatile uint32_t epInt = ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE];
				epInt &= ctrl_common.dc->base[DOEPMSK];

				/* XFRC */
				if (epInt & 1) {
					__asm__ volatile("1: b 1b");
					ctrl_common.dc->base[DOEPINT0 + epNum * EP_STRIDE] |= 1;

					// if (epNum == 0 && ctrl_common.data->endpts->out.xfer_len) {
					// 	clbc_ep0OutStart();
					// }
					// (void)PCD_EP_OutXfrComplete_int(hpcd, epnum);
				}

				if (dupa == 2) {
					__asm__ volatile("1: b 1b");
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
				uint32_t epMsk = ctrl_common.dc->base[DIEPMSK];
				uint32_t epEmp = ctrl_common.dc->base[DIEPEMPMSK];

				epMsk = ((epEmp >> (epNum & 0xFU)) & 0x1U) << 7;

				uint32_t epInt = ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE];
				epInt &= epMsk;

				/* XFRC */
				if (epInt & 1) {
					__asm__ volatile("1: b 1b");
					uint32_t fifoemptymsk = (uint32_t)(0x1UL << (epNum & 0xFU));
					ctrl_common.dc->base[DIEPEMPMSK] &= ~fifoemptymsk;
					ctrl_common.dc->base[DIEPINT0 + epNum * EP_STRIDE] |= 1;
					clbc_dataInStage(epNum);
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

					clbc_flushTxFifo(epNum);

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

					stm32n6_endpt_t *ep = &ctrl_common.data->endpts[epNum];

					// if (ep->in.xfer_count < ep->in.xfer_len) {

					// 	uint32_t len = ep->in.xfer_len - ep->in.xfer_count;
					// 	if (len > ep->in.maxpacket) {
					// 		len = ep->in.maxpacket;
					// 	}

					// 	clbc_sendEpData(epNum, ep->in.xfer_buf + ep->in.xfer_count, len);

					// 	ep->in.xfer_count += len;
					// }
					clbc_sendEpData(epNum, ep->in.xfer_buf, ep->in.xfer_len);
					dupa = 1;
					uint32_t fifoemptymsk = (uint32_t)(0x1UL << (epNum & 0xFU));
					ctrl_common.dc->base[DIEPEMPMSK] &= ~fifoemptymsk;
					// if (ep->in.xfer_count >= ep->in.xfer_len) {
					// 	uint32_t fifoemptymsk = (uint32_t)(0x1UL << (epNum & 0xFU));
					// 	ctrl_common.dc->base[DIEPEMPMSK] &= ~fifoemptymsk;
					// }

					// (void)PCD_WriteEmptyTxFifo(hpcd, epnum);
					// clbc_sendEpData(ctrl_common.fifoTxPrep.ep, ctrl_common.fifoTxPrep.virtAdress, ctrl_common.fifoTxPrep.nBytes);
					ctrl_common.dc->base[DIEPEMPMSK] &= ~(0x1);
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

		clbc_flushTxFifo(0x10U);

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

		clbc_ep0OutStart();

		// ep0OutStart();

		ctrl_common.dc->base[GINTSTS] |= (1 << 12);
	}

	/* ENUMDNE */
	if ((gintstsClear >> 13) & 1) {
		ctrl_common.dc->base[DIEPCTL0] &= ~(0x7FFUL);
		ctrl_common.dc->base[DCTL] |= (0x1UL << 8UL);

		clbc_enumdne();
		clbc_reset();

		ctrl_common.dc->base[GINTSTS] |= (1 << 13);
	}


	/* SOF */
	if ((gintstsClear >> 3) & 1) {

		// callback

		ctrl_common.dc->base[GINTSTS] |= (1 << 3);
	}


	/* GONAKEFFM / BOUTNAKEFF */
	if ((gintstsClear >> 7) & 1) {

		ctrl_common.dc->base[GINTMSK] &= ~(1 << 7);

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
}


int ctrl_execTransfer(int endpt, uint8_t *virtAddr, int nBytes)
{
	ctrl_common.fifoTxPrep.ep = endpt;
	ctrl_common.fifoTxPrep.virtAdress = virtAddr;
	ctrl_common.fifoTxPrep.nBytes = nBytes;
	/* setting XFERSIZ */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x7F);
	ctrl_common.dc->base[DIEPTSIZ0] |= (0x7F & 1);
	/* setting PKTCNT */
	ctrl_common.dc->base[DIEPTSIZ0] &= ~(0x180000);
	ctrl_common.dc->base[DIEPTSIZ0] |= (1 << 19);
	ctrl_common.dc->base[DIEPCTL0] |= (1 << 29);
	/* clear NAK and EPENA */
	ctrl_common.dc->base[DIEPCTL0] |= ((1 << 26) | (1 << 31));
	ctrl_common.dc->base[DIEPEMPMSK] |= 1;

	return 0;
}


void ctrl_lfIrq(void)
{
}


int ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	ctrl_common.dc = dc_in;
	ctrl_common.data = usb_data_in;

	clbc_endptInit();
	return 0;
}
