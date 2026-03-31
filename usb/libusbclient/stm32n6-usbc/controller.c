/*
 * Phoenix-RTOS
 *
 * libusbclient (STM32 N6)
 *
 * IRQ controller
 *
 * Copyright 2026 Phoenix Systems
 * Author: Radosław Szewczyk, Rafał Mikielis
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

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

#define USB_REG(x) *(ctrl_common.dc->base + x)

struct {
	usb_dc_t *dc;
	usb_common_data_t *data;
} ctrl_common;


void ctrl_init(usb_common_data_t *usb_data_in, usb_dc_t *dc_in)
{
	ctrl_common.data = usb_data_in;
	ctrl_common.dc = dc_in;
}


/* quick actions to perform on interrupt */
void ctrl_hifiq_handler(uint32_t *irqStatus)
{
	uint8_t epNum;
	uint32_t reg;
	uint32_t daintClear;

	/* USBRST */
	if (IS_IRQ(*irqStatus, GINTSTS_USBRST)) {

		/* clear IRQ */
		USB_REG(otg_gintsts) = (1UL << GINTSTS_USBRST);
	}

	/* ENUMDNE */
	if (IS_IRQ(*irqStatus, GINTSTS_ENUMDNE)) {

		/* clear IRQ */
		USB_REG(otg_gintsts) = (1UL << GINTSTS_ENUMDNE);
	}

	/* RXFLVL */
	if (IS_IRQ(*irqStatus, GINTSTS_RXFLVL)) {

		/* mask IRQ */
		USB_REG(otg_gintmsk) &= ~(1UL << GINTSTS_RXFLVL);

		clbc_rxFifoData();

		/* unmask IRQ */
		USB_REG(otg_gintmsk) |= (1UL << GINTSTS_RXFLVL);
		*irqStatus &= ~(1UL << GINTSTS_RXFLVL);
	}

	/* OEPINT */
	if (IS_IRQ(*irqStatus, GINTSTS_OEPINT)) {

		epNum = 0U;
		daintClear = USB_REG(otg_daint);
		daintClear &= USB_REG(otg_daintmsk);
		daintClear &= 0xFFFF0000UL;
		ctrl_common.dc->daintClear |= daintClear;

		daintClear >>= 16;

		while (daintClear != 0U) {
			if ((daintClear & 0x1U) != 0U) {
				reg = USB_REG(otg_doepint0 + epNum * EP_STRIDE);

				/* save DOEPINT snapshot */
				ctrl_common.dc->irqPendingDOEPINT[epNum] |= reg;

				/* clear IRQ */
				USB_REG(otg_doepint0 + epNum * EP_STRIDE) = reg;
			}
			epNum++;
			daintClear >>= 1U;
		}
	}

	/* IEPINT */
	if (IS_IRQ(*irqStatus, GINTSTS_IEPINT)) {

		epNum = 0U;
		daintClear = USB_REG(otg_daint);
		daintClear &= USB_REG(otg_daintmsk);
		daintClear &= 0xFFFFUL;
		ctrl_common.dc->daintClear |= daintClear;

		while (daintClear != 0U) {
			if ((daintClear & 0x1U) != 0U) {
				reg = USB_REG(otg_diepint0 + epNum * EP_STRIDE);

				/* save DIEPINT snapshot */
				ctrl_common.dc->irqPendingDIEPINT[epNum] |= reg;

				if (IS_IRQ(reg, DIEPINT_TXFE)) {
					/* TXFE must be serviced in ISR */
					USB_REG(otg_diepempmsk) &= ~(1UL << epNum);
					ctrl_common.dc->irqPendingDIEPINT[epNum] &= ~(1UL << DIEPINT_TXFE);
					clbc_sendEpData(epNum);
				}

				/* clear IRQ */
				USB_REG(otg_diepint0 + epNum * EP_STRIDE) = reg;
			}
			epNum++;
			daintClear >>= 1U;
		}
	}

	/* MMIS */
	if (IS_IRQ(*irqStatus, GINTSTS_MMIS)) {

		USB_REG(otg_gintsts) = (1 << GINTSTS_MMIS);
		*irqStatus &= ~(1UL << GINTSTS_MMIS);
	}

	/* WKUPINT */
	if (IS_IRQ(*irqStatus, GINTSTS_WKUPINT)) {

		/* RWUSIG */
		USB_REG(otg_dctl) &= ~(0x1);

		USB_REG(otg_gintsts) = (1UL << GINTSTS_WKUPINT);
		*irqStatus &= ~(1UL << GINTSTS_WKUPINT);
	}

	/* ESUSP */
	if (IS_IRQ(*irqStatus, GINTSTS_ESUSP)) {

		USB_REG(otg_gintsts) = (1UL << GINTSTS_ESUSP);
		*irqStatus &= ~(1UL << GINTSTS_ESUSP);
	}

	/* USBSUSP */
	if (IS_IRQ(*irqStatus, GINTSTS_USBSUSP)) {

		USB_REG(otg_gintsts) = (1UL << GINTSTS_USBSUSP);
	}

	/* SOF */
	if (IS_IRQ(*irqStatus, GINTSTS_SOF)) {

		USB_REG(otg_gintsts) = (1UL << GINTSTS_SOF);
		*irqStatus &= ~(1UL << GINTSTS_SOF);
	}

	/* OTGINT */
	if (IS_IRQ(*irqStatus, GINTSTS_OTGINT)) {

		reg = USB_REG(otg_gotgint);
		USB_REG(otg_gotgint) = reg;
		*irqStatus &= ~(1UL << GINTSTS_OTGINT);
	}

	__asm__ volatile("dsb");
}


void ctrl_lifiq_handler(uint32_t *irqStatus)
{
	uint8_t epNum;

	/* USBRST */
	if (IS_IRQ(*irqStatus, GINTSTS_USBRST)) {

		*irqStatus &= ~(1UL << GINTSTS_USBRST);
		ctrl_common.dc->currEvent = USBCLIENT_EV_RESET;

		_clbc_USBRST();
	}

	/* ENUMDNE */
	if (IS_IRQ(*irqStatus, GINTSTS_ENUMDNE)) {

		ctrl_common.dc->deviceState = USBD_STATE_DEFAULT;
		*irqStatus &= ~(1UL << GINTSTS_ENUMDNE);
		ctrl_common.dc->currEvent = USBCLIENT_EV_CONNECT;

		_clbc_ENUMDNE();
	}

	/* USBSUSP */
	if (IS_IRQ(*irqStatus, GINTSTS_USBSUSP)) {

		ctrl_common.dc->deviceState = USBD_STATE_DEFAULT;
		ctrl_common.dc->ep0State = USBD_EP0_IDLE;
		ctrl_common.dc->currEvent = USBCLIENT_EV_DISCONNECT;

		/* Release semaphores, terminate transfers */
		for (epNum = 0U; epNum < ENDPOINTS_NUMBER; epNum++) {
			ctrl_common.data->endpts[epNum].in.xfer_active = 0U;
			ctrl_common.data->endpts[epNum].out.xfer_active = 0U;
		}

		if (ctrl_common.dc->semBulkRx.v == 0U) {
			semaphoreUp(&ctrl_common.dc->semBulkRx);
		}
		if (ctrl_common.dc->semBulkTx.v == 0U) {
			semaphoreUp(&ctrl_common.dc->semBulkTx);
		}

		*irqStatus &= ~(1UL << GINTSTS_USBSUSP);
	}

	/* OEPINT */
	if (IS_IRQ(*irqStatus, GINTSTS_OEPINT)) {

		*irqStatus &= ~(1UL << GINTSTS_OEPINT);

		_clbc_OEPINT();
	}

	/* IEPINT */
	if (IS_IRQ(*irqStatus, GINTSTS_IEPINT)) {

		*irqStatus &= ~(1UL << GINTSTS_IEPINT);

		_clbc_IEPINT();
	}
}
