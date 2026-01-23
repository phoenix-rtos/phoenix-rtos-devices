#include "client.h"

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
	if (epType == USB_EP_DIR_IN) {
		clbc_common.dc->base[DAINTMSK] |= (0xFFFF & ((uint32_t)(1UL << (epNum & 0xFU))));

		if (((clbc_common.dc->base[DIEPCTL0 + epNum * EP_STRIDE] >> 15) & 1) == 0) {
			clbc_common.dc->base[DIEPCTL0 + epNum * EP_STRIDE] |= (maxPacket & 0x7FF) |
					((uint32_t)epType << 18) | (epNum << 22) |
					(0x1UL << 28UL) | (0x1UL << 15UL);
		}
	}
	else {
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
	activateEndpoint(0, USB_EP_DIR_IN, USB_EP_TYPE_CTRL, 64U);
}


static void ep0OutStart(void)
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
			ep0OutStart();
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
