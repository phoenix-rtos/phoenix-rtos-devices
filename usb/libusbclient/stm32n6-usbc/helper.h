#ifndef _STM32N6_LIBUSB_HELPER_
#define _STM32N6_LIBUSB_HELPER_

#include <stdio.h>
#include <stdint.h>


enum {
	HELPER_GINSTSTS,
	HELPER_DOEPINT,
	HELPER_DIEPINT,
	HELPER_DOEPCTL0,
	HELPER_DIEPCTL0
};


extern void helper_showRegisterInfo(uint32_t regVal, int registerH);

#endif
