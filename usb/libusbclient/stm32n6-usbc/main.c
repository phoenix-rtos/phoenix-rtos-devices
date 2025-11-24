/*
 * STM32N6 USB - physical layer test
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include "phy.h"


#define GINT_USBRST  (1 << 12)
#define GINT_ENUMDNE (1 << 13)
#define GINT_SOF     (1 << 3)


void debug_dump_regs(volatile uint32_t *otg)
{
	printf("\n--- USB REG DUMP ---\n");
	printf("GOTGCTL (Control):  0x%08X\n", otg[0x000 / 4]);
	printf("GINTSTS (Interrupt):0x%08X\n", otg[0x014 / 4]);
	printf("DSTS    (Dev Stat): 0x%08X\n", otg[0x808 / 4]);
	printf("GCCFG   (Core Cfg): 0x%08X\n", otg[0x038 / 4]);
	printf("DCTL    (Dev Ctrl): 0x%08X\n", otg[0x804 / 4]);
	printf("--------------------\n");
}


int main(void)
{
	printf("\n=== Phoenix-RTOS USB Tester ===\n");

	if (phy_init() < 0) {
		printf("Critical Error: PHY Init failed\n");
		return -1;
	}

	volatile uint32_t *otg = phy_getOtgBase();
	otg[DCTL] &= ~(1 << 1);

	// give D+ some time
	usleep(100000);

	// clear all alarms
	otg[GINTSTS] = 0xFFFFFFFF;
	// unmask Resent and Enum Interrupts
	otg[GINTMSK] = GINT_USBRST | GINT_ENUMDNE;

	// enable pull-up
	otg[DCTL] &= ~(1 << 1);

	printf("[INFO] Waiting for Host interaction...\n");

	while (1) {
		volatile uint32_t status = otg[GINTSTS];

		if (status == 0) {
			usleep(1000);
			continue;
		}

		if (status & GINT_USBRST) {
			printf("[IRQ] USB RESET detected!\n");
			// Clear flag
			otg[GINTSTS] = GINT_USBRST;
		}

		// ENUM DONE (Resent end - speed negotiation)
		if (status & GINT_ENUMDNE) {
			uint32_t dsts = otg[DSTS];
			uint32_t speed = (dsts >> 1) & 0x3;

			printf("[IRQ] ENUMERATION DONE! Speed: ");
			switch (speed) {
				case 0: printf("High Speed (PHY OK!)\n"); break;
				case 1: printf("Full Speed\n"); break;
				case 2: printf("Low Speed\n"); break;
				case 3: printf("Full Speed (PHY 48MHz)\n"); break;
			}

			// clear flag
			otg[GINTSTS] = GINT_ENUMDNE;
		}

		// clear other flags
		if (status & (GINT_SOF)) {
			otg[GINTSTS] = GINT_SOF;
		}

		usleep(5000);
		// debug_dump_regs(otg);
	}

	return 0;
}
