/*
 * Phoenix-RTOS
 *
 * STM32 XSPI driver header file
 *
 * Copyright 2026 Phoenix Systems
 * Author: Jacek Maksymowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _XSPI_N6_H_
#define _XSPI_N6_H_

#include <stdbool.h>
#include <stdlib.h>
#include <board_config.h>

/* It's not practical to automatically determine if a given XSPI bus uses HyperBus protocol
 * or regular commands, so use compile-time configuration for it. */

#ifndef XSPI1_IS_HYPERBUS
#define XSPI1_IS_HYPERBUS 0
#endif

#ifndef XSPI2_IS_HYPERBUS
#define XSPI2_IS_HYPERBUS 0
#endif

#ifndef XSPI3_IS_HYPERBUS
#define XSPI3_IS_HYPERBUS 0
#endif

/* Set default divider to result in 50 MHz frequency. This is slow, but safe. */

#ifndef XSPI1_CLOCK_DIV
#define XSPI1_CLOCK_DIV 4
#endif

#ifndef XSPI2_CLOCK_DIV
#define XSPI2_CLOCK_DIV 4
#endif

#ifndef XSPI3_CLOCK_DIV
#define XSPI3_CLOCK_DIV 4
#endif

#ifndef XSPI1_STORAGE
#define XSPI1_STORAGE 0
#endif

#ifndef XSPI2_STORAGE
#define XSPI2_STORAGE 0
#endif

#ifndef XSPI3_STORAGE
#define XSPI3_STORAGE 0
#endif

#define XSPI_FIFO_SIZE     64 /* Size of hardware FIFO */
#define XSPI_N_CONTROLLERS 1
#define XSPI_MCE_REGIONS   1 /* Hardware supports up to 4 */

#define XSPI_SR_BUSY (1UL << 5) /* Controller busy */
#define XSPI_SR_TOF  (1UL << 4) /* Timeout */
#define XSPI_SR_SMF  (1UL << 3) /* Status match in auto-polling mode */
#define XSPI_SR_FTF  (1UL << 2) /* FIFO threshold */
#define XSPI_SR_TCF  (1UL << 1) /* Transfer complete */
#define XSPI_SR_TEF  (1UL << 0) /* Transfer error */

#define XSPI_CR_MODE_IWRITE   (0UL << 28) /* Indirect write mode */
#define XSPI_CR_MODE_IREAD    (1UL << 28) /* Indirect read mode */
#define XSPI_CR_MODE_AUTOPOLL (2UL << 28) /* Auto-polling mode */
#define XSPI_CR_MODE_MEMORY   (3UL << 28) /* Memory-mapped mode */
#define XSPI_CR_MODE_MASK     (3UL << 28) /* Mask of mode bits */
#define XSPI_CR_NOPREF_AXI    (1UL << 26) /* Disable prefetch when the AXI transaction is signaled as not-prefetchable */
#define XSPI_CR_NOPREF        (1UL << 25) /* Disable prefetch always */
#define XSPI_CR_TOIE          (1UL << 20)
#define XSPI_CR_SMIE          (1UL << 19)
#define XSPI_CR_FTIE          (1UL << 18)
#define XSPI_CR_TCIE          (1UL << 17)
#define XSPI_CR_TEIE          (1UL << 16)
#define XSPI_CR_TCEN          (1UL << 3) /* Enable timeout in memory-mapped mode */

/* Default settings for prefetch and timeout */
#define XSPI_DEFAULT_PREFETCH (XSPI_CR_NOPREF_AXI)
#define XSPI_DEFAULT_TIMEOUT  (XSPI_CR_TCEN)

#define XSPIM_PORT1   0
#define XSPIM_PORT2   1
#define XSPIM_MUX_OFF 0UL /* No multiplexed accesses */
#define XSPIM_MUX_ON  1UL /* XSPI1 and XSPI2 do multiplexed accesses on the same port */
/* If MUX_OFF: XSPI1 to Port 1, XSPI2 to Port 2
 * If MUX_ON: XSPI1 and XSPI2 muxed to Port 1, XSPI3 to Port 2*/
#define XSPIM_MODE_DIRECT 0UL
/* If MUX_OFF: XSPI1 to Port 2, XSPI2 to Port 1
 * If MUX_ON: XSPI1 and XSPI2 muxed to Port 2, XSPI3 to Port 1 */
#define XSPIM_MODE_SWAPPED (1UL << 1)

#define XSPI_CHIPSELECT_NCS1 0UL
#define XSPI_CHIPSELECT_NCS2 1UL

enum xspi_regs {
	xspi_cr = 0x0,
	xspi_dcr1 = 0x2,
	xspi_dcr2,
	xspi_dcr3,
	xspi_dcr4,
	xspi_sr = 0x8,
	xspi_fcr,
	xspi_dlr = 0x10,
	xspi_ar = 0x12,
	xspi_dr = 0x14,
	xspi_psmkr = 0x20,
	xspi_psmar = 0x22,
	xspi_pir = 0x24,
	xspi_ccr = 0x40,
	xspi_tcr = 0x42,
	xspi_ir = 0x44,
	xspi_abr = 0x48,
	xspi_lptr = 0x4c,
	xspi_wpccr = 0x50,
	xspi_wptcr = 0x52,
	xspi_wpir = 0x54,
	xspi_wpabr = 0x58,
	xspi_wccr = 0x60,
	xspi_wtcr = 0x62,
	xspi_wir = 0x64,
	xspi_wabr = 0x68,
	xspi_hlcr = 0x80,
	xspi_calfcr = 0x84,
	xspi_calmr = 0x86,
	xspi_calsor = 0x88,
	xspi_calsir = 0x8a,
};


typedef struct {
	int16_t port;
	int8_t pin;
} xspi_pin_t;


typedef struct {
	void *start;
	uint32_t size;
	volatile uint32_t *ctrl;
	struct {
		uint16_t sel; /* Clock mux (ipclk_xspi?sel) */
		uint8_t val;  /* Clock source (one of ipclk_sel_*) */
	} clksel;
	uint16_t divider_slow; /* Divider used for initialization - resulting clock must be under 50 MHz */
	uint16_t divider;      /* Divider used for normal operation - can be as fast as Flash can handle */
	uint16_t dev;
	uint16_t mceDev;
	uint16_t irq;
	xspi_pin_t resetPin; /* Hardware reset pin for device (set to -1 if unused) */
	uint8_t enabled;
	uint8_t spiPort;
	uint8_t chipSelect;
	uint8_t isHyperbus;
} xspi_ctrlParams_t;

#endif /* _XSPI_N6_H_ */
