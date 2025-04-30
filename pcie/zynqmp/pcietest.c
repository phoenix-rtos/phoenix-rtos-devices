/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <endian.h>
#include <sys/interrupt.h>
#include <sys/mman.h>
#include <sys/msg.h>
#include <sys/platform.h>
#include <sys/threads.h>
#include <posix/utils.h>

#include <phoenix/arch/aarch64/zynqmp/zynqmp.h>

#include <board_config.h>

#include <i2c.h>

#include "si5345.h"

#define SERDES_SIZE    0x20000
#define SERDES_ADDRESS 0x00fd400000

#define L0_TM_PLL_DIG_37          0x2094
#define L0_L0_REF_CLK_SEL         0x2860
#define L0_PLL_SS_STEPS_0_LSB     0x2368
#define L0_PLL_SS_STEPS_1_MSB     0x236c
#define L0_PLL_SS_STEP_SIZE_0_LSB 0x2370
#define L0_PLL_SS_STEP_SIZE_1     0x2374
#define L0_PLL_SS_STEP_SIZE_2     0x2378
#define L0_PLL_SS_STEP_SIZE_3_MSB 0x237c
#define L0_PLL_STATUS_READ_1      0x23e4
#define PLL_REF_SEL0              0x10000
#define ICM_CFG0                  0x10010


static uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}

static void writeReg(uint32_t *base, uint32_t offset, uint32_t value)
{
	*((volatile uint32_t *)((char *)base + offset)) = value;
}

static void writeRegMsk(uint32_t *base, uint32_t offset, uint32_t clr, uint32_t set)
{
	uint32_t value = readReg(base, offset);
	value &= ~clr;
	value |= set;
	writeReg(base, offset, value);
}

static int pcitest_configPsGtr(void)
{
	/* Map SERDES registers memory */
	uint32_t *serdes = mmap(NULL, SERDES_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			SERDES_ADDRESS);
	if (NULL == serdes) {
		printf("pcietest: fail to map SERDES registers memory\n");
		return -1;
	}

	/* "Enable coarse saturation" */
	writeReg(serdes, L0_TM_PLL_DIG_37, 0x10);

	/* Set external reference clock equal 100MHz */
	writeRegMsk(serdes, PLL_REF_SEL0, 0x1f, 0x0d);

	/* Set that clock is not shared among 4 transceivers */
	writeRegMsk(serdes, L0_L0_REF_CLK_SEL, 0x9f, (1 << 7));

	/* Continue configuring PLL  */
	uint32_t step_size = 87533;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_0_LSB, 0xff, step_size & 0xff);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_1, 0xff, step_size & 0xff);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_2, 0xff, step_size & 0xff);
	uint32_t steps = 1058;
	writeRegMsk(serdes, L0_PLL_SS_STEPS_0_LSB, 0xff, steps & 0xff);
	writeRegMsk(serdes, L0_PLL_SS_STEPS_1_MSB, 0x07, (steps >> 8) & 0x07);
	step_size >>= 8;
	writeRegMsk(serdes, L0_PLL_SS_STEP_SIZE_3_MSB, 0x3, (step_size & 0x3) | 0x10 | 0x20);

	/* Set PCIe protocol for PS GTR lane 0 */
	writeRegMsk(serdes, ICM_CFG0, 0x07, 0x1);

	/* Wait for PLL lock */
	usleep(10 * 1000);

	/* Read PLL status */
	uint32_t pll_status = readReg(serdes, 0x23E4);
	if (pll_status & 0x10) {
		printf("pcietest: PLL locked, status 0x%x\n", pll_status);
	}
	else {
		printf("pcietest: fail to lock PLL, status 0x%x\n", pll_status);
	}

	/* Unmap SERDES memory */
	munmap((void *)serdes, SERDES_SIZE);

	return 0;
}

static int pcietest_releasePcieCfgRst(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_ctrl,
		.devreset.state = 0,
	};
	return platformctl(&ctl);
}

static int pcietest_releaseGtRst(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_gt,
		.devreset.state = 0,
	};
	return platformctl(&ctl);
}

static int pcietest_releasePcieBridgeRst(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_bridge,
		.devreset.state = 0,
	};
	return platformctl(&ctl);
}

static int pcietest_initPcieClock(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devclock,
		.devclock.dev = pctl_devclock_fpd_pcie,
		.devclock.src = 0,  /* source: IOPLL_TO_FPD (500 MHz) */
		.devclock.div0 = 2, /* divide by 2 = 250 MHz */
		.devclock.div1 = 0,
		.devclock.active = 0x1,
	};

	return platformctl(&ctl);
}

static int pcietest_init(void)
{
	int ret = 0;

	/* Initialise PCI Express reference clock */
	ret = si5345_initPcieClk();
	if (ret != 0) {
		printf("pcietest: fail to init si5345\n");
		return ret;
	}

	/* Release reset from required subsystems */
	printf("pcietest: release resets...\n");
	ret = pcietest_releasePcieCfgRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from PCI Express Config\n");
		return ret;
	}
	ret = pcietest_releaseGtRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from GT\n");
		return ret;
	}
	ret = pcietest_releasePcieBridgeRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from PCI Express Bridge\n");
		return ret;
	}

	/* Program PCI Express peripheral clock */
	printf("pcietest: init PCI Express clock...\n");
	ret = pcietest_initPcieClock();
	if (ret != 0) {
		printf("pcietest: fail to config PCI Express clock\n");
		return ret;
	}

	/* Configure PS GTR transceivers */
	printf("pcietest: configure PS GTR transceivers...\n");
	ret = pcitest_configPsGtr();
	if (ret != 0) {
		printf("pcietest: fail to configure PS GTR transceivers\n");
		return ret;
	}

	return 0;
}

int main(int argc, char **argv)
{
	if (pcietest_init() < 0) {
		fprintf(stderr, "pcietest: cannot initialize PCI Express\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
