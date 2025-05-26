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
#include <stdbool.h>
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

#define SERDES_SIZE    0x20000
#define SERDES_ADDRESS 0xfd400000

#define BREG_SIZE	   0x1000
#define BREG_ADDRESS   0xfd0e0000

#define PCIREG_SIZE    0x1000
#define PCIREG_ADDRESS 0xfd480000

#define CFG_SIZE 0x10000000
#define CFG_ADDRESS 0x8000000000

#define GIC_PCIE_INTX_IRQ 148

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

#define BRCFG_PCIE_RX0			0x00000000
#define BRCFG_PCIE_RX1			0x00000004
#define BRCFG_INTERRUPT			0x00000010
#define BRCFG_PCIE_RX_MSG_FILTER	0x00000020

#define E_BREG_CAPABILITIES		0x00000200
#define E_BREG_CONTROL			0x00000208
#define E_BREG_BASE_LO			0x00000210
#define E_BREG_BASE_HI			0x00000214
#define E_ECAM_CAPABILITIES		0x00000220
#define E_ECAM_CONTROL			0x00000228
#define E_ECAM_BASE_LO			0x00000230
#define E_ECAM_BASE_HI			0x00000234

#define I_MSII_CAPABILITIES		0x00000300
#define I_MSII_CONTROL			0x00000308
#define I_MSII_BASE_LO			0x00000310
#define I_MSII_BASE_HI			0x00000314
#define I_ISUB_CONTROL			0x000003E8

#define PS_LINKUP_OFFSET		0x00000238

#define upper_32_bits(n) ((uint32_t)(((n) >> 16) >> 16))

#define lower_32_bits(n) ((uint32_t)((n) & 0xffffffff))

/* Data structure used by the driver */
static struct {
	uint32_t *breg;
	uint32_t *pcireg;
	uint32_t *cfg;
	handle_t cond;               /**< Conditional variable for synchronizing interrupts */
	handle_t inth;               /**< Interrupt handler object */
	handle_t lock;               /**< Mutex used with the conditional variable for synchronization */
} pcie;

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

static int pcietest_pciIntxIrqHandler(unsigned int id, void *arg)
{
	return 0;
}

static int pcietest_initPcie(void)
{
	/* Map registers memory */
	pcie.breg = mmap(NULL, BREG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		BREG_ADDRESS);
	if (NULL == pcie.breg) {
		printf("pcietest: fail to map AXI PCIE MAIN registers memory\n");
		return -1;
	}
	pcie.pcireg = mmap(NULL, PCIREG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		PCIREG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcietest: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}
	pcie.cfg = mmap(NULL, CFG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		CFG_ADDRESS);
	if (NULL == pcie.pcireg) {
		printf("pcietest: fail to map PCIE CFG registers memory\n");
		return -1;
	}

	/* Initialize synchronization primitives */
	if (condCreate(&pcie.cond) != 0) {
		printf("pcietest: failed to create conditional variable\n");
		return -1;
	}
	if (mutexCreate(&pcie.lock) != 0) {
		printf("pcietest: failed to create mutex\n");
		return -1;
	}

	/* Register PCI INTx interrupt handler */
	if (interrupt(GIC_PCIE_INTX_IRQ, pcietest_pciIntxIrqHandler, &pcie, pcie.cond, &pcie.inth) != 0) {
		printf("pcietest: failed to register interrupt handler\n");
		return -1;
	}

	usleep(1000);

	/* Check if BREG is present */
	uint32_t breg_value = readReg(pcie.breg, E_BREG_CAPABILITIES);
	if (!(breg_value & 0x1)) {
		printf("pcietest: fail lack of Egress Bridge Register Translation\n");
		return -1;
	}

	uint32_t ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	/* Map the bridge register aperture */
	writeReg(pcie.breg, E_BREG_BASE_LO, BREG_ADDRESS);
	writeReg(pcie.breg, E_BREG_BASE_HI, 0x0);

	/* Enable BREG */
	writeReg(pcie.breg, E_BREG_CONTROL, 0x1);

	ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	/* Disable DMA */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX0, 0x7, 0x7);

	/* Enable Ingress subtractive decode translation */
	writeReg(pcie.breg, I_ISUB_CONTROL, 0x1);

	/* Enable msg filtering details */
	writeReg(pcie.breg, BRCFG_PCIE_RX_MSG_FILTER, ((1 << 1) | (1 << 2) | (1 << 3)));

	/* This routes the PCIe DMA traffic to go through CCI path (?) */
	writeRegMsk(pcie.breg, BRCFG_PCIE_RX1, 0xff, 0xff);

	/* Check if the phy link is up or not */
	bool phy_link_up = false;
	static const int LINK_WAIT_MAX_RETRIES = 10;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (link_status & 0x2) {
			phy_link_up = true;
			break;
		}
		usleep(100000);
	}

	if (phy_link_up) {
		printf("pcietest: phy link up\n");
	}
	else {
		printf("pcietest: fail, phy link down\n");
		return -1;
	}

	uint32_t ecam_value = readReg(pcie.breg, E_ECAM_CAPABILITIES);
	if (ecam_value & 0x1) {
		printf("pcietest: ecam present\n");
	}
	else {
		printf("pcietest: fail ecam is not present\n");
		return -1;
	}

	/* Enable ECAM */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, 0x1, 0x1);
	/* Set size of translation window */
	writeRegMsk(pcie.breg, E_ECAM_CONTROL, (0x1f << 16), (16 << 16)); // 256 MB

	writeReg(pcie.breg, E_ECAM_BASE_LO, lower_32_bits(CFG_ADDRESS));
	writeReg(pcie.breg, E_ECAM_BASE_HI, upper_32_bits(CFG_ADDRESS));


	/* Check link status in loop */
    uint32_t ltsm_previous = 0;
    for(uint32_t i = 0; i < 100000; i++) {
        uint32_t ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
        uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
        if (ltsm != ltsm_previous) {
            printf("status: ltsm 0x%x link status 0x%x\n", ltsm, pcie_link_status);
        }
        ltsm_previous = ltsm;
        usleep(1 * 100);
    }



#if 0
	/* Check link status */
	bool pcie_link_up = false;
	for (int retries = 0; retries < LINK_WAIT_MAX_RETRIES; retries++) {
		uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
		if (pcie_link_status & 0x1) {
			pcie_link_up = true;
			break;
		}
		usleep(100000);
	}

	ltsm = (readReg(pcie.pcireg, 0x228) >> 3);
	printf("pcietest: ltsm 0x%x\n", ltsm);

	if (pcie_link_up) {
		printf("pcietest: pcie link up\n");
	}
	else {
		printf("pcietest: fail pcie link down\n");
		return -1;
	}
#endif
	/* Unmap memory */
	munmap((void *)pcie.breg, BREG_SIZE);
	munmap((void *)pcie.pcireg, PCIREG_SIZE);

	return 0;
}

int main(int argc, char **argv)
{
	int ret = 0;

	/* Configure PS GTR transceivers */
	printf("pcietest: configure PS GTR transceivers...\n");
	ret = pcitest_configPsGtr();
	if (ret != 0) {
		printf("pcietest: fail to configure PS GTR transceivers\n");
		return ret;
	}

	/* Finaly configure PCI peripheral */
	printf("pcietest: configure PCI Express peripheral...\n");
	ret = pcietest_initPcie();
	if (ret != 0) {
		printf("pcietest: fail to configure PCI Express peripheral\n");
		return ret;
	}

	return 0;
}
