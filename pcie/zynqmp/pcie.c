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

#define PCIE_ATTRIB_ATTR_4_OFFSET     0X0010
#define PCIE_ATTRIB_ATTR_8_OFFSET     0X0020
#define PCIE_ATTRIB_ATTR_7_OFFSET     0X001C
#define PCIE_ATTRIB_ATTR_9_OFFSET     0X0024
#define PCIE_ATTRIB_ATTR_10_OFFSET    0X0028
#define PCIE_ATTRIB_ATTR_11_OFFSET    0X002C
#define PCIE_ATTRIB_ATTR_12_OFFSET    0X0030
#define PCIE_ATTRIB_ATTR_13_OFFSET    0X0034
#define PCIE_ATTRIB_ATTR_14_OFFSET    0X0038
#define PCIE_ATTRIB_ATTR_15_OFFSET    0X003C
#define PCIE_ATTRIB_ATTR_16_OFFSET    0X0040
#define PCIE_ATTRIB_ATTR_17_OFFSET    0X0044
#define PCIE_ATTRIB_ATTR_18_OFFSET    0X0048
#define PCIE_ATTRIB_ATTR_24_OFFSET    0X0060
#define PCIE_ATTRIB_ATTR_25_OFFSET    0X0064
#define PCIE_ATTRIB_ATTR_25_OFFSET    0X0064
#define PCIE_ATTRIB_ATTR_27_OFFSET    0X006C
#define PCIE_ATTRIB_ATTR_34_OFFSET    0X0088
#define PCIE_ATTRIB_ATTR_35_OFFSET    0X008C
#define PCIE_ATTRIB_ATTR_37_OFFSET    0X0094
#define PCIE_ATTRIB_ATTR_41_OFFSET    0X00A4
#define PCIE_ATTRIB_ATTR_43_OFFSET    0X00AC
#define PCIE_ATTRIB_ATTR_44_OFFSET    0X00B0
#define PCIE_ATTRIB_ATTR_45_OFFSET    0X00B4
#define PCIE_ATTRIB_ATTR_46_OFFSET    0X00B8
#define PCIE_ATTRIB_ATTR_47_OFFSET    0X00BC
#define PCIE_ATTRIB_ATTR_48_OFFSET    0X00C0
#define PCIE_ATTRIB_ATTR_50_OFFSET    0X00C8
#define PCIE_ATTRIB_ATTR_53_OFFSET    0X00D4
#define PCIE_ATTRIB_ATTR_79_OFFSET    0X013C
#define PCIE_ATTRIB_ATTR_89_OFFSET    0X0164
#define PCIE_ATTRIB_ATTR_93_OFFSET    0X0174
#define PCIE_ATTRIB_ATTR_97_OFFSET    0X0184
#define PCIE_ATTRIB_ATTR_100_OFFSET   0X0190
#define PCIE_ATTRIB_ATTR_101_OFFSET   0X0194
#define PCIE_ATTRIB_ATTR_105_OFFSET   0X01A4
#define PCIE_ATTRIB_ATTR_106_OFFSET   0X01A8
#define PCIE_ATTRIB_ATTR_107_OFFSET   0X01AC
#define PCIE_ATTRIB_ATTR_108_OFFSET   0X01B0
#define PCIE_ATTRIB_ATTR_109_OFFSET   0X01B4
#define PCIE_ATTRIB_CB_OFFSET         0X031C
#define PCIE_ATTRIB_ID_OFFSET         0X0200
#define PCIE_ATTRIB_REV_ID_OFFSET     0X0208
#define PCIE_ATTRIB_SUBSYS_ID_OFFSET  0X0204

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

static int pcietest_releasePcieRst(void)
{
	platformctl_t ctl = {
		.action = pctl_set,
		.type = pctl_devreset,
		.devreset.dev = pctl_devreset_fpd_pcie_cfg,
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

	printf("pcitest: setup pcie attributes...\n");
	/* Lets to it! */
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_25_OFFSET, 0x00000200U, 0x00000200U);
	writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_7_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_8_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_9_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_10_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_11_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_12_OFFSET, 0x0000FFFFU, 0x000000FFU);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_13_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_14_OFFSET, 0x0000FFFFU, 0x0000FFFFU);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_15_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_16_OFFSET, 0x0000FFFFU, 0x0000FFF0U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_17_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_18_OFFSET, 0x0000FFFFU, 0x0000FFF1U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_27_OFFSET, 0x00000738U, 0x00000100U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_50_OFFSET, 0x0000FFF0U, 0x00000040U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_105_OFFSET, 0x000007FFU, 0x000000CDU);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_106_OFFSET, 0x00003FFFU, 0x00000624U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_107_OFFSET, 0x000007FFU, 0x00000018U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_108_OFFSET, 0x000007FFU, 0x000000B5U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_109_OFFSET, 0x0000FFFFU, 0x00007E20U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_34_OFFSET, 0x000000FFU, 0x00000001U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_53_OFFSET, 0x000000FFU, 0x00000060U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_41_OFFSET, 0x000003FFU, 0x00000000U);
	writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_97_OFFSET, 0x00000FFFU, 0x00000041U);
	writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_100_OFFSET, 0x00000040U, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_101_OFFSET, 0x0000FFE2U, 0x0000FFE2U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_37_OFFSET, 0x00007E00U, 0x00004A00U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_93_OFFSET, 0x0000FFFFU, 0x00009000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ID_OFFSET, 0xFFFFFFFFU, 0x10EED021U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_SUBSYS_ID_OFFSET, 0xFFFFFFFFU, 0x10EE0007U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_REV_ID_OFFSET, 0x000000FFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_24_OFFSET, 0x0000FFFFU, 0x00000400U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_25_OFFSET, 0x000001FFU, 0x00000006U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_4_OFFSET, 0x00001000U, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_89_OFFSET, 0x00001FFEU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_79_OFFSET, 0x00000020U, 0x00000020U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_43_OFFSET, 0x00000100U, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_48_OFFSET, 0x000007FFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_46_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_47_OFFSET, 0x00001FFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_44_OFFSET, 0x0000FFFFU, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_45_OFFSET, 0x0000FFF8U, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_CB_OFFSET, 0x00000002U, 0x00000000U);
    writeRegMsk(pcie.pcireg, PCIE_ATTRIB_ATTR_35_OFFSET, 0x0000B000U, 0x00008000U);

	/* Release reset from required subsystems */
	int ret = pcietest_releasePcieRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from PCI Express\n");
		return ret;
	}

	usleep(1000);

	/* Check if BREG is present */
	uint32_t breg_value = readReg(pcie.breg, E_BREG_CAPABILITIES);
	if (!(breg_value & 0x1)) {
		printf("pcietest: fail lack of Egress Bridge Register Translation\n");
		return -1;
	}

	/* Map the bridge register aperture */
	writeReg(pcie.breg, E_BREG_BASE_LO, BREG_ADDRESS);
	writeReg(pcie.breg, E_BREG_BASE_HI, 0x0);

	/* Enable BREG */
	writeReg(pcie.breg, E_BREG_CONTROL, 0x1);

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

	usleep(100000);

	/* Check link status */
	uint32_t pcie_link_status = readReg(pcie.pcireg, PS_LINKUP_OFFSET);
	if (pcie_link_status & 0x1) {
		printf("pcietest: pcie link up\n");
	}
	else {
		printf("pcietest: fail pcie link down\n");
		return -1;
	}

	/* Unmap memory */
	munmap((void *)pcie.breg, BREG_SIZE);
	munmap((void *)pcie.pcireg, PCIREG_SIZE);

	return 0;
}

static int pcietest_init(void)
{
	int ret = 0;

	/* Program PCI Express peripheral clock */
	printf("pcietest: init PCI Express clock...\n");
	ret = pcietest_initPcieClock();
	if (ret != 0) {
		printf("pcietest: fail to config PCI Express clock\n");
		return ret;
	}
	printf("pcietest: release resets...\n");
	ret = pcietest_releasePcieCfgRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from PCI Express Config\n");
		return ret;
	}
	ret = pcietest_releasePcieBridgeRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from PCI Express Bridge\n");
		return ret;
	}
	ret = pcietest_releaseGtRst();
	if (ret != 0) {
		printf("pcietest: fail to release reset from GT\n");
		return ret;
	}
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

int main(int argc, char **argv)
{
	if (pcietest_init() < 0) {
		fprintf(stderr, "pcietest: cannot initialize PCI Express\n");
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}
