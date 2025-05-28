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

static uint32_t readReg(uint32_t *base, uint32_t offset)
{
	return *((volatile uint32_t *)((char *)base + offset));
}


int main(int argc, char **argv)
{
	//int ret = 0;

	uint32_t* pcireg = mmap(NULL, PCIREG_SIZE,
		PROT_WRITE | PROT_READ,
		MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
		-1,
		PCIREG_ADDRESS);
	if (NULL == pcireg) {
		printf("status: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}

    uint32_t ltsm_previous = 0;
    for(uint32_t i = 0; i < 5000; i++)
    {
        uint32_t ltsm = (readReg(pcireg, 0x228) >> 3);
        uint32_t pcie_link_status = readReg(pcireg, PS_LINKUP_OFFSET);

        if (ltsm != ltsm_previous) {
            printf("status: ltsm 0x%x link status 0x%x\n", ltsm, pcie_link_status);
        }

        ltsm_previous = ltsm;
        usleep(1 * 1000);
    }

	munmap((void *)pcireg, PCIREG_SIZE);

	return 0;
}

