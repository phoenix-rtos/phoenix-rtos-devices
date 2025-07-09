/*
 * Phoenix-RTOS
 *
 * PCI Express Xilinx NWL driver
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

#include <pcie.h>


/* Register set "AXIPCIE_MAIN" */
#define AXIPCIE_MAIN_SIZE    0x1000
#define AXIPCIE_MAIN_ADDRESS 0xfd0e0000


/* Register set "PCIE_ATTRIB"  */
#define PCIE_ATTRIB_SIZE    0x1000
#define PCIE_ATTRIB_ADDRESS 0xfd480000


/* Registers inside "AXIPCIE_MAIN" registers set */
#define E_BREG_CAPABILITIES 0x00000200
#define E_BREG_CONTROL      0x00000208
#define E_BREG_BASE_LO      0x00000210
#define E_BREG_BASE_HI      0x00000214
#define E_ECAM_CONTROL      0x00000228
#define E_ECAM_BASE_LO      0x00000230
#define E_ECAM_BASE_HI      0x00000234


/* Registers inside "PCIE_ATTRIB" registers set */
#define PCIE_STATUS 0x00000238


static struct {
	uint32_t *axipcie_main; /* Register set "AXIPCIE_MAIN" */
	uint32_t *pcie_attrib;  /* Register set "PCIE_ATTRIB"  */
	uint32_t *ecam;         /* Memory range used for ECAM space "PCIe HIGH" */
} common;


static int pcie_initPcie(void)
{
	/* Map the bridge register aperture */
	writeReg(common.axipcie_main, E_BREG_BASE_LO, LOWER_32_BITS(AXIPCIE_MAIN_ADDRESS));
	writeReg(common.axipcie_main, E_BREG_BASE_HI, UPPER_32_BITS(AXIPCIE_MAIN_ADDRESS));

	/* Enable BREG */
	writeReg(common.axipcie_main, E_BREG_CONTROL, 0x1);

	/* Check PHY link */
	bool phy_link_up = false;
	static const int link_wait_retries_max = 10;
	for (int retries = 0; retries < link_wait_retries_max; retries++) {
		uint32_t link_status = readReg(common.pcie_attrib, PCIE_STATUS);
		if (link_status & 0x2) {
			phy_link_up = true;
			break;
		}
		usleep(100000);
	}
	if (phy_link_up) {
		printf("pcie-xilinx-nwl: phy link up\n");
	}
	else {
		fprintf(stderr, "pcie-xilinx-nwl: fail, phy link down\n");
		return -1;
	}

	/* Enable ECAM */
	writeRegMsk(common.axipcie_main, E_ECAM_CONTROL, 0x1, 0x1);
	/* Set size of translation window */
	writeRegMsk(common.axipcie_main, E_ECAM_CONTROL, (0x1f << 16), (16 << 16));
	/* Configure address for ECAM space */
	writeReg(common.axipcie_main, E_ECAM_BASE_LO, LOWER_32_BITS(ECAM_ADDRESS));
	writeReg(common.axipcie_main, E_ECAM_BASE_HI, UPPER_32_BITS(ECAM_ADDRESS));

	/* Check PCI Express link */
	bool pcie_link_up = false;
	for (int retries = 0; retries < link_wait_retries_max; retries++) {
		uint32_t pcie_link_status = readReg(common.pcie_attrib, PCIE_STATUS);
		if (pcie_link_status & 0x1) {
			pcie_link_up = true;
			break;
		}
		usleep(100000);
	}
	if (pcie_link_up) {
		printf("pcie-xilinx-nwl: pcie link up\n");
	}
	else {
		fprintf(stderr, "pcie-xilinx-nwl: fail pcie link down\n");
		return -1;
	}

	/* Read local config space */
	uint32_t config = readReg(common.ecam, 0x4);

	/* Enable needed functionality */
	config |= (PCI_CMD_IO_ENABLE | PCI_CMD_MEM_ENABLE | PCI_CMD_MASTER_ENABLE |
			PCI_CMD_PARITY_ERR_ENABLE | PCI_CMD_SERR_ERR_ENABLE);
	writeReg(common.ecam, 0x4, config);

	return 0;
}


int pcie_xilinx_nwl_init(void)
{
	int ret = 0;

	/* Map registers memory */
	common.axipcie_main = mmap(NULL, AXIPCIE_MAIN_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			AXIPCIE_MAIN_ADDRESS);
	if (MAP_FAILED == common.axipcie_main) {
		fprintf(stderr, "pcie-xilinx-nwl: fail to map AXI PCIE MAIN registers memory\n");
		return -1;
	}
	common.pcie_attrib = mmap(NULL, PCIE_ATTRIB_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			PCIE_ATTRIB_ADDRESS);
	if (MAP_FAILED == common.pcie_attrib) {
		munmap((void *)common.axipcie_main, AXIPCIE_MAIN_SIZE);
		fprintf(stderr, "pcie-xilinx-nwl: fail to map PCIE ATTRIB registers memory\n");
		return -1;
	}
	common.ecam = mmap(NULL, ECAM_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			ECAM_ADDRESS);
	if (MAP_FAILED == common.ecam) {
		munmap((void *)common.axipcie_main, AXIPCIE_MAIN_SIZE);
		munmap((void *)common.pcie_attrib, PCIE_ATTRIB_SIZE);
		fprintf(stderr, "pcie-xilinx-nwl: fail to map PCIE CFG registers memory\n");
		return -1;
	}

	/* Configure PCI peripheral */
	printf("pcie-xilinx-nwl: configure PCI Express peripheral...\n");
	ret = pcie_initPcie();
	if (ret != 0) {
		fprintf(stderr, "pcie-xilinx-nwl: fail to configure PCI Express peripheral\n");
	}

	/* Unmap memory */
	munmap((void *)common.axipcie_main, AXIPCIE_MAIN_SIZE);
	munmap((void *)common.pcie_attrib, PCIE_ATTRIB_SIZE);
	munmap((void *)common.ecam, ECAM_SIZE);

	return ret;
}
