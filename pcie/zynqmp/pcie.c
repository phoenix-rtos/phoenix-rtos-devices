/*
 * Phoenix-RTOS
 *
 * ZynqMP PCI Express driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Dariusz Sabala, Mikolaj Matalowski
 *
 * %LICENSE%
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdatomic.h>
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

#include "core/pci_utils.h"
#include "core/pci_core.h"
#include "core/pci_link.h"

#include "Hailo/common/pcie_common.h"
#include "Hailo/src/pcie.h"
#include "Hailo/phoenix/phoenix_wrappers.h"


static struct {
	uint32_t *fpga_gpio;
	uint32_t *pcie;
	uint32_t irq_counter;
} common;


void test_NVME(pci_dev_t *root)
{
	pci_dev_t *nvme_dev = pci_getDevFromTree(root, 0x1e4b, 0x1202);
	printf("Found device %p\n", nvme_dev);

	uint32_t bar0_test = *((volatile uint32_t *)nvme_dev->bar[0]);
	printf("Reading from BAR0: %x\n", bar0_test);

	pci_dumpBAR(nvme_dev, 0);
}

void test_HAILO(pci_dev_t *root)
{
	pci_dev_t *hailo_dev = pci_getDevFromTree(root, 0x1e60, 0x2864);
	printf("Found device %p\n", hailo_dev);

	uint32_t bar0_test = *((volatile uint32_t *)hailo_dev->bar[0]);
	printf("Reading from BAR0: %x\n", bar0_test);
}

uint32_t irq_status_reg = 0;

int irq_cb(unsigned int, void *arg)
{
	/* Disable irq pin assertion */
	writeReg(common.pcie, 0x13c, 0u);

	irq_status_reg = readReg(common.pcie, 0x138);

	/* Clear irq FIFO */
	while ((readReg(common.pcie, 0x148) & (1u << 18)) != 0) {
		(void)readReg(common.pcie, 0x158);
		writeReg(common.pcie, 0x158, 1u << 31);
	}

	/* Clear irq FIFO overflow */
	// writeReg(common.pcie, 0x148, 1u << 19);

	writeReg(common.pcie, 0x138, irq_status_reg);
	common.irq_counter++;

	/* Enable irq pin assertion */
	writeReg(common.pcie, 0x13c, ~0u);
	return 1;
}

typedef struct {
	handle_t mutex;
	handle_t cond;
	pci_dev_t *devices;
} thread_arg_t;

char irq_handlerStack[4096];

void irq_handlerThread(void *data)
{
	thread_arg_t *arg = (thread_arg_t *)data;
	pci_dev_t *devices = arg->devices;
	pci_dev_t *hailo = pci_getDevFromTree(devices, 0x1e60, 0x2864);

	printf("pcie: IRQ handler thread started\n");
	printf("pcie: Hailo irq pin value is %d\n", ecamRead8((uintptr_t)common.pcie, hailo->bus_no, hailo->dev_no, hailo->func_no, 0x3D));

	while (1) {
		mutexLock(arg->mutex);
		condWait(arg->cond, arg->mutex, 0);

		printf("IRQ status reg (handler call no. %u) : 0x%x\n", common.irq_counter, irq_status_reg);

		if (irq_status_reg & (1 << 17)) {
			/* Received MSI interrupt */
			if (hailo != NULL && hailo->private_handler != NULL && hailo->handler_data != NULL) {
				/* Calling this callback cannot be called -> this causes data abort */
				hailo->private_handler(0, hailo->handler_data);
			}
		}

		mutexUnlock(arg->mutex);
	}

	/* Should never get here */
}

int main(int argc, char **argv)
{
	printf("Entering PCIe test app\n");
	usleep(10000);

	common.irq_counter = 0;

	/* Map memory */
	common.fpga_gpio = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0xa0020000);
	if (NULL == common.fpga_gpio) {
		printf("pcie: fail to map FPGA GPIO memory\n");
		return -1;
	}
	common.pcie = mmap(NULL, 0x10000000, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, 0x500000000);
	if (MAP_FAILED == common.pcie) {
		printf("pcie: fail to map AXI PCIE bridge memory\n");
		return -1;
	}

	phoenix_wrapper_pci_base = common.pcie;

	/* Fix AXI configuration */
	pci_prepareAXI();

	pci_resetAXIandRC(common.fpga_gpio);

	/* Check Bridge whoami info */
	pci_readBridgeInfoReg(common.pcie);

	/* Check PHY link status */
	phy_link_status phy_link_status = pci_checkLinkStatus(common.pcie);
	if (phy_link_status.link_up) {
		printf("pcie: phy LINK UP, link rate: %u, link width x%u, ltssm state: %s\n",
				phy_link_status.link_rate,
				phy_link_status.link_width,
				pci_ltssm_state2str(phy_link_status.ltssm_state));
	}
	else {
		printf("pcie: PHY LINK DOWN, phy status reg: 0x%x\n", phy_link_status.raw_register_val);
		return -1;
	}

	printf("Disable interrupts\n");
	/* Disable interrupt */
	writeReg(common.pcie, 0x13c, 0x0);

	printf("Clear MSIx and legacy interrupts\n");
	/* Clear pending interrupts */
	writeRegMsk(common.pcie, 0x138, 0x0ff30fe9, 0x0ff30fe9);

	/* MSI decode mode */
	writeReg(common.pcie, 0x178, 0xffffffff);
	writeReg(common.pcie, 0x17c, 0xffffffff);

	/* Make sure that root complex is turned off */
	writeReg(common.pcie, 0x148, 0);

	pci_dev_t root = { 0 };
	root.bus_no = 0xff;
	root.dev_no = 0xff;
	root.func_no = 0xff;
	pci_enumerateRoot(common.pcie, &root, 0, 0);
	pci_printTree(&root, 0);

	/* Enable MSI and INTx interrupts */
	handle_t my_cond;
	handle_t my_lock;
	if (condCreate(&my_cond) < 0 || mutexCreate(&my_lock)) {
		printf("Failed to create conditional/mutex passed to irq handler\n");
		return 0;
	}


	thread_arg_t a = { .mutex = my_lock, .cond = my_cond, .devices = &root };

	beginthread(irq_handlerThread, 1, irq_handlerStack, 4096, &a);
	interrupt(PCI_ROOT_COMPLEX_IRQ_NO, irq_cb, &root, my_cond, NULL);
	writeReg(common.pcie, 0x13c, ~0u);
	writeReg(common.pcie, 0x134, 0);
	/* Enable root complex */
	pci_enableRootComplex(common.pcie);
	usleep(10 * 1000);

	pci_dev_t *hailo_card = pci_getDevFromTree(&root, 0x1e60, 0x2864);

	if (hailo_card == NULL) {
		printf("Root complex did not detect Hailo accelerator card");
	}
	else {
		int ret = hailo_pcie_probe(hailo_card);
		printf("Managed to finish probing Hailo : %d\n", ret);
	}

	munmap((void *)(uintptr_t)common.pcie, 0x10000000);
	munmap((void *)common.fpga_gpio, 0x1000);

	return 0;
}
