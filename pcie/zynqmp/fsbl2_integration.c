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

#include <fsbl2.h>
#include <fsbl2_integration.h>

#define SERDES_SIZE    0x20000
#define SERDES_ADDRESS 0xfd400000

#define PCIREG_SIZE    0x1000
#define PCIREG_ADDRESS 0xfd480000

#define CRF_APB_SIZE    0x1000
#define CRF_APB_ADDRESS 0xfd1a0000

#define SIOU_SIZE       0x1000
#define SIOU_ADDRESS    0xfd3d0000

#define GPIO_SIZE       0x1000
#define GPIO_ADDRESS    0Xff0a0000

static uint32_t *serdes  = NULL;
static uint32_t *pcireg  = NULL;
static uint32_t *crf_apb = NULL;
static uint32_t *siou    = NULL;
static uint32_t *gpio    = NULL;

/* Initialise integrastion layer */
int fsbl2_integration_init(void)
{
    /* Map SERDES registers memory */
	serdes = mmap(NULL, SERDES_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, SERDES_ADDRESS);
	if (NULL == serdes) {
		printf("fsbl2: fail to map SERDES registers memory\n");
		return -1;
	}

    /* Map PCIE registers memory */
    pcireg = mmap(NULL, PCIREG_SIZE,
			PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS,
			-1,
			PCIREG_ADDRESS);
	if (NULL == pcireg) {
		printf("fsbl2: fail to map PCIE registers memory\n");
		return -1;
	}

    /* Map CRF APB registers memory */
	crf_apb = mmap(NULL, CRF_APB_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, CRF_APB_ADDRESS);
	if (NULL == serdes) {
		printf("fsbl2: fail to map CRF APB registers memory\n");
		return -1;
	}

    /* Map SIOU registers memory */
	siou = mmap(NULL, SIOU_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, SIOU_ADDRESS);
	if (NULL == serdes) {
		printf("fsbl2: fail to map SIOU registers memory\n");
		return -1;
	}

    /* Map GPIO registers memory */
	gpio = mmap(NULL, GPIO_SIZE, PROT_WRITE | PROT_READ,
			MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, GPIO_ADDRESS);
	if (NULL == gpio) {
		printf("fsbl2: fail to map GPIO registers memory\n");
		return -1;
	}

    return 0;
}

/* Translate physical address to virtual address returned by mmap() */
static inline volatile uint32_t *phys_to_virt(uintptr_t phys)
{
    if (phys >= SERDES_ADDRESS && phys < SERDES_ADDRESS + SERDES_SIZE) {
        return (volatile uint32_t *)((uint8_t *)serdes + (phys - SERDES_ADDRESS));
    }
    if (phys >= CRF_APB_ADDRESS && phys < CRF_APB_ADDRESS + CRF_APB_SIZE) {
        return (volatile uint32_t *)((uint8_t *)crf_apb + (phys - CRF_APB_ADDRESS));
    }
    if (phys >= PCIREG_ADDRESS && phys < PCIREG_ADDRESS + PCIREG_SIZE) {
        return (volatile uint32_t *)((uint8_t *)pcireg + (phys - PCIREG_ADDRESS));
    }
    if (phys >= SIOU_ADDRESS && phys < SIOU_ADDRESS + SIOU_SIZE) {
        return (volatile uint32_t *)((uint8_t *)siou + (phys - SIOU_ADDRESS));
    }
    if (phys >= GPIO_ADDRESS && phys < GPIO_ADDRESS + GPIO_SIZE) {
        return (volatile uint32_t *)((uint8_t *)gpio + (phys - GPIO_ADDRESS));
    }

    /* Unknown range - return NULL so the caller can catch the error */
    return NULL;
}

uint32_t Xil_In32(uintptr_t Addr)
{
    volatile uint32_t *vaddr = phys_to_virt(Addr);
    if (!vaddr) {
        fprintf(stderr, "Xil_In32: unsupported address 0x%lx\n", (unsigned long)Addr);
        return 0xffffffff;
    }

    return *vaddr;
}

void Xil_Out32(uintptr_t Addr, uint32_t Value)
{
    volatile uint32_t *vaddr = phys_to_virt(Addr);
    if (!vaddr) {
        fprintf(stderr, "Xil_Out32: unsupported address 0x%lx\n", (unsigned long)Addr);
        return;
    }

    *vaddr = Value;
}

void PSU_Mask_Write(uintptr_t offset, uint32_t mask, uint32_t val)
{
    uint32_t RegVal = Xil_In32(offset);
    RegVal &= ~mask;
    RegVal |= (val & mask);
    Xil_Out32(offset, RegVal);
}

int mask_poll(uint32_t phys_addr, uint32_t mask)
{
    volatile uint32_t *vaddr = phys_to_virt(phys_addr);
    if (!vaddr) {
        fprintf(stderr, "mask_poll: unsupported address 0x%x\n", phys_addr);
        return -1;
    }

    for (int i = 0; i < PSU_MASK_POLL_TIME; ++i) {
        if (*vaddr & mask) /* condition met - success */
            return 1;
    }
    return 0; /* timeout */
}

uint32_t mask_read(uint32_t phys_addr, uint32_t mask)
{
    return Xil_In32(phys_addr) & mask;
}

void mask_delay(u32 delay)
{
	usleep(delay);
}
