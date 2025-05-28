#include <stdint.h>
#include <stdio.h>

#define xil_printf(fmt, ...)  printf(fmt , ##__VA_ARGS__)

#define PSU_MASK_POLL_TIME 1100000

typedef uint32_t u32;
typedef uint64_t u64;

int fsbl2_integration_init(void);

uint32_t Xil_In32(uintptr_t Addr);

void Xil_Out32(uintptr_t Addr, uint32_t Value);

void PSU_Mask_Write(uintptr_t offset, uint32_t mask, uint32_t val);

int mask_poll(uint32_t phys_addr, uint32_t mask);

uint32_t mask_read(uint32_t phys_addr, uint32_t mask);

void mask_delay(u32 delay);
