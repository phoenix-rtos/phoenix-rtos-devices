#include <errno.h>
#include <paths.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

#include "i2c.h"
#include "axi-i2c.h"


/* I2C registers, from PG090 Table 4 */
#define I2C_REG_GIE          (0x01C / 4)
#define I2C_REG_ISR          (0x020 / 4)
#define I2C_REG_IER          (0x028 / 4)
#define I2C_REG_SOFTR        (0x040 / 4)
#define I2C_REG_CR           (0x100 / 4)
#define I2C_REG_SR           (0x104 / 4)
#define I2C_REG_TX_FIFO      (0x108 / 4)
#define I2C_REG_RX_FIFO      (0x10C / 4)
#define I2C_REG_ADR          (0x110 / 4)
#define I2C_REG_TX_FIFO_OCY  (0x114 / 4)
#define I2C_REG_RX_FIFO_OCY  (0x118 / 4)
#define I2C_REG_TEN_ADR      (0x11C / 4)
#define I2C_REG_RX_FIFO_PIRQ (0x120 / 4)
#define I2C_REG_GPO          (0x124 / 4)
#define I2C_REG_TSUSTA       (0x128 / 4)
#define I2C_REG_TSUSTO       (0x12C / 4)
#define I2C_REG_THDSTA       (0x130 / 4)
#define I2C_REG_TSUDAT       (0x134 / 4)
#define I2C_REG_TBUF         (0x138 / 4)
#define I2C_REG_THIGH        (0x13C / 4)
#define I2C_REG_TLOW         (0x140 / 4)
#define I2C_REG_THDDAT       (0x144 / 4)

#define GIE_EN 31

#define ISR_ARB_LOST           0
#define ISR_TX_ERR_OR_COMPLETE 1
#define ISR_TX_FIFO_EMPY       2
#define ISR_RX_FIFO_FULL       3
#define ISR_BUS_NOT_BUSY       4
#define ISR_ADDR_AS_SLAVE      5
#define ISR_NOT_ADDR_AS_SLAVE  6
#define ISR_TX_FIFO_HALF_EMPTY 7

#define CR_EN          0
#define CR_TX_FIFO_RST 1
#define CR_MSMS        2
#define CR_TX          3
#define CR_TXAK        4
#define CR_RSTA        5

#define SR_ABGC          0
#define SR_AAS           1
#define SR_BB            2
#define SR_SRW           3
#define SR_TX_FIFO_FULL  4
#define SR_RX_FIFO_FULL  5
#define SR_RX_FIFO_EMPTY 6
#define SR_TX_FIFO_EMPTY 7

#define DEV_ADDR 0x41
// #define DEV_ADDR  0x0
#define BASE_ADDR 0xA0030000

#define TCA_REG_OUTPUT   0x01
#define TCA_REG_POLARITY 0x02
#define TCA_REG_CONFIG   0x03
#define TCA_REG_SPECIAL  0x50

inline static void clearInterrupts(volatile uint32_t *base)
{
	/*
	 * Clear all interrupts.
	 */
	int val = *(base + I2C_REG_ISR);
	// printf("before clear: 0x%08x\n", val);
	*(base + I2C_REG_ISR) = val;
}


inline static int send_data(uint8_t dev_addr, volatile uint32_t *base)
{
	/* Soft reset */
	clearInterrupts(base);
	/*
	 * Enable the device.
	 */
	*(base + I2C_REG_CR) = (1 << CR_EN) | (1 << CR_TX_FIFO_RST);
	*(base + I2C_REG_CR) = (1 << CR_EN);

	/*
	 * Set Rx FIFO Occupancy depth to throttle at first byte (after reset = 0)
	 * */
	*(base + I2C_REG_RX_FIFO_PIRQ) = 15;

	/*
	 * Disable interrupt*/
	*(base + I2C_REG_GIE) = 0;

	// *(base + I2C_REG_TX_FIFO) = (1 << 8);
	*(base + I2C_REG_TX_FIFO) = (dev_addr << 1);
	// *(base + I2C_REG_TX_FIFO) = 0x01;
	// *(base + I2C_REG_TX_FIFO) = 0x5b;

	*(base + I2C_REG_ISR) &= (1 << ISR_TX_ERR_OR_COMPLETE);  //| (1 << ISR_TX_FIFO_EMPY);
	*(base + I2C_REG_IER) |= (1 << ISR_TX_ERR_OR_COMPLETE);  //| (1 << ISR_TX_FIFO_EMPY);
	*(base + I2C_REG_CR) = (1 << CR_EN) | (1 << CR_MSMS) | (1 << CR_TX);

	usleep(2);
	uint32_t val = *(base + I2C_REG_ISR);
	// printf("status for i=0x%02x is: 0x%08x\n", dev_addr, val);
	// if (0 == (val & (1 << ISR_TX_ERR_OR_COMPLETE))) {
	if (0 == (val & (1 << ISR_TX_ERR_OR_COMPLETE))) {
		if (0x10 == dev_addr)
			printf("Address found: 0x%02x\n", dev_addr);
		return 1;
	}
	return 0;
	// *(base + I2C_REG_GIE) = (1 << GIE_EN);
}

static inline int probe(volatile uint32_t *base)
{
	int res = 0;
	for (int i = 0x1; i < 0x80; i++) {
		*(base + I2C_REG_SOFTR) = 0xa;
		res += send_data(i, base);
	}
	return res;
}

static inline void probe_multi(volatile uint32_t *base)
{

	uint8_t output;
	uint8_t polarity;
	uint8_t config;
	uint16_t special;
	int val = 0;
	for (output = 0x0; output < 0x10; output++) {
		i2c_regWrite(DEV_ADDR, TCA_REG_OUTPUT, &output, 1);
		for (polarity = 0x0; polarity < 0x10; polarity++) {
			i2c_regWrite(DEV_ADDR, TCA_REG_POLARITY, &polarity, 1);
			for (config = 0x0; config < 0x10; config++) {
				i2c_regWrite(DEV_ADDR, TCA_REG_CONFIG, &config, 1);
				for (special = 0x0; special < 0x100; special += 0x40) {
					i2c_regWrite(DEV_ADDR, TCA_REG_SPECIAL, (uint8_t *)&special, 1);
					val = probe(base);
					if (2 == val) {
						printf("regs: 0x%02x, 0x%02x, 0x%02x, 0x%02x\n", output, polarity, config, special);
					}
				}
			}
		}
	}
}


int main(int argc, char **argv)
{
	char *i2cDevice = "/dev/i2c0";
	oid_t i2cDev;

	standard_proc(0);

	// printf("hello\n");
	if (lookup(i2cDevice, NULL, &i2cDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}


	volatile uint32_t *base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, BASE_ADDR);
	probe(base);

	return 0;
}
