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

#include "mipi-regs.h"

#define DEV_ADDR         0x10
#define BASE_ADDR_I2C    0xA0030000
#define BASE_ADDR_MIPI   0xA0040000
#define BASE_ADDR_FRMBUF 0xB0100000

struct imx219_reg {
	uint16_t addr;
	uint8_t value;
};

static struct imx219_reg imx219_reginit[256] = {
	/* 3280x2464P15 */

	/* Control seq to access 0x3000+ addresses */
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x0C },
	{ 0x300A, 0xFF },
	{ 0x300B, 0xFF },
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x09 },
	/* 2 Lane MIPI */

	/* Output Set-up */
	{ 0x0110, 0x01 },  // CSI channel ID
	{ 0x0114, 0x01 },  // CSI_LANE_MODE: 2-Lane
	{ 0x0128, 0x00 },  // MIPI global timing set to auto mode
	{ 0x012A, 0x18 },  // INCK (input clock) frequency set to 0x1800 24 MHz
	{ 0x012B, 0x00 },

	/* Frame Bank "A" */
	{ 0x0160, 0x09 },  // frame_length_lines set to 0x09C8 1Lines (2504)
	{ 0x0161, 0xC8 },
	{ 0x0162, 0x0D },  // Line length set to 0x0D78 pixels (3448)
	{ 0x0163, 0x78 },
	{ 0x0164, 0x00 },  // x_addr_start  set to 0
	{ 0x0165, 0x00 },
	{ 0x0166, 0x0C },  // x_addr_end set to 0xCCF (3279)
	{ 0x0167, 0xCF },
	{ 0x0168, 0x00 },  // y_addr_start set to 0
	{ 0x0169, 0x00 },
	{ 0x016A, 0x09 },  // y_addr_end set to 0x99F (2463)
	{ 0x016B, 0x9F },
	{ 0x016C, 0x0C },  // x_output_size set to 0xCD0 pixels/line (3280)
	{ 0x016D, 0xD0 },
	{ 0x016E, 0x09 },  // y_output_size set to 0x9A0 lines (2464)
	{ 0x016F, 0xA0 },
	{ 0x0170, 0x01 },  // x incr for odd pixels: 1
	{ 0x0171, 0x01 },  // y incr for odd pixels: 1
	{ 0x0172, 0x03 },  // image orientation: set for both hori, vert
	{ 0x0174, 0x00 },  // binning mode hori: no-binning
	{ 0x0175, 0x00 },  // binning mode vert: no-binning
	{ 0x018C, 0x0A },  // CSI_DATA_FORMAT_A set to 0x0A0A (RAW10)
	{ 0x018D, 0x0A },

	/* Clock Set-up */
	{ 0x0301, 0x05 },  // Video Timing Pixel Clock Divider value: 5
	{ 0x0303, 0x01 },  // Video Timing System Clock Divider value: 1
	{ 0x0304, 0x03 },  // Pre PLL clock Video Timing System Divider Value: 3
	{ 0x0305, 0x03 },  // Pre PLL clock Output System Divider Value: 3
	{ 0x0306, 0x00 },  // PLL Video Timing System multiplier value: 0x2B (43)
	{ 0x0307, 0x2B },
	{ 0x0309, 0x0A },  // Output Pixel Clock Divider value: 0x0A (10)
	{ 0x030B, 0x01 },  // Output System Clock Divider value: 0x01
	{ 0x030C, 0x00 },  // PLL Output System multiplier value: 0x55 (85)
	{ 0x030D, 0x55 },

	/* Here be dragons (CIS tuning)*/
	{ 0x455E, 0x00 },
	{ 0x471E, 0x4B },
	{ 0x4767, 0x0F },
	{ 0x4750, 0x14 },
	{ 0x4540, 0x00 },
	{ 0x47B4, 0x14 },
	{ 0x4713, 0x30 },
	{ 0x478B, 0x10 },
	{ 0x478F, 0x10 },
	{ 0x4793, 0x10 },
	{ 0x4797, 0x0E },
	{ 0x479B, 0x0E },

	{ 0xFFFF, 0x00 } /* end of the list */
};

static struct imx219_reg imx219_secondInit[256] = {
	/* 1920x1080P48 */
	/* Control seq to access 0x3000+ addresses */
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x0C },
	{ 0x300A, 0xFF },
	{ 0x300B, 0xFF },
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x09 },

	/* Output Set-up */
	{ 0x0114, 0x01 },  // CSI_LANE_MODE: 2-Lane
	{ 0x0128, 0x00 },  // MIPI global timing set to auto mode
	{ 0x012A, 0x18 },  // INCK (input clock) frequency set to 24 MHz
	{ 0x012B, 0x00 },

	/* Frame Bank "A" */
	{ 0x0160, 0x04 },  // frame length set to 0x0459 lines (1113)
	{ 0x0161, 0x59 },
	{ 0x0162, 0x0D },  // line length set to 0x0D78 pixels (3448)
	{ 0x0163, 0x78 },
	{ 0x0164, 0x02 },  // x_addr_start set to 0x02A8 (680)
	{ 0x0165, 0xA8 },
	{ 0x0166, 0x0A },  // x_addr_end set to 0x0A27 (2599 = 680 + 1920 - 1)
	{ 0x0167, 0x27 },
	{ 0x0168, 0x02 },  // y_addr_start set to 0x02B4 (692)
	{ 0x0169, 0xB4 },
	{ 0x016A, 0x06 },  // y_addr_end set to 0x06EB (1771 = 692 + 1080 - 1)
	{ 0x016B, 0xEB },
	{ 0x016C, 0x07 },  // x_output_size set to 0x780 pixels/line (1920)
	{ 0x016D, 0x80 },
	{ 0x016E, 0x04 },  // y_output_size set to 0x438 lines (1080)
	{ 0x016F, 0x38 },
	{ 0x0170, 0x01 },  // x incr for odd pixels: 1
	{ 0x0171, 0x01 },  // y incr for odd pixels: 1
	{ 0x0174, 0x00 },  // binning mode hori: no-binning
	{ 0x0175, 0x00 },  // binning mode vert: no-binning
	{ 0x018C, 0x0A },  // CSI_DATA_FORMAT_A set to 0x0A0A (RAW10)
	{ 0x018D, 0x0A },

	/* Clock Set-up */
	{ 0x0301, 0x05 },  // Video Timing Pixel Clock Divider value: 5
	{ 0x0303, 0x01 },  // Video Timing System Clock Divider value: 1
	{ 0x0304, 0x03 },  // Pre PLL clock Video Timing System Divider Value: 3
	{ 0x0305, 0x03 },  // Pre PLL clock Output System Divider Value: 3
	{ 0x0306, 0x00 },  // PLL Video Timing System multiplier value: 0x39 (57)
	{ 0x0307, 0x39 },
	{ 0x0309, 0x0A },  // Output Pixel Clock Divider value: 0x0A (10)
	{ 0x030B, 0x01 },  // Output System Clock Divider value: 0x01
	{ 0x030C, 0x00 },  // PLL Output System multiplier value: 0x72 (114)
	{ 0x030D, 0x72 },

	/* Here be the same dragons, plus one extra (CIS tuning)*/
	{ 0x455E, 0x00 },
	{ 0x471E, 0x4B },
	{ 0x4767, 0x0F },
	{ 0x4750, 0x14 },
	{ 0x4540, 0x00 },
	{ 0x47B4, 0x14 },
	{ 0x4713, 0x30 },
	{ 0x478B, 0x10 },
	{ 0x478F, 0x10 },
	{ 0x4793, 0x10 },  // This is the extra dragon
	{ 0x4797, 0x0E },
	{ 0x479B, 0x0E },

	{ 0xFFFF, 0x00 } /* end of the list */
};


static struct imx219_reg testInit[256] = {
	{ 0x0100, 0x00 },  // 0=OFF, 1=Stream, 2=MAX
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x0C },
	{ 0x300A, 0xFF },
	{ 0x300B, 0xFF },
	{ 0x30EB, 0x05 },
	{ 0x30EB, 0x09 },
	{ 0x0114, 0x01 },  // CSI MIPI Lanes [1:0]  (0x01=2, 0x03=4)
	{ 0x0128, 0x00 },  // DPHY_CNTRL
	{ 0x012A, 0x18 },  // EXCK_FREQ [15:8]
	{ 0x012B, 0x00 },  // EXCK_FREQ [7:0]
	{ 0x0157, 0x00 },  // Analog Gain
	{ 0x0158, 0x00 },  // Digital Gain [15:8]
	{ 0x0159, 0x00 },  // Digital Gain [7:0]
					   // {0x015A, 0x01},		// Shutter/Integration Time [15:8]
					   // {0x015B, 0x00},		// Shutter/Integration Time [7:0]
	{ 0x0160, 0x09 },  // Frame Length [15:8]
	{ 0x0161, 0xC8 },  // Frame Length [7:0]
	{ 0x0162, 0x0D },  // Line Length [15:8]
	{ 0x0163, 0x78 },  // Line Length [7:0]
	{ 0x0164, 0x00 },
	{ 0x0165, 0x00 },
	{ 0x0166, 0x0C },
	{ 0x0167, 0xCF },
	{ 0x0168, 0x00 },
	{ 0x0169, 0x00 },
	{ 0x016A, 0x09 },
	{ 0x016B, 0x9F },
	{ 0x016C, 0x0C },
	{ 0x016D, 0xD0 },
	{ 0x016E, 0x09 },
	{ 0x016F, 0xA0 },
	{ 0x0170, 0x01 },  // X_ODD_INC [2:0]
	{ 0x0171, 0x01 },  // Y_ODD_INC [2:0]
	{ 0x0172, 0x03 },
	{ 0x0174, 0x00 },  // Binning Mode H_A
	{ 0x0175, 0x00 },  // Binning Mode V_A
	{ 0x018C, 0x0A },  // CSI Data Format [15:8]
	{ 0x018D, 0x0A },  // CSI Data Format [7:0]
	{ 0x0301, 0x05 },  // VTPXCK_DIV
	{ 0x0303, 0x01 },  // VTSYCK_DIV
	{ 0x0304, 0x03 },  // PREPLLCK_VT_DIV [3:0]
	{ 0x0305, 0x03 },  // PREPLLCK_OP_DIV [3:0]
	{ 0x0306, 0x00 },  // PLL_VT_MPY [10:8]
	{ 0x0307, 0x2B },  // PLL_VT_MPY [7:0]
	{ 0x0309, 0x0A },  // OPPXCK_DIV [4:0]
	{ 0x030B, 0x01 },  // OPSYCK_DIV
	{ 0x030C, 0x00 },  // PLL_OP_MPY [10:8]
	{ 0x030D, 0x55 },  // PLL_OP_MPY [7:0]
	{ 0x455E, 0x00 },  // CIS Tuning ?
	{ 0x471E, 0x4B },  // CIS Tuning ?
	{ 0x4767, 0x0F },  // CIS Tuning ?
	{ 0x4750, 0x14 },  // CIS Tuning ?
	{ 0x4540, 0x00 },  // CIS Tuning ?
	{ 0x47B4, 0x14 },  // CIS Tuning ?
	{ 0x4713, 0x30 },  // CIS Tuning ?
	{ 0x478B, 0x10 },  // CIS Tuning ?
	{ 0x478F, 0x10 },  // CIS Tuning ?
	{ 0x4797, 0x0E },  // CIS Tuning ?
	{ 0x479B, 0x0E },  // CIS Tuning ?
	{ 0x0100, 0x01 },

	{ 0xFFFF, 0x00 } /* end of the list */

};

static struct imx219_reg imx219_testPattern[256] = {

	{ 0x0600, 0x00 },
	{ 0x0601, 0x02 },  // color bar

	{ 0x060A, 0x01 },
	{ 0x060B, 0x00 },
	{ 0x060C, 0x02 },
	{ 0x060D, 0x00 },
	{ 0x060E, 0x01 },
	{ 0x060F, 0x00 },
	{ 0x0610, 0x02 },
	{ 0x0611, 0x00 },

	{ 0x0620, 0x00 },
	{ 0x0621, 0x00 },
	{ 0x0622, 0x00 },
	{ 0x0623, 0x00 },
	{ 0x0624, 0x09 },
	{ 0x0625, 0xC8 },
	{ 0x0626, 0x0D },
	{ 0x0627, 0x78 },


	{ 0xFFFF, 0x00 } /* end of the list */
};

static struct imx219_reg imx219_startStream[256] = {
	{ 0x0100, 0x01 },
	{ 0xFFFF, 0x00 }
};

static struct imx219_reg imx219_stopStream[256] = {
	{ 0x0100, 0x00 },
	{ 0xFFFF, 0x00 }
};

static struct {
	volatile uint32_t *base;
	int initialized;
	handle_t lock; /* I2C IRQ mutex */
	handle_t cond; /* I2C IRQ cond */
	handle_t inth; /* I2C IRQ handle */
	volatile uint32_t st;
} mipi = { 0 };

static int mipi_isr(unsigned int n, void *arg)
{
	// printf("delete this printf, pls\n");
	return 1;
}

static inline int mipi_init(void)
{
	int err = EOK;

	err = mutexCreate(&mipi.lock);
	if (err < 0) {
		return err;
	}

	err = condCreate(&mipi.cond);
	if (err < 0) {
		resourceDestroy(mipi.lock);
		return err;
	}

	/* map i2c registers */
	mipi.base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, BASE_ADDR_MIPI);
	if (mipi.base == MAP_FAILED) {
		resourceDestroy(mipi.cond);
		resourceDestroy(mipi.lock);
		return -ENOMEM;
	}

	volatile uint32_t *mipi_base = mmap(NULL, _PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, BASE_ADDR_MIPI);
	*(mipi_base + MIPI_REG_CORE_CONFIG) = 0;
	while (1 == (*(mipi_base + MIPI_REG_CORE_STATUS) & MIPI_STATUS_RESET))
		usleep(1);
	// *(mipi_base + MIPI_REG_CORE_CONFIG) = 0;

	*(mipi_base + MIPI_REG_IRQ_ENABLE) = MIPI_IRQ_FRAME_RX | MIPI_IRQ_VCX_FRAME_ERR;
	// *(mipi_base + MIPI_REG_IRQ_ENABLE) = ~0;
	// *(mipi_base + MIPI_REG_PROTOCOL_CONFIG) = MIPI_2ACTIVE_LANES;  // two lanes
	*(mipi_base + MIPI_REG_PROTOCOL_CONFIG) = 0x0;  // one lane
	*(mipi_base + MIPI_REG_DYN_VC_SELECT) = 0x1;
	*(mipi_base + MIPI_REG_GIE) = MIPI_GIE;
	*(mipi_base + MIPI_REG_CORE_CONFIG) = MIPI_CORE_ENABLE;

	interrupt(121, mipi_isr, NULL, mipi.cond, &mipi.inth);

	mipi.initialized = 1;

	return err;
}

int imx219_regWriteBatch(struct imx219_reg *regs)
{
	int ret = 0;
	int i;
	for (i = 0; i < 256 && regs[i].addr != 0xFFFF; i++) {
		ret = i2c_regWrite16(DEV_ADDR, regs[i].addr, &(regs[i].value), 1);
		if (ret != 0) {
			return ret;
		}
	}
	return ret;
}

void imx219_regCheckBatch(struct imx219_reg *regs)
{
	int i;
	uint8_t byte;
	for (i = 0; i < 256 && regs[i].addr != 0xFFFF; i++) {
		i2c_regRead16(DEV_ADDR, regs[i].addr, &byte, 1);
		if (byte == regs[i].value) {
			printf("0x%04x: 0x%02x (match)\n", regs[i].addr, byte);
		}
		else {
			printf("0x%04x: mismatch detected! - written: 0x%02x, read: 0x%02x\n", regs[i].addr, regs[i].value, byte);
		}
	}
}

static void printReg(uint16_t reg)
{
	uint8_t val;
	i2c_regRead16(DEV_ADDR, reg, &val, 1);
	printf("0x%04x: 0x%02x\n", reg, val);
}

static void writeReg(uint16_t reg, uint8_t val)
{
	i2c_regWrite16(DEV_ADDR, reg, &val, 1);
}


int main(int argc, char **argv)
{
	char *i2cDevice = "/dev/i2c0";
	oid_t i2cDev;

	standard_proc(0);

	if (lookup(i2cDevice, NULL, &i2cDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}


	printf("Successfully checked i2c device oid\n");


	mipi_init();

	printReg(0x0018);
	printReg(0x0019);
	printReg(0x001A);
	printReg(0x001B);
	if (0) {
		imx219_regWriteBatch(imx219_reginit);
		imx219_regCheckBatch(imx219_reginit);
	}
	if (0) {
		imx219_regWriteBatch(testInit);
		imx219_regCheckBatch(testInit);
	}
	if (1) {
		imx219_regWriteBatch(imx219_secondInit);
		imx219_regCheckBatch(imx219_secondInit);
	}


	if (0)
		writeReg(0x0600, 0);
	// writeReg(0x0600, 3);
	// usleep(100);

	if (0)
		imx219_regWriteBatch(imx219_testPattern);
	// imx219_regCheckBatch(imx219_testPattern);
	// if (0)
	imx219_regWriteBatch(imx219_startStream);

	printf("\n\tFrame bank regs:\n");
	printReg(0x0150);
	printReg(0x0151);
	printReg(0x0152);

	// usleep(5000);
	printf("\n\tMIPI irq status: 0x%08x\n", *(mipi.base + MIPI_REG_IRQ_STATUS));
	for (int i = 0; i < 32; i += 2) {
		printf("\t0x%08x\n", *(mipi.base + MIPI_REG_IMG_INFO + i));
	}

	printf("\n\tModel id:\n");
	printReg(0x0000);
	printReg(0x0001);

	printf("\n\tFrame counter:\n");
	printReg(0x0018);
	sleep(1);
	printf("after 1 second\n");
	printReg(0x0018);
	usleep(500000);
	printf("after another half a second\n");
	printReg(0x0018);
	imx219_regWriteBatch(imx219_stopStream);

	return 0;
}
