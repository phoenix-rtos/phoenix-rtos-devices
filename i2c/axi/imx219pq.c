/*
 * Phoenix-RTOS
 *
 * IMX219PQ AXI I2C (axi_iic) BUS driver
 *
 * Copyright 2025 Phoenix Systems
 * Author: Krzysztof Szostek
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdint.h>
#include <stddef.h>
#include <i2c.h>

#include "imx219pq.h"
#include "i2c.h"

#define DEV_ADDR 0x10

struct imx219_reg {
	uint16_t addr;
	uint8_t value;
};


enum imx219_mode {
	IMX219_MODE_1280x720P30 = 0,
	IMX219_MODE_1920x1080P30,
	IMX219_MODE_3840x2160P30,
	IMX219_MODE_1280x720P60,
	IMX219_MODE_1920x1080P60,
	IMX219_MODE_3840x2160P60,
};


static const struct imx219_reg imx219_reginit[2][256] = {
	{
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

		/* Here be dragons (called CIS Tuning)*/
		{ 0x455E, 0x00 },
		{ 0x471E, 0x4B },
		{ 0x4767, 0x0F },
		{ 0x4750, 0x14 },
		{ 0x4540, 0x00 },
		{ 0x47B4, 0x14 },
		{ 0x4713, 0x30 },
		{ 0x478B, 0x10 },
		{ 0x478F, 0x10 },
		{ 0x4797, 0x0E },
		{ 0x479B, 0x0E },
		{ 0xFFFF, 0x00 } /* end of the list */
	},
	{
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

		/* Here be the same dragons (called CIS Tuning), plus one extra */
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
	}
};

static const struct imx219_reg startStream[256] = {
	{ 0x0100, 0x1 },
	{ 0xFFFF, 0x0 }
};

static const struct imx219_reg endStream[256] = {
	{ 0x0100, 0x0 },
	{ 0xFFFF, 0x0 }
};

// int imx219_regWrite(uint8_t dev_addr, uint16_t reg_addr, uint8_t value)
int imx219_regWrite(uint8_t dev_addr, imx219_reg reg)
{
	// value = __builtin_bswap16(value);
	return i2c_regWrite16(dev_addr, reg.addr, reg.value, sizeof(reg.value));
}

int imx219_regRead(uint8_t dev_addr, uint16_t reg_addr, uint8_t *value)
{
	int res = i2c_regRead16(dev_addr, reg_addr, (uint8_t *)value, sizeof(value));
	// *value = __builtin_bswap32(*value);
	return res;
}


int imx219_regWriteBatch(uint8_t dev_addr, struct imx219_reg *regs)
{
	int ret = 0;
	int i;
	for (i = 0; i < 256 && regs[i].addr != 0xFFFF; i++) {
		ret = imx219_regWrite(dev_addr, regs[i]);
		if (ret != 0) {
			return ret;
		}
	}
	return ret;
}

int main(int argc, char *argv[])
{
	char *i2cDevice = "/dev/i2c0";
	oid_t i2cDev;

	standard_proc(0);

	if (lookup(i2cDevice, NULL, &i2cDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}


	printf("Successfully checked i2c device oid\n");
	// Set config
	// set streaming
	// where frame?
	imx219_regWriteBatch(DEV_ADDR, imx219_reginit[0]);
	imx219_regWriteBatch(DEV_ADDR, startStream);
	usleep(500);
	// imx219_regWriteBatch(DEV_ADDR, endStream);
	return EXIT_SUCCESS;
}
