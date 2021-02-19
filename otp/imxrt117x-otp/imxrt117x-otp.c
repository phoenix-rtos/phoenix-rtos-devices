/*
 * Phoenix-RTOS
 *
 * IMXRT117x OTP tool.
 *
 * Writes or reads OTP fuses
 *
 * Copyright 2020 Phoenix Systems
 * Author: Aleksander Kaminski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdint.h>
#include <errno.h>
#include <string.h>

#define FUSE_MIN 0x800
#define FUSE_MAX 0x18F0
#define OTP_BASE ((void *)0x40cac000)

enum { otp_ctrl = 0, otp_ctrl_set, otp_ctrl_clr, otp_ctrl_tog, otp_pdn,
	otp_data = 8, otp_read_ctrl = 12, otp_out_status = 36, otp_out_status_set, otp_out_status_clr,
	otp_out_status_tog, otp_version = 44, otp_read_fuse_data0 = 64, otp_read_fuse_data1 = 68,
	otp_read_fuse_data2 = 72, otp_read_fuse_data3 = 76, otp_sw_lock = 80, otp_bit_lock = 84,
	otp_locked0 = 384, otp_locked1 = 388, otp_locked2 = 392, otp_locked3 = 396, otp_locked4 = 400,
	otp_fuse = 512 };


struct {
	volatile uint32_t *base;
} otp_common;


void otp_clrError(void)
{
	*(otp_common.base + otp_ctrl_clr) = 1 << 11;
	*(otp_common.base + otp_out_status_clr) = ~0;
}


void otp_waitBusy(void)
{
	while (*(otp_common.base + otp_ctrl) & (1 << 10))
		usleep(100 * 1000);

	/* Wait some more (at least 2 us) */
	usleep(1000);
}


unsigned int fuse2addr(int fuse)
{
	return ((fuse - 0x800) >> 4) & 0x3ff;
}


int read_fuse(int fuse, uint32_t *val)
{
	unsigned int t;

	otp_clrError();
	otp_waitBusy();

	/* Set fuse address */
	t = *(otp_common.base + otp_ctrl) & ~0x3ff;
	*(otp_common.base + otp_ctrl) = t | fuse2addr(fuse);

	/* Start read */
	t = *(otp_common.base + otp_read_ctrl) & ~0x1f;
	*(otp_common.base + otp_read_ctrl) = t | 0x7;

	otp_waitBusy();

	*val = *(otp_common.base + otp_read_fuse_data0);

	if (*(otp_common.base + otp_out_status) & (1 << 24)) {
		*(otp_common.base + otp_out_status_clr) |= 1 << 24;

		return -EIO;
	}

	return 0;
}


int write_fuse(int fuse, uint32_t val)
{
	int res = 0;
	unsigned int t;

	otp_clrError();
	otp_waitBusy();

	/* Set fuse address and unlock write */
	t = *(otp_common.base + otp_ctrl) & ~0xffff03ff;
	*(otp_common.base + otp_ctrl) = t | (0x3e77 << 16) | fuse2addr(fuse);

	/* Program word */
	*(otp_common.base + otp_data) = val;

	otp_waitBusy();

	t = *(otp_common.base + otp_out_status);

	if (t & (1 << 12))
		res = -EIO;

	if (t & (1 << 11))
		res = -EPERM;

	if (res != 0)
		*(otp_common.base + otp_out_status_clr) |= 0x3 << 11;

	return res;
}


int main(int argc, char *argv[])
{
	int opt, read = 0, write = 0, fuse = -1, usage = 0, res;
	uint32_t val = 0;

	otp_common.base = OTP_BASE;

	/* Fast path for being started from syspage */
	if (argc == 1)
		return 0;

	while ((opt = getopt(argc, argv, "f:rw:")) >= 0) {
		switch (opt) {
		case 'f':
			fuse = (int)strtoul(optarg, NULL, 0);
			break;

		case 'r':
			read = 1;
			break;

		case 'w':
			write = 1;
			val = (uint32_t)strtoul(optarg, NULL, 0);
			break;

		default:
			usage = 1;
			break;
		}
	}

	if (!(read ^ write) || fuse < FUSE_MIN || fuse > FUSE_MAX || usage) {
		fprintf(stderr, "Tool for management otp of i.MX RT117x MCU. Usage:\n");
		fprintf(stderr, "%s [-r | -w value] -f fuse\n", argv[0]);
		fprintf(stderr, "\t-r\t\tRead fuse\n");
		fprintf(stderr, "\t-w val\t\tWrite fuse with value [val] (32-bit number)\n");
		fprintf(stderr, "\t-f fuse\t\tSelect fuse to read/write\n");

		return 1;
	}

	if (read) {
		if ((res = read_fuse(fuse, &val)) < 0) {
			fprintf(stderr, "Fuse reading failed! (%s)\n", strerror(res));
			return -1;
		}

		printf("0x%08x\n", val);
	}
	else if (write) {
		if ((res = write_fuse(fuse, val)) < 0) {
			fprintf(stderr, "Fuse write failed! (%s)\n", strerror(res));
			return -1;
		}
	}

	return 0;
}
