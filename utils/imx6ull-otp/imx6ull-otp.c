#/*
 * Phoenix-RTOS
 *
 * IMX6ULL NAND tool.
 *
 * Writes image file to flash
 *
 * Copyright 2018 Phoenix Systems
 * Author: Kamil Amanowicz
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define OTP_BASE_ADDR 0x21BC000

#define IPG_CLK_RATE (66 * 1000 * 1000)

enum { ocotp_ctrl, ocotp_ctrl_set, ocotp_ctrl_clr, ocotp_ctrl_tog, ocotp_timing, ocotp_data = 0x8, ocotp_read_ctrl = 0xc,
	ocotp_read_fuse_data = 0x10, ocotp_sw_sticky = 0x14, ocotp_scs = 0x18, ocotp_scs_set, ocotp_scs_clr, ocotp_scs_tog,
	ocotp_crc_addr = 0x1c, ocotp_crc_value = 0x20, ocotp_version = 0x24, ocotp_timing2 = 0x40, ocotp_lock = 0x100,
	ocotp_cfg0 = 0x104, ocotp_cfg1 = 0x108, ocotp_cfg2 = 0x10c, ocopt_cfg3 = 0x110, ocotp_cfg4 = 0x114, ocotp_cfg5 = 0x118,
	ocotp_cfg6 = 0x11c, ocotp_mem0 = 0x120, ocotp_mem1 = 0x124, ocotp_mem2 = 0x128, ocotp_mem3 = 0x12c, ocotp_mem4 = 0x130,
	ocotp_ana0 = 0x134, ocotp_ana1 = 0x138, ocotp_ana2 = 0x13c //TODO: rest of otp shadow regs
	};

typedef struct _otp_t {
	volatile u32 *base;
} otp_t;

otp_t otp = { 0 };

#define OTP_BUSY	0x100
#define OTP_ERROR	0x200
#define OTP_RELOAD	0x400

#define OTP_WR_UNLOCK (0x3e77 << 16)

struct common_s {
	int blow_fuses;
	int get_uid;
	int display_usage;
} common;

int blow_fuses(void)
{
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("OTP error\n");
		return -1;
	}
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);

	*(otp.base + ocotp_ctrl_set) = 0x6 | OTP_WR_UNLOCK;
	*(otp.base + ocotp_data) = 0x10;

	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("BT_FUSE_SEL error\n");
		return -1;
	}
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	*(otp.base + ocotp_ctrl_clr) = 0x6 | OTP_WR_UNLOCK;
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	*(otp.base + ocotp_ctrl_set) = 0x5 | OTP_WR_UNLOCK;
	*(otp.base + ocotp_data) = 0x1090;

	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("BOOT_CFG error\n");
		return -1;
	}
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	*(otp.base + ocotp_ctrl_set) = OTP_RELOAD;

	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("RELOAD error\n");
		return -1;
	}
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	printf("Fuses blown\n");
	return 0;
}

int get_unique_id(void)
{
	uint64_t uid;

	uid = ((uint64_t)*(otp.base + ocotp_cfg1)) << 32 | *(otp.base + ocotp_cfg0);

	printf("0x%llx\n", uid);

	return 0;
}

int main(int argc, char **argv)
{
	int res;

	otp.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, OTP_BASE_ADDR);
	if (otp.base == NULL) {
		printf("OTP mmap failed\n");
		return -1;
	}

	while ((res = getopt(argc, argv, "bu")) >= 0) {
		switch (res) {
		case 'b':
			common.blow_fuses = 1;
			break;
		case 'u':
			common.get_uid = 1;
			break;
		default:
			common.display_usage = 1;
			break;
		}
	}

	if (common.display_usage || (!common.blow_fuses && !common.get_uid)) {
		printf("Usage: imx6ull-otp [-b] [-u]\n\r");
		printf("    -b    Blow fuses\n\r");
		printf("    -u    Get unique ID\n\r");
		return 1;
	}

	if (common.blow_fuses)
		blow_fuses();

	if (common.get_uid)
		get_unique_id();

	return 0;
}
