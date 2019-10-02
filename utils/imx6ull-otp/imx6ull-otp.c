/*
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
	int blow_boot;
	int get_uid;
	int blow_mac;
	int display_usage;
} common;


int otp_write(unsigned addr, unsigned data)
{
	/*check busy */
	if (*(otp.base + ocotp_ctrl) & OTP_BUSY) {
		printf("otp busy\n");
		return -1;
	}

	/* clear error */
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR)
		*(otp.base + ocotp_ctrl) &= ~OTP_ERROR;

	/* check address */
	if (addr & ~0x7f) {
		printf("invalid address\n");
		return -1;
	}

	/* set address and data */
	*(otp.base + ocotp_ctrl_set) = addr | OTP_WR_UNLOCK;
	*(otp.base + ocotp_data) = data;

	/* check error */
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("data write error\n");
		return -1;
	}

	/* wait for completion */
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	/* clear address */
	*(otp.base + ocotp_ctrl_clr) = addr | OTP_WR_UNLOCK;

	/* wait for completion */
	while (*(otp.base + ocotp_ctrl) & OTP_BUSY) usleep(10000);
	usleep(100000);

	return 0;
}


int otp_reload(void)
{
	/* check busy */
	if (*(otp.base + ocotp_ctrl) & OTP_BUSY) {
		printf("otp busy\n");
		return -1;
	}

	/* clear error */
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR)
		*(otp.base + ocotp_ctrl) &= ~OTP_ERROR;

	/* set reload */
	*(otp.base + ocotp_ctrl_set) = OTP_RELOAD;

	/* check error */
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR) {
		printf("reload error\n");
		return -1;
	}

	/*wait for completion */
	while (*(otp.base + ocotp_ctrl) & (OTP_BUSY & OTP_RELOAD)) usleep(10000);
	usleep(100000);

	return 0;
}


int otp_write_reload(unsigned addr, unsigned data)
{
	return otp_write(addr, data) ? -1 : otp_reload();
}


int blow_boot_fuses(void)
{
	if (otp_write(0x5, 0x1090)) /* set nand options (64 pages per block, 4 fcb)*/
		return -1;

	if (otp_write_reload(0x6, 0x10)) /* set internal boot fuse */
		return -1;

	printf("Boot fuses blown\n");
	return 0;
}


unsigned long long get_unique_id(void)
{
	uint64_t uid;

	uid = ((uint64_t)*(otp.base + ocotp_cfg1)) << 32 | *(otp.base + ocotp_cfg0);

	printf("0x%llx\n", uid);

	return uid;
}

int blow_mac_fuses(char *mac1_str, char *mac2_str) {
	unsigned mac1[6] = { 0 };
	unsigned mac2[6] = { 0 };

	if (mac1_str == NULL || mac2_str == NULL) {
		printf("Invalid argument\n");
		return -1;
	}

	if (sscanf(mac1_str, "%2x:%2x:%2x:%2x:%2x:%2x",
		&mac1[0], &mac1[1], &mac1[2], &mac1[3], &mac1[4], &mac1[5]) != 6) {
		printf("Invalid MAC address format\n");
		return -1;
	}

	if (sscanf(mac2_str, "%2x:%2x:%2x:%2x:%2x:%2x",
		&mac2[0], &mac2[1], &mac2[2], &mac2[3], &mac2[4], &mac2[5]) != 6) {
		printf("Invalid MAC address format\n");
		return -1;
	}

	printf("MAC addr 1: %x:%x:%x:%x:%x:%x\n",
		mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]);
	printf("MAC addr 2: %x:%x:%x:%x:%x:%x\n",
		mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]);

	if (otp_write(0x22, (unsigned)(mac1[5] | mac1[4] << 8 | mac1[3] << 16 | mac1[2] << 24)))
		return -1;

	if (otp_write(0x23, (unsigned)(mac1[1] | mac1[0] << 8 | mac2[5] << 16 | mac2[4] << 24)))
		return -1;

	if (otp_write(0x24, (unsigned)(mac2[3] | mac2[2] << 8 | mac2[1] << 16 | mac2[0] << 24)))
		return -1;

	if (otp_reload())
		return -1;

	printf("MAC addresses fuses blown\n");
	return 0;
}


int main(int argc, char **argv)
{
	int res;
	char *mac[2] = { 0 };

	otp.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, OTP_BASE_ADDR);
	if (otp.base == NULL) {
		printf("OTP mmap failed\n");
		return -1;
	}

	while ((res = getopt(argc, argv, "bum:")) >= 0) {
		switch (res) {
		case 'b':
			common.blow_boot = 1;
			break;
		case 'u':
			common.get_uid = 1;
			break;
		case 'm':
			mac[0] = argv[optind - 1];
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				common.display_usage = 1;
				break;
			}
			mac[1] = argv[optind];
			optind++;
			common.blow_mac = 1;
			break;
		default:
			common.display_usage = 1;
			break;
		}
	}

	if (common.display_usage || (!common.blow_boot && !common.get_uid && !common.blow_mac)) {
		printf("Usage: imx6ull-otp [-b] [-u] [-m MAC1 MAC2]\n\r");
		printf("    -b              Blow boot fuses\n\r");
		printf("    -u              Get unique ID\n\r");
		printf("    -m MAC1 MAC2    Blow MAC addresses (MAC format XX:XX:XX:XX:XX:XX)\n\r");
		return 1;
	}

	if (common.blow_boot)
		blow_boot_fuses();

	if (common.get_uid)
		get_unique_id();

	if (common.blow_mac)
		blow_mac_fuses(mac[0], mac[1]);

	return 0;
}
