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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define OTP_BASE_ADDR 0x21BC000

#define IPG_CLK_RATE (66 * 1000 * 1000)

enum { ocotp_ctrl, ocotp_ctrl_set, ocotp_ctrl_clr, ocotp_ctrl_tog, ocotp_timing = 0x4, ocotp_data = 0x8, ocotp_read_ctrl = 0xc,
	ocotp_read_fuse_data = 0x10, ocotp_sw_sticky = 0x14, ocotp_scs = 0x18, ocotp_scs_set, ocotp_scs_clr, ocotp_scs_tog,
	ocotp_crc_addr = 0x1c, ocotp_crc_value = 0x20, ocotp_version = 0x24, ocotp_timing2 = 0x40, ocotp_lock = 0x100,
	ocotp_cfg0 = 0x104, ocotp_cfg1 = 0x108, ocotp_cfg2 = 0x10c, ocopt_cfg3 = 0x110, ocotp_cfg4 = 0x114, ocotp_cfg5 = 0x118,
	ocotp_cfg6 = 0x11c, ocotp_mem0 = 0x120, ocotp_mem1 = 0x124, ocotp_mem2 = 0x128, ocotp_mem3 = 0x12c, ocotp_mem4 = 0x130,
	ocotp_ana0 = 0x134, ocotp_ana1 = 0x138, ocotp_ana2 = 0x13c /* fast forward */, ocotp_mac0 = 0x188, ocotp_mac1 = 0x18c,
	ocotp_mac = 0x190, ocotp_gp1 = 0x198, ocotp_gp2 = 0x19c, ocotp_sw_gp0 = 0x1a0, ocotp_sw_gp1 = 0x1a4, ocotp_sw_gp2 = 0x1a8,
	ocotp_sw_gp3 = 0x1ac, ocotp_sw_gp4 = 0x1b0, /* ff */ ocotp_gp3_0 = 0x220, ocotp_gp3_1 = 0x224,
	ocotp_gp3_2 = 0x228, ocotp_gp3_3 = 0x22c //TODO: rest of otp shadow regs
	};

typedef struct _otp_t {
	volatile uint32_t *base;
} otp_t;

otp_t otp = { 0 };


/* otp op type */
#define OTP_OP_WRITE 1
#define OTP_OP_READ 2

/* PLC MAC otp address */
#define OTP_ADDR_PLC_MAC0	0x26
#define OTP_ADDR_PLC_MAC1 	0x27

/* serial number otp addresses */
#define OTP_ADDR_SN0	0x28
#define OTP_ADDR_SN1 	0x29
#define OTP_ADDR_SN2 	0x2a
#define OTP_ADDR_SN3 	0x2b

/* MAC otp addresses */
#define OTP_ADDR_MAC0 	0x22
#define OTP_ADDR_MAC1 	0x23
#define OTP_ADDR_MAC2 	0x24


/* otp ctrl flags */
#define OTP_NONE	0x0
#define OTP_BUSY	0x100
#define OTP_ERROR	0x200
#define OTP_RELOAD	0x400
#define OTP_WR_UNLOCK (0x3e77 << 16)


struct common_s {
	int blow_boot;
	int get_uid;
	int rw_mac;
	int r_mac_no;
	int rw_sn;
	int display_usage;
} common;


int otp_check_err(void)
{
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR)
		return 1;
	return 0;
}


void otp_clear_err(void)
{
	*(otp.base + ocotp_ctrl_clr) = OTP_ERROR;
	while (*(otp.base + ocotp_ctrl) & (OTP_BUSY | OTP_ERROR)) usleep(10000);
	usleep(100000);
}


int otp_cnc_err(void)
{
	int res;

	if ((res = otp_check_err()))
		otp_clear_err();
	return res;
}


void otp_wait(unsigned extra_fl)
{
	while (*(otp.base + ocotp_ctrl) & (OTP_BUSY | extra_fl)) usleep(10000);
	usleep(100000);
}


int otp_write(unsigned addr, unsigned data)
{
	int ret = 0;

	/* check busy */
	otp_wait(OTP_NONE);

	/* clear error */
	otp_cnc_err();

	/* check address */
	if (addr & ~0x3f) {
		printf("invalid address\n");
		return -1;
	}

	/* set address and data */
	*(otp.base + ocotp_ctrl_set) = addr | OTP_WR_UNLOCK;
	*(otp.base + ocotp_data) = data;

	/* check and clear error */
	if (otp_cnc_err()) {
		printf("Write at 0x%x failed\n", addr);
		ret = -1;
	}

	/* wait for completion */
	otp_wait(OTP_NONE);

	/* clear address */
	*(otp.base + ocotp_ctrl_clr) = addr | OTP_WR_UNLOCK;

	/* wait for completion */
	otp_wait(OTP_NONE);

	return ret;
}


int otp_reload(void)
{
	/* check busy */
	otp_wait(OTP_NONE);

	/* clear error */
	otp_cnc_err();

	/* set reload */
	*(otp.base + ocotp_ctrl_set) = OTP_RELOAD;

	/* check and clear error */
	if (otp_cnc_err()) {
		printf("Reload error\n");
		return -1;
	}

	/*wait for completion */
	otp_wait(OTP_RELOAD);

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


int check_hex(char c)
{
	if (c >= 'A' && c <= 'F')
		return 0;
	else if (c >= '0' && c <= '9')
		return 0;
	return -1;
}

int verify_mac(char *mac[])
{
	int i;

	for (i = 0; i < 6; i++) {
		if (strlen(mac[i]) != 2)
			return -EINVAL;

		if (check_hex(mac[i][0]) || check_hex(mac[i][1]))
			return -EINVAL;
	}
	return EOK;
}


int read_mac(int silent, int mac_no)
{
	unsigned int mac0, mac1, mac, res = 1, plc0, plc1;
	unsigned m1[6] = { 0 };
	unsigned m2[6] = { 0 };
	unsigned m3[6] = { 0 };

	mac0 = *(otp.base + ocotp_mac0);
	mac1 = *(otp.base + ocotp_mac1);
	mac = *(otp.base + ocotp_mac);

	plc0 = *(otp.base + ocotp_gp1);
	plc1 = *(otp.base + ocotp_gp2);

	if (!mac0 && !mac1 && !mac && !plc0 && !plc1)
		res = 0;

	m1[0] = (mac1 >> 8) & 0xFF;
	m1[1] = (mac1) & 0xFF;
	m1[2] = (mac0 >> 24) & 0xFF;
	m1[3] = (mac0 >> 16) & 0xFF;
	m1[4] = (mac0 >> 8) & 0xFF;
	m1[5] = (mac0) & 0xFF;

	m2[0] = (mac >> 24) & 0xFF;
	m2[1] = (mac >> 16) & 0xFF;
	m2[2] = (mac >> 8) & 0xFF;
	m2[3] = (mac) & 0xFF;
	m2[4] = (mac1 >> 24) & 0xFF;
	m2[5] = (mac1 >> 16) & 0xFF;

	m3[0] = (plc1 >> 8) & 0xFF;
	m3[1] = (plc1) & 0xFF;
	m3[2] = (plc0 >> 24) & 0xFF;
	m3[3] = (plc0 >> 16) & 0xFF;
	m3[4] = (plc0 >> 8) & 0xFF;
	m3[5] = (plc0) & 0xFF;


	if (!silent) {
		if (!mac_no || mac_no == 1)
			printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n", !mac_no ? "MAC1: " : "",
				m1[0], m1[1], m1[2], m1[3], m1[4], m1[5]);
		if (!mac_no || mac_no == 2)
			printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n", !mac_no ? "MAC2: " : "",
				m2[0], m2[1], m2[2], m2[3], m2[4], m2[5]);
		if (!mac_no || mac_no == 3)
			printf("%s%02X:%02X:%02X:%02X:%02X:%02X\n", !mac_no ? "PLC: " : "",
				m3[0], m3[1], m3[2], m3[3], m3[4], m3[5]);
	}

	return res;
}


int write_mac(char *mac1_str, char *mac2_str, char *mac3_str) {

	int i, ret = 0;
	char *mac1[6] = { 0 };
	char *mac2[6] = { 0 };
	char *mac3[6] = { 0 };
	unsigned m1[6] = { 0 };
	unsigned m2[6] = { 0 };
	unsigned m3[6] = { 0 };

	if (read_mac(1, 0)) {
		printf("MACs are already written\n");
		return -1;
	}

	if (mac1_str == NULL || mac2_str == NULL || mac3_str == NULL) {
		printf("Invalid argument\n");
		return -1;
	}

	for (i = 0; i < 6; i++) {
		mac1[i] = calloc(1, 4);
		mac2[i] = calloc(1, 4);
		mac3[i] = calloc(1, 4);
	}
	printf("mac1: %s mac2: %s mac3: %s\n", mac1_str, mac2_str, mac3_str);
	if (sscanf(mac1_str, "%2s:%2s:%2s:%2s:%2s:%3s",
		mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]) != 6) {
		printf("Invalid MAC1 address format\n");
		ret = -1;
	}

	if (!ret && sscanf(mac2_str, "%2s:%2s:%2s:%2s:%2s:%3s",
		mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]) != 6) {
		printf("Invalid MAC2 address format\n");
		ret = -1;
	}

	if (!ret && sscanf(mac3_str, "%2s:%2s:%2s:%2s:%2s:%3s",
		mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]) != 6) {
		printf("Invalid PLC MAC address format\n");
		ret = -1;
	}

	if (!ret && verify_mac(mac1)) {
		printf("Invalid MAC1 address format\n");
		ret = -1;
	}

	if (!ret && verify_mac(mac2)) {
		printf("Invalid MAC2 address format\n");
		ret = -1;
	}

	if (!ret && verify_mac(mac3)) {
		printf("Invalid PLC MAC address format\n");
		ret = -1;
	}

	if (!ret) {
		for (i = 0; i < 6; i++) {
			sscanf(mac1[i], "%X", &m1[i]);
			sscanf(mac2[i], "%X", &m2[i]);
			sscanf(mac3[i], "%X", &m3[i]);
		}

		ret |= otp_write(OTP_ADDR_MAC0, (unsigned)(m1[5] | m1[4] << 8 | m1[3] << 16 | m1[2] << 24));
		ret |= otp_write(OTP_ADDR_MAC1, (unsigned)(m1[1] | m1[0] << 8 | m2[5] << 16 | m2[4] << 24));
		ret |= otp_write(OTP_ADDR_MAC2, (unsigned)(m2[3] | m2[2] << 8 | m2[1] << 16 | m2[0] << 24));

		ret |= otp_write(OTP_ADDR_PLC_MAC0, (unsigned)(m3[5] | m3[4] << 8 | m3[3] << 16 | m3[2] << 24));
		ret |= otp_write(OTP_ADDR_PLC_MAC1,  (unsigned)(m3[1] | m3[0] << 8) & 0xFFFF);

		otp_reload();
	}

	for (i = 0; i < 6; i++) {
		free(mac1[i]);
		free(mac2[i]);
	}

	printf("MAC addrs %s\n", !ret ? "written" : "write failed");
	return ret;
}


int check_sn(const char *sn, const size_t len)
{
	int i;

	if (len < 8) {
		printf("Serial number too short\n");
		return -EINVAL;
	}

	if (len > 16) {
		printf("Serial number too long\n");
		return -EINVAL;
	}

	for (i = 0; i < 3; i++) {
		if (sn[i] < 'A' || sn[i] > 'Z') {
			printf("Invalid manufacturer id (only capital letters)\n");
			return -EINVAL;
		}
	}

	for (; i < len; i++) {
		if (sn[i] < '0' || sn[i] > '9') {
			printf("Invalid s/n (only digits after manufacturer id)\n");
			return -EINVAL;
		}
	}

	return 0;
}


int read_sn(int silent)
{
	int res = 1;
	unsigned sn0, sn1, sn2, sn3, lock;
	char sn[17] = { 0 };

	otp_reload();
	lock = *(otp.base + ocotp_lock);
	if ((lock >> 31) & 0x1) {
		printf("Reading S/N is locked\n");
		return -1;
	}

	sn0 = *(otp.base + ocotp_sw_gp0);
	sn1 = *(otp.base + ocotp_sw_gp1);
	sn2 = *(otp.base + ocotp_sw_gp2);
	sn3 = *(otp.base + ocotp_sw_gp3);

	if (!sn0 && !sn1 && !sn2 && !sn3)
		res = 0;

	sn[0] = (char)((sn0 >> 24) & 0xFF);
	sn[1] = (char)((sn0 >> 16) & 0xFF);
	sn[2] = (char)((sn0 >> 8) & 0xFF);
	sn[3] = (char)((sn0) & 0xFF);

	sn[4] = (char)((sn1 >> 24) & 0xFF);
	sn[5] = (char)((sn1 >> 16) & 0xFF);
	sn[6] = (char)((sn1 >> 8) & 0xFF);
	sn[7] = (char)((sn1) & 0xFF);

	sn[8] = (char)((sn2 >> 24) & 0xFF);
	sn[9] = (char)((sn2 >> 16) & 0xFF);
	sn[10] = (char)((sn2 >> 8) & 0xFF);
	sn[11] = (char)((sn2) & 0xFF);

	sn[12] = (char)((sn3 >> 24) & 0xFF);
	sn[13] = (char)((sn3 >> 16) & 0xFF);
	sn[14] = (char)((sn3 >> 8) & 0xFF);
	sn[15] = (char)((sn3) & 0xFF);

	if (strlen(sn) == 0)
		sprintf(sn, "DEV000001");

	if (!silent)
		printf("%s\n", sn);

	return res;
}


int write_sn(const char *sn)
{
	int ret = 0, i = 0;
	size_t len;
	unsigned sn0 = 0, sn1 = 0, sn2 = 0, sn3 = 0, lock = 0;

	if (read_sn(1)) {
		printf("S/N already written\n");
		return -1;
	}

	if (!sn)
		return -1;

	len = strlen(sn);

	if (check_sn(sn, len))
		return -1;

	sn0 |= ((unsigned)sn[i++] << 24) & (0xFF << 24);
	sn0 |= ((unsigned)sn[i++] << 16) & (0xFF << 16);
	sn0 |= ((unsigned)sn[i++] << 8) & (0xFF << 8);
	sn0 |= ((unsigned)sn[i++]) & 0xFF;

	sn1 |= ((unsigned)sn[i++] << 24) & (0xFF << 24);
	sn1 |= ((unsigned)sn[i++] << 16) & (0xFF << 16);
	sn1 |= ((unsigned)sn[i++] << 8) & (0xFF << 8);
	sn1 |= ((unsigned)sn[i++]) & 0xFF;

	sn2 |= len > i ? ((unsigned)sn[i++] << 24) & (0xFF << 24) : 0;
	sn2 |= len > i ? ((unsigned)sn[i++] << 16) & (0xFF << 16) : 0;
	sn2 |= len > i ? ((unsigned)sn[i++] << 8) & (0xFF << 8) : 0;
	sn2 |= len > i ? (((unsigned)sn[i++]) & 0xFF) : 0;

	sn3 |= len > i ? ((unsigned)sn[i++] << 24) & (0xFF << 24) : 0;
	sn3 |= len > i ? ((unsigned)sn[i++] << 16) & (0xFF << 16) : 0;
	sn3 |= len > i ? ((unsigned)sn[i++] << 8) & (0xFF << 8) : 0;
	sn3 |= len > i ? (((unsigned)sn[i++]) & 0xFF) : 0;

	otp_reload();
	lock = *(otp.base + ocotp_lock);
	if ((lock >> 15) & 0x1) {
		printf("Writing S/N is locked\n");
		return -1;
	}

	ret |= otp_write(OTP_ADDR_SN0, sn0);
	ret |= otp_write(OTP_ADDR_SN1, sn1);
	ret |= otp_write(OTP_ADDR_SN2, sn2);
	ret |= otp_write(OTP_ADDR_SN3, sn3);

	otp_reload();

	if (ret)
		printf("Writing S/N failed\n");
	else
		read_sn(0);

	return ret;
}


int main(int argc, char **argv)
{
	int res;
	char *mac[3] = { 0 };
	char *sn = NULL;

	otp.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE, OID_PHYSMEM, OTP_BASE_ADDR);
	if (otp.base == NULL) {
		printf("OTP mmap failed\n");
		return -1;
	}

	while ((res = getopt(argc, argv, "Ss:buMm:")) >= 0) {
		switch (res) {
		case 'b':
			common.blow_boot = 1;
			break;
		case 'u':
			common.get_uid = 1;
			break;
		case 'm':
			if (argc < 5) {
				common.display_usage = 1;
				break;
			}
			mac[0] = argv[optind - 1];
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				common.display_usage = 1;
				break;
			}
			mac[1] = argv[optind];
			optind++;
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				common.display_usage = 1;
				break;
			}
			mac[2] = argv[optind];
			optind++;
			common.rw_mac = OTP_OP_WRITE;
			break;
		case 'M':
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				common.r_mac_no = 0;
			}
			else if (sscanf(argv[optind], "%d", &common.r_mac_no) == 1) {
				if (common.r_mac_no <= 0 || common.r_mac_no > 3)
					common.display_usage = 1;
			}
			else {
				common.display_usage = 1;
			}
			common.rw_mac = OTP_OP_READ;
			break;
		case 's':
			sn = optarg;
			common.rw_sn = OTP_OP_WRITE;
			break;
		case 'S':
			common.rw_sn = OTP_OP_READ;
			break;
		default:
			common.display_usage = 1;
			break;
		}
	}

	if (common.display_usage || (!common.blow_boot && !common.get_uid && !common.rw_mac && !common.rw_sn)) {
		printf("Usage: imx6ull-otp [-b] [-u] [-m MAC1 MAC2]\n\r");
		printf("\t-b\t\tBlow boot fuses\n\r");
		printf("\t-u\t\tGet unique ID\n\r");
		printf("\t-m MAC1 MAC2 PLC_MAC\twrite MAC addresses (MAC format XX:XX:XX:XX:XX:XX)\n\r");
		printf("\t-M [1 - 3]\t\tprint MAC address [1 = MAC1, 2 = MAC2, 3 = PLC] or all [no arg]\n\r");
		printf("\t-s S/N\t\twrite serial number\n\r");
		printf("\t-S\t\tprint serial number\n\r");
		return 1;
	}

	if (common.rw_sn == OTP_OP_WRITE)
		write_sn(sn);

	if (common.rw_sn == OTP_OP_READ)
		read_sn(0);

	if (common.blow_boot)
		blow_boot_fuses();

	if (common.get_uid)
		get_unique_id();

	if (common.rw_mac == OTP_OP_WRITE)
		write_mac(mac[0], mac[1], mac[2]);

	if (common.rw_mac == OTP_OP_READ)
		read_mac(0, common.r_mac_no);

	return 0;
}