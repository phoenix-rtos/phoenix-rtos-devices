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

#include <ctype.h>
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

#define SN_FORMAT_RAW "DDDDDDDDDDDDDDDD"
#define SN_FORMAT     "DDDDD-DDDDDDDD-DD-D"

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
#define OTP_ADDR_PLC_MAC0 0x26
#define OTP_ADDR_PLC_MAC1 0x27 /* lower 2 bytes occupied by PLC mac */
#define OTP_ADDR_HW_REV   0x27 /* MSB */

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


#define READ_ADDR_UNDEFINED 0xffff

static struct {
	int blow_boot;
	int get_uid;
	int rw_mac;
	int r_mac_no;
	int rw_sn;
	int rw_revision;
	int force;
	int dry_run;

	uint32_t read_addr;
} common = {
	.read_addr = READ_ADDR_UNDEFINED,
};


static int otp_check_err(void)
{
	if (*(otp.base + ocotp_ctrl) & OTP_ERROR)
		return 1;
	return 0;
}


static void otp_clear_err(void)
{
	*(otp.base + ocotp_ctrl_clr) = OTP_ERROR;
	while (*(otp.base + ocotp_ctrl) & (OTP_BUSY | OTP_ERROR)) usleep(10000);
	usleep(100000);
}


static int otp_cnc_err(void)
{
	int res;

	if ((res = otp_check_err()))
		otp_clear_err();
	return res;
}


static void otp_wait(unsigned extra_fl)
{
	while (*(otp.base + ocotp_ctrl) & (OTP_BUSY | extra_fl)) usleep(10000);
	usleep(100000);
}


static int otp_write(unsigned addr, unsigned data)
{
	int ret = 0;

	if (common.dry_run == 1)
		return 0;

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


static int otp_reload(void)
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


static int otp_write_reload(unsigned addr, unsigned data)
{
	return otp_write(addr, data) ? -1 : otp_reload();
}


static int blow_boot_fuses(void)
{
	if (otp_write(0x5, 0x1090)) /* set nand options (64 pages per block, 4 fcb)*/
		return -1;

	if (otp_write_reload(0x6, 0x10)) /* set internal boot fuse */
		return -1;

	printf("Boot fuses blown\n");
	return 0;
}


static unsigned long long get_unique_id(void)
{
	uint64_t uid;

	uid = ((uint64_t)*(otp.base + ocotp_cfg1)) << 32 | *(otp.base + ocotp_cfg0);

	printf("0x%llx\n", uid);

	return uid;
}


static int check_hex(char c)
{
	if (c >= 'A' && c <= 'F')
		return 0;
	else if (c >= '0' && c <= '9')
		return 0;
	return -1;
}

static int verify_mac(char *mac[])
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


/* returns non-zero if ETH1 and ETH2 macs are non-zero */
static int read_mac(int silent, int mac_no)
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

	if (!mac0 && !mac1 && !mac && !plc0 && ((plc1 & 0xFFFF) == 0))
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


static int write_mac(char *mac1_str, char *mac2_str, char *mac3_str)
{

	int i, ret = 0;
	char *mac1[6] = { 0 };
	char *mac2[6] = { 0 };
	char *mac3[6] = { 0 };
	unsigned m1[6] = { 0 };
	unsigned m2[6] = { 0 };
	unsigned m3[6] = { 0 };

	if (read_mac(1, 0) && (common.force == 0)) {
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
	if (sscanf(mac1_str, "%2s:%2s:%2s:%2s:%2s:%2s", mac1[0], mac1[1], mac1[2], mac1[3], mac1[4], mac1[5]) != 6) {
		printf("Invalid MAC1 address format\n");
		ret = -1;
	}

	if (!ret && sscanf(mac2_str, "%2s:%2s:%2s:%2s:%2s:%2s", mac2[0], mac2[1], mac2[2], mac2[3], mac2[4], mac2[5]) != 6) {
		printf("Invalid MAC2 address format\n");
		ret = -1;
	}

	if (!ret && sscanf(mac3_str, "%2s:%2s:%2s:%2s:%2s:%2s", mac3[0], mac3[1], mac3[2], mac3[3], mac3[4], mac3[5]) != 6) {
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
		ret |= otp_write(OTP_ADDR_PLC_MAC1, (unsigned)(m3[1] | m3[0] << 8) & 0xFFFF);

		otp_reload();
	}

	for (i = 0; i < 6; i++) {
		free(mac1[i]);
		free(mac2[i]);
		free(mac3[i]);
	}

	printf("MAC addrs %s\n", !ret ? "written" : "write failed");
	return ret;
}


static int check_sn(const char *sn, const size_t len, int raw, unsigned *out)
{
	int i, j;
	int ret = 0;

	if ((raw && len != 16) || (!raw && len != 19)) {
		printf("Wrong serial number length!\n");
		return -EINVAL;
	}

	j = 0;
	for (i = 0; i < len; i++) {
		if (raw) {
			if (isdigit(sn[i])) {
				out[j++] = sn[i];
			}
			else {
				ret = -EINVAL;
				break;
			}
		}
		else {
			if (isdigit(sn[i]) && i != 5 && i != 14 && i != 17) {
				out[j++] = sn[i];
			}
			else if (sn[i] == '-' && (i == 5 || i == 14 || i == 17)) {
				continue;
			}
			else {
				ret = -EINVAL;
				break;
			}
		}
	}

	if (ret < 0)
		printf("Wrong serial number format. Expected: %s\n", raw ? SN_FORMAT_RAW : SN_FORMAT);

	return ret;
}


static int read_sn(int silent, int raw)
{
	int res = 1;
	unsigned sn0, sn1, sn2, sn3, lock;
	char sn[20] = { 0 };
	int i;

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

	i = 0;
	sn[i++] = (char)((sn0 >> 24) & 0xFF);
	sn[i++] = (char)((sn0 >> 16) & 0xFF);
	sn[i++] = (char)((sn0 >> 8) & 0xFF);
	sn[i++] = (char)(sn0 & 0xFF);
	sn[i++] = (char)((sn1 >> 24) & 0xFF);

	if (!raw)
		sn[i++] = '-';
	sn[i++] = (char)((sn1 >> 16) & 0xFF);
	sn[i++] = (char)((sn1 >> 8) & 0xFF);
	sn[i++] = (char)(sn1 & 0xFF);
	sn[i++] = (char)((sn2 >> 24) & 0xFF);
	sn[i++] = (char)((sn2 >> 16) & 0xFF);
	sn[i++] = (char)((sn2 >> 8) & 0xFF);
	sn[i++] = (char)(sn2 & 0xFF);
	sn[i++] = (char)((sn3 >> 24) & 0xFF);

	if (!raw)
		sn[i++] = '-';
	sn[i++] = (char)((sn3 >> 16) & 0xFF);
	sn[i++] = (char)((sn3 >> 8) & 0xFF);

	if (!raw)
		sn[i++] = '-';
	sn[i++] = (char)(sn3 & 0xFF);

	if (strlen(sn) == 0)
		sprintf(sn, "%s", raw ? "0000000000000000" : "00000-00000000-00-0");

	if (!silent)
		printf("%s\n", sn);

	return res;
}


static int write_sn(const char *sn, int raw)
{
	int ret = 0, i = 0;
	size_t len;
	unsigned sn0 = 0, sn1 = 0, sn2 = 0, sn3 = 0, lock = 0;
	unsigned buf[16];

	if (read_sn(1, raw) && (common.force == 0)) {
		printf("S/N already written\n");
		return -1;
	}

	if (!sn)
		return -1;

	len = strlen(sn);

	if (check_sn(sn, len, raw, buf))
		return -1;

	sn0 |= (buf[i++] << 24) & (0xFF << 24);
	sn0 |= (buf[i++] << 16) & (0xFF << 16);
	sn0 |= (buf[i++] << 8) & (0xFF << 8);
	sn0 |= buf[i++] & 0xFF;

	sn1 |= (buf[i++] << 24) & (0xFF << 24);
	sn1 |= (buf[i++] << 16) & (0xFF << 16);
	sn1 |= (buf[i++] << 8) & (0xFF << 8);
	sn1 |= buf[i++] & 0xFF;

	sn2 |= (buf[i++] << 24) & (0xFF << 24);
	sn2 |= (buf[i++] << 16) & (0xFF << 16);
	sn2 |= (buf[i++] << 8) & (0xFF << 8);
	sn2 |= buf[i++] & 0xFF;

	sn3 |= (buf[i++] << 24) & (0xFF << 24);
	sn3 |= (buf[i++] << 16) & (0xFF << 16);
	sn3 |= (buf[i++] << 8) & (0xFF << 8);
	sn3 |= buf[i++] & 0xFF;

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
		read_sn(0, raw);

	return ret;
}


static int read_hwrev(int silent)
{
	uint32_t plc1 = *(otp.base + ocotp_gp2);

	/* NOTE: keeping (hw_rev - 1) in FUSE bits */
	uint8_t hw_rev = (plc1 >> 24 & 0xff) + 1;
	if (silent == 0) {
		printf("%u\n", hw_rev);
	}

	return hw_rev;
}


static int write_hwrev(uint8_t hw_rev)
{

	if ((read_hwrev(1) != 1) && (common.force == 0)) {
		printf("HW rev already written\n");
		return -1;
	}

	/* NOTE: no need to read current plc1 word, HW does it automatically */
	/* NOTE: keeping (hw_rev - 1) in FUSE bits */
	uint32_t hwrev32 = (uint32_t)(hw_rev - 1) << 24;

	otp_reload();

	int ret = otp_write(OTP_ADDR_PLC_MAC1, hwrev32);

	otp_reload();

	if (ret) {
		printf("Writing hw rev failed\n");
	}
	else {
		read_hwrev(0);
	}

	return ret;
}


static void usage(const char *progname)
{
	printf("Usage: %s [-b] [-u] [-m MAC1 MAC2 PLC_MAC]\n", progname);
	printf("\t-b                       Blow boot fuses\n");
	printf("\t-u                       Get unique ID\n");
	printf("\t-m MAC1 MAC2 PLC_MAC     write MAC addresses (MAC format XX:XX:XX:XX:XX:XX)\n");
	printf("\t-M [1 - 3]               print MAC address [1 = MAC1, 2 = MAC2, 3 = PLC] or all [no arg]\n");
	printf("\t-r " SN_FORMAT_RAW "      write serial number format raw\n");
	printf("\t-R                       print serial number format raw " SN_FORMAT_RAW "\n");
	printf("\t-s " SN_FORMAT "   write serial number format\n");
	printf("\t-S                       print serial number format " SN_FORMAT "\n");
	printf("\t-e [byte]                write HW revision (0-255)\n");
	printf("\t-E                       read HW revision\n");
	printf("\t-X [addr]                read raw 32bit value from OTP [addr]\n");
	printf("\t-f                       force OTP writing even if already written\n");
	printf("\t-n                       dry run - perform all checks apart from real OTP writing\n");
	printf("\t-h                       this help message\n");
}


int main(int argc, char **argv)
{
	int res;
	int raw = 0;
	char *mac[3] = { 0 };
	char *sn = NULL;
	char *endptr;
	unsigned long hw_rev = 0;

	while ((res = getopt(argc, argv, "r:RSs:buMm:fnX:Ee:h")) >= 0) {
		switch (res) {
		case 'b':
			common.blow_boot = 1;
			break;
		case 'u':
			common.get_uid = 1;
			break;
		case 'm':
			if (argc - (optind - 1) < 3) {
				usage(argv[0]);
				return EXIT_FAILURE;
			}
			/* WARN: using multiple positional arguments with getopt options is implementation-defined */
			mac[0] = argv[optind - 1];
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				usage(argv[0]);
				return EXIT_FAILURE;
			}
			mac[1] = argv[optind];
			optind++;
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				usage(argv[0]);
				return EXIT_FAILURE;
			}
			mac[2] = argv[optind];
			optind++;
			common.rw_mac = OTP_OP_WRITE;
			break;
		case 'M': /* phoenix getopt doesn't support optional arguments, parse by hand */
			if (argv[optind] == NULL || argv[optind][0] == '-') {
				common.r_mac_no = 0;
			}
			else {
				common.r_mac_no = strtol(argv[optind], NULL, 10);
			}

			common.rw_mac = OTP_OP_READ;
			break;
		case 'r':
			sn = optarg;
			raw = 1;
			common.rw_sn = OTP_OP_WRITE;
			break;
		case 'R':
			raw = 1;
			common.rw_sn = OTP_OP_READ;
			break;
		case 's':
			sn = optarg;
			raw = 0;
			common.rw_sn = OTP_OP_WRITE;
			break;
		case 'S':
			raw = 0;
			common.rw_sn = OTP_OP_READ;
			break;
		case 'e':
			common.rw_revision = OTP_OP_WRITE;
			hw_rev = strtoul(optarg, &endptr, 0);
			if ((endptr == optarg) || (*endptr != '\0') || (hw_rev > 255) || (hw_rev == 0)) {
				printf("%s: invalid HW revision value (%s)\n", argv[0], optarg);
				return EXIT_FAILURE;
			}
			break;
		case 'E':
			common.rw_revision = OTP_OP_READ;
			break;
		case 'X':
			common.read_addr = strtoul(optarg, &endptr, 0);
			if ((endptr == optarg) || (*endptr != '\0') || (common.read_addr > 0x4f)) {
				printf("%s: invalid OTP address value (%s)\n", argv[0], optarg);
				return EXIT_FAILURE;
			}
			break;
		case 'f':
			common.force = 1;
			break;
		case 'n':
			common.dry_run = 1;
			break;
		case 'h':
			usage(argv[0]);
			return EXIT_SUCCESS;
		default:
			usage(argv[0]);
			return EXIT_FAILURE;
		}
	}

	if (!common.blow_boot && !common.get_uid && !common.rw_mac && !common.rw_sn && common.rw_revision == 0 && common.read_addr == READ_ADDR_UNDEFINED) {
		printf("Nothing to do\n");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (common.r_mac_no < 0 || common.r_mac_no > 3) {
		printf("Invalid MAC number\n");
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	otp.base = mmap(NULL, 0x1000, PROT_WRITE | PROT_READ, MAP_DEVICE | MAP_PHYSMEM | MAP_ANONYMOUS, -1, OTP_BASE_ADDR);
	if (otp.base == MAP_FAILED) {
		printf("OTP mmap failed\n");
		return EXIT_FAILURE;
	}

	res = 0;
	if (common.rw_sn == OTP_OP_WRITE)
		res |= write_sn(sn, raw);

	if (common.rw_sn == OTP_OP_READ)
		res |= read_sn(0, raw);

	if (common.blow_boot)
		res |= blow_boot_fuses();

	if (common.get_uid)
		res |= get_unique_id();

	if (common.rw_mac == OTP_OP_WRITE)
		res |= write_mac(mac[0], mac[1], mac[2]);

	if (common.rw_mac == OTP_OP_READ)
		res |= read_mac(0, common.r_mac_no);

	if (common.rw_revision == OTP_OP_WRITE) {
		res |= write_hwrev(hw_rev);
	}

	if (common.rw_revision == OTP_OP_READ) {
		read_hwrev(0);
	}

	if (common.read_addr != READ_ADDR_UNDEFINED) {
		uint32_t val = *(otp.base + ocotp_lock + common.read_addr * 4);
		printf("[0x%02x]: 0x%02x%02x%02x%02x\n", common.read_addr, (val >> 24) & 0xff, (val >> 16) & 0xff, (val >> 8) & 0xff, val & 0xff);
	}

	munmap((void *)otp.base, 0x1000);
	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
