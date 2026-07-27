/*
 * Phoenix-RTOS
 *
 * Driver for GPS module ubx
 *
 * Copyright 2022, 2026 Phoenix Systems
 * Author: Damian Loewnau, Mateusz Niewiadomski, Jakub Smolaga
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <sys/time.h>
#include <sys/threads.h>

#include <libsensors/sensor.h>
#include <libsensors/gps/receiver.h>

/* for gga_fix_* constants - fix quality values shared with nmea based drivers */
#include <libsensors/gps/nmea.h>

#include <board_config.h>

#define UBX_STR "ubx:"

/* frame layout */
#define UBX_SYNC1   0xb5U
#define UBX_SYNC2   0x62U
#define UBX_HDRLEN  6U
#define UBX_CHKLEN  2U
#define UBX_SYNCLEN 2U
#define UBX_MINLEN  (UBX_HDRLEN + UBX_CHKLEN)
#define UBX_MAXLEN  512U
#define UBX_BUFSZ   1024U

#define UBX_STKSZ 2048U

/* timing */
#define UBX_NEVER_US     (0x0fffffffffffffffLL)
#define UBX_POLL_US      (50LL * 1000LL)
#define UBX_TOUT_US      (500LL * 1000LL)
#define UBX_SETTLE_US    (100LL * 1000LL)
#define UBX_RETRY_US     (1000LL * 1000LL)
#define UBX_OPEN_RETRIES 10000
#define UBX_MAX_TOUT_US  (2LL * 1000LL * 1000LL)

/* default configuration */
#ifndef UBX_DEFAULT_BAUDRATE
#define UBX_DEFAULT_BAUDRATE 115200
#endif

#define UBX_DEFAULT_RATE_HZ 10U
#define UBX_MAX_RATE_HZ     20U
#define UBX_DYN_KEEP        0xffU /* dynModel argument value: keep module setting */

/* CFG-VALSET keys */
#define UBX_KEY_UART1_OUT_PROT_UBX   0x10740001UL
#define UBX_KEY_UART1_OUT_PROT_NMEA  0x10740002UL
#define UBX_KEY_UART1_BAUDRATE       0x40520001UL
#define UBX_KEY_RATE_MEAS            0x30210001UL
#define UBX_KEY_RATE_NAV             0x30210002UL
#define UBX_KEY_RATE_TIMEREF         0x20210003UL
#define UBX_KEY_MSGOUT_NAV_PVT_UART1 0x20910007UL
#define UBX_KEY_NAVSPG_DYNMODEL      0x20110021UL
#define UBX_KEY_SBAS_ENA             0x10310020UL
#define UBX_KEY_SBAS_L1CA_ENA        0x10310005UL
#define UBX_KEY_SBAS_USE_RANGING     0x10360003UL
#define UBX_KEY_SBAS_USE_DIFFCORR    0x10360004UL
#define UBX_KEY_SBAS_USE_INTEGRITY   0x10360005UL
#define UBX_KEY_SBAS_PRNSCANMASK     0x50360006UL
#define UBX_KEY_GAL_ENABLED          0x10310021UL
#define UBX_KEY_GAL_E1_ENABLED       0x10310007UL
#define UBX_KEY_BDS_ENABLED          0x10310022UL
#define UBX_KEY_BDS_B1_ENABLED       0x1031000DUL

/* SBAS PRN scan masks: bit (N - 120) selects SBAS satellite PRN N */
#define UBX_SBAS_PRNMASK_EU 0x0000000000010049ULL /* PRNs 120, 123, 126, 136 (EGNOS) */
#define UBX_SBAS_PRNMASK_NA 0x000000000004A800ULL /* PRNs 131, 133, 135, 138 (WAAS)  */

/* CFG-GNSS: GNSS identifiers and flags (legacy M8) */
#define UBX_GNSSID_GPS       0
#define UBX_GNSSID_SBAS      1
#define UBX_GNSSID_GALILEO   2
#define UBX_GNSSID_BEIDOU    3
#define UBX_GNSSID_QZSS      5
#define UBX_GNSSID_GLONASS   6
#define UBX_GNSS_FLAG_ENABLE 0x01U
#define UBX_GNSS_HDRLEN      4U
#define UBX_GNSS_BLKLEN      8U
#define UBX_GNSS_MAXBLK      7U

/* fix values used by ubx protocol */

typedef enum {
	ubx_class_nav = 0x01,
	ubx_class_ack = 0x05,
	ubx_class_cfg = 0x06,
	ubx_class_mon = 0x0a,
} ubx_class_t;

/* message ids */
typedef enum {
	ubx_id_posllh = 0x02,  /* ubx_class_nav */
	ubx_id_dop = 0x04,     /* ubx_class_nav */
	ubx_id_sol = 0x06,     /* ubx_class_nav */
	ubx_id_pvt = 0x07,     /* ubx_class_nav */
	ubx_id_velned = 0x12,  /* ubx_class_nav */
	ubx_id_timeutc = 0x21, /* ubx_class_nav */
	ubx_id_nak = 0x00,     /* ubx_class_ack */
	ubx_id_ack = 0x01,     /* ubx_class_ack */
	ubx_id_prt = 0x00,     /* ubx_class_cfg */
	ubx_id_msg = 0x01,     /* ubx_class_cfg */
	ubx_id_rate = 0x08,    /* ubx_class_cfg */
	ubx_id_nav5 = 0x24,    /* ubx_class_cfg */
	ubx_id_sbas = 0x16,    /* ubx_class_cfg */
	ubx_id_gnss = 0x3e,    /* ubx_class_cfg */
	ubx_id_valset = 0x8a,  /* ubx_class_cfg */
	ubx_id_ver = 0x04,     /* ubx_class_mon */
} ubx_id_t;

typedef enum {
	ubx_fix_none = 0x00,   /* no fix */
	ubx_fix_dr = 0x01,     /* dead reckoning only */
	ubx_fix_2d = 0x02,     /* 2d fix */
	ubx_fix_3d = 0x03,     /* 3d fix */
	ubx_fix_gnssDr = 0x04, /* gnss + dead reckoning combined */
	ubx_fix_time = 0x05,   /* time only fix */
} ubx_fix_t;

typedef enum {
	ubx_gen_unknown = 0,
	ubx_gen_m5,
	ubx_gen_m6,
	ubx_gen_m7,
	ubx_gen_m8,
	ubx_gen_m9,
	ubx_gen_m10
} ubx_gen_t;

typedef enum {
	ubx_sbas_off = 0,
	ubx_sbas_eu,
	ubx_sbas_na,
} ubx_sbas_t;

typedef enum {
	ubx_constellationMode_keep = 0,
	ubx_constellationMode_on,
	ubx_constellationMode_off,
} ubx_constellationMode_t;

typedef struct {
	uint8_t buf[UBX_BUFSZ];
	size_t wrIdx;
	size_t rdIdx;
} ubx_parser_t;

typedef struct {
	uint8_t buf[UBX_BUFSZ];
	size_t pos;
} ubx_builder_t;

typedef struct {
	uint8_t *pld;
	uint16_t len;
	ubx_id_t id;
	ubx_class_t class;
} ubx_msg_t;

/* runtime options, set through driver arguments */
typedef struct {
	uint16_t rateMs;  /* measurement period [ms] */
	uint8_t dynModel; /* dynamic platform model, UBX_DYN_KEEP = do not change */
	ubx_sbas_t sbas;
	ubx_constellationMode_t galileo;
	ubx_constellationMode_t beidou;
} ubx_opts_t;

typedef struct {
	sensor_event_t ev;
	int fd;
	char stack[UBX_STKSZ] __attribute__((aligned(8)));
	handle_t tid;
	ubx_parser_t p;
	ubx_builder_t b;
	ubx_msg_t msg;
	ubx_gen_t gen;
	ubx_opts_t opts;

	/* information for grouping incoming messages based on TOW */
	uint32_t batchTOW;        /* time-of-week for current batch */
	uint8_t batchMask;        /* bitmask of messages received in the current batch */
	uint8_t batchRequirement; /* bitmask of messages required for a full batch */

	char path[64];

	volatile bool run;
} ubx_ctx_t;


/*
 * utilities
 */


static bool ubx_isDeadlineDue(time_t deadlineUs)
{
	time_t nowUs = 0;
	gettime(&nowUs, NULL);
	return nowUs >= deadlineUs;
}


static time_t ubx_getDeadline(time_t toutUs)
{
	time_t nowUs = 0;
	gettime(&nowUs, NULL);
	return nowUs + toutUs;
}


/* calculate checksum using the Fletcher algorithm */
static void ubx_chk(const uint8_t *buf, size_t bufsz, uint8_t *a, uint8_t *b)
{
	*a = 0;
	*b = 0;
	for (size_t i = 0; i < bufsz; i++) {
		*a += buf[i];
		*b += *a;
	}
}


/* deg * 1e-5 to mrad */
static int32_t ubx_degE5ToMRad(int32_t deg)
{
	static const int64_t scaler = (int64_t)((1e9 / 360.0) * 2.0 * M_PI);
	static const int64_t divider = (int64_t)1e11;

	return (int32_t)(((int64_t)deg * scaler) / divider);
}


/* deg * 1e-2 to mrad */
static int32_t ubx_degE2ToMRad(int32_t deg)
{
	static const int64_t scaler = (int64_t)((1e9 / 360.0) * 2.0 * M_PI);
	static const int64_t divider = (int64_t)1e8;

	return (int32_t)(((int64_t)deg * scaler) / divider);
}


/* convert ubx fix to the fix quality values shared with nmea based drivers */
static uint8_t ubx_fixToGgaFix(uint8_t fix)
{
	uint8_t ret;

	switch (fix) {
		case ubx_fix_2d:
		case ubx_fix_3d:
			ret = gga_fix_gnss;
			break;

		case ubx_fix_dr:
		case ubx_fix_gnssDr:
			ret = gga_fix_estimated;
			break;

		default:
			ret = gga_fix_invalid;
			break;
	}

	return ret;
}


/*
 * parsing
 */


static uint8_t ubx_readU8(uint8_t *p)
{
	return *p;
}


static uint16_t ubx_readU16(uint8_t *p)
{
	uint8_t lo = ubx_readU8(p);
	uint8_t hi = ubx_readU8(p + 1);

	return (uint16_t)(((uint16_t)lo) | ((uint16_t)((uint16_t)hi << 8U)));
}


static uint32_t ubx_readU32(uint8_t *p)
{
	uint16_t lo = ubx_readU16(p);
	uint16_t hi = ubx_readU16(p + 2);

	return ((uint32_t)lo) | (((uint32_t)hi) << 16U);
}


static int16_t ubx_readI16(uint8_t *p)
{
	uint16_t val = ubx_readU16(p);
	int16_t ret;
	memcpy(&ret, &val, sizeof(ret));
	return ret;
}


static int32_t ubx_readI32(uint8_t *p)
{
	uint32_t val = ubx_readU32(p);
	int32_t ret;
	memcpy(&ret, &val, sizeof(ret));
	return ret;
}


/*
 * tries to find a message
 * if message was not found - returns false
 * if message was found - returns true and writes data to msg
 */
static bool ubx_parserRun(ubx_parser_t *p, ubx_msg_t *msg)
{
	bool found = false;

	while (!found) {
		/* don't try parsing if there is no chance for a full frame */
		size_t avail = p->wrIdx - p->rdIdx;
		if (avail < UBX_MINLEN) {
			break;
		}

		/* extract header */
		uint8_t sync1 = ubx_readU8(p->buf + p->rdIdx + 0);
		uint8_t sync2 = ubx_readU8(p->buf + p->rdIdx + 1);

		/* check sync bytes */
		if ((sync1 != UBX_SYNC1) || (sync2 != UBX_SYNC2)) {
			p->rdIdx++;
			continue;
		}

		uint16_t len = ubx_readU16(p->buf + p->rdIdx + 4);

		/* reject messages which are too long to parse */
		if (len > (UBX_MAXLEN - UBX_HDRLEN - UBX_CHKLEN)) {
			p->rdIdx++;
			continue;
		}

		/* wait for more data if the frame is incomplete */
		if ((size_t)len > (avail - UBX_HDRLEN - UBX_CHKLEN)) {
			break;
		}

		/* verify checksum */
		uint8_t *chkBeg = p->buf + p->rdIdx + UBX_SYNCLEN;
		uint8_t *chkEnd = p->buf + p->rdIdx + UBX_HDRLEN + len;
		uint8_t chkA = 0;
		uint8_t chkB = 0;
		ubx_chk(chkBeg, (size_t)(chkEnd - chkBeg), &chkA, &chkB);
		if ((chkA != chkEnd[0]) || (chkB != chkEnd[1])) {
			p->rdIdx++;
			continue;
		}

		/* copy data to the output message */
		msg->pld = p->buf + p->rdIdx + UBX_HDRLEN;
		msg->len = len;
		msg->class = ubx_readU8(p->buf + p->rdIdx + 2);
		msg->id = ubx_readU8(p->buf + p->rdIdx + 3);
		p->rdIdx += UBX_HDRLEN + len + UBX_CHKLEN;

		found = true;
	}

	return found;
}


/* read more data into the parser buffer */
static ssize_t ubx_parserRead(ubx_ctx_t *ctx, time_t deadlineUs)
{
	ssize_t ret = -1;

	while (ctx->run) {
		uint8_t *buf = ctx->p.buf + ctx->p.wrIdx;
		size_t bufsz = sizeof(ctx->p.buf) - ctx->p.wrIdx;

		ret = read(ctx->fd, buf, bufsz);
		if (ret > 0) {
			ctx->p.wrIdx += (size_t)ret;
			break;
		}
		else if (ret == 0) {
			ret = -1;
			break;
		}
		else {
			int err = errno;
			if (err != EAGAIN && err != EWOULDBLOCK && err != EINTR) {
				break;
			}
		}

		if (ubx_isDeadlineDue(deadlineUs)) {
			ret = -1;
			break;
		}

		usleep(UBX_POLL_US);
	}

	return ret;
}


static void ubx_parserCompact(ubx_parser_t *p)
{
	/* move read/write heads to the front if there is no data left */
	if (p->wrIdx <= p->rdIdx) {
		p->wrIdx = 0;
		p->rdIdx = 0;
	}

	/* reset the parser if a single message has filled the entire buffer */
	if ((p->rdIdx == 0) && (p->wrIdx >= UBX_BUFSZ)) {
		p->wrIdx = 0;
		p->rdIdx = 0;
	}

	/* compact the buffer if it's too far */
	if ((p->rdIdx > 0) && ((p->wrIdx + UBX_MAXLEN) > UBX_BUFSZ)) {
		size_t leftover = p->wrIdx - p->rdIdx;
		memmove(p->buf, p->buf + p->rdIdx, leftover);
		p->wrIdx = leftover;
		p->rdIdx = 0;
	}
}


static void ubx_parserReset(ubx_parser_t *p)
{
	p->wrIdx = 0;
	p->rdIdx = 0;
}


static int ubx_msgGet(ubx_ctx_t *ctx, time_t deadlineUs)
{
	int ret = -1;

	while (ctx->run) {
		/* try to parse what is already in the buffer */
		if (ubx_parserRun(&ctx->p, &ctx->msg)) {
			ret = 0;
			break;
		}

		/* make sure we have enough space for the read */
		ubx_parserCompact(&ctx->p);

		/* try to read more data */
		if (ubx_parserRead(ctx, deadlineUs) < 0) {
			break;
		}
	}

	return ret;
}


static int ubx_msgGetByClass(ubx_ctx_t *ctx, ubx_class_t cls, time_t deadlineUs)
{
	int ret = -1;

	while (ctx->run) {
		if (ubx_msgGet(ctx, deadlineUs) < 0) {
			break;
		}
		if (ctx->msg.class == cls) {
			ret = 0;
			break;
		}
	}

	return ret;
}


static int ubx_msgGetExact(ubx_ctx_t *ctx, ubx_class_t cls, ubx_id_t id, time_t deadlineUs)
{
	int ret = -1;

	while (ctx->run) {
		if (ubx_msgGetByClass(ctx, cls, deadlineUs) < 0) {
			break;
		}
		if (ctx->msg.id == id) {
			ret = 0;
			break;
		}
	}

	return ret;
}


/* wait for ack for the (cls, id) command, fail on nak */
static int ubx_msgGetAck(ubx_ctx_t *ctx, ubx_class_t cls, ubx_id_t id, time_t deadlineUs)
{
	int ret = -1;

	while (ctx->run) {
		if (ubx_msgGetByClass(ctx, ubx_class_ack, deadlineUs) < 0) {
			break;
		}
		if (ctx->msg.len < 2U) {
			continue;
		}
		if ((ctx->msg.pld[0] != cls) || (ctx->msg.pld[1] != id)) {
			/* ack for a different command */
			continue;
		}
		ret = (ctx->msg.id == ubx_id_ack) ? 0 : -1;
		break;
	}

	return ret;
}


/*
 * message builder
 */


static void ubx_pushU8(ubx_ctx_t *ctx, uint8_t data)
{
	/* always reserve space for the header */
	ctx->b.pos = (ctx->b.pos < UBX_HDRLEN) ? UBX_HDRLEN : ctx->b.pos;

	/* check if data will fit */
	if ((ctx->b.pos + 1) >= UBX_BUFSZ) {
		return;
	}

	ctx->b.buf[ctx->b.pos] = data;
	ctx->b.pos++;
}


static void ubx_pushU16(ubx_ctx_t *ctx, uint16_t data)
{
	ubx_pushU8(ctx, (uint8_t)(data & 0xffU));
	ubx_pushU8(ctx, (uint8_t)((data >> 8U) & 0xffU));
}


static void ubx_pushU32(ubx_ctx_t *ctx, uint32_t data)
{
	ubx_pushU16(ctx, (uint16_t)(data & 0xffffUL));
	ubx_pushU16(ctx, (uint16_t)((data >> 16U) & 0xffffUL));
}


static int ubx_send(ubx_ctx_t *ctx, ubx_class_t cls, ubx_id_t id)
{
	/* always reserve space for the header */
	ctx->b.pos = (ctx->b.pos < UBX_HDRLEN) ? UBX_HDRLEN : ctx->b.pos;

	/* check if the checksum will fit */
	if ((ctx->b.pos + UBX_CHKLEN) >= UBX_BUFSZ) {
		ctx->b.pos = UBX_HDRLEN;
		return -1;
	}

	/* construct the header */
	size_t pldLen = ctx->b.pos - UBX_HDRLEN;
	ctx->b.buf[0] = UBX_SYNC1;
	ctx->b.buf[1] = UBX_SYNC2;
	ctx->b.buf[2] = cls;
	ctx->b.buf[3] = id;
	ctx->b.buf[4] = (uint8_t)(pldLen & 0xffU);
	ctx->b.buf[5] = (uint8_t)((pldLen >> 8U) & 0xffU);

	/* calculate checksum */
	uint8_t *chkA = ctx->b.buf + ctx->b.pos;
	uint8_t *chkB = ctx->b.buf + ctx->b.pos + 1U;
	ubx_chk(ctx->b.buf + 2, ctx->b.pos - 2U, chkA, chkB);

	/* calculate total amount of data to be sent, reset the builder */
	size_t total = ctx->b.pos + UBX_CHKLEN;
	ctx->b.pos = UBX_HDRLEN;

	/* write the bytes */
	size_t sent = 0;
	while ((sent < total) && ctx->run) {
		ssize_t n = write(ctx->fd, ctx->b.buf + sent, total - sent);
		if (n > 0) {
			sent += (size_t)n;
		}
		else if (n == 0) {
			break;
		}
		else {
			int err = errno;
			if (err == EAGAIN || err == EWOULDBLOCK || err == EINTR) {
				usleep(1000);
			}
			else {
				break;
			}
		}
	}

	if (sent < total) {
		return -1;
	}

	return (int)sent;
}


static int ubx_sendAndAck(ubx_ctx_t *ctx, ubx_class_t cls, ubx_id_t id)
{
	int ret = ubx_send(ctx, cls, id);

	if (ret < 0) {
		return ret;
	}

	return ubx_msgGetAck(ctx, cls, id, ubx_getDeadline(UBX_TOUT_US));
}


/*
 * configuration commands
 */


/* CFG-PRT: configure uart1 protocol and baudrate */
static int ubx_sendCfgPrt(ubx_ctx_t *ctx, uint32_t baud)
{
	ubx_pushU8(ctx, 1U);        /* port id = uart1 */
	ubx_pushU8(ctx, 0U);        /* reserved */
	ubx_pushU16(ctx, 0U);       /* tx ready flags */
	ubx_pushU32(ctx, 0x08d0UL); /* mode = 8N1 */
	ubx_pushU32(ctx, baud);     /* baudrate */
	ubx_pushU16(ctx, 0x1U);     /* input protocol = ubx */
	ubx_pushU16(ctx, 0x1U);     /* output protocol = ubx (no nmea) */
	ubx_pushU16(ctx, 0U);       /* extra flags */
	ubx_pushU16(ctx, 0U);       /* reserved */

	return ubx_send(ctx, ubx_class_cfg, ubx_id_prt);
}


/* CFG-RATE: set navigation solution rate */
static int ubx_sendCfgRate(ubx_ctx_t *ctx, uint16_t measRate)
{
	ubx_pushU16(ctx, measRate); /* measurement rate [ms] */
	ubx_pushU16(ctx, 1U);       /* one measurement per solution */
	ubx_pushU16(ctx, 1U);       /* time reference = gps time */

	return ubx_sendAndAck(ctx, ubx_class_cfg, ubx_id_rate);
}


/* CFG-MSG: set message output rate (every x navigation solutions) */
static int ubx_sendCfgMsg(ubx_ctx_t *ctx, ubx_class_t cls, ubx_id_t id, uint8_t rate)
{
	ubx_pushU8(ctx, cls);
	ubx_pushU8(ctx, id);
	ubx_pushU8(ctx, rate);

	return ubx_sendAndAck(ctx, ubx_class_cfg, ubx_id_msg);
}


/* CFG-NAV5: set dynamic platform model, leave everything else untouched */
static int ubx_sendCfgNav5(ubx_ctx_t *ctx, uint8_t dynModel)
{
	ubx_pushU16(ctx, 0x0001U); /* mask: apply dynModel only */
	ubx_pushU8(ctx, dynModel);
	ubx_pushU8(ctx, 0U);  /* fixMode */
	ubx_pushU32(ctx, 0U); /* fixedAlt */
	ubx_pushU32(ctx, 0U); /* fixedAltVar */
	ubx_pushU8(ctx, 0U);  /* minElev */
	ubx_pushU8(ctx, 0U);  /* drLimit */
	ubx_pushU16(ctx, 0U); /* pDop */
	ubx_pushU16(ctx, 0U); /* tDop */
	ubx_pushU16(ctx, 0U); /* pAcc */
	ubx_pushU16(ctx, 0U); /* tAcc */
	ubx_pushU8(ctx, 0U);  /* staticHoldThresh */
	ubx_pushU8(ctx, 0U);  /* dgnssTimeout */
	ubx_pushU8(ctx, 0U);  /* cnoThreshNumSV */
	ubx_pushU8(ctx, 0U);  /* cnoThresh */
	ubx_pushU16(ctx, 0U); /* reserved */
	ubx_pushU16(ctx, 0U); /* staticHoldMaxDist */
	ubx_pushU8(ctx, 0U);  /* utcStandard */
	ubx_pushU32(ctx, 0U); /* reserved */
	ubx_pushU8(ctx, 0U);  /* reserved */

	return ubx_sendAndAck(ctx, ubx_class_cfg, ubx_id_nav5);
}


/* CFG-SBAS: enable SBAS augmentation with region-specific scan mask */
static int ubx_sendCfgSbas(ubx_ctx_t *ctx, ubx_sbas_t sbas)
{
	uint64_t prnMask = 0ULL;

	if (sbas == ubx_sbas_eu) {
		prnMask = UBX_SBAS_PRNMASK_EU;
	}
	else if (sbas == ubx_sbas_na) {
		prnMask = UBX_SBAS_PRNMASK_NA;
	}

	ubx_pushU8(ctx, 1U);                        /* mode: bit0 = enabled */
	ubx_pushU8(ctx, 0x07U);                     /* usage: ranging | diffcorr | integrity */
	ubx_pushU8(ctx, 3U);                        /* maxSBAS: 3 channels */
	ubx_pushU8(ctx, (uint8_t)(prnMask >> 32U)); /* scanmode2: PRN 152-158 */
	ubx_pushU32(ctx, (uint32_t)prnMask);        /* scanmode1: PRN 120-151 */

	return ubx_sendAndAck(ctx, ubx_class_cfg, ubx_id_sbas);
}


/* CFG-GNSS: enable Galileo/BeiDou constellations (legacy M8) */
static int ubx_sendCfgGnss(ubx_ctx_t *ctx)
{
	time_t deadline;
	uint8_t numTrkChHw;
	uint8_t numTrkChUse;
	uint8_t blockCount = 0;
	uint8_t newBlockCount = 0;
	bool hasGalileo = false;
	bool hasBeidou = false;
	int ret;

	ret = ubx_send(ctx, ubx_class_cfg, ubx_id_gnss);
	if (ret < 0) {
		return ret;
	}

	deadline = ubx_getDeadline(UBX_TOUT_US);
	ret = ubx_msgGetExact(ctx, ubx_class_cfg, ubx_id_gnss, deadline);
	if (ret < 0) {
		return ret;
	}

	if (ctx->msg.len < UBX_GNSS_HDRLEN) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	numTrkChHw = ubx_readU8(p + 1);
	numTrkChUse = ubx_readU8(p + 2);
	blockCount = ubx_readU8(p + 3);

	if (blockCount > UBX_GNSS_MAXBLK) {
		return -1;
	}

	if (ctx->msg.len < (size_t)(UBX_GNSS_HDRLEN + (size_t)blockCount * UBX_GNSS_BLKLEN)) {
		return -1;
	}

	for (size_t i = 0; i < blockCount; i++) {
		uint8_t gnssId = ubx_readU8(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN);
		if (gnssId == UBX_GNSSID_GALILEO) {
			hasGalileo = true;
		}
		else if (gnssId == UBX_GNSSID_BEIDOU) {
			hasBeidou = true;
		}
	}

	newBlockCount = blockCount;
	if (ctx->opts.galileo != ubx_constellationMode_keep && !hasGalileo) {
		newBlockCount++;
	}
	if (ctx->opts.beidou != ubx_constellationMode_keep && !hasBeidou) {
		newBlockCount++;
	}

	ubx_pushU8(ctx, 0); /* version */
	ubx_pushU8(ctx, numTrkChHw);
	ubx_pushU8(ctx, numTrkChUse);
	ubx_pushU8(ctx, newBlockCount);

	for (size_t i = 0; i < blockCount; i++) {
		uint8_t gnssId = ubx_readU8(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN);
		uint8_t resTrkCh = ubx_readU8(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN + 1);
		uint8_t maxTrkCh = ubx_readU8(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN + 2);
		uint8_t reserved1 = ubx_readU8(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN + 3);
		uint32_t flags = ubx_readU32(p + UBX_GNSS_HDRLEN + i * UBX_GNSS_BLKLEN + 4);

		if (gnssId == UBX_GNSSID_GALILEO) {
			if (ctx->opts.galileo == ubx_constellationMode_on) {
				flags |= UBX_GNSS_FLAG_ENABLE;
			}
			else if (ctx->opts.galileo == ubx_constellationMode_off) {
				flags &= ~UBX_GNSS_FLAG_ENABLE;
			}
		}
		else if (gnssId == UBX_GNSSID_BEIDOU) {
			if (ctx->opts.beidou == ubx_constellationMode_on) {
				flags |= UBX_GNSS_FLAG_ENABLE;
			}
			else if (ctx->opts.beidou == ubx_constellationMode_off) {
				flags &= ~UBX_GNSS_FLAG_ENABLE;
			}
		}

		ubx_pushU8(ctx, gnssId);
		ubx_pushU8(ctx, resTrkCh);
		ubx_pushU8(ctx, maxTrkCh);
		ubx_pushU8(ctx, reserved1);
		ubx_pushU32(ctx, flags);
	}

	if (ctx->opts.galileo != ubx_constellationMode_keep && !hasGalileo) {
		ubx_pushU8(ctx, UBX_GNSSID_GALILEO);
		ubx_pushU8(ctx, 1);
		ubx_pushU8(ctx, 4);
		ubx_pushU8(ctx, 0);
		if (ctx->opts.galileo == ubx_constellationMode_on) {
			ubx_pushU32(ctx, UBX_GNSS_FLAG_ENABLE | (0x01U << 16U));
		}
		else {
			ubx_pushU32(ctx, 0U);
		}
	}
	if (ctx->opts.beidou != ubx_constellationMode_keep && !hasBeidou) {
		ubx_pushU8(ctx, UBX_GNSSID_BEIDOU);
		ubx_pushU8(ctx, 1);
		ubx_pushU8(ctx, 4);
		ubx_pushU8(ctx, 0);
		if (ctx->opts.beidou == ubx_constellationMode_on) {
			ubx_pushU32(ctx, UBX_GNSS_FLAG_ENABLE | (0x01U << 16U));
		}
		else {
			ubx_pushU32(ctx, 0U);
		}
	}

	return ubx_sendAndAck(ctx, ubx_class_cfg, ubx_id_gnss);
}


static void ubx_pushValSetItem(ubx_ctx_t *ctx, uint32_t key, uint32_t val)
{
	uint8_t sizeBits = (uint8_t)((key >> 28U) & 0x7U);

	ubx_pushU32(ctx, key);
	if (sizeBits == 0x04U) {
		ubx_pushU32(ctx, val);
	}
	else if (sizeBits == 0x03U) {
		ubx_pushU16(ctx, (uint16_t)val);
	}
	else {
		/* sizeBits 0x01, 0x02: small values */
		ubx_pushU8(ctx, (uint8_t)val);
	}
}


static void ubx_pushValSetItemU64(ubx_ctx_t *ctx, uint32_t key, uint64_t val)
{
	ubx_pushU32(ctx, key);
	ubx_pushU32(ctx, (uint32_t)(val & 0xffffffffULL));
	ubx_pushU32(ctx, (uint32_t)((val >> 32ULL) & 0xffffffffULL));
}


/*
 * configuration workflows
 */


/* configuration for M9/M10 devices (CFG-VALSET interface) */
static int ubx_configureM9M10(ubx_ctx_t *ctx, uint32_t baud)
{
	int ret;

	ubx_pushU8(ctx, 0x00U); /* message version */
	ubx_pushU8(ctx, 0x01U); /* ram layer */
	ubx_pushU16(ctx, 0x0000U);
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_OUT_PROT_UBX, 1UL);
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_OUT_PROT_NMEA, 0UL);
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_MEAS, ctx->opts.rateMs);
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_NAV, 1UL);
	ubx_pushValSetItem(ctx, UBX_KEY_RATE_TIMEREF, 1UL);
	ubx_pushValSetItem(ctx, UBX_KEY_MSGOUT_NAV_PVT_UART1, 1UL);
	if (ctx->opts.dynModel != UBX_DYN_KEEP) {
		ubx_pushValSetItem(ctx, UBX_KEY_NAVSPG_DYNMODEL, ctx->opts.dynModel);
	}
	if (ctx->opts.sbas != ubx_sbas_off) {
		uint64_t prnMask = 0ULL;
		if (ctx->opts.sbas == ubx_sbas_eu) {
			prnMask = UBX_SBAS_PRNMASK_EU;
		}
		else if (ctx->opts.sbas == ubx_sbas_na) {
			prnMask = UBX_SBAS_PRNMASK_NA;
		}
		ubx_pushValSetItem(ctx, UBX_KEY_SBAS_ENA, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_SBAS_L1CA_ENA, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_SBAS_USE_RANGING, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_SBAS_USE_DIFFCORR, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_SBAS_USE_INTEGRITY, 1UL);
		ubx_pushValSetItemU64(ctx, UBX_KEY_SBAS_PRNSCANMASK, prnMask);
	}

	/* galileo */
	if (ctx->opts.galileo == ubx_constellationMode_on) {
		ubx_pushValSetItem(ctx, UBX_KEY_GAL_ENABLED, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_GAL_E1_ENABLED, 1UL);
	}
	else if (ctx->opts.galileo == ubx_constellationMode_off) {
		ubx_pushValSetItem(ctx, UBX_KEY_GAL_ENABLED, 0UL);
		ubx_pushValSetItem(ctx, UBX_KEY_GAL_E1_ENABLED, 0UL);
	}

	/* beidou */
	if (ctx->opts.beidou == ubx_constellationMode_on) {
		ubx_pushValSetItem(ctx, UBX_KEY_BDS_ENABLED, 1UL);
		ubx_pushValSetItem(ctx, UBX_KEY_BDS_B1_ENABLED, 1UL);
	}
	else if (ctx->opts.beidou == ubx_constellationMode_off) {
		ubx_pushValSetItem(ctx, UBX_KEY_BDS_ENABLED, 0UL);
		ubx_pushValSetItem(ctx, UBX_KEY_BDS_B1_ENABLED, 0UL);
	}

	/* baudrate is applied last, the module switches after acking */
	ubx_pushValSetItem(ctx, UBX_KEY_UART1_BAUDRATE, baud);

	ret = ubx_send(ctx, ubx_class_cfg, ubx_id_valset);
	if (ret < 0) {
		return ret;
	}

	ret = ubx_msgGetAck(ctx, ubx_class_cfg, ubx_id_valset, ubx_getDeadline(UBX_TOUT_US));
	if (ret < 0) {
		return ret;
	}

	/* the device applies the new baudrate after acking - switch the host side */
	ret = gps_serialSetup(ctx->fd, baud, NULL);
	if (ret < 0) {
		return ret;
	}

	ubx_parserReset(&ctx->p);

	return 0;
}


/* configuration for M7/M8 devices (legacy CFG commands, NAV-PVT output) */
static int ubx_configureM7M8(ubx_ctx_t *ctx, uint32_t baud)
{
	int ret = ubx_sendCfgPrt(ctx, baud);

	if (ret < 0) {
		return ret;
	}

	/* wait for the device to switch baudrate, then follow */
	usleep(UBX_SETTLE_US);
	ret = gps_serialSetup(ctx->fd, baud, NULL);
	if (ret < 0) {
		return ret;
	}
	ubx_parserReset(&ctx->p);

	ret = ubx_sendCfgRate(ctx, ctx->opts.rateMs);
	if (ret < 0) {
		return ret;
	}

	/* enable nav-pvt messages on uart1 (factory default outputs nmea only) */
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_pvt, 1U);
	if (ret < 0) {
		return ret;
	}

	if (ctx->opts.dynModel != UBX_DYN_KEEP) {
		ret = ubx_sendCfgNav5(ctx, ctx->opts.dynModel);
	}
	if (ret == 0 && ctx->opts.sbas != ubx_sbas_off) {
		ret = ubx_sendCfgSbas(ctx, ctx->opts.sbas);
	}
	if (ret == 0 && (ctx->opts.galileo != ubx_constellationMode_keep || ctx->opts.beidou != ubx_constellationMode_keep)) {
		ret = ubx_sendCfgGnss(ctx);
	}

	return ret;
}


/* configuration for pre NAV-PVT devices, M5/M6 (legacy CFG + NAV messages) */
static int ubx_configurePrePvt(ubx_ctx_t *ctx, uint32_t baud)
{
	int ret = ubx_sendCfgPrt(ctx, baud);

	if (ret < 0) {
		return ret;
	}

	/* wait for the device to switch baudrate, then follow */
	usleep(UBX_SETTLE_US);
	ret = gps_serialSetup(ctx->fd, baud, NULL);
	if (ret < 0) {
		return ret;
	}
	ubx_parserReset(&ctx->p);

	ret = ubx_sendCfgRate(ctx, ctx->opts.rateMs);
	if (ret < 0) {
		return ret;
	}

	/* enable the set of messages which replaces nav-pvt */
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_posllh, 1U);
	if (ret < 0) {
		return ret;
	}
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_velned, 1U);
	if (ret < 0) {
		return ret;
	}
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_dop, 1U);
	if (ret < 0) {
		return ret;
	}
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_sol, 1U);
	if (ret < 0) {
		return ret;
	}
	ret = ubx_sendCfgMsg(ctx, ubx_class_nav, ubx_id_timeutc, 1U);
	if (ret < 0) {
		return ret;
	}

	if (ctx->opts.dynModel != UBX_DYN_KEEP) {
		ret = ubx_sendCfgNav5(ctx, ctx->opts.dynModel);
	}

	return ret;
}


static int ubx_configure(ubx_ctx_t *ctx, uint32_t baud)
{
	int ret;

	/* trim the settings based on generation */
	if (ctx->gen < ubx_gen_m8 && ctx->opts.galileo != ubx_constellationMode_keep) {
		fprintf(stderr, "%s Galileo support requires M8+ - ignoring configuration", UBX_STR);
		ctx->opts.galileo = ubx_constellationMode_keep;
	}
	if (ctx->gen < ubx_gen_m7 && ctx->opts.beidou != ubx_constellationMode_keep) {
		fprintf(stderr, "%s BeiDou support requires M7+ - ignoring configuration", UBX_STR);
		ctx->opts.beidou = ubx_constellationMode_keep;
	}

	switch (ctx->gen) {
		case ubx_gen_m10:
		case ubx_gen_m9:
			ret = ubx_configureM9M10(ctx, baud);
			break;

		case ubx_gen_m8:
		case ubx_gen_m7:
			ret = ubx_configureM7M8(ctx, baud);
			break;

		default:
			ret = ubx_configurePrePvt(ctx, baud);
			break;
	}

	return ret;
}


/*
 * module generation detection
 */


/* parse hw version string from MON-VER payload into module generation */
static ubx_gen_t ubx_genFromHwStr(const char *str)
{
	static const struct {
		const char *str;
		ubx_gen_t gen;
	} hwVers[] = {
		{ "00040005", ubx_gen_m5 },
		{ "00040007", ubx_gen_m6 },
		{ "00070000", ubx_gen_m7 },
		{ "00080000", ubx_gen_m8 },
		{ "00190000", ubx_gen_m9 },
		{ "000A0000", ubx_gen_m10 },
	};
	ubx_gen_t gen = ubx_gen_unknown;

	for (size_t i = 0; i < (sizeof(hwVers) / sizeof(hwVers[0])); i++) {
		/* hwVersion is a fixed 8-byte field, not necessarily NUL-terminated */
		if (strncmp(str, hwVers[i].str, 8U) == 0) {
			gen = hwVers[i].gen;
			break;
		}
	}

	return gen;
}


static const char *ubx_genToStr(ubx_gen_t gen)
{
	static const char *const genStr[] = {
		[ubx_gen_unknown] = "unknown",
		[ubx_gen_m5] = "M5",
		[ubx_gen_m6] = "M6",
		[ubx_gen_m7] = "M7",
		[ubx_gen_m8] = "M8",
		[ubx_gen_m9] = "M9",
		[ubx_gen_m10] = "M10",
	};
	const char *str = "unknown";

	if (gen < (sizeof(genStr) / sizeof(genStr[0]))) {
		str = genStr[gen];
	}

	return str;
}


/*
 * scan baudrates and find the module:
 * returns 0 and fills ctx->gen when a u-blox module answered the MON-VER
 * poll on one of the baudrates
 */
static int ubx_detect(ubx_ctx_t *ctx)
{
	/* list of baudrate options from most common to least common */
	static const unsigned baudRates[] = {
		38400,  /* default configuration on modern devices */
		9600,   /* default configuration on older devices */
		115200, /* default on some high-end devices */
		57600,  /* custom */
		19200,  /* custom */
		230400, /* custom */
		4800,   /* ancient nmea standard, too slow for most devices */
	};
	int ret = -1;

	for (size_t i = 0; i < (sizeof(baudRates) / sizeof(baudRates[0])); i++) {
		unsigned baud = baudRates[i];

		if (gps_serialSetup(ctx->fd, baud, NULL) < 0) {
			continue;
		}
		ubx_parserReset(&ctx->p);

		/* send version request */
		if (ubx_send(ctx, ubx_class_mon, ubx_id_ver) < 0) {
			continue;
		}

		/* wait for response */
		if (ubx_msgGetExact(ctx, ubx_class_mon, ubx_id_ver, ubx_getDeadline(UBX_TOUT_US)) < 0) {
			continue;
		}

		if (ctx->msg.len < 40U) {
			continue;
		}

		/* hw version string is at offset 30 of the payload */
		ctx->gen = ubx_genFromHwStr((const char *)(ctx->msg.pld + 30U));

		printf("%s detected u-blox %s at %u baud\n", UBX_STR, ubx_genToStr(ctx->gen), baud);

		ret = 0;
		break;
	}

	return ret;
}


static uint8_t ubx_msgGetBatchBit(ubx_id_t id)
{
	switch (id) {
		case ubx_id_posllh: return 1U << 0U;
		case ubx_id_velned: return 1U << 1U;
		case ubx_id_dop: return 1U << 2U;
		case ubx_id_sol: return 1U << 3U;
		case ubx_id_timeutc: return 1U << 4U;
		case ubx_id_pvt: return 1U << 5U;
		default: return 0U;
	}
}


static uint8_t ubx_getRequiredBatchMask(ubx_gen_t gen)
{
	switch (gen) {
		/* for M7+ devices, a single NAV-PVT message contains all required data */
		case ubx_gen_m7:
		case ubx_gen_m8:
		case ubx_gen_m9:
		case ubx_gen_m10:
			return ubx_msgGetBatchBit(ubx_id_pvt);

		/* for pre-M7 devices, the data is split across 5 different messages */
		default:
			return ubx_msgGetBatchBit(ubx_id_posllh) |
					ubx_msgGetBatchBit(ubx_id_velned) |
					ubx_msgGetBatchBit(ubx_id_dop) |
					ubx_msgGetBatchBit(ubx_id_sol) |
					ubx_msgGetBatchBit(ubx_id_timeutc);
	}
}


static int ubx_setup(ubx_ctx_t *ctx)
{
	int ret = ubx_detect(ctx);

	if (ret < 0) {
		return ret;
	}

	/* set batch requirement */
	ctx->batchRequirement = ubx_getRequiredBatchMask(ctx->gen);

	return ubx_configure(ctx, UBX_DEFAULT_BAUDRATE);
}


/*
 * message handlers
 */


static int ubx_handlePvt(ubx_ctx_t *ctx)
{
	/* u-blox 7 emits an 84-byte NAV-PVT, the headVeh/magDec/magAcc fields exist only from u-blox 8 (92 bytes) onward */
	if (ctx->msg.len < 84U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	uint16_t year = ubx_readU16(p + 4);     /* year */
	uint8_t month = ubx_readU8(p + 6);      /* month [1; 12] */
	uint8_t day = ubx_readU8(p + 7);        /* day of month [1; 31] */
	uint8_t hour = ubx_readU8(p + 8);       /* hour of day [0; 23] */
	uint8_t min = ubx_readU8(p + 9);        /* minute of hour [0; 59] */
	uint8_t sec = ubx_readU8(p + 10);       /* second of minute [0; 60] */
	uint8_t valid = ubx_readU8(p + 11);     /* validity flags */
	int32_t nano = ubx_readI32(p + 16);     /* fraction of second [-1e9; +1e9] */
	uint8_t fix = ubx_readU8(p + 20);       /* gnss fix type */
	uint8_t numSV = ubx_readU8(p + 23);     /* number of satellites used */
	int32_t lon = ubx_readI32(p + 24);      /* longitude [deg * 1e-7] */
	int32_t lat = ubx_readI32(p + 28);      /* latitude [deg * 1e-7] */
	int32_t height = ubx_readI32(p + 32);   /* height above ellipsoid [mm] */
	int32_t hMSL = ubx_readI32(p + 36);     /* height above mean sea level [mm] */
	uint32_t hAcc = ubx_readU32(p + 40);    /* horizontal accuracy estimate [mm] */
	uint32_t vAcc = ubx_readU32(p + 44);    /* vertical accuracy estimate [mm] */
	int32_t velN = ubx_readI32(p + 48);     /* ned north velocity [mm/s] */
	int32_t velE = ubx_readI32(p + 52);     /* ned east velocity [mm/s] */
	int32_t velD = ubx_readI32(p + 56);     /* ned down velocity [mm/s] */
	int32_t gSpeed = ubx_readI32(p + 60);   /* ground speed 2d [mm/s] */
	int32_t headMot = ubx_readI32(p + 64);  /* heading of motion 2d [deg * 1e-5] */
	uint32_t sAcc = ubx_readU32(p + 68);    /* speed accuracy estimate [mm/s] */
	uint32_t headAcc = ubx_readU32(p + 72); /* heading accuracy estimate [deg * 1e-5] */
	uint16_t pDOP = ubx_readU16(p + 76);    /* position dop [1e-2] */
	int16_t magDec = 0;                     /* magnetic declination [deg * 1e-2] */
	if ((ctx->msg.len >= 92U) && ((valid & 0x08U) != 0U)) {
		magDec = ubx_readI16(p + 88);
	}

	/* fail if time or date are not valid or not resolved yet */
	if ((valid & 0x03U) != 0x03U || year < 1900 || month < 1) {
		return -1;
	}

	struct tm tmUTC = {
		.tm_year = year - 1900,
		.tm_mon = month - 1,
		.tm_mday = day,
		.tm_hour = hour,
		.tm_min = min,
		.tm_sec = sec,
		.tm_isdst = 0,
	};
	time_t epochSec = timegm(&tmUTC);
	if (epochSec < 0) {
		return -1;
	}

	ctx->ev.gps.utc = (uint64_t)((int64_t)epochSec * 1000000LL + (nano / 1000LL));

	if (headMot < 0) {
		headMot += 36000000;
	}

	ctx->ev.gps.fix = ubx_fixToGgaFix(fix);
	ctx->ev.gps.satsNb = numSV;
	ctx->ev.gps.lon = (int64_t)lon * 100LL;
	ctx->ev.gps.lat = (int64_t)lat * 100LL;
	ctx->ev.gps.altEllipsoid = height;
	ctx->ev.gps.alt = hMSL;
	ctx->ev.gps.eph = hAcc;
	ctx->ev.gps.epv = vAcc;
	ctx->ev.gps.velNorth = velN;
	ctx->ev.gps.velEast = velE;
	ctx->ev.gps.velDown = velD;
	ctx->ev.gps.groundSpeed = (gSpeed > 0) ? (uint32_t)gSpeed : 0U;
	ctx->ev.gps.evel = sAcc;
	ctx->ev.gps.heading = (uint16_t)ubx_degE5ToMRad(headMot);
	ctx->ev.gps.headingAccur = (uint16_t)ubx_degE5ToMRad((int32_t)headAcc);
	ctx->ev.gps.hdop = pDOP;
	ctx->ev.gps.vdop = pDOP;
	ctx->ev.gps.headingOffs = (int16_t)ubx_degE2ToMRad(magDec);

	return 0;
}


static int ubx_handlePosllh(ubx_ctx_t *ctx)
{
	if (ctx->msg.len < 28U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	int32_t lon = ubx_readI32(p + 4);     /* longitude [deg * 1e-7] */
	int32_t lat = ubx_readI32(p + 8);     /* latitude [deg * 1e-7] */
	int32_t height = ubx_readI32(p + 12); /* height above ellipsoid [mm] */
	int32_t hMSL = ubx_readI32(p + 16);   /* height above mean sea level [mm] */
	uint32_t hAcc = ubx_readU32(p + 20);  /* horizontal accuracy estimate [mm] */
	uint32_t vAcc = ubx_readU32(p + 24);  /* vertical accuracy estimate [mm] */

	ctx->ev.gps.lon = (int64_t)lon * 100LL;
	ctx->ev.gps.lat = (int64_t)lat * 100LL;
	ctx->ev.gps.altEllipsoid = height;
	ctx->ev.gps.alt = hMSL;
	ctx->ev.gps.eph = hAcc;
	ctx->ev.gps.epv = vAcc;

	return 0;
}


static int ubx_handleVelned(ubx_ctx_t *ctx)
{
	if (ctx->msg.len < 36U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	int32_t velN = ubx_readI32(p + 4);     /* ned north velocity [cm/s] */
	int32_t velE = ubx_readI32(p + 8);     /* ned east velocity [cm/s] */
	int32_t velD = ubx_readI32(p + 12);    /* ned down velocity [cm/s] */
	uint32_t gSpeed = ubx_readU32(p + 20); /* ground speed 2d [cm/s] */
	int32_t heading = ubx_readI32(p + 24); /* heading of motion 2d [deg * 1e-5] */
	uint32_t sAcc = ubx_readU32(p + 28);   /* speed accuracy estimate [cm/s] */
	uint32_t cAcc = ubx_readU32(p + 32);   /* heading accuracy estimate [deg * 1e-5] */

	if (heading < 0) {
		heading += 36000000;
	}

	/* nav-velned reports speeds in cm/s, the event uses mm/s */
	ctx->ev.gps.velNorth = velN * 10;
	ctx->ev.gps.velEast = velE * 10;
	ctx->ev.gps.velDown = velD * 10;
	ctx->ev.gps.groundSpeed = gSpeed * 10U;
	ctx->ev.gps.evel = sAcc * 10U;
	ctx->ev.gps.heading = (uint16_t)ubx_degE5ToMRad(heading);
	ctx->ev.gps.headingAccur = (uint16_t)ubx_degE5ToMRad((int32_t)cAcc);

	return 0;
}


static int ubx_handleDop(ubx_ctx_t *ctx)
{
	if (ctx->msg.len < 18U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	uint16_t vDOP = ubx_readU16(p + 10); /* vertical dop [1e-2] */
	uint16_t hDOP = ubx_readU16(p + 12); /* horizontal dop [1e-2] */

	ctx->ev.gps.hdop = hDOP;
	ctx->ev.gps.vdop = vDOP;

	return 0;
}


static int ubx_handleSol(ubx_ctx_t *ctx)
{
	if (ctx->msg.len < 52U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	uint8_t fix = ubx_readU8(p + 10);   /* gnss fix type */
	uint8_t numSV = ubx_readU8(p + 47); /* number of satellites used */

	ctx->ev.gps.fix = ubx_fixToGgaFix(fix);
	ctx->ev.gps.satsNb = numSV;

	return 0;
}


static int ubx_handleTimeUTC(ubx_ctx_t *ctx)
{
	if (ctx->msg.len < 20U) {
		return -1;
	}

	uint8_t *p = ctx->msg.pld;
	int32_t nano = ubx_readI32(p + 8); /* fraction of second [ns] */
	uint16_t year = ubx_readU16(p + 12);
	uint8_t month = ubx_readU8(p + 14);
	uint8_t day = ubx_readU8(p + 15);
	uint8_t hour = ubx_readU8(p + 16);
	uint8_t min = ubx_readU8(p + 17);
	uint8_t sec = ubx_readU8(p + 18);
	uint8_t valid = ubx_readU8(p + 19);

	/* fail if time is invalid */
	if ((valid & 0x07U) != 0x07U || year < 1900 || month < 1) {
		return -1;
	}

	struct tm tmUTC = {
		.tm_year = year - 1900,
		.tm_mon = month - 1,
		.tm_mday = day,
		.tm_hour = hour,
		.tm_min = min,
		.tm_sec = sec,
		.tm_isdst = 0,
	};
	time_t epochSec = timegm(&tmUTC);
	if (epochSec < 0) {
		return -1;
	}

	ctx->ev.gps.utc = (uint64_t)((int64_t)epochSec * 1000000LL + (nano / 1000LL));

	return 0;
}


static bool ubx_handleNavMsg(ubx_ctx_t *ctx)
{
	/* handle data contained in the message */
	int ret;
	switch (ctx->msg.id) {
		case ubx_id_posllh:
			ret = ubx_handlePosllh(ctx);
			break;

		case ubx_id_velned:
			ret = ubx_handleVelned(ctx);
			break;

		case ubx_id_dop:
			ret = ubx_handleDop(ctx);
			break;

		case ubx_id_sol:
			ret = ubx_handleSol(ctx);
			break;

		case ubx_id_timeutc:
			ret = ubx_handleTimeUTC(ctx);
			break;

		case ubx_id_pvt:
			ret = ubx_handlePvt(ctx);
			break;

		default:
			ret = -1;
	}

	/* ignore invalid messages */
	if (ret < 0) {
		return false;
	}

	/* undersized payload (the per-message handler bailed out) - do not touch the batch */
	if (ctx->msg.len < 4U) {
		return false;
	}

	/* update TOW for current batch (TOW is always the first field for exactly this purpose) */
	uint32_t tow = ubx_readU32(ctx->msg.pld + 0U);
	uint8_t bit = ubx_msgGetBatchBit(ctx->msg.id);
	if (tow == ctx->batchTOW) {
		/* add message to the current batch */
		ctx->batchMask |= bit;
	}
	else {
		/* reset the batch */
		ctx->batchTOW = tow;
		ctx->batchMask = bit;
	}

	/* handle finished batch */
	if ((ctx->batchMask & ctx->batchRequirement) == ctx->batchRequirement) {
		gettime(&ctx->ev.timestamp, NULL);
		ctx->batchMask = 0U;
		return true;
	}

	return false;
}


/*
 * main thread
 */


static int ubx_runSetup(ubx_ctx_t *ctx)
{
	for (;;) {
		if (!ctx->run) {
			return -1;
		}
		if (ctx->fd >= 0 && ubx_setup(ctx) == 0) {
			return 0;
		}
		if (ctx->fd >= 0) {
			close(ctx->fd);
			ctx->fd = -1;
		}
		usleep(UBX_RETRY_US);
		ctx->fd = open(ctx->path, O_RDWR | O_NOCTTY | O_NONBLOCK);
	}
}


static void ubx_threadPublish(void *data)
{
	sensor_info_t *info = (sensor_info_t *)data;
	struct __errno_t errnoNew;
	ubx_ctx_t *ctx = info->ctx;

	if (ubx_runSetup(ctx) < 0) {
		endthread();
		return;
	}

	/* Redirecting errno to keep backward compatibility (in case of errno not working correctly) */
	_errno_new(&errnoNew);

	while (ctx->run) {
		if (ubx_msgGetByClass(ctx, ubx_class_nav, ubx_getDeadline(UBX_MAX_TOUT_US)) < 0) {
			/* if maximum timeout has been reached then the device has been lost - try to reconnect */
			(void)ubx_runSetup(ctx);
			continue;
		}
		if (!ubx_handleNavMsg(ctx)) {
			continue;
		}
		(void)sensors_publish(info->id, &ctx->ev);
	}

	endthread();
}


static int ubx_start(sensor_info_t *info)
{
	int err;
	ubx_ctx_t *ctx = info->ctx;

	ctx->ev.type = SENSOR_TYPE_GPS;
	ctx->ev.gps.devId = info->id;
	ctx->run = true;

	err = beginthreadex(ubx_threadPublish, THREAD_PRIORITY_SENSOR, ctx->stack, sizeof(ctx->stack), info, &ctx->tid);
	if (err < 0) {
		close(ctx->fd);
		free(ctx);
		info->ctx = NULL;
	}
	else {
		printf("%s launched sensor\n", UBX_STR);
	}

	return err;
}


/*
 * driver interface
 */


static int ubx_parseDynModel(const char *str, uint8_t *dynModel)
{
	static const struct {
		const char *str;
		uint8_t model;
	} models[] = {
		{ "portable", 0 },
		{ "stationary", 2 },
		{ "pedestrian", 3 },
		{ "automotive", 4 },
		{ "sea", 5 },
		{ "air1", 6 },  /* airborne < 1g acceleration */
		{ "air2", 7 },  /* airborne < 2g acceleration */
		{ "air4", 8 },  /* airborne < 4g acceleration */
		{ "wrist", 9 }, /* wrist worn watch */
		{ "bike", 10 },
	};
	int ret = -1;

	for (size_t i = 0; i < (sizeof(models) / sizeof(models[0])); i++) {
		if (strcmp(str, models[i].str) == 0) {
			*dynModel = models[i].model;
			ret = 0;
			break;
		}
	}

	return ret;
}


static int ubx_parseSbas(const char *str, ubx_sbas_t *sbas)
{
	static const struct {
		const char *str;
		ubx_sbas_t sbas;
	} regions[] = {
		{ "eu", ubx_sbas_eu },
		{ "na", ubx_sbas_na },
	};

	for (size_t i = 0; i < (sizeof(regions) / sizeof(regions[0])); i++) {
		if (strcmp(str, regions[i].str) == 0) {
			*sbas = regions[i].sbas;
			return 0;
		}
	}

	return -1;
}


static int ubx_parseOpt(char *opt, ubx_opts_t *opts)
{
	int ret = -1;
	char *val = strchr(opt, '=');

	if (val == NULL) {
		return -1;
	}
	*val = '\0';
	val++;

	if (strcmp(opt, "rate") == 0) {
		/* solution rate in Hz */
		unsigned long rate = strtoul(val, NULL, 10);
		if ((rate >= 1UL) && (rate <= UBX_MAX_RATE_HZ)) {
			opts->rateMs = (uint16_t)(1000UL / rate);
			ret = 0;
		}
	}
	else if (strcmp(opt, "dyn") == 0) {
		/* dynamic platform model */
		ret = ubx_parseDynModel(val, &opts->dynModel);
	}
	else if (strcmp(opt, "sbas") == 0) {
		ret = ubx_parseSbas(val, &opts->sbas);
	}
	else if (strcmp(opt, "galileo") == 0) {
		if (strcmp(val, "1") == 0) {
			opts->galileo = ubx_constellationMode_on;
			ret = 0;
		}
		else if (strcmp(val, "0") == 0) {
			opts->galileo = ubx_constellationMode_off;
			ret = 0;
		}
	}
	else if (strcmp(opt, "beidou") == 0) {
		if (strcmp(val, "1") == 0) {
			opts->beidou = ubx_constellationMode_on;
			ret = 0;
		}
		else if (strcmp(val, "0") == 0) {
			opts->beidou = ubx_constellationMode_off;
			ret = 0;
		}
	}
	else {
		/* unknown options are ignored to stay forward compatible */
		fprintf(stderr, "%s ignoring unknown option: %s\n", UBX_STR, opt);
		ret = 0;
	}

	return ret;
}


/*
 * driver arguments format: <device-path>[:opt=val,opt=val,...]
 * supported options:
 *   rate=<1..20>  navigation solution rate in Hz (default 10)
 *   dyn=<model>   dynamic platform model: portable, stationary, pedestrian,
 *                 automotive, sea, air1, air2, air4, wrist, bike
 *   sbas=<region> SBAS augmentation region: eu (EGNOS), na (WAAS)
 *   galileo=1    enable Galileo constellation (M8+)
 *   beidou=1     enable BeiDou constellation (M8+)
 * Module generation is auto-detected via UBX-MON-VER.
 */
static int ubx_parseArgs(const char *args, ubx_ctx_t *ctx)
{
	char buf[128];
	char *opts;

	if ((args == NULL) || (strlen(args) >= sizeof(buf))) {
		fprintf(stderr, "%s invalid arguments\n", UBX_STR);
		return -EINVAL;
	}

	strcpy(buf, args);

	opts = strchr(buf, ':');
	if (opts != NULL) {
		*opts = '\0';
		opts++;
	}

	if ((strlen(buf) == 0U) || (strlen(buf) >= sizeof(ctx->path))) {
		fprintf(stderr, "%s invalid device path\n", UBX_STR);
		return -EINVAL;
	}
	strcpy(ctx->path, buf);

	ctx->opts.rateMs = 1000U / UBX_DEFAULT_RATE_HZ;
	ctx->opts.dynModel = UBX_DYN_KEEP;
	ctx->opts.sbas = ubx_sbas_off;
	ctx->opts.galileo = ubx_constellationMode_keep;
	ctx->opts.beidou = ubx_constellationMode_keep;

	/* parse comma separated options */
	while ((opts != NULL) && (*opts != '\0')) {
		char *next = strchr(opts, ',');
		if (next != NULL) {
			*next = '\0';
			next++;
		}

		if (ubx_parseOpt(opts, &ctx->opts) < 0) {
			fprintf(stderr, "%s invalid option: %s\n", UBX_STR, opts);
			return -EINVAL;
		}

		opts = next;
	}

	return 0;
}


static int ubx_alloc(sensor_info_t *info, const char *args)
{
	int cnt = 0;
	int err;
	ubx_ctx_t *ctx;

	ctx = malloc(sizeof(ubx_ctx_t));
	if (ctx == NULL) {
		return -ENOMEM;
	}
	memset(ctx, 0, sizeof(*ctx));

	info->types = SENSOR_TYPE_GPS;

	err = ubx_parseArgs(args, ctx);
	if (err != 0) {
		free(ctx);
		fprintf(stderr,
				"%s usage: <device-path>[:rate=<1..20>][,dyn=<model>][,sbas=<eu|na>][,galileo=1][,beidou=1]\n"
				"%s for example: /dev/uart0:rate=5,dyn=air4,sbas=eu,galileo=1\n",
				UBX_STR, UBX_STR);
		return err;
	}

	/* Opening serial device */
	do {
		ctx->fd = open(ctx->path, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (ctx->fd < 0) {
			cnt++;
			if (cnt > UBX_OPEN_RETRIES) {
				err = -errno;
				fprintf(stderr, "%s Can't open %s: %s\n", UBX_STR, ctx->path, strerror(-err));
				free(ctx);
				return err;
			}
			else {
				usleep(10 * 1000);
			}
		}
	} while (ctx->fd < 0);

	if (isatty(ctx->fd) != 1) {
		fprintf(stderr, "%s %s not a tty\n", UBX_STR, ctx->path);
		close(ctx->fd);
		free(ctx);
		return -ENOTTY;
	}

	info->ctx = ctx;

	return 0;
}


static int ubx_dealloc(sensor_info_t *info)
{
	ubx_ctx_t *ctx;

	if ((info == NULL) || (info->ctx == NULL)) {
		return -EINVAL;
	}

	ctx = (ubx_ctx_t *)info->ctx;
	ctx->run = false;

	if (ctx->tid != 0) {
		(void)threadJoin(ctx->tid, (time_t)-1);
	}

	(void)close(ctx->fd);

	free(ctx);
	info->ctx = NULL;

	return 0;
}


void __attribute__((constructor)) ubx_register(void)
{
	static sensor_drv_t sensor = {
		.name = "ubx",
		.alloc = ubx_alloc,
		.start = ubx_start,
		.dealloc = ubx_dealloc,
	};

	sensors_register(&sensor);
}
