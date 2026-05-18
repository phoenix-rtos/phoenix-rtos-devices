/*
 * Phoenix-RTOS
 *
 * NMEA
 *
 * Copyright 2022 Phoenix Systems
 * Author: Damian Loewnau, Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include <libsensors/gps/nmea.h>


/* Moves pointer to the n-th comma character in str (excluding the first character).
 * Returns the pointer with the new value or NULL if search failed */
static const char *nmea_nField(const char *str, int n)
{
	int i;

	for (i = 0; i < n && str != NULL; i++) {
		str = strchr(str + 1, ',');
	}

	return str;
}


static void nmea_parsegsa(const char *str, nmea_t *out)
{
	const char *p = str;
	nmea_gsa_t in = { 0 };

	out->type = nmea_broken;

	p = nmea_nField(p, field_gsa_fix);
	if (p == NULL) {
		return;
	}
	/* if failed, gsa_fix_notavailable will be assigned */
	in.fix = strtoul(p + 1, NULL, 10);
	if (in.fix < gsa_fix_notavailable || in.fix > gsa_fix_3d) {
		in.fix = gsa_fix_notavailable;
	}

	p = nmea_nField(p, field_gsa_pdop - field_gsa_fix);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.pdop = strtod(p + 1, NULL);

	p = nmea_nField(p, field_gsa_hdop - field_gsa_pdop);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.hdop = strtod(p + 1, NULL);

	p = nmea_nField(p, field_gsa_vdop - field_gsa_hdop);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.vdop = strtod(p + 1, NULL);

	out->msg.gsa = in;
	out->type = nmea_gsa;
}


static void nmea_parsevtg(const char *str, nmea_t *out)
{
	const char *p = str;
	nmea_vtg_t in = { 0 };

	out->type = nmea_broken;

	p = nmea_nField(p, field_vtg_track);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.track = strtod(p + 1, NULL);

	p = nmea_nField(p, field_vtg_speedknots - field_vtg_track);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.speed_knots = strtod(p + 1, NULL);

	p = nmea_nField(p, field_vtg_speedkmh - field_vtg_speedknots);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.speed_kmh = strtod(p + 1, NULL);

	out->msg.vtg = in;
	out->type = nmea_vtg;
}


static void nmea_parsegga(const char *str, nmea_t *out)
{
	const char *p = str, *sign;
	nmea_gga_t in = { 0 };

	out->type = nmea_broken;

	/* utc fix time */
	p = nmea_nField(p, field_gga_utc);
	if (p == NULL) {
		return;
	}
	in.utc = strtod(p + 1, NULL);

	/* latitude read */
	p = nmea_nField(p, field_gga_lat - field_gga_utc);
	if (p == NULL) {
		return;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.lat = strtod(p + 1, NULL);
	/* conversion from ddmm.mmmm to dd.dddd */
	in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60;
	in.lat = (sign[1] == 'S') ? -in.lat : in.lat;

	/* longitude read */
	p = nmea_nField(p, field_gga_lon - field_gga_lat);
	if (p == NULL) {
		return;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.lon = strtod(p + 1, NULL);
	/* conversion from ddmm.mmmm to dd.dddd */
	in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60;
	in.lon = (sign[1] == 'W') ? -in.lon : in.lon;

	/* GPS Quality indicator read */
	p = nmea_nField(p, field_gga_fix - field_gga_lon);
	if (p == NULL) {
		return;
	}

	in.fix = strtoul(p + 1, NULL, 10);
	if (in.fix > gga_fix_simulation) {
		in.fix = gga_fix_invalid;
	}

	/* number of satellites in use read */
	p = nmea_nField(p, field_gga_sats - field_gga_fix);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.sats = strtoul(p + 1, NULL, 10);

	/* horizontal dilution of precision read */
	p = nmea_nField(p, field_gga_hdop - field_gga_sats);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.hdop = strtod(p + 1, NULL);

	/* Antenna altitude above mean-sea-level read */
	p = nmea_nField(p, field_gga_h_asl - field_gga_hdop);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.h_asl = strtod(p + 1, NULL);

	/* Geoidal separation read */
	p = nmea_nField(p, field_gga_h_wgs - field_gga_h_asl);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.h_wgs = strtod(p + 1, NULL);

	out->msg.gga = in;
	out->type = nmea_gga;
}


static void nmea_parsermc(const char *str, nmea_t *out)
{
	const char *p = str, *sign;
	nmea_rmc_t in = { 0 };

	out->type = nmea_broken;

	/* latitude read */
	p = nmea_nField(p, field_rmc_lat);
	if (p == NULL) {
		return;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.lat = strtod(p + 1, NULL);
	in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
	in.lat = (sign[1] == 'S') ? -in.lat : in.lat;

	/* longitude read */
	p = nmea_nField(p, field_rmc_lon - field_rmc_lat);
	if (p == NULL) {
		return;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.lon = strtod(p + 1, NULL);
	in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
	in.lon = (sign[1] == 'W') ? -in.lon : in.lon;

	/* speed in knots read */
	p = nmea_nField(p, field_rmc_speedknots - field_rmc_lon);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.speed = strtod(p + 1, NULL);

	/* course read */
	p = nmea_nField(p, field_rmc_course - field_rmc_speedknots);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.course = strtod(p + 1, NULL);

	/* magnetic variation read */
	p = nmea_nField(p, field_rmc_magvar - field_rmc_course);
	if (p == NULL) {
		return;
	}
	/* when error occurred 0 is assigned */
	in.magvar = strtod(p + 1, NULL);

	out->msg.rmc = in;
	out->type = nmea_rmc;
}


static int nmea_checksum(const char *buff, size_t buffSz)
{
	unsigned int ii = 1;
	unsigned char csum = 0;
	char csumStr[3];

	if (buffSz == 0) {
		return -1;
	}

	if (buff[0] != '$') {
		return -1;
	}

	while ((ii < buffSz) && (buff[ii] != '*')) {
		csum ^= buff[ii];
		ii++;
	}

	if ((ii >= buffSz) || (buff[ii] != '*')) {
		return -1;
	}

	if ((ii + 2) >= buffSz) {
		return -1;
	}

	snprintf(csumStr, sizeof(csumStr), "%02X", csum);
	if ((csumStr[0] != buff[ii + 1]) || (csumStr[1] != buff[ii + 2])) {
		return -1;
	}

	return 0;
}


void nmea_interpreter(const char *str, nmea_t *out)
{
	if (strncmp(str, "$G", 2) != 0 || (str[2] == '\0')) {
		out->type = nmea_unknown;
		return;
	}

	if (strncmp(&str[3], "GSA", 3) == 0) {
		nmea_parsegsa(str, out);
	}
	else if (strncmp(&str[3], "VTG", 3) == 0) {
		nmea_parsevtg(str, out);
	}
	else if (strncmp(&str[3], "GGA", 3) == 0) {
		nmea_parsegga(str, out);
	}
	else if (strncmp(&str[3], "RMC", 3) == 0) {
		nmea_parsermc(str, out);
	}
	else {
		out->type = nmea_unknown;
	}
}


int nmea_scan(nmea_scanCtx_t *ctx, const char *buff, size_t buffSz, const char **frame)
{
	if ((ctx == NULL) || (buff == NULL) || (buffSz == 0) || (frame == NULL)) {
		return -1;
	}
	*frame = NULL;

	while (ctx->rdIdx < buffSz) {
		/* pull data from input buffer into frame if frame buffer index is aligned with buffer data count */
		if ((ctx->buffIdx == ctx->buffCnt) && (ctx->buffCnt < sizeof(ctx->buff))) {
			ctx->buff[ctx->buffIdx] = buff[ctx->rdIdx];
			ctx->rdIdx++;
			ctx->buffCnt++;
		}

		switch (ctx->state) {
			case state_header:
				/* check for frame buffer space */
				if (ctx->buffCnt == sizeof(ctx->buff)) {
					/* buffer full - no header found, discard buffer content */
					ctx->buffIdx = 0;
					ctx->buffCnt = 0;
					break;
				}

				/* search for header in buffered data */
				if (ctx->buff[ctx->buffIdx] != '$') {
					ctx->buffIdx++;
					break;
				}
				ctx->headIdx = ctx->buffIdx;
				ctx->footIdx = ctx->headIdx;
				ctx->state = state_pld;
				ctx->buffIdx++;
				break;

			case state_pld:
				/* check for frame buffer space */
				if (ctx->buffCnt == sizeof(ctx->buff)) {
					if (ctx->headIdx == 0) {
						/* this is not valid frame - search for header */
						ctx->buffIdx = 1;
						ctx->state = state_header;
					}
					else {
						/* frame cannot be discarded yet - move data to beginning of buffer */
						ctx->buffCnt -= ctx->headIdx;
						memmove(ctx->buff, &ctx->buff[ctx->headIdx], ctx->buffCnt);
						ctx->buffIdx -= ctx->headIdx;
						ctx->headIdx = 0;
					}
					break;
				}

				/* search for footer in buffered data */
				if (ctx->buff[ctx->buffIdx] != '\n') {
					ctx->buffIdx++;
					break;
				}
				ctx->footIdx = ctx->buffIdx;

				/* validate frame */
				ctx->state = state_header;
				uint32_t frameLen = ctx->footIdx - ctx->headIdx + 1;
				if (nmea_checksum(&ctx->buff[ctx->headIdx], frameLen) != 0) {
					ctx->buffIdx = ctx->headIdx + 1; /* align current frame buffer index with header index */
					break;
				}

				/* frame valid - return with success */
				ctx->buff[ctx->footIdx] = '\0'; /* substitute newline with null character */
				*frame = &ctx->buff[ctx->headIdx];
				ctx->buffIdx = ctx->footIdx; /* align current frame buffer index with header index */
				return frameLen;

			default:
				ctx->rdIdx = 0;
				ctx->headIdx = 0;
				ctx->footIdx = 0;
				ctx->buffIdx = 0;
				ctx->buffCnt = 0;
				ctx->state = state_header;
				break;
		}
	}

	/* we should reach this only if whole buffer was scanned */
	ctx->rdIdx = 0;

	return 0;
}
