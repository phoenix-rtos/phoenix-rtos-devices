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

#include "nmea.h"


/* Moves pointer to the n-th comma character in str (excluding the first character).
 * Returns the pointer with the new value or NULL if search failed */
static char *nmea_nField(char *str, int n)
{
	int i;

	for (i = 0; i < n && str != NULL; i++) {
		str = strchr(str + 1, ',');
	}

	return str;
}


static int nmea_parsegsa(char *str, nmea_t *out)
{
	char *p = str;
	nmea_gsa_t in;

	p = nmea_nField(p, field_gsa_fix);
	if (p == NULL) {
		return nmea_broken;
	}
	/* if failed, gsa_fix_notavailable will be assigned */
	in.fix = strtoul(p + 1, NULL, 10);
	if (in.fix < gsa_fix_notavailable || in.fix > gsa_fix_3d) {
		in.fix = gsa_fix_notavailable;
	}

	p = nmea_nField(p, field_gsa_pdop - field_gsa_fix);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.pdop = strtod(p + 1, NULL);

	p = nmea_nField(p, field_gsa_hdop - field_gsa_pdop);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.hdop = strtod(p + 1, NULL);

	p = nmea_nField(p, field_gsa_vdop - field_gsa_hdop);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.vdop = strtod(p + 1, NULL);

	out->msg.gsa = in;
	out->type = nmea_gsa;

	return nmea_gsa;
}


static int nmea_parsevtg(char *str, nmea_t *out)
{
	char *p = str;
	nmea_vtg_t in;

	p = nmea_nField(p, field_vtg_track);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.track = strtod(p + 1, NULL);

	p = nmea_nField(p, field_vtg_speedknots - field_vtg_track);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.speed_knots = strtod(p + 1, NULL);

	p = nmea_nField(p, field_vtg_speedkmh - field_vtg_speedknots);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.speed_kmh = strtod(p + 1, NULL);

	out->msg.vtg = in;
	out->type = nmea_vtg;

	return nmea_vtg;
}


static int nmea_parsegga(char *str, nmea_t *out)
{
	char *p = str, *sign;
	nmea_gga_t in;

	/* latitude read */
	p = nmea_nField(p, field_gga_lat);
	if (p == NULL) {
		return nmea_broken;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.lat = strtod(p + 1, NULL);
	/* conversion from ddmm.mmmm to dd.dddd */
	in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60;
	in.lat = (sign[1] == 'S') ? -in.lat : in.lat;

	/* longitude read */
	p = nmea_nField(p, field_gga_lon - field_gga_lat);
	if (p == NULL) {
		return nmea_broken;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.lon = strtod(p + 1, NULL);
	/* conversion from ddmm.mmmm to dd.dddd */
	in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60;
	in.lon = (sign[1] == 'W') ? -in.lon : in.lon;

	/* GPS Quality indicator read */
	p = nmea_nField(p, field_gga_fix - field_gga_lon);
	if (p == NULL) {
		return nmea_broken;
	}

	in.fix = strtoul(p + 1, NULL, 10);
	if (in.fix > gga_fix_simulation) {
		in.fix = gga_fix_invalid;
	}

	/* number of satellites in use read */
	p = nmea_nField(p, field_gga_sats - field_gga_fix);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.sats = strtoul(p + 1, NULL, 10);

	/* horizontal dilution of precision read */
	p = nmea_nField(p, field_gga_hdop - field_gga_sats);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.hdop = strtod(p + 1, NULL);

	/* Antenna altitude above mean-sea-level read */
	p = nmea_nField(p, field_gga_h_asl - field_gga_hdop);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.h_asl = strtod(p + 1, NULL);

	/* Geoidal separation read */
	p = nmea_nField(p, field_gga_h_wgs - field_gga_h_asl);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.h_wgs = strtod(p + 1, NULL);

	out->msg.gga = in;
	out->type = nmea_gga;

	return nmea_gga;
}


static int nmea_parsermc(char *str, nmea_t *out)
{
	char *p = str, *sign;
	nmea_rmc_t in;

	/* latitude read */
	p = nmea_nField(p, field_rmc_lat);
	if (p == NULL) {
		return nmea_broken;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.lat = strtod(p + 1, NULL);
	in.lat = ((int)in.lat / 100) + (in.lat - ((int)in.lat / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
	in.lat = (sign[1] == 'S') ? -in.lat : in.lat;

	/* longitude read */
	p = nmea_nField(p, field_rmc_lon - field_rmc_lat);
	if (p == NULL) {
		return nmea_broken;
	}
	sign = nmea_nField(p, 1);
	if (sign == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.lon = strtod(p + 1, NULL);
	in.lon = ((int)in.lon / 100) + (in.lon - ((int)in.lon / 100) * 100) / 60; /* conversion from ddmm.mmmm to dd.dddd */
	in.lon = (sign[1] == 'W') ? -in.lon : in.lon;

	/* speed in knots read */
	p = nmea_nField(p, field_rmc_speedknots - field_rmc_lon);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.speed = strtod(p + 1, NULL);

	/* course read */
	p = nmea_nField(p, field_rmc_course - field_rmc_speedknots);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.course = strtod(p + 1, NULL);

	/* magnetic variation read */
	p = nmea_nField(p, field_rmc_magvar - field_rmc_course);
	if (p == NULL) {
		return nmea_broken;
	}
	/* when error occurred 0 is assigned */
	in.magvar = strtod(p + 1, NULL);

	out->msg.rmc = in;
	out->type = nmea_rmc;

	return nmea_rmc;
}


int nmea_interpreter(char *str, nmea_t *out)
{
	int ret;

	if (strncmp(str, "$GPGSA", 6) == 0) {
		ret = nmea_parsegsa(str, out);
	}
	else if (strncmp(str, "$GPVTG", 6) == 0) {
		ret = nmea_parsevtg(str, out);
	}
	else if (strncmp(str, "$GPGGA", 6) == 0) {
		ret = nmea_parsegga(str, out);
	}
	else if (strncmp(str, "$GPRMC", 6) == 0) {
		ret = nmea_parsermc(str, out);
	}
	else {
		ret = nmea_unknown;
	}

	return ret;
}


int nmea_countlines(char *buf)
{
	int n = 0;
	char *curr;

	curr = strchr(buf, '$');

	while (curr != NULL) {
		/* assert that '*' and checksum characters are available */
		curr = strchr(curr, '*');
		if (curr != NULL) {
			if ((curr[1] != '\0') && (curr[2] != '\0')) {
				n++;
			}
			curr = strchr(curr, '$');
		}
	}

	return n;
}


char **nmea_getlines(char *buf, int n)
{
	char **lines;
	char *curr, *start;
	int i;

	curr = buf;

	lines = malloc(sizeof(char *) * n);

	if (lines == NULL) {
		return NULL;
	}

	for (i = 0; i < n; i++) {
		start = strchr(curr, '$');
		if (start == NULL) {
			break;
		}

		curr = strchr(start, '*');
		if (curr == NULL) {
			break;
		}

		curr[3] = '\0';
		curr += 4;

		lines[i] = start;
	}

	/* something went wrong - aborting */
	if (i < n) {
		free(lines);
		return NULL;
	}
	else {
		return lines;
	}
}


int nmea_assertChecksum(const char *line)
{
	char countedChecksum;
	unsigned int readChecksum;
	int i, len;

	countedChecksum = 0;
	readChecksum = 0;

	/* example message: `$GPVTG,9.75,T,,M,0.02,N,0.04,K,A*30` */
	/* `*30` is the value of checksum, it has to be skipped when counting */
	len = strlen(line) - 3;

	for (i = 1; i < len; i++) {
		countedChecksum ^= line[i];
	}
	sscanf(&line[i + 1], "%x", &readChecksum);

	return (countedChecksum == (char)readChecksum) ? EOK : -1;
}
