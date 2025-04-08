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

#ifndef _NMEA_H_
#define _NMEA_H_

/* clang-format off */
/* types of messages interpreted, together with unknown, and broken message */
enum nmea_type { nmea_gga, nmea_gsa, nmea_rmc, nmea_vtg, nmea_unknown, nmea_broken };
/* clang-format on */

/* GPGSA nmea type definitions: GPS DOP and active satellites */
typedef struct {
	unsigned int fix;
	float pdop;
	float hdop;
	float vdop;
} nmea_gsa_t;

/* clang-format off */
enum nmea_gsa_fix { gsa_fix_notavailable = 1, gsa_fix_2d, gsa_fix_3d };


enum nmea_fields_gsa { field_gsa_fix = 2, field_gsa_pdop = 15, field_gsa_hdop = 16, field_gsa_vdop = 17 };
/* clang-format on */


/* GPRMC nmea type definitions: Recommended minimum specific GPS/Transit data */
typedef struct {
	float lat;
	float lon;
	float speed;
	float course;
	float magvar;
} nmea_rmc_t;

/* clang-format off */
enum nmea_fields_rmc { field_rmc_lat = 3, field_rmc_lon = 5, field_rmc_speedknots = 7, field_rmc_course = 8,
	field_rmc_magvar = 10 };
/* clang-format on */


/* GPVTG nmea type definitions: Track Made Good and Ground Speed. */
typedef struct {
	float track;
	char track_type;
	float speed_knots;
	float speed_kmh;
} nmea_vtg_t;


/* clang-format off */
enum nmea_fields_vtg { field_vtg_track = 1, field_vtg_tracktype = 2, field_vtg_speedknots = 5, field_vtg_speedkmh = 7 };
/* clang-format on */


/* GPGGA nmea type definitions: Global Positioning System Fix Data  */
typedef struct {
	double lat;
	double lon;
	float utc;
	unsigned int fix;
	unsigned int sats;
	float hdop;
	float h_asl;
	float h_wgs;
} nmea_gga_t;

/* clang-format off */
enum nmea_gga_fix { gga_fix_invalid, gga_fix_gnss, gga_fix_dpgs, gga_fix_pps, gga_fix_rtkinematic,
	gga_fix_estimated = 6,gga_fix_maninput, gga_fix_simulation };


enum nmea_fields_gga { field_gga_utc = 1, field_gga_lat = 2, field_gga_lon = 4, field_gga_fix = 6, field_gga_sats = 7,
	field_gga_hdop = 8, field_gga_h_asl = 9, field_gga_h_wgs = 11 };
/* clang-format on */


/* incoming message storage */
typedef struct {
	int type;
	union {
		nmea_gsa_t gsa;
		nmea_rmc_t rmc;
		nmea_vtg_t vtg;
		nmea_gga_t gga;
	} msg;
} nmea_t;


/* interpret one line of gps output into nmea message */
extern void nmea_interpreter(char *str, nmea_t *out);

#endif
