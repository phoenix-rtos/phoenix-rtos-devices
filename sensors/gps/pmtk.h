/*
 * Phoenix-RTOS
 *
 * PMTK protocol header file.
 *
 * Checksum validation can be performed using online checksum generators:
 * http://www.hhhh.org/wiml/proj/nmeaxor.html (access: february 2023)
 *
 * Copyright 2023 Phoenix Systems
 * Author: Mateusz Niewiadomski
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */


#ifndef _PAH_PMTK_H_
#define _PAH_PMTK_H_

/* universal pmtk protocol message header */
#define PMTK_HEADER "$PMTK"

/* Command acknowledgement return */
#define PMTK_ID_ACK      001
#define PMTK_ACK_HEADER  "$PMTK001,"
#define PMTK_ACK_MSG_LEN 17

enum pmtk_ack { pmtk_ack_none = -1, pmtk_ack_invalid = 0, pmtk_ack_nosupport = 1, pmtk_ack_failed = 2, pmtk_ack_success = 4 };


/* Hot start request */
#define PMTK_ID_HOT_START  011
#define PMTK_CMD_HOT_START "MTKGPS*08"


/* Baudrates */
#define PMTK_ID_SET_NMEA_BAUDRATE     251
#define PMTK_SET_NMEA_BAUDRATE_115200 "115200*1F" /* sets communication baudrate to 115200 */
#define PMTK_SET_NMEA_BAUDRATE_57600  "57600*2C"  /* sets communication baudrate to  57600 */
#define PMTK_SET_NMEA_BAUDRATE_19200  "19200*22"  /* sets communication baudrate to  19200 */
#define PMTK_SET_NMEA_BAUDRATE_9600   "9600*17"   /* sets communication baudrate to   9600 */


/* Update rates (frequency at which data is send to client) */
#define PMTK_ID_SET_NMEA_UPDATERATE  220
#define PMTK_SET_NMEA_UPDATERATE_10k "10000*2F" /* nmea message period 10000 ms */
#define PMTK_SET_NMEA_UPDATERATE_5k  "5000*1B"  /* nmea message period 5000 ms */
#define PMTK_SET_NMEA_UPDATERATE_1k  "1000*1F"  /* nmea message period 1000 ms */
#define PMTK_SET_NMEA_UPDATERATE_500 "500*2B"   /* nmea message period 500 ms */
#define PMTK_SET_NMEA_UPDATERATE_200 "250*29"   /* nmea message period 200 ms */
#define PMTK_SET_NMEA_UPDATERATE_100 "100*2F"   /* nmea message period 100 ms */


/* Position fix rates (frequency at which gps performs position fix calculation) */
#define PMTK_ID_API_SET_FIX_CTL  300
#define PMTK_API_SET_FIX_CTL_10k "10000,0,0,0,0*2C" /* position fixing activity peroiod 10000 ms */
#define PMTK_API_SET_FIX_CTL_5k  "5000,0,0,0,0*18"  /* position fixing activity peroiod 5000 ms */
#define PMTK_API_SET_FIX_CTL_1k  "1000,0,0,0,0*1C"  /* position fixing activity peroiod 1000 ms */
#define PMTK_API_SET_FIX_CTL_200 "200,0,0,0,0*2F"   /* position fixing activity peroiod 200 ms */
#define PMTK_API_SET_FIX_CTL_100 "100,0,0,0,0*2F"   /* position fixing activity peroiod 200 ms */


/* Update output data selectors */
#define PMTK_ID_SET_NMEA_OUTPUT          314
#define PMTK_SET_NMEA_OUTPUT_RMC         "0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" /* only GPRMC present on update */
#define PMTK_SET_NMEA_OUTPUT_GGA         "0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" /* only GPGGA present on update */
#define PMTK_SET_NMEA_OUTPUT_GSA         "0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" /* only GPGSA present on update */
#define PMTK_SET_NMEA_OUTPUT_GSA_GGA_RMC "0,1,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" /* GSA/GGA/RMC present */
#define PMTK_SET_NMEA_OUTPUT_GSA_GGA_VTG "0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29" /* GSA/GGA/VTG present */

#endif
