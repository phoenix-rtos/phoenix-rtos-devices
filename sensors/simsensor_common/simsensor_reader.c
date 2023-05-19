/*
 * Phoenix-RTOS
 *
 * Simsensor reader reads and parses data form special scenario file
 * than it pushes it to event_queue
 *
 * Copyright 2023 Phoenix Systems
 * Author: Piotr Nieciecki
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <limits.h>
#include <sys/types.h>
#include <string.h>
#include <sys/time.h>

#include "simsensor_reader.h"

#define READER_END_SCENARIO_INDICATOR NO_SENSOR
#define READER_DATA_SEPARATOR         ','

#define READER_TIMESTAMP_NOT_SET (time_t) LLONG_MAX


/* Reads actual field (`actField`) and parses it to `res`. Updates `actField` so it's pointing to next field. Returns -1 if error occurs. */
static int reader_getFieldLLong(const char **actField, long long *res)
{
	char *endptr;
	long long value = strtoll(*actField, &endptr, 10);

	if (!(*endptr == READER_DATA_SEPARATOR || *endptr == '\0' || *endptr == '\n' || *endptr == '\r' || *actField == endptr)) {
		printf("simsensor: Invalid file\n");
		return -1;
	}

	if (*endptr == READER_DATA_SEPARATOR) {
		*actField = endptr + 1;
	}

	*res = value;

	return 0;
}


static int reader_accelDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_ACCEL;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelX = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelY = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelZ = tmp;

	return 0;
}


static int reader_baroDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_BARO;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->baro.pressure = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->baro.temp = tmp;

	return 0;
}


static int reader_gpsDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_GPS;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.alt = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.lat = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.lon = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.hdop = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.vdop = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.altEllipsoid = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.groundSpeed = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.velNorth = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.velEast = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.velDown = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.eph = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.epv = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.evel = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.heading = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.headingOffs = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.headingAccur = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.satsNb = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gps.fix = tmp;

	return 0;
}


static int reader_gyroDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_GYRO;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.gyroX = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.gyroY = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.gyroZ = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.dAngleX = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.dAngleY = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->gyro.dAngleZ = tmp;

	return 0;
}


static int reader_magDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_MAG;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->mag.magX = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->mag.magY = tmp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->mag.magZ = tmp;

	return 0;
}


static int reader_tempDataParse(const char *data, time_t timestamp, sensor_event_t *result)
{
	const char *actField;
	long long tmp;

	actField = data;

	result->type = SENSOR_TYPE_TEMP;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(&actField, &tmp) < 0) {
		return -1;
	}
	result->temp.temp = tmp;

	return 0;
}


int reader_read(simsens_reader_t *rd, event_queue_t *queue)
{
	const char *actField;
	long long tmp;
	int err = 0, emptyIter = 100;
	sensor_type_t sensorID;
	ssize_t lineLen;
	time_t timestamp, timeStart = READER_TIMESTAMP_NOT_SET;
	sensor_event_t parsed;

	if (rd == NULL || queue == NULL) {
		return -1;
	}

	while ((eventQueue_full(queue) == 0) && (emptyIter > 0)) {
		lineLen = getline(&rd->lineBuf, &rd->bufLen, rd->scenarioFile);
		if (lineLen == -1) {
			if (!feof(rd->scenarioFile)) {
				err = -1;
				break;
			}

			clearerr(rd->scenarioFile);

			/* Going to the beginning of the file and omitting file header */
			if (fseek(rd->scenarioFile, rd->headerLen, SEEK_SET) != 0) {
				err = -1;
				break;
			}
			emptyIter--;

			continue;
		}

		actField = rd->lineBuf;
		if (reader_getFieldLLong(&actField, &tmp) < 0) {
			err = -1;
			break;
		}
		sensorID = tmp;

		/* Checking if we are parsing our header */
		if ((sensorID & rd->sensorTypes) == 0) {
			if (sensorID == READER_END_SCENARIO_INDICATOR) {
				parsed.type = READER_STOP_EVENT;
				if (eventQueue_enqueue(queue, &parsed) < 0) {
					err = -1;
				}

				break;
			}
			emptyIter--;

			continue;
		}

		if (reader_getFieldLLong(&actField, &tmp) < 0) {
			err = -1;
			break;
		}
		timestamp = tmp;

		/* Increasing timeOffset on scenario timestamp decrease (as on file loop) */
		if (timestamp < rd->timeLast && rd->timeLast != READER_TIMESTAMP_NOT_SET) {
			rd->timeOffset += rd->timeLast - timestamp;
		}
		rd->timeLast = timestamp;
		timestamp += rd->timeOffset;

		if (timeStart == READER_TIMESTAMP_NOT_SET) {
			timeStart = timestamp;
		}

		if (timestamp - timeStart > rd->timeHorizon) {
			/* Go to start of the line */
			if (fseek(rd->scenarioFile, -lineLen, SEEK_CUR) != 0) {
				err = -1;
			}
			break;
		}

		switch (sensorID) {
			case SENSOR_TYPE_ACCEL:
				err = reader_accelDataParse(actField, timestamp, &parsed);
				break;
			case SENSOR_TYPE_BARO:
				err = reader_baroDataParse(actField, timestamp, &parsed);
				break;
			case SENSOR_TYPE_GPS:
				err = reader_gpsDataParse(actField, timestamp, &parsed);
				break;
			case SENSOR_TYPE_GYRO:
				err = reader_gyroDataParse(actField, timestamp, &parsed);
				break;
			case SENSOR_TYPE_MAG:
				err = reader_magDataParse(actField, timestamp, &parsed);
				break;
			case SENSOR_TYPE_TEMP:
				err = reader_tempDataParse(actField, timestamp, &parsed);
				break;
			default:
				fprintf(stderr, "%s: Unknown sensor type: %d\n", __FUNCTION__, sensorID);
				err = -1;
				break;
		}

		if (err != 0) {
			break;
		}

		if (eventQueue_enqueue(queue, &parsed) < 0) {
			err = -1;
			break;
		}
	}

	return err;
}


void reader_close(simsens_reader_t *reader)
{
	if (reader == NULL) {
		return;
	}

	if (reader->scenarioFile != NULL) {
		fclose(reader->scenarioFile);
		reader->scenarioFile = NULL;
	}

	free(reader->lineBuf);
	reader->lineBuf = NULL;
}


int reader_open(simsens_reader_t *rd, const char *path, sensor_type_t sensorTypes, time_t timeHorizon)
{
	ssize_t lineLen;
	time_t act_time;
	const char *p;

	if (rd == NULL || path == NULL) {
		return -1;
	}

	rd->scenarioFile = fopen(path, "r");
	if (rd->scenarioFile == NULL) {
		return -1;
	}

	rd->lineBuf = NULL;
	rd->bufLen = 0;
	rd->sensorTypes = sensorTypes;
	rd->timeHorizon = timeHorizon;
	rd->timeLast = READER_TIMESTAMP_NOT_SET;
	rd->headerLen = 0;

	lineLen = getline(&rd->lineBuf, &rd->bufLen, rd->scenarioFile);
	if (lineLen < 1) {
		fclose(rd->scenarioFile);
		return -1;
	}

	/* Checking if current line is header */
	if (rd->lineBuf[0] == 's' || rd->lineBuf[0] == 'S') {
		/* Read next line */
		if (getline(&rd->lineBuf, &rd->bufLen, rd->scenarioFile) < 1) {
			free(rd->lineBuf);
			fclose(rd->scenarioFile);
			return -1;
		}
		rd->headerLen = lineLen;
	}

	/* Omitting first sensorID, to get first event timestamp for timeOffset */
	p = strchr(rd->lineBuf, READER_DATA_SEPARATOR);
	if (p == NULL) {
		free(rd->lineBuf);
		fclose(rd->scenarioFile);
		return -1;
	}

	p++;
	if (reader_getFieldLLong(&p, &rd->timeOffset) != 0) {
		free(rd->lineBuf);
		fclose(rd->scenarioFile);
		return -1;
	}

	gettime(&act_time, NULL);
	rd->timeOffset = act_time - rd->timeOffset;

	if (fseek(rd->scenarioFile, rd->headerLen, SEEK_SET) != 0) {
		free(rd->lineBuf);
		fclose(rd->scenarioFile);
		return -1;
	}

	return 0;
}
