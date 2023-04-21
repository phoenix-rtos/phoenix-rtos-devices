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


static int reader_getFieldLLong(const char *actField, char **nextField, long long *res)
{
	char *endptr;
	long long value = strtoll(actField, &endptr, 10);
	if (!(*endptr == READER_DATA_SEPARATOR || *endptr == '\0' || *endptr == '\n') || actField == endptr) {
		printf("simsensor: Invalid file\n");
		return -1;
	}

	if (*endptr == READER_DATA_SEPARATOR && nextField != NULL) {
		*nextField = endptr + 1;
	}

	*res = value;

	return 0;
}


static int reader_accelDataParse(char *startOfDataSection, time_t timestamp, sensor_event_t *result)
{
	char *actField, *nextField;
	long long tmp;

	actField = startOfDataSection;

	result->type = SENSOR_TYPE_ACCEL;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelX = tmp;
	actField = nextField;

	if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelY = tmp;
	actField = nextField;

	if (reader_getFieldLLong(actField, NULL, &tmp) < 0) {
		return -1;
	}
	result->accels.accelZ = tmp;

	return 0;
}


static int reader_baroDataParse(char *startOfDataSection, time_t timestamp, sensor_event_t *result)
{
	char *actField, *nextField;
	long long tmp;

	actField = startOfDataSection;

	result->type = SENSOR_TYPE_BARO;
	result->timestamp = timestamp;

	if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
		return -1;
	}
	result->baro.pressure = tmp;
	actField = nextField;

	if (reader_getFieldLLong(actField, NULL, &tmp) < 0) {
		return -1;
	}
	result->baro.temp = tmp;

	return 0;
}


int reader_read(simsens_reader_t *rd, event_queue_t *queue)
{
	char *actField, *nextField;
	long long tmp;
	int sensorID, err = 0, emptyIter = 100;
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

		if (reader_getFieldLLong(rd->lineBuf, &nextField, &tmp) < 0) {
			err = -1;
			break;
		}
		sensorID = tmp;
		actField = nextField;

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

		if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
			err = -1;
			break;
		}
		timestamp = tmp;
		actField = nextField;

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


int reader_open(simsens_reader_t *rd, const char *path, int sensorTypes, time_t timeHorizon)
{
	ssize_t lineLen;
	time_t act_time;

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
	char *p = strchr(rd->lineBuf, READER_DATA_SEPARATOR);
	if (p == NULL || reader_getFieldLLong(p + 1, NULL, &rd->timeOffset) != 0) {
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
