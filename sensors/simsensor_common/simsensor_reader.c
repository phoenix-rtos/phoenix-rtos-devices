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
#include <stdbool.h>

#include "simsensor_reader.h"

#define READER_END_SCENARIO_INDICATOR NO_SENSOR
#define READER_DATA_SEPARATOR         ','

#define READER_TIMESTAMP_NOT_SET -1


static int reader_skipHeader(simsens_reader_t *rd)
{
	size_t lineLen = getline(&rd->lineBuf, &rd->bufLen, rd->scenarioFile);
	if (lineLen < 1) {
		return -1;
	}

	/* Checking if current line is header */
	if (rd->lineBuf[0] == 's' || rd->lineBuf[0] == 'S') {
		return 0;
	}

	if (fseek(rd->scenarioFile, -lineLen, SEEK_CUR) != 0) {
		return -1;
	}

	clearerr(rd->scenarioFile);

	return 0;
}


int reader_open(simsens_reader_t *rd, const char *path, int sensorTypes, time_t timeHorizon)
{
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

	rd->timeStruct.firstTimestamp = READER_TIMESTAMP_NOT_SET;
	rd->timeStruct.lastTimestamp = READER_TIMESTAMP_NOT_SET;
	rd->timeStruct.loops = 0;

	if (reader_skipHeader(rd) != 0) {
		fclose(rd->scenarioFile);
		free(rd->lineBuf);
		return -1;
	}

	return 0;
}


void reader_close(simsens_reader_t *reader)
{
	if (reader == NULL) {
		return;
	}

	if (reader->scenarioFile != NULL) {
		fclose(reader->scenarioFile);
	}

	free(reader->lineBuf);
}


static int reader_getFieldLLong(const char *actField, char **nextField, long long *res)
{
	char *endptr;
	long long value = strtoll(actField, &endptr, 10);

	if (!(*endptr == READER_DATA_SEPARATOR || *endptr == '\0' || *endptr == '\n')) {
		return -1;
	}

	if (*endptr == READER_DATA_SEPARATOR) {
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

	if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
		return -1;
	}
	result->accels.accelZ = tmp;
	actField = nextField;

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

	if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
		return -1;
	}
	result->baro.temp = tmp;
	actField = nextField;

	return 0;
}


static time_t reader_nextTimeStampGet(simsens_time_t *timeStruct, time_t timestampFromFile)
{
	if (timeStruct->firstTimestamp == READER_TIMESTAMP_NOT_SET) {
		timeStruct->firstTimestamp = timestampFromFile;
		timeStruct->lastTimestamp = timestampFromFile;
	}
	else if (timeStruct->firstTimestamp == timestampFromFile) {
		timeStruct->loops++;
	}
	else if (timeStruct->lastTimestamp < timestampFromFile) {
		timeStruct->lastTimestamp = timestampFromFile;
	}

	return timestampFromFile + timeStruct->loops * (timeStruct->lastTimestamp - timeStruct->firstTimestamp);
}


int reader_read(simsens_reader_t *rd, event_queue_t *queue)
{
	char *actField, *nextField;
	long long tmp;
	int sensorID, err = 0, maxInter = 200;
	size_t curLineLen;
	time_t timestamp, first_timestamp = READER_TIMESTAMP_NOT_SET;
	sensor_event_t parsedResult;

	while (!eventQueue_full(queue) && maxInter > 0) {
		maxInter--;
		curLineLen = getline(&rd->lineBuf, &rd->bufLen, rd->scenarioFile);
		if (curLineLen == -1) {
			if (!feof(rd->scenarioFile)) {
				err = -1;
				break;
			}

			/* Going to the beginning of the file */
			if (fseek(rd->scenarioFile, 0, SEEK_SET) != 0) {
				err = -1;
				break;
			}
			clearerr(rd->scenarioFile);

			if (reader_skipHeader(rd) != 0) {
				err = -1;
				break;
			}

			continue;
		}

		actField = rd->lineBuf;

		if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
			err = -1;
			break;
		}
		sensorID = tmp;


		if ((sensorID & rd->sensorTypes) == 0) {
			if (sensorID == READER_END_SCENARIO_INDICATOR) {
				break;
			}

			continue;
		}
		actField = nextField;

		if (reader_getFieldLLong(actField, &nextField, &tmp) < 0) {
			err = -1;
			break;
		}
		timestamp = tmp;
		actField = nextField;

		timestamp = reader_nextTimeStampGet(&rd->timeStruct, timestamp);

		if (first_timestamp == READER_TIMESTAMP_NOT_SET) {
			first_timestamp = timestamp;
		}

		if (timestamp - first_timestamp > rd->timeHorizon) {
			/* Go to start of the line */
			if (fseek(rd->scenarioFile, -curLineLen, SEEK_CUR) != 0) {
				err = -1;
			}
			break;
		}

		switch (sensorID) {
			case SENSOR_TYPE_ACCEL:
				err = reader_accelDataParse(actField, timestamp, &parsedResult);
				break;
			case SENSOR_TYPE_BARO:
				err = reader_baroDataParse(actField, timestamp, &parsedResult);
				break;
			default:
				fprintf(stderr, "%s: Unknown sensor type\n", __FUNCTION__);
				err = -1;
				break;
		}

		if (err < 0) {
			break;
		}

		if (eventQueue_enqueue(queue, &parsedResult) < 0) {
			err = -1;
			break;
		}
	}

	return (err < 0) ? err : 0;
}
