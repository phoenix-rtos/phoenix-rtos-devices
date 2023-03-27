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

#include "simsensor_reader.h"

#define READER_END_SCENARIO_INDICATOR NO_SENSOR
#define READER_DATA_SEPARATOR         ','

#define READER_TIMESTAMP_NOT_SET -1

#define READER_ACCEL_DEV_ID 1
#define READER_BARO_DEV_ID  2


typedef struct {
	time_t first_scenario_time;
	time_t last_scenario_timestamp;
	int loops_cnt;
} simsens_time_t;


struct simsens_reader_t {
	FILE *scenario_file;
	int sensor_types;

	time_t time_horizon;

	simsens_time_t time_struct;
};


static void reader_timeInit(simsens_time_t *timeStruct)
{
	timeStruct->first_scenario_time = READER_TIMESTAMP_NOT_SET;
	timeStruct->last_scenario_timestamp = READER_TIMESTAMP_NOT_SET;
	timeStruct->loops_cnt = 0;
}


simsens_reader_t *reader_open(const char *path, int sensor_types, time_t time_horizon)
{
	simsens_reader_t *result = malloc(sizeof(simsens_reader_t));
	if (result == NULL) {
		return NULL;
	}

	result->scenario_file = fopen(path, "r");
	if (result->scenario_file == NULL) {
		free(result);
		return NULL;
	}

	result->sensor_types = sensor_types;
	result->time_horizon = time_horizon;

	reader_timeInit(&result->time_struct);

	return result;
}


void reader_close(simsens_reader_t *reader)
{
	if (reader == NULL) {
		return;
	}

	fclose(reader->scenario_file);
	free(reader);
}


static inline int reader_isLineHeader(const char *line)
{
	return line[0] == 's' || line[0] == 'S';
}


static long int reader_getFieldLong(const char *actField, char **nextField, int *error)
{
	char *endptr;
	long int result = strtol(actField, &endptr, 10);

	if (!(*endptr == READER_DATA_SEPARATOR || *endptr == '\0' || *endptr == '\n')) {
		*error = -1;
		return -1;
	}

	if (*endptr == READER_DATA_SEPARATOR) {
		*nextField = endptr + 1;
	}

	return result;
}


static int reader_accelDataParse(char *startOfDataSection, time_t timestamp, sensor_event_t *result)
{
	char *actField, *nextField;
	int err = 0;

	actField = startOfDataSection;


	result->type = SENSOR_TYPE_ACCEL;
	result->timestamp = timestamp;
	result->accels.devId = READER_ACCEL_DEV_ID;

	result->accels.accelX = reader_getFieldLong(actField, &nextField, &err);
	if (err != 0) {
		return -1;
	}
	actField = nextField;

	result->accels.accelY = reader_getFieldLong(actField, &nextField, &err);
	if (err != 0) {
		return -1;
	}
	actField = nextField;

	result->accels.accelZ = reader_getFieldLong(actField, &nextField, &err);
	if (err != 0) {
		return -1;
	}
	actField = nextField;

	return 0;
}


static int reader_baroDataParse(char *startOfDataSection, time_t timestamp, sensor_event_t *result)
{
	char *actField, *nextField;
	int err = 0;

	actField = startOfDataSection;


	result->type = SENSOR_TYPE_BARO;
	result->timestamp = timestamp;
	result->baro.devId = READER_BARO_DEV_ID;

	result->baro.pressure = reader_getFieldLong(actField, &nextField, &err);
	if (err != 0) {
		return -1;
	}
	actField = nextField;

	result->baro.temp = reader_getFieldLong(actField, &nextField, &err);
	if (err != 0) {
		return -1;
	}
	actField = nextField;

	return 0;
}


static time_t reader_nextTimeStampGet(simsens_time_t *timeStruct, time_t timestampFromFile)
{
	if (timeStruct->first_scenario_time == READER_TIMESTAMP_NOT_SET) {
		timeStruct->first_scenario_time = timestampFromFile;
		timeStruct->last_scenario_timestamp = timestampFromFile;
	}
	else if (timeStruct->first_scenario_time == timestampFromFile) {
		timeStruct->loops_cnt++;
	}
	else if (timeStruct->last_scenario_timestamp < timestampFromFile) {
		timeStruct->last_scenario_timestamp = timestampFromFile;
	}

	return timestampFromFile + timeStruct->loops_cnt * (timeStruct->last_scenario_timestamp - timeStruct->first_scenario_time);
}


int reader_read(simsens_reader_t *rd, event_queue_t *queue)
{
	char *lineBuf = NULL, *actField, *nextField;
	int sensorID, err = 0;
	size_t bufLen = 0, curLineLen;
	time_t timestamp, first_timestamp = READER_TIMESTAMP_NOT_SET;
	sensor_event_t parsed_result;

	while (eventQueue_canEnqueue(queue)) {
		curLineLen = getline(&lineBuf, &bufLen, rd->scenario_file);
		if (curLineLen == -1) {
			if (!feof(rd->scenario_file)) {
				err = -1;
				break;
			}

			if (fseek(rd->scenario_file, 0, SEEK_SET) != 0) {
				err = -1;
				break;
			}
			continue;
		}

		if (reader_isLineHeader(lineBuf)) {
			continue;
		}

		actField = lineBuf;

		sensorID = reader_getFieldLong(actField, &nextField, &err);
		if (err < 0) {
			break;
		}

		if (sensorID == READER_END_SCENARIO_INDICATOR) {
			break;
		}

		if ((sensorID & rd->sensor_types) == 0) {
			continue;
		}
		actField = nextField;

		timestamp = reader_getFieldLong(actField, &nextField, &err);
		if (err < 0) {
			break;
		}
		actField = nextField;

		timestamp = reader_nextTimeStampGet(&rd->time_struct, timestamp);

		if (first_timestamp == READER_TIMESTAMP_NOT_SET) {
			first_timestamp = timestamp;
		}

		if (timestamp - first_timestamp > rd->time_horizon) {
			/* Go to start of the line */
			if (fseek(rd->scenario_file, -curLineLen, SEEK_CUR) != 0) {
				err = -1;
			}
			break;
		}

		switch (sensorID) {
			case SENSOR_TYPE_ACCEL:
				if (reader_accelDataParse(actField, timestamp, &parsed_result) < 0) {
					err = -1;
				}
				break;
			case SENSOR_TYPE_BARO:
				if (reader_baroDataParse(actField, timestamp, &parsed_result) < 0) {
					err = -1;
				}
				break;
		}

		if (err < 0) {
			break;
		}

		if (eventQueue_enqueue(queue, &parsed_result) < 0) {
			free(lineBuf);
			return -1;
		}
	}

	free(lineBuf);

	return err < 0 ? err : 0;
}
