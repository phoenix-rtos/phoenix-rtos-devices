#include <errno.h>
#include <string.h>

#include <sys/msg.h>
#include <sys/time.h>

#include "imx6ull-sensi2c.h"


static msg_t msg = { 0 };


static int getData(enum dev_types device)
{
	memset(&msg, 0, sizeof(msg));
	int err;

	msg.type = mtRead;

	msg.i.raw[field_devtype] = device;

	msg.i.data = NULL;
	msg.i.size = 0;
	msg.o.data = (void *)NULL;
	msg.o.size = 0;

	err = msgSend(1, &msg);
	if (err != 0) {
		return set_errno(err);
	}

	return EOK;
}


int sensImu(struct sens_imu_t *imu_data)
{
	float *data = (float *)msg.o.raw;
	struct timeval *timestamp;

	if (getData(type_imu) != EOK || imu_data == NULL) {
		return -1;
	}

	timestamp = (struct timeval *)(&data[7]);
	if (timestamp->tv_usec == imu_data->timestamp.tv_usec && timestamp->tv_sec == imu_data->timestamp.tv_sec) {
		return 0;
	}

	imu_data->accel_x = data[0];
	imu_data->accel_y = data[1];
	imu_data->accel_z = data[2];
	imu_data->gyr_x = data[3];
	imu_data->gyr_y = data[4];
	imu_data->gyr_z = data[5];
	imu_data->temp = data[6];
	imu_data->timestamp = *timestamp;

	return 7;
}


int sensMag(struct sens_mag_t *mag_data)
{
	float *data = (float *)msg.o.raw;
	struct timeval *timestamp;

	if (getData(type_magmeter) != EOK || mag_data == NULL) {
		return -1;
	}

	mag_data->mag_x = data[0];
	mag_data->mag_y = data[1];
	mag_data->mag_z = data[2];

	timestamp = (struct timeval *)(&data[3]);
	if (timestamp->tv_usec == mag_data->timestamp.tv_usec && timestamp->tv_sec == mag_data->timestamp.tv_sec) {
		return 0;
	}

	mag_data->timestamp = *timestamp;

	return 3;
}


int sensBaro(struct sens_baro_t *baro_data)
{
	float *data = (float *)msg.o.raw;
	struct timeval *timestamp;

	if (getData(type_baro) != EOK || baro_data == NULL) {
		return -1;
	}

	timestamp = (struct timeval *)(&data[2]);
	if (timestamp->tv_usec == baro_data->timestamp.tv_usec && timestamp->tv_sec == baro_data->timestamp.tv_sec) {
		return 0;
	}

	baro_data->press = data[0];
	baro_data->baro_temp = data[1];
	baro_data->timestamp = *timestamp;

	return 2;
}
