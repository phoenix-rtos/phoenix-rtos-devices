# Sensor manager

Module provides a basic interface for a variety of sensors.

Assumptions:
 - the client communicates with manager via messages to **/dev/sensors**,
 - there is an external library **libsensors** containing sensor structures and message events - external API for the client
 - sensors server takes sensors description as arguments list:  <sensor_name:bus:others>; arguments might be specific for the sensor driver,
 - each sensor registers itself in a sensor manager via constructor,

# Examples


## Server usage
``` sh
sensors -s lsm9dsxx:/dev/spi1.0:argv -s lps25xx:/dev/i2c0`
```

## Client app:

```c
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <libsensors.h>

int main(int argc, char **argv)
{
	int fd;
	sensors_data_t *data;
	unsigned char buff[0x400];
	sensor_type_t types;
	sensors_ops_t ops = {0};

	fd = open("/dev/sensors", O_RDWR);
	if (fd < 0) {
		return EXIT_FAILURE;
	}

	/* Get available sensor types */
	ioctl(fd, SMIOC_SENSORSAVAIL, &types);

	/* Define sensor types request */
	ops.types = (types & SENSOR_TYPE_BARO) | (types & SENSOR_TYPE_ACCEL) | (types & SENSOR_TYPE_GYRO);
	ioctl(fd, SMIOC_SENSORSSET, &ops);

	if ((ops.evtSz * sizeof(sensor_event_t) + sizeof(((sensors_data_t *)0)->size)) > sizeof(buff)) {
		fprintf(stderr, "Buff is too small\n");
		return EXIT_FAILURE;
	}

	for (int i = 0; i < 15; ++i) {
		sleep(1);
		read(fd, buff, 0x600);

		data = (sensors_data_t *)buff;
		printf("Sz: %d\n", data->size);

		for (int j = 0; j < data->size; ++j) {
			switch (data->events[j].type) {
			case SENSOR_TYPE_ACCEL:
				printf("Accel, timestamp: %llu\n", data->events[j].timestamp);
				printf("acce - x: %u, y: %u, z: %u\n", data->events[j].accels.accelX, data->events[j].accels.accelY, data->events[j].accels.accelZ);
				break;

			case SENSOR_TYPE_BARO:
				printf("Baro, timestamp: %llu\n", data->events[j].timestamp);
				printf("baro - pressure: %u\n", data->events[j].baro.pressure);
				break;

			case SENSOR_TYPE_GYRO:
				printf("Gyro, timestamp: %llu\n", data->events[j].timestamp);
				printf("gyro - x: %u, y: %u, z: %u\n", data->events[j].gyro.gyroX, data->events[j].gyro.gyroY, data->events[j].gyro.gyroZ);
				break;

			default:
				break;
			}
		}
	}

	close(fd);

	return EXIT_SUCCESS;
}
```
</details>