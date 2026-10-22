#include <stdio.h>
#include <time.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>

#include "grlib-can-core.h"
#include "grlib-can-shared.h"

#define TEST_MSG_BUFFER_SIZE 200
#define TEST_BAUDRATE        1000000

int main(int argc, char **argv)
{
	/* Query for devices and configure */
	grlibCan_dev_t devices[GRLIB_MAX_CAN_DEVICES];
	grlibCan_queryForDevices(devices);

	grlibCan_dev_t *device = &devices[1];
	grlibCan_initDevices(device, 1);

	grlibCan_applyDefConf(device);

	grlibCan_config_t config;
	grlibCan_copyConfig(device, &config);

	config.dataBdRate = TEST_BAUDRATE;
	config.nomBdRate = TEST_BAUDRATE;

	/* Turn off loopback */
	config.conf &= ~((1 << 7) | (1 << 6));

	grlibCan_applyConfig(device, &config);

	/* Send 1MB of data */
	int bytes_to_send = 1024 * 1024;

	grlibCan_msg_t msg_buffer[TEST_MSG_BUFFER_SIZE];
	for (int i = 0; i < TEST_MSG_BUFFER_SIZE; i++) {
		msg_buffer[i].frame.head = (1 << 31) | (0xFA << 17);
		msg_buffer[i].frame.stat = 8 << 28;
		memset(msg_buffer[i].frame.payload, 0xFA, 8);
	}

	time_t start = time(NULL);

	while (bytes_to_send > 0) {
		int ret = grlibCan_transmitAsync(device, msg_buffer, 1);
		if (ret <= 0) {
			continue;
		}

		bytes_to_send -= 8 * ret;
	}

	time_t end = time(NULL);

	printf("SEND: Sent 1MB of data, took: %lld\n", end - start);
	printf("SEND: Avg rate: %fbps effective\n", (float)(1024 * 1024 * 8) / (end - start));

	return 0;
}
