#include <stdio.h>
#include <time.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/threads.h>

#include "grlib-can-core.h"
#include "grlib-can-shared.h"

#define TEST_MSG_BUFFER_SIZE 256
#define TEST_BAUDRATE        1000000

int frames;
char my_stack[4096];

void temp(void *)
{
	for (;;) {
		printf("%d\n", frames);
		sleep(1);
	}
}

int main(int argc, char **argv)
{
	/* Set priority of current thread for RX interrupt */
	priority(1);

	/* Query for devices and configure */
	grlibCan_dev_t devices[GRLIB_MAX_CAN_DEVICES];
	grlibCan_queryForDevices(devices);

	grlibCan_dev_t *device = &devices[0];
	grlibCan_initDevices(device, 1);

	grlibCan_applyDefConf(device);

	grlibCan_config_t config;
	grlibCan_copyConfig(device, &config);

	config.dataBdRate = TEST_BAUDRATE;
	config.nomBdRate = TEST_BAUDRATE;

	grlibCan_applyConfig(device, &config);


	/* Read 1MB of data */
	int bytes_to_read = 1024 * 1024;

	grlibCan_msg_t msg[TEST_MSG_BUFFER_SIZE];

	beginthread(temp, 4, my_stack, 4096, NULL);

	printf("Starting listening\n");
	time_t start = time(NULL);

	uint32_t pending;

	while (bytes_to_read > 0) {
		int ret = grlibCan_recvSync(device, msg, 1, &pending);

		frames += ret * 8;

		for (int i = 0; i < ret; i++) {
			bytes_to_read -= (msg[i].frame.stat >> 28);
		}
	}

	time_t end = time(NULL);

	printf("RECV: Managed to read 1MB of data");
	printf("RECV: Took %lld s\n", end - start);

	return 0;
}
