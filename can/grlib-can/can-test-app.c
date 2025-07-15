#include <errno.h>
#include <paths.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/debug.h>
#include <sys/file.h>
#include <sys/msg.h>
#include <sys/interrupt.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/threads.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <posix/utils.h>

#include "grlib-can-if.h"

int main(int argc, char **argv)
{
	char *canDevice = "/dev/can0?";
	oid_t canDev;

	while (lookup(canDevice, NULL, &canDev) < 0) {
		printf("Device does not exist\n");
		return EXIT_FAILURE;
	}

	printf("Successfully checked CAN device oid\n");

	uint32_t status;
	grlibCan_getStatus(canDev, &status);
	printf("Can device status: %x\n", status);

	grlibCan_config_t config;

	grlibCan_getConfig(canDev, &config);
	printf("Can device baud rate - nom: %d, data: %d\n", config.nomBdRate, config.dataBdRate);

	grlibCan_msg_t frames[10];

	frames->frame.head = (1 << 18);
	frames->frame.stat = 0x10u << 24u;
	frames->frame.payload[0] = 0x0F;

	for (int i = 1; i < 10; i++) {
		memcpy((void *)&frames[i], (void *)frames, sizeof(grlibCan_msg_t));
		frames[i].frame.payload[0] = 0x0F + i;
	}

	int ret = grlibCan_Send(canDev, frames, 10, true);
	printf("Can device managed to send %d frames\n", ret);

	grlibCan_msg_t buf[10];

	ret = grlibCan_Send(canDev, buf, 10, true);
	printf("Can device managed to receive %d frames\n", ret);


	return EXIT_SUCCESS;
}
